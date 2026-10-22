// bear_yolo_node: RGB YOLO bear detection on the drone camera feed.
//
// Subscribes:  image_topic (sensor_msgs/Image, default "camera/image", i.e. /parrot1/camera/image
//              when launched in the parrot1 namespace)
// Publishes:   bear_detections        vision_msgs/Detection2DArray  (one per processed frame;
//                                     empty array = frame processed, no bear)
//              bear_detected          std_msgs/Bool                 (convenience flag, same frame)
//              bear_detections/image  sensor_msgs/Image (bgr8)      (annotated, only if subscribed)
// All outputs carry the source image's header (timestamp + frame_id).
//
// Threading: the camera callback only stores the newest frame (no queue, no backlog). A timer in a
// separate callback group runs inference at process_rate_hz on that newest frame, so slow
// inference drops frames instead of building latency, and never blocks image reception.
//
// The detector reports image-plane boxes only. It does not estimate distance or world position.
// Thermal detection is not part of this node and is not required for it to run.

#include <algorithm>
#include <cstdint>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "grizzlyguard_perception/yolo_detector.hpp"

namespace gp = grizzlyguard_perception;
using namespace std::chrono_literals;

class BearYoloNode : public rclcpp::Node
{
public:
  BearYoloNode()
  : Node("bear_yolo_node")
  {
    const auto model_param = declare_parameter<std::string>("model_path", "yolo26n.onnx");
    const auto image_topic = declare_parameter<std::string>("image_topic", "camera/image");
    target_class_ = declare_parameter<std::string>("target_class", "bear");
    const auto target_id_param = declare_parameter<int>("target_class_id", -1);
    auto class_names = declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{});
    if (class_names.size() == 1 && class_names[0].empty()) {class_names.clear();}
    const double conf = declare_parameter<double>("confidence_threshold", 0.25);
    const double iou = declare_parameter<double>("iou_threshold", 0.45);
    const int imgsz = declare_parameter<int>("inference_size", 640);
    const double rate = declare_parameter<double>("process_rate_hz", 3.0);
    max_frame_age_s_ = declare_parameter<double>("max_frame_age_s", 1.0);
    const auto provider = declare_parameter<std::string>("execution_provider", "cpu");
    const int threads = declare_parameter<int>("intra_op_threads", 2);
    const auto fmt = declare_parameter<std::string>("output_format", "auto");
    publish_annotated_ = declare_parameter<bool>("publish_annotated", true);
    annotate_other_classes_ = declare_parameter<bool>("annotate_other_classes", false);

    if (rate <= 0.0) {
      throw std::runtime_error("process_rate_hz must be > 0");
    }

    // ---- Load the model once. Any failure aborts start-up with a clear reason.
    gp::YoloDetector::Config cfg;
    cfg.model_path = resolveModelPath(model_param);
    cfg.input_size = imgsz;
    cfg.conf_threshold = static_cast<float>(conf);
    cfg.iou_threshold = static_cast<float>(iou);
    cfg.execution_provider = provider;
    cfg.intra_op_threads = threads;
    cfg.output_format = gp::parseOutputFormat(fmt);
    cfg.class_names = class_names;
    detector_ = std::make_unique<gp::YoloDetector>(cfg);

    // ---- Resolve the target class from the model's class map (or explicit id/names).
    if (target_id_param >= 0) {
      target_id_ = static_cast<int>(target_id_param);
      auto it = detector_->classNames().find(target_id_);
      if (!detector_->classNames().empty() && it == detector_->classNames().end()) {
        throw std::runtime_error("target_class_id " + std::to_string(target_id_) +
                                 " is not a class of this model");
      }
      if (it != detector_->classNames().end()) {target_class_ = it->second;}
    } else {
      target_id_ = gp::resolveClassId(detector_->classNames(), target_class_);
      if (target_id_ < 0) {
        throw std::runtime_error("target_class '" + target_class_ + "' not found in the model's " +
                                 "class names; set class_names or target_class_id");
      }
    }

    RCLCPP_INFO(get_logger(),
      "Model %s loaded: %s, input %dx%d, provider %s, target '%s' = class %d, conf >= %.2f",
      cfg.model_path.c_str(), gp::toString(detector_->outputFormat()).c_str(),
      detector_->inputSize(), detector_->inputSize(), detector_->providerInUse().c_str(),
      target_class_.c_str(), target_id_, conf);
    if (provider == "cuda" && detector_->providerInUse() != "cuda") {
      RCLCPP_WARN(get_logger(), "Falling back to CPU inference: %s",
        detector_->providerInUse().c_str());
    }

    // ---- ROS interfaces.
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("bear_detections", 10);
    flag_pub_ = create_publisher<std_msgs::msg::Bool>("bear_detected", 10);
    if (publish_annotated_) {
      img_pub_ = create_publisher<sensor_msgs::msg::Image>("bear_detections/image", 5);
    }

    rx_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    infer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = rx_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(std::move(msg));}, sub_opts);

    // Wall-clock timer: inference pacing must not depend on /clock (sim time may pause).
    timer_ = rclcpp::create_timer(
      this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME),
      rclcpp::Duration::from_seconds(1.0 / rate), [this]() {processLatest();}, infer_group_);

    RCLCPP_INFO(get_logger(), "Listening on %s at up to %.1f Hz",
      image_sub_->get_topic_name(), rate);
    start_ = Steady::now();
    last_stats_ = start_;
  }

private:
  using Steady = std::chrono::steady_clock;

  std::string resolveModelPath(const std::string & p)
  {
    if (p.empty()) {throw std::runtime_error("model_path parameter is empty");}
    std::filesystem::path path(p);
    if (!path.is_absolute()) {
      path = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("grizzlyguard_perception")) / "models" / p;
    }
    if (!std::filesystem::exists(path)) {
      throw std::runtime_error("model file not found: " + path.string() +
                               " (see grizzlyguard_perception/models/README.md to export it)");
    }
    return path.string();
  }

  void onImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(msg);   // overwrite: only the newest frame is ever processed
    latest_rx_ = Steady::now();
    ++latest_seq_;
  }

  void processLatest()
  {
    sensor_msgs::msg::Image::ConstSharedPtr msg;
    Steady::time_point rx;
    uint64_t seq;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_;
      rx = latest_rx_;
      seq = latest_seq_;
    }
    const auto now = Steady::now();

    if (!msg) {
      if (now - start_ > 5s) {
        RCLCPP_WARN_THROTTLE(get_logger(), *steady_clock_, 5000,
          "No camera frames received on %s yet", image_sub_->get_topic_name());
      }
      return;
    }
    if (seq == processed_seq_) {
      if (now - rx > 5s) {
        RCLCPP_WARN_THROTTLE(get_logger(), *steady_clock_, 5000,
          "Camera frames stopped (last one %.1f s ago); not publishing stale detections",
          std::chrono::duration<double>(now - rx).count());
      }
      return;   // nothing new: never re-publish an old frame's result
    }
    processed_seq_ = seq;

    const double age = std::chrono::duration<double>(now - rx).count();
    if (age > max_frame_age_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *steady_clock_, 5000,
        "Dropping frame that waited %.2f s (> max_frame_age_s %.2f)", age, max_frame_age_s_);
      return;
    }

    cv_bridge::CvImageConstPtr cv;
    try {
      cv = cv_bridge::toCvShare(msg, "bgr8");   // handles rgb8 (Gazebo), bgra8, mono8, ...
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *steady_clock_, 5000,
        "Cannot convert image (encoding '%s'): %s", msg->encoding.c_str(), e.what());
      return;
    }
    if (cv->image.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *steady_clock_, 5000, "Received an empty image");
      return;
    }

    std::vector<gp::Detection> all;
    const auto t0 = Steady::now();
    try {
      all = detector_->detect(cv->image);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *steady_clock_, 5000, "Inference failed: %s", e.what());
      return;
    }
    const double infer_ms = std::chrono::duration<double, std::milli>(Steady::now() - t0).count();

    vision_msgs::msg::Detection2DArray out;
    out.header = msg->header;   // timestamp + frame_id of the source image
    std::vector<gp::Detection> bears;
    for (const auto & d : all) {
      if (d.class_id != target_id_) {continue;}
      bears.push_back(d);
      vision_msgs::msg::Detection2D det;
      det.header = msg->header;
      det.bbox.center.position.x = d.box.x + d.box.width / 2.0;
      det.bbox.center.position.y = d.box.y + d.box.height / 2.0;
      det.bbox.center.theta = 0.0;
      det.bbox.size_x = d.box.width;
      det.bbox.size_y = d.box.height;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = d.label;
      hyp.hypothesis.score = d.score;
      det.results.push_back(hyp);
      out.detections.push_back(det);
    }
    det_pub_->publish(out);
    std_msgs::msg::Bool flag;
    flag.data = !bears.empty();
    flag_pub_->publish(flag);

    if (img_pub_ && img_pub_->get_subscription_count() > 0) {
      cv::Mat vis = cv->image.clone();
      gp::drawDetections(vis, annotate_other_classes_ ? all : bears, target_id_);
      img_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg());
    }

    // Latency statistics so a growing backlog is visible in the log.
    const double total_ms = std::chrono::duration<double, std::milli>(Steady::now() - rx).count();
    ++n_frames_;
    sum_infer_ms_ += infer_ms;
    max_total_ms_ = std::max(max_total_ms_, total_ms);
    if (!bears.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *steady_clock_, 2000, "%zu bear(s), best %.2f",
        bears.size(), bears.front().score);
    }
    if (now - last_stats_ > 10s) {
      RCLCPP_INFO(get_logger(),
        "%lu frames processed, mean inference %.1f ms, max receive-to-publish %.1f ms",
        static_cast<unsigned long>(n_frames_), sum_infer_ms_ / n_frames_, max_total_ms_);
      last_stats_ = now;
      n_frames_ = 0;
      sum_infer_ms_ = 0.0;
      max_total_ms_ = 0.0;
    }
  }

  // Log throttling on wall time, so warnings still appear if simulation time is paused.
  rclcpp::Clock::SharedPtr steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  std::unique_ptr<gp::YoloDetector> detector_;
  std::string target_class_;
  int target_id_ = -1;
  double max_frame_age_s_ = 1.0;
  bool publish_annotated_ = true;
  bool annotate_other_classes_ = false;

  rclcpp::CallbackGroup::SharedPtr rx_group_, infer_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_;
  Steady::time_point latest_rx_;
  uint64_t latest_seq_ = 0;
  uint64_t processed_seq_ = 0;

  Steady::time_point start_, last_stats_;
  uint64_t n_frames_ = 0;
  double sum_infer_ms_ = 0.0, max_total_ms_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<BearYoloNode> node;
  try {
    node = std::make_shared<BearYoloNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("bear_yolo_node"), "Start-up failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}