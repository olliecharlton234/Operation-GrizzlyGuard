// Copyright 2026 Operation GrizzlyGuard (Group 15)

#include "grizzlyguard_perception/thermal_perception_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.hpp>

namespace grizzlyguard_perception
{

ThermalPerceptionNode::ThermalPerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("thermal_perception_node", options)
{
  declareParameters();

  const std::string image_topic = get_parameter("image_topic").as_string();
  publish_pseudo_thermal_ = get_parameter("publish_pseudo_thermal").as_bool();
  log_throttle_ms_ = static_cast<int64_t>(
    get_parameter("log_throttle_seconds").as_double() * 1000.0);

  // Publishers use plain sensor_msgs/Image publishers (rather than
  // image_transport) to keep this prototype's dependencies minimal; the
  // topic names below are fixed by the GrizzlyGuard perception interface.
  overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(
    "/grizzlyguard/perception/thermal_overlay", rclcpp::SensorDataQoS());
  mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
    "/grizzlyguard/perception/thermal_mask", rclcpp::SensorDataQoS());
  pseudo_thermal_pub_ = create_publisher<sensor_msgs::msg::Image>(
    "/grizzlyguard/perception/pseudo_thermal", rclcpp::SensorDataQoS());
  heat_detected_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/grizzlyguard/perception/heat_detected", rclcpp::QoS(10));
  // No custom "heat detection" interface exists yet in grizzlyguard_interfaces,
  // so the centre is published as a plain geometry_msgs/Point (pixel
  // coordinates, z unused). A custom message combining centre + size +
  // confidence would be a natural addition there later -- see the README.
  heat_centre_pub_ = create_publisher<geometry_msgs::msg::Point>(
    "/grizzlyguard/perception/heat_centre", rclcpp::QoS(10));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    std::bind(&ThermalPerceptionNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Thermal perception prototype started, subscribing to '%s'. "
    "NOTE: this uses colour segmentation on the RGB camera as a placeholder "
    "for a real thermal sensor -- it is a visual simulation only.",
    image_topic.c_str());
}

void ThermalPerceptionNode::declareParameters()
{
  const ThermalDetectorParams defaults;

  declare_parameter<std::string>("image_topic", "/parrot1/camera/image");
  declare_parameter<bool>("publish_pseudo_thermal", true);
  declare_parameter<double>("log_throttle_seconds", 2.0);

  declare_parameter<double>("min_contour_area", defaults.min_contour_area);
  declare_parameter<double>("confidence_threshold", defaults.confidence_threshold);

  declare_parameter<int>("morph_kernel_size", defaults.morph_kernel_size);
  declare_parameter<int>("morph_open_iterations", defaults.morph_open_iterations);
  declare_parameter<int>("morph_close_iterations", defaults.morph_close_iterations);

  declareHsvRangeParameters("red_low", defaults.red_low);
  declareHsvRangeParameters("red_high", defaults.red_high);
  declareHsvRangeParameters("orange", defaults.orange);
  declareHsvRangeParameters("yellow", defaults.yellow);
}

void ThermalPerceptionNode::declareHsvRangeParameters(
  const std::string & prefix, const HsvRange & defaults)
{
  declare_parameter<int>(prefix + ".h_min", defaults.h_min);
  declare_parameter<int>(prefix + ".h_max", defaults.h_max);
  declare_parameter<int>(prefix + ".s_min", defaults.s_min);
  declare_parameter<int>(prefix + ".s_max", defaults.s_max);
  declare_parameter<int>(prefix + ".v_min", defaults.v_min);
  declare_parameter<int>(prefix + ".v_max", defaults.v_max);
}

HsvRange ThermalPerceptionNode::readHsvRangeParameter(const std::string & prefix) const
{
  HsvRange range;
  range.h_min = static_cast<int>(get_parameter(prefix + ".h_min").as_int());
  range.h_max = static_cast<int>(get_parameter(prefix + ".h_max").as_int());
  range.s_min = static_cast<int>(get_parameter(prefix + ".s_min").as_int());
  range.s_max = static_cast<int>(get_parameter(prefix + ".s_max").as_int());
  range.v_min = static_cast<int>(get_parameter(prefix + ".v_min").as_int());
  range.v_max = static_cast<int>(get_parameter(prefix + ".v_max").as_int());
  return range;
}

ThermalDetectorParams ThermalPerceptionNode::readParameters() const
{
  // Re-read every frame (cheap relative to the image processing below) so
  // that `ros2 param set` changes to HSV thresholds take effect live -- this
  // is handy for tuning the demo without restarting the node.
  ThermalDetectorParams params;
  params.red_low = readHsvRangeParameter("red_low");
  params.red_high = readHsvRangeParameter("red_high");
  params.orange = readHsvRangeParameter("orange");
  params.yellow = readHsvRangeParameter("yellow");

  params.morph_kernel_size = static_cast<int>(get_parameter("morph_kernel_size").as_int());
  params.morph_open_iterations = static_cast<int>(
    get_parameter("morph_open_iterations").as_int());
  params.morph_close_iterations = static_cast<int>(
    get_parameter("morph_close_iterations").as_int());

  params.min_contour_area = get_parameter("min_contour_area").as_double();
  params.confidence_threshold = get_parameter("confidence_threshold").as_double();

  return params;
}

void ThermalPerceptionNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // Handle missing/invalid frames safely rather than crashing.
  if (!msg || msg->height == 0 || msg->width == 0 || msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "Received an empty/invalid camera frame; skipping this frame.");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "cv_bridge conversion failed (encoding '%s'): %s", msg->encoding.c_str(), e.what());
    return;
  }

  if (cv_ptr->image.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "Converted camera frame is empty; skipping this frame.");
    return;
  }

  const cv::Mat & bgr_image = cv_ptr->image;
  const ThermalDetectorParams params = readParameters();

  const cv::Mat warm_mask = computeWarmMask(bgr_image, params);
  const DetectionResult result = detectWarmRegion(warm_mask, params);
  const cv::Mat overlay = drawDetectionOverlay(bgr_image, result);

  if (result.detected) {
    // Throttled: reports detections without printing on every frame.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "WARM OBJECT DETECTED centre=(%.0f, %.0f) bbox=[%d,%d,%dx%d] area=%.0fpx^2 confidence=%.2f",
      static_cast<double>(result.centre.x), static_cast<double>(result.centre.y),
      result.bounding_box.x, result.bounding_box.y,
      result.bounding_box.width, result.bounding_box.height,
      result.area, result.confidence);
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_, "NO WARM OBJECT DETECTED");
  }

  std_msgs::msg::Bool heat_detected_msg;
  heat_detected_msg.data = result.detected;
  heat_detected_pub_->publish(heat_detected_msg);

  geometry_msgs::msg::Point centre_msg;
  centre_msg.x = static_cast<double>(result.centre.x);
  centre_msg.y = static_cast<double>(result.centre.y);
  centre_msg.z = 0.0;
  heat_centre_pub_->publish(centre_msg);

  overlay_pub_->publish(
    *cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg());
  mask_pub_->publish(
    *cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, warm_mask).toImageMsg());

  if (publish_pseudo_thermal_) {
    const cv::Mat pseudo_thermal = computePseudoThermal(bgr_image, result);
    pseudo_thermal_pub_->publish(
      *cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::BGR8, pseudo_thermal).toImageMsg());
  }
}

}  // namespace grizzlyguard_perception
