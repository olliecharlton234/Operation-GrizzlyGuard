// Copyright 2026 Operation GrizzlyGuard (Group 15)

#ifndef GRIZZLYGUARD_PERCEPTION__THERMAL_PERCEPTION_NODE_HPP_
#define GRIZZLYGUARD_PERCEPTION__THERMAL_PERCEPTION_NODE_HPP_

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "grizzlyguard_perception/thermal_detector.hpp"

namespace grizzlyguard_perception
{

/// ROS 2 node wrapping the colour-segmentation "warm object" pipeline
/// (see thermal_detector.hpp) around a live sensor_msgs/Image topic.
///
/// This is a *prototype placeholder* for a real thermal camera: it treats
/// red/orange/yellow pixels in the drone's simulated RGB camera as "warm",
/// and looks for a sufficiently large warm blob to flag as a possible bear
/// heat signature. See the package README for limitations and the
/// recommended next step.
class ThermalPerceptionNode : public rclcpp::Node
{
public:
  explicit ThermalPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void declareHsvRangeParameters(const std::string & prefix, const HsvRange & defaults);
  HsvRange readHsvRangeParameter(const std::string & prefix) const;
  ThermalDetectorParams readParameters() const;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pseudo_thermal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heat_detected_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr heat_centre_pub_;

  bool publish_pseudo_thermal_ = true;
  int64_t log_throttle_ms_ = 2000;
};

}  // namespace grizzlyguard_perception

#endif  // GRIZZLYGUARD_PERCEPTION__THERMAL_PERCEPTION_NODE_HPP_
