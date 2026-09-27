#include <rclcpp/rclcpp.hpp>

#include "grizzlyguard_perception/thermal_perception_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<grizzlyguard_perception::ThermalPerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
