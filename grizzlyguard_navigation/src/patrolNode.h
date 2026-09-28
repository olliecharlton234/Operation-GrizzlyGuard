#ifndef IGNITION_41068_BRINGUP__PATROL_NODE_HPP_
#define IGNITION_41068_BRINGUP__PATROL_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bringup_41068
{

/// Sends a fixed, looping list of waypoints to Nav2's FollowWaypoints action
/// server so the robot patrols the same route indefinitely.
///
/// Topic/action names are relative, matching the convention used elsewhere in
/// this package: when launched in the /parrot1 namespace, the action name
/// "follow_waypoints" resolves to "/parrot1/follow_waypoints".
class PatrolNode : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  PatrolNode();

private:
  /// Build a PoseStamped in the robot's map frame from a flat (x, y) point.
  /// Orientation is left as identity; Nav2's planner/controller choose heading.
  geometry_msgs::msg::PoseStamped makePose(double x, double y) const;

  /// Timer callback: waits for the action server, then hands off to
  /// sendPatrol() and stops ticking (sendPatrol re-arms the loop itself via
  /// its result callback).
  void tick();

  /// Stamps the waypoints and sends them as one FollowWaypoints goal. On
  /// completion (success or failure), immediately sends the same goal again,
  /// producing an indefinite patrol loop.
  void sendPatrol();

  std::string robot_name_;
  std::string map_frame_;

  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  bool goal_active_ = false;
};

}  // namespace bringup_41068

#endif  // IGNITION_41068_BRINGUP__PATROL_NODE_HPP_