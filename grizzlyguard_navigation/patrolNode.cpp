#include "patrolNode.h"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace bringup_41068
{

PatrolNode::PatrolNode()
: Node("patrol_node")
{
  this->declare_parameter("robot_name", "parrot1");
  robot_name_ = this->get_parameter("robot_name").as_string();
  map_frame_ = robot_name_ + "_map";

  // Relative action name -> becomes /parrot1/follow_waypoints when this node
  // is launched inside the parrot1 namespace, matching the rest of the
  // package's namespacing convention.
  client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

  // Hardcoded patrol route. Replace with real coordinates from your world
  // (read them off the SLAM map in RViz, or from known-free points in the
  // .sdf world file).
  waypoints_ = {
    makePose(2.0, 2.0),
    makePose(5.0, -3.0),
    makePose(-4.0, -2.0),
    makePose(-1.0, 4.0),
  };

  timer_ = this->create_wall_timer(1s, std::bind(&PatrolNode::tick, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Patrol node started for robot \"%s\" with %zu waypoints in frame \"%s\".",
    robot_name_.c_str(), waypoints_.size(), map_frame_.c_str());
}

geometry_msgs::msg::PoseStamped PatrolNode::makePose(double x, double y) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = map_frame_;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

void PatrolNode::tick()
{
  if (goal_active_) {
    return;
  }

  if (!client_->wait_for_action_server(0s)) {
    RCLCPP_INFO(this->get_logger(), "Waiting for follow_waypoints action server...");
    return;
  }

  // We're ready: stop polling on the timer and hand control to the
  // goal-send/result-callback loop in sendPatrol().
  timer_->cancel();
  sendPatrol();
}

void PatrolNode::sendPatrol()
{
  for (auto & pose : waypoints_) {
    pose.header.stamp = this->get_clock()->now();
  }

  auto goal_msg = FollowWaypoints::Goal();
  goal_msg.poses = waypoints_;

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const GoalHandleFollowWaypoints::WrappedResult & result)
    {
      (void)result;
      RCLCPP_INFO(this->get_logger(), "Patrol loop finished. Restarting.");
      goal_active_ = false;
      sendPatrol();  // loop forever
    };

  goal_active_ = true;
  RCLCPP_INFO(this->get_logger(), "Sending patrol waypoints.");
  client_->async_send_goal(goal_msg, send_goal_options);
}

}  // namespace bringup_41068

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bringup_41068::PatrolNode>());
  rclcpp::shutdown();
  return 0;
}