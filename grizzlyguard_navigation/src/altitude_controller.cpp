// altitude_controller.cpp
//
// Adds z (altitude) control on top of Nav2's 2D velocity commands.
//
//   Nav2 controller --cmd_vel_nav-->  [altitude_controller]  --cmd_vel--> Gazebo
//                                            ^          ^
//                       target_altitude -----+          +----- odom (for current z)
//
// It copies Nav2's x / yaw velocity straight through and fills in linear.z
// with a P-controller toward the target altitude. Nav2 never knows z exists.
//
// For this to move the drone, the Gazebo plugin on the parrot must accept
// linear.z (VelocityControl, not DiffDrive), and Nav2's output must be
// remapped from cmd_vel to cmd_vel_nav. See Z_INTEGRATION.md.
//
//   ros2 run grizzlyguard_navigation altitude_controller --ros-args -r __ns:=/parrot1

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class AltitudeController : public rclcpp::Node
{
public:
  AltitudeController() : Node("altitude_controller")
  {
    kp_ = declare_parameter<double>("kp", 0.8);
    max_vz_ = declare_parameter<double>("max_vz", 1.0);          // m/s
    min_alt_ = declare_parameter<double>("min_altitude", 1.0);   // never go below
    max_alt_ = declare_parameter<double>("max_altitude", 12.0);
    tolerance_ = declare_parameter<double>("tolerance", 0.15);   // "reached" band
    nav_timeout_ = declare_parameter<double>("nav_timeout", 0.5);  // s
    target_ = declare_parameter<double>("initial_altitude", 2.0);
    //manual_sub_ = declare_parameter<std::string>("manual_sub", "/manual_mode");

    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) {
        nav_cmd_ = *m;
        last_nav_ = now();
      });

    target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "target_altitude", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
        const double t = std::clamp(m->data, min_alt_, max_alt_);
        if (std::abs(t - target_) > 1e-3) {
          RCLCPP_INFO(get_logger(), "New target altitude %.2f m", t);
        }
        target_ = t;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odom_topic", "odometry"), 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        z_ = m->pose.pose.position.z;
        have_odom_ = true;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("altitude_reached", 10);

    last_nav_ = now();
    timer_ = create_wall_timer(50ms, [this]() { tick(); });

    manual_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/manual_mode",                                   // absolute, not relative
    rclcpp::QoS(1).transient_local().reliable(),      // receive the latched value
    [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data && !manual_) {
        cmd_pub_->publish(geometry_msgs::msg::Twist{}); // final zero before going quiet
        RCLCPP_INFO(get_logger(), "Manual mode: releasing cmd_vel");
        }
        manual_ = m->data;
    });
  }

private:
  void tick()
  {
    if (manual_) return;
    geometry_msgs::msg::Twist out;

    // Pass Nav2's horizontal command through, unless it has gone stale
    // (Nav2 finished or crashed) - then hold position horizontally.
    if ((now() - last_nav_).seconds() < nav_timeout_) {
      out = nav_cmd_;
    }

    std_msgs::msg::Bool reached;
    if (have_odom_) {
      const double err = target_ - z_;
      out.linear.z = std::clamp(kp_ * err, -max_vz_, max_vz_);
      reached.data = std::abs(err) < tolerance_;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No odometry yet - not commanding z.");
    }

    cmd_pub_->publish(out);
    reached_pub_->publish(reached);
  }

  double kp_, max_vz_, min_alt_, max_alt_, tolerance_, nav_timeout_;
  double target_, z_{0.0};
  bool have_odom_{false};
  geometry_msgs::msg::Twist nav_cmd_;
  rclcpp::Time last_nav_;
  bool manual_{false};
  
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AltitudeController>());
  rclcpp::shutdown();
  return 0;
}