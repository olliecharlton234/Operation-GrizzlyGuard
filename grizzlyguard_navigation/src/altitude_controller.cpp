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
// While the GUI's /manual_mode is true it publishes nothing, so the operator's
// /cmd_vel is the only command reaching Gazebo.
//
//   ros2 run grizzlyguard_navigation altitude_controller --ros-args \
//       -r __ns:=/parrot1 -p min_altitude:=0.2

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <string>
#include <thread>

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

    // SensorDataQoS (best effort) accepts both best-effort and reliable
    // publishers. A default (reliable) subscription silently receives nothing
    // from a best-effort publisher such as the Gazebo bridge.
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "odometry");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        z_ = m->pose.pose.position.z;
        if (!have_odom_) {
          RCLCPP_INFO(get_logger(), "Odometry received, current z = %.2f m", z_);
        }
        have_odom_ = true;
      });

    // GUI manual-mode hand-off. Absolute topic (this node runs in /parrot1) and
    // transient_local so the latched value published before we started arrives.
    manual_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/manual_mode", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data && !manual_) {
          cmd_pub_->publish(geometry_msgs::msg::Twist{});  // final zero before going quiet
          RCLCPP_INFO(get_logger(), "Manual mode: releasing cmd_vel");
        } else if (!m->data && manual_) {
          RCLCPP_INFO(get_logger(), "Manual mode off: resuming control");
        }
        manual_ = m->data;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("altitude_reached", 10);

    RCLCPP_INFO(get_logger(), "Listening for odometry on %s",
      get_node_topics_interface()->resolve_topic_name(odom_topic).c_str());

    last_nav_ = now();
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

  // Send zero velocity a few times so Gazebo stops holding the last command.
  void stopDrone()
  {
    if (manual_) return;  // the GUI owns cmd_vel; don't override the operator
    RCLCPP_INFO(get_logger(), "Shutting down: sending zero velocity");
    for (int i = 0; i < 5; ++i) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      std::this_thread::sleep_for(20ms);
    }
  }

private:
  void tick()
  {
    if (manual_) return;  // GUI owns /cmd_vel while the operator is flying

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
  bool manual_{false};
  geometry_msgs::msg::Twist nav_cmd_;
  rclcpp::Time last_nav_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Gazebo's VelocityControl keeps applying the LAST velocity it received, forever.
// If this node is killed mid-climb/descent the drone keeps moving (through the
// ground, since collisions are off). So handle Ctrl+C ourselves and send a final
// zero command before shutting down.
static std::atomic<bool> g_stop{false};

void stopHandler(int) { g_stop = true; }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stopHandler);
  std::signal(SIGTERM, stopHandler);

  auto node = std::make_shared<AltitudeController>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !g_stop) {
    exec.spin_some(50ms);
  }

  node->stopDrone();
  rclcpp::shutdown();
  return 0;
}