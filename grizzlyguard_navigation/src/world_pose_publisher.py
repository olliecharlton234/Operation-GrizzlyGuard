#!/usr/bin/env python3
"""Publish the parrot's pose in the Gazebo WORLD frame for the GUI map.

    ros2 run grizzlyguard_navigation world_pose_publisher.py --ros-args \
        -p spawn_x:=0.0 -p spawn_y:=0.0 -p spawn_yaw:=0.0

The Gazebo odometry (/parrot1/odometry, frame parrot1_odom) starts at the
spawn point, so world = rotate(odom, spawn_yaw) + spawn. Set the spawn_*
parameters to the parrot's add_robot(...) values in the launch file.

Check it: at startup the GUI's blue marker must sit on the green start
square. Then fly somewhere known and check the marker follows.

  SUB /parrot1/odometry     nav_msgs/Odometry
  PUB /parrot1/world_pose   geometry_msgs/PoseStamped  (frame_id: world)
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_multiply(a, b):
    """Hamilton product a * b, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class WorldPosePublisher(Node):
    def __init__(self):
        super().__init__('world_pose_publisher')
        odom_topic = self.declare_parameter('odom_topic', '/parrot1/odometry').value
        out_topic = self.declare_parameter('world_pose_topic', '/parrot1/world_pose').value
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        self.sx = float(self.declare_parameter('spawn_x', 0.0).value)
        self.sy = float(self.declare_parameter('spawn_y', 0.0).value)
        self.sz = float(self.declare_parameter('spawn_z', 0.0).value)
        yaw = float(self.declare_parameter('spawn_yaw', 0.0).value)
        rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)

        self.cos_y, self.sin_y = math.cos(yaw), math.sin(yaw)
        self.q_spawn = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        self.min_period = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self.last_pub = None

        self.pub = self.create_publisher(PoseStamped, out_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.get_logger().info(
            f'{odom_topic} -> {out_topic} with spawn ({self.sx}, {self.sy}, {self.sz}), '
            f'yaw {yaw:.3f} rad')

    def on_odom(self, msg):
        now = self.get_clock().now()
        if self.last_pub is not None and \
                (now - self.last_pub).nanoseconds * 1e-9 < self.min_period:
            return
        self.last_pub = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.position.x = self.cos_y * p.x - self.sin_y * p.y + self.sx
        out.pose.position.y = self.sin_y * p.x + self.cos_y * p.y + self.sy
        out.pose.position.z = p.z + self.sz
        qx, qy, qz, qw = quat_multiply(self.q_spawn, (q.x, q.y, q.z, q.w))
        out.pose.orientation.x, out.pose.orientation.y = qx, qy
        out.pose.orientation.z, out.pose.orientation.w = qz, qw
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(WorldPosePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()