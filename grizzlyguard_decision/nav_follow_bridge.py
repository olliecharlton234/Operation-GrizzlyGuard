#!/usr/bin/env python3
"""ROS/Nav2 bridge that turns bear detections into follow goals.

This node sits on top of the threat decision layer and sends the drone to a goal
position derived from the detected bear location. It intentionally keeps the
movement behavior simple and deterministic:

- GREEN: hold current patrol / do not chase
- YELLOW: follow with a moderate offset
- RED: follow with a tighter offset
- EMERGENCY: maintain a critical tracking offset

It is designed to be a thin integration layer between the detection pipeline and
Nav2, not a full autonomy stack.
"""

from __future__ import annotations

from typing import Optional

from bear_follow_controller import BearFollowController
from bear_perception_bridge import BearPerceptionBridge
from threat_zones import ThreatLevel

try:  # pragma: no cover - optional ROS dependency
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.node import Node
    from vision_msgs.msg import Detection2DArray
except Exception:  # pragma: no cover
    rclpy = None  # type: ignore
    BasicNavigator = None  # type: ignore
    Node = object  # type: ignore
    Detection2DArray = None  # type: ignore
    PoseStamped = None  # type: ignore


class BearNavFollowNode(Node):
    """Turn bear detections into Nav2 goal commands based on threat level."""

    def __init__(self, namespace: str = 'parrot1', camera_frame: str = 'camera_link', world_frame: str = 'map'):
        if rclpy is None:
            raise RuntimeError('rclpy is not available; cannot create the Nav2 follow node.')
        super().__init__('bear_nav_follow_bridge')

        self.namespace = namespace
        self.bridge = BearPerceptionBridge(node=self, camera_frame=camera_frame, world_frame=world_frame)
        self.follow_controller = BearFollowController(controller=self.bridge.controller)

        self.navigator = BasicNavigator(namespace=self.namespace)
        self.subscription = self.create_subscription(
            Detection2DArray,
            f'/{self.namespace}/bear_detections',
            self._detection_callback,
            10,
        )

        self._last_goal = None

    def _goal_pose(self, x: float, y: float, z: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = f'{self.namespace}_map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def _send_goal(self, x: float, y: float) -> None:
        self._last_goal = (x, y)
        self.navigator.goToPose(self._goal_pose(x, y))

    def _detection_callback(self, msg):
        if not msg.detections:
            return

        detection = msg.detections[0]
        center_x = detection.bbox.center.position.x
        center_y = detection.bbox.center.position.y

        x_world, y_world, decision = self.bridge.detection_to_decision(
            center_x,
            center_y,
            stamp=msg.header.stamp,
            camera_frame=self.bridge.camera_frame,
            world_frame=self.bridge.world_frame,
        )

        current_threat, action, target = self.follow_controller.evaluate_bear(x_world, y_world)
        self.get_logger().info(
            'Bear detected at (%.2f, %.2f): %s -> %s | target=(%.2f, %.2f)',
            x_world,
            y_world,
            current_threat.value,
            action,
            target[0],
            target[1],
        )

        if current_threat == ThreatLevel.GREEN:
            self.get_logger().info('Bear is green; maintaining patrol and not chasing.')
            return

        self._send_goal(target[0], target[1])


def main(args=None):
    """ROS entrypoint for the Nav2 follow bridge."""
    if rclpy is None:
        raise RuntimeError('rclpy is not available; cannot launch the Nav2 follow bridge.')

    rclpy.init(args=args)
    node = BearNavFollowNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
