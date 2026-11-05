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

import time
from typing import Optional

from bear_follow_controller import BearFollowController
from bear_perception_bridge import BearPerceptionBridge
from threat_zones import ThreatLevel

try:  # pragma: no cover - optional ROS dependency
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from vision_msgs.msg import Detection2DArray
except Exception:  # pragma: no cover
    rclpy = None  # type: ignore
    BasicNavigator = None  # type: ignore
    Node = object  # type: ignore
    Detection2DArray = None  # type: ignore
    PoseStamped = None  # type: ignore
    String = None  # type: ignore
    Bool = None  # type: ignore


class BearNavFollowNode(Node):
    """Turn bear detections into Nav2 goal commands based on threat level."""

    def __init__(self, namespace: str = 'parrot1', camera_frame: str = 'camera_link', world_frame: str = 'parrot1_map'):
        if rclpy is None:
            raise RuntimeError('rclpy is not available; cannot create the Nav2 follow node.')
        super().__init__('bear_nav_follow_bridge')

        self.namespace = namespace
        self.world_frame = world_frame or f'{self.namespace}_map'
        self.bridge = BearPerceptionBridge(node=self, camera_frame=camera_frame, world_frame=self.world_frame)
        self.follow_controller = BearFollowController(controller=self.bridge.controller)
        self._drone_ready_for_bear_tracking = False

        self.navigator = BasicNavigator(namespace=self.namespace)
        self.subscription = self.create_subscription(
            Detection2DArray,
            f'/{self.namespace}/bear_detections',
            self._detection_callback,
            10,
        )
        self.status_subscription = self.create_subscription(
            String,
            '/drone/status',
            self._status_callback,
            10,
        )
        self.decision_subscription = self.create_subscription(
            String,
            f'/{self.namespace}/decision',
            self._decision_callback,
            10,
        )
        self.manual_mode_publisher = self.create_publisher(Bool, '/manual_mode', 10)

        self._last_goal = None
        self.latest_decision = None
        self.latest_world_point = None
        self._last_threat = None
        self._last_follow_target = None
        self._bear_detected = False
        self._last_detection_time = 0.0
        self._detection_timeout_s = 4.0

    def _status_callback(self, msg):
        text = (msg.data or '').upper()
        self._drone_ready_for_bear_tracking = ('TAKEOFF COMPLETE' in text) or ('PATROL' in text)

    def _decision_callback(self, msg):
        self.latest_decision = (msg.data or '').strip()
        self.get_logger().info(f'Received decision update: {self.latest_decision}')

    def _set_manual_mode(self, enabled: bool) -> None:
        if self.manual_mode_publisher is None:
            return
        msg = Bool()
        msg.data = bool(enabled)
        self.manual_mode_publisher.publish(msg)

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
        self._set_manual_mode(True)
        self.navigator.cancelTask()
        self.navigator.goToPose(self._goal_pose(x, y))

    def _detection_callback(self, msg):
        if not msg.detections:
            if time.monotonic() - self._last_detection_time > self._detection_timeout_s:
                self._bear_detected = False
            return
        if not self._drone_ready_for_bear_tracking:
            self.get_logger().info('Ignoring bear detection: drone is not airborne yet.')
            return

        self._bear_detected = True
        self._last_detection_time = time.monotonic()

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
        self.latest_world_point = (x_world, y_world)
        self.bridge.latest_world_point = (x_world, y_world)
        self.bridge.latest_decision = decision

        current_threat, action, target = self.follow_controller.evaluate_bear(x_world, y_world)
        self._last_threat = current_threat
        self.get_logger().info(
            f'Bear detected at ({x_world:.2f}, {y_world:.2f}): '
            f'{current_threat.value} -> {action} | target=({target[0]:.2f}, {target[1]:.2f})'
        )

        # Follow policy: once a bear has been detected, and while it remains in the
        # yellow/red/emergency zones, keep the drone offset-following the bear.
        active_threats = (ThreatLevel.YELLOW, ThreatLevel.RED, ThreatLevel.EMERGENCY)
        if current_threat not in active_threats:
            self.get_logger().info('Bear is currently green or stale; patrol resumes and chase disengages.')
            self._set_manual_mode(False)
            self._last_follow_target = None
            return

        # Avoid repeated goal cancellations on the same target while the bear stays in
        # the same threat zone; this was causing the route to churn at startup.
        if self._last_follow_target == target and self._last_threat == current_threat:
            return

        self._last_follow_target = target
        self._last_threat = current_threat

        # Keep the drone offset 5–10 m from the bear so it remains in frame while
        # the patrol is suspended and the vehicle follows it.
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
