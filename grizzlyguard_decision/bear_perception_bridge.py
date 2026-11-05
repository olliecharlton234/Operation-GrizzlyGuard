#!/usr/bin/env python3
"""Bridge between the YOLO perception output and the threat decision logic.

This module is intentionally lightweight and assumes the detector is already
publishing bear detections in a ROS message such as
`vision_msgs/msg/Detection2DArray`. It reads the center of the detection box,
translates that image-space point into a simple local map-space coordinate,
and then feeds the resulting world position into the decision controller.
"""

from __future__ import annotations

import math
import time
from typing import Iterable, Optional, Tuple

from decision_controller import DecisionController
from decision_logic import decision_for_world_position

try:  # pragma: no cover - optional ROS dependency
    import rclpy
    from geometry_msgs.msg import PointStamped
    from rclpy.node import Node
    from std_msgs.msg import String
    from tf2_geometry_msgs import do_transform_point
    from tf2_ros import Buffer, TransformListener
    from vision_msgs.msg import Detection2DArray
except Exception:  # pragma: no cover
    rclpy = None  # type: ignore
    PointStamped = None  # type: ignore
    Node = object  # type: ignore
    Detection2DArray = None  # type: ignore
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore
    do_transform_point = None  # type: ignore
    String = None  # type: ignore


class BearPerceptionBridge:
    """Convert YOLO bear detections into a threat decision."""

    def __init__(self, image_width: float = 640.0, image_height: float = 480.0, scale_m_per_pixel: float = 0.05,
                 border_x: float = 0.0, border_y: float = 0.0, camera_frame: str = 'camera_link',
                 world_frame: str = 'parrot1_map', node: Optional[Node] = None):
        self.image_width = float(image_width)
        self.image_height = float(image_height)
        self.scale_m_per_pixel = float(scale_m_per_pixel)
        self.border_x = float(border_x)
        self.border_y = float(border_y)
        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.controller = DecisionController()
        self.node = node
        self.tf_buffer = None
        self.tf_listener = None
        self.latest_world_point = None
        self.latest_decision = None

        if rclpy is not None and node is not None and Buffer is not None and TransformListener is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, node)

    def detection_center_to_world(
        self,
        center_x: float,
        center_y: float,
        stamp=None,
        camera_frame: Optional[str] = None,
        world_frame: Optional[str] = None,
    ) -> Tuple[float, float]:
        """Convert the image-plane detection center to world coordinates.

        If TF is available and a valid camera/world frame transform exists, it is
        used. Otherwise, a simple local fallback is used to keep the bridge usable
        without a full calibration stack.
        """
        camera_frame_name = camera_frame or self.camera_frame
        world_frame_name = world_frame or self.world_frame

        if self.tf_buffer is not None and camera_frame_name and world_frame_name and PointStamped is not None:
            try:
                if stamp is None and self.node is not None:
                    stamp = self.node.get_clock().now().to_msg()

                point = PointStamped()
                point.header.frame_id = camera_frame_name
                point.header.stamp = stamp
                point.point.x = center_x
                point.point.y = center_y
                point.point.z = 0.0

                transform = self.tf_buffer.lookup_transform(
                    world_frame_name,
                    camera_frame_name,
                    stamp,
                    rclpy.time.Duration(seconds=0.1),
                )
                transformed = do_transform_point(point, transform)
                return transformed.point.x, transformed.point.y
            except Exception:
                pass

        local_x = (center_x - (self.image_width / 2.0)) * self.scale_m_per_pixel
        local_y = ((self.image_height / 2.0) - center_y) * self.scale_m_per_pixel
        return self.border_x + local_x, self.border_y + local_y

    def detection_to_decision(
        self,
        center_x: float,
        center_y: float,
        stamp=None,
        camera_frame: Optional[str] = None,
        world_frame: Optional[str] = None,
    ) -> tuple[float, float, object]:
        """Convert a detection center into world coordinates and return the resulting decision."""
        x_world, y_world = self.detection_center_to_world(center_x, center_y, stamp=stamp, camera_frame=camera_frame, world_frame=world_frame)
        decision = decision_for_world_position(x_world, y_world, border_x=self.border_x, border_y=self.border_y)
        self.latest_world_point = (x_world, y_world)
        self.latest_decision = decision
        self.controller.last_threat = decision.threat_level
        self.controller.update_state_from_threat(decision.threat_level)
        return x_world, y_world, decision

    def process_detections(self, detections: Iterable[object]) -> list[tuple[float, float, object]]:
        """Process a list of detections and return a decision for each one."""
        results: list[tuple[float, float, object]] = []
        for detection in detections:
            if detection is None:
                continue
            center_x = getattr(detection.bbox.center.position, 'x', 0.0)
            center_y = getattr(detection.bbox.center.position, 'y', 0.0)
            results.append(self.detection_to_decision(center_x, center_y))
        return results


if rclpy is not None:
    class BearYoloDecisionNode(Node):
        """ROS 2 node that subscribes to detections and passes them to the decision controller."""

        def __init__(self, namespace: str = 'parrot1', camera_frame: str = 'camera_link', world_frame: str = 'parrot1_map'):
            super().__init__('bear_yolo_decision_bridge')
            self.bridge = BearPerceptionBridge(node=self, camera_frame=camera_frame, world_frame=world_frame)
            self.subscription = self.create_subscription(
                Detection2DArray,
                f'/{namespace}/bear_detections',
                self._detection_callback,
                10,
            )
            self.latest_decision = None
            self.latest_world_point = None
            self.decision_publisher = self.create_publisher(String, f'/{namespace}/decision', 10)
            self.deterrent_publisher = self.create_publisher(String, '/bear_deterrent', 10)
            self.deterrent_interval_s = self.declare_parameter('deterrent_interval_s', 3.0).value
            self._last_deterrent_time = 0.0
            self._last_deterrent_level = None

        def _publish_deterrent(self, threat_level):
            if self.deterrent_publisher is None:
                return
            if threat_level == 'green':
                self._last_deterrent_level = None
                return
            now = time.monotonic()
            if self._last_deterrent_level == threat_level and now - self._last_deterrent_time < self.deterrent_interval_s:
                return
            action_name = {
                'yellow': 'sound',
                'red': 'spray',
                'emergency': 'spray',
            }.get(threat_level, 'sound')
            msg = String()
            msg.data = f'all {action_name}'
            self.deterrent_publisher.publish(msg)
            self._last_deterrent_time = now
            self._last_deterrent_level = threat_level
            self.get_logger().info(f'Published deterrent command: {msg.data}')

        def _detection_callback(self, msg):
            if not msg.detections:
                self.get_logger().info('No bear detections received')
                return

            for detection in msg.detections:
                center_x = detection.bbox.center.position.x
                center_y = detection.bbox.center.position.y
                x_world, y_world, decision = self.bridge.detection_to_decision(
                    center_x,
                    center_y,
                    stamp=msg.header.stamp,
                    camera_frame=self.bridge.camera_frame,
                    world_frame=self.bridge.world_frame,
                )
                self.latest_decision = decision
                self.latest_world_point = (x_world, y_world)
                self.bridge.latest_decision = decision
                self.bridge.latest_world_point = (x_world, y_world)
                if self.decision_publisher is not None:
                    msg_out = String()
                    msg_out.data = decision.action
                    self.decision_publisher.publish(msg_out)
                if decision.threat_level.value in {'yellow', 'red', 'emergency'}:
                    self._publish_deterrent(decision.threat_level.value)
                self.get_logger().info(
                    f'Bear detected at world=({x_world:.2f}, {y_world:.2f}) -> '
                    f'{decision.threat_level.value} :: {decision.action}'
                )


def main(args=None):
    """ROS entrypoint for the bridge node."""
    if rclpy is None:
        raise RuntimeError('rclpy is not available; cannot launch the ROS bridge.')

    rclpy.init(args=args)
    node = BearYoloDecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
