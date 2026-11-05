#!/usr/bin/env python3
"""Drive the GUI traffic light and breach popup from threat_zones.py.

    ros2 run grizzlyguard_decision perimeter_status_node.py

Every bear in /bears/detections is classified with
threat_zones.classify_world_position(). The traffic light shows the WORST
bear currently tracked.

  SUB /bears/detections   visualization_msgs/MarkerArray  bear poses, frame "world"
                          (the same topic the GUI uses for its red markers)
  SUB /bears/lost         std_msgs/String  "ns:id" of a bear to forget
  PUB /perimeter/status   std_msgs/UInt8   0 UNKNOWN, 1 SAFE, 2 CAUTION, 3 DANGER
  PUB /perimeter/alert    std_msgs/String  sent when the level changes;
                          "BREACH:<metres>" opens the GUI popup

ThreatLevel -> traffic light
  GREEN -> 1 SAFE | YELLOW -> 2 CAUTION | RED -> 3 DANGER | EMERGENCY -> 3 DANGER (+ BREACH popup)

Status is 0 (UNKNOWN) until the first detections message arrives. After that
it is 1 (SAFE) whenever no bear is tracked. A bear not updated for
`track_timeout` seconds is dropped.

Test without perception (also shows a red marker on the GUI map):
  ros2 topic pub -r 2 /bears/detections visualization_msgs/msg/MarkerArray \
    "{markers: [{header: {frame_id: world}, ns: bear, id: 1, action: 0, \
      pose: {position: {x: 12.0, y: 0.0}}}]}"
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8
from visualization_msgs.msg import Marker, MarkerArray

from threat_zones import (BORDER_X, BORDER_Y, ThreatLevel, classify_threat,
                          classify_world_position, distance_to_border_m)

UNKNOWN, SAFE, CAUTION, DANGER = 0, 1, 2, 3

GUI_STATUS = {
    ThreatLevel.GREEN: SAFE,
    ThreatLevel.YELLOW: CAUTION,
    ThreatLevel.RED: DANGER,
    ThreatLevel.EMERGENCY: DANGER,
}

# Worst first, so the most dangerous bear wins.
SEVERITY = {ThreatLevel.GREEN: 0, ThreatLevel.YELLOW: 1,
            ThreatLevel.RED: 2, ThreatLevel.EMERGENCY: 3}


def classify(x: float, y: float, border_x: float, border_y: float) -> ThreatLevel:
    """classify_world_position, but never raises.

    It raises ValueError outside x -75..75 / y -40..235. Every point outside
    that box is > 40 m from the border centre, so it is GREEN anyway; use the
    plain distance classifier there instead of crashing the node.
    """
    try:
        return classify_world_position(x, y, border_x=border_x, border_y=border_y)
    except ValueError:
        return classify_threat(distance_to_border_m(x, y, border_x, border_y))


class PerimeterStatusNode(Node):
    def __init__(self):
        super().__init__('perimeter_status_node')
        self.border_x = float(self.declare_parameter('border_x', BORDER_X).value)
        self.border_y = float(self.declare_parameter('border_y', BORDER_Y).value)
        self.timeout = float(self.declare_parameter('track_timeout', 5.0).value)
        self.frame = self.declare_parameter('world_frame', 'world').value
        rate = float(self.declare_parameter('publish_rate_hz', 2.0).value)

        # id -> (x, y, last_seen_monotonic)
        self.bears: dict[str, tuple[float, float, float]] = {}
        self.have_data = False
        self.last_level: ThreatLevel | None = None
        self.warned_frames: set[str] = set()

        self.status_pub = self.create_publisher(UInt8, '/perimeter/status', 10)
        self.alert_pub = self.create_publisher(String, '/perimeter/alert', 10)
        self.create_subscription(MarkerArray, '/bears/detections', self.on_detections, 10)
        self.create_subscription(String, '/bears/lost', self.on_lost, 10)
        # Periodic publish so a GUI started later still gets the current status.
        self.create_timer(1.0 / rate, self.update)

        self.get_logger().info(
            f'Border centre ({self.border_x}, {self.border_y}); '
            f'publishing /perimeter/status at {rate} Hz')

    # ---------------- inputs ----------------
    def on_detections(self, msg: MarkerArray):
        self.have_data = True
        now = time.monotonic()
        for m in msg.markers:
            bear_id = f'{m.ns}:{m.id}'           # same id format as the GUI
            if m.action in (Marker.DELETE, Marker.DELETEALL):
                if m.action == Marker.DELETEALL:
                    self.bears.clear()
                else:
                    self.bears.pop(bear_id, None)
                continue
            frame = m.header.frame_id
            if frame and frame != self.frame and frame not in self.warned_frames:
                self.warned_frames.add(frame)
                self.get_logger().warn(
                    f"Bear {bear_id} is in frame '{frame}', expected '{self.frame}'. "
                    'Zones assume world coordinates.')
            self.bears[bear_id] = (m.pose.position.x, m.pose.position.y, now)
        self.update()

    def on_lost(self, msg: String):
        self.bears.pop(msg.data, None)
        self.update()

    # ---------------- output ----------------
    def update(self):
        now = time.monotonic()
        for bear_id in [b for b, (_, _, t) in self.bears.items() if now - t > self.timeout]:
            self.get_logger().info(f'Bear {bear_id} not seen for {self.timeout:.0f} s - dropped')
            del self.bears[bear_id]

        if not self.have_data:
            self.status_pub.publish(UInt8(data=UNKNOWN))
            return

        # Worst bear decides the light.
        level, worst_id, worst_d = ThreatLevel.GREEN, None, None
        for bear_id, (x, y, _) in self.bears.items():
            lvl = classify(x, y, self.border_x, self.border_y)
            d = distance_to_border_m(x, y, self.border_x, self.border_y)
            if worst_id is None or SEVERITY[lvl] > SEVERITY[level] or \
                    (lvl == level and d < worst_d):
                level, worst_id, worst_d = lvl, bear_id, d

        self.status_pub.publish(UInt8(data=GUI_STATUS[level]))

        if level != self.last_level:
            self.send_alert(level, worst_id, worst_d)
            self.last_level = level

    def send_alert(self, level: ThreatLevel, bear_id, d):
        if level == ThreatLevel.EMERGENCY:
            text = f'BREACH:{d:.1f}'             # GUI parses the number after ':'
        elif level == ThreatLevel.GREEN:
            text = 'SAFE: no bear inside the 40 m zone'
        else:
            text = f'{level.value.upper()}: bear {bear_id} {d:.1f} m from border'
        self.alert_pub.publish(String(data=text))
        self.get_logger().info(f'Threat level -> {level.value} ({text})')


def main():
    rclpy.init()
    node = PerimeterStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()