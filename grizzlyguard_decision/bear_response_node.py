#!/usr/bin/env python3
"""Decide how the drone responds to what YOLO sees.

This joins perception to the rest of the system:

  YOLO box + drone pose -> bear position on the map -> threat zone -> response

Response per zone (the deterrent comes from decision_logic.py):
  GREEN      note the bear and its location, keep patrolling
  YELLOW     play sound, follow the bear until it is back in green
  RED        spray, follow the bear; changes to sound once it reaches yellow
  EMERGENCY  no deterrent, follow the bear, send the critical alert

Subscribes:
  /parrot1/bear_detections      vision_msgs/Detection2DArray  from bear_yolo_node
  /parrot1/nonbear_detections   vision_msgs/Detection2DArray  people, not bears
  /parrot1/odometry             nav_msgs/Odometry             drone pose
  /parrot1/scan                 sensor_msgs/LaserScan         obstacles ahead
  /drone/status                 std_msgs/String               from patrol_node
  /drone/emergency_stop         std_msgs/Empty                from the GUI
  /drone/return_to_home         std_msgs/Empty                from the GUI

Publishes:
  /bears/detections             visualization_msgs/MarkerArray  bear on the GUI map
  /perimeter/status             std_msgs/UInt8     1 safe, 2 caution, 3 danger
  /perimeter/alert              std_msgs/String    GUI status bar; "BREACH:<m>" opens the alert
  /deterrent/status             std_msgs/Bool      deterrent on or off
  /bear_deterrent               std_msgs/String    tells the bears they were scared
  /drone/follow                 std_msgs/Bool      pauses the patrol while following
  /parrot1/cmd_vel              geometry_msgs/Twist  steers the drone while following
  /perception/not_bear          std_msgs/String    a person was seen, and ignored
"""
import math
import statistics
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String, UInt8
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from decision_logic import decision_for_threat
from threat_zones import ThreatLevel, classify_threat, distance_to_border_m

LOST_AFTER_S = 15.0       # no sighting for this long means the bear is gone
SEARCH_AFTER_S = 10.0     # no sighting for this long: stop closing in, just turn to it
ODOM_BUFFER_S = 2.0       # odometry kept for matching a detection's timestamp
# From 2 m up, a far bear is close to the horizon, where one pixel covers metres of
# ground, so positions past this range are too unreliable to judge a zone from.
MAX_RANGE_M = 15.0
FIXES_KEPT = 3            # positions are the median of this many, to steady them
FIXES_TO_CHANGE = 3       # sightings in a new zone before believing it, so one false
CONFIRM_WITHIN_S = 6.0    # box can't trigger a response; they must be this close in time
FOLLOW_GAP_M = 4.0        # how far from the bear the drone tries to stay
# YOLO tends to lose a bear walking away near the green line, so get closer there to
# make it look bigger in the camera and see it cross.
NEAR_GREEN_M = 33.0       # bear this far from town counts as nearing the green line
NEAR_GREEN_GAP_M = 3.0    # how far from the bear to stay while it does
MAX_SPEED = 1.5           # m/s, enough to keep up with a sprayed bear
TRACK_WINDOW_S = 4.0      # seconds of sightings used to judge the bear's velocity
PREDICT_FOR_S = 8.0       # how long to assume it keeps walking when it isn't seen
MAX_TURN = 0.8            # rad/s
FACING_RAD = 0.6          # only move forward when roughly facing the bear
STOP_DISTANCE_M = 2.5     # stop if the laser sees something this close ahead
AHEAD_RAD = 0.5           # width either side of straight ahead to check
NOT_BEAR_EVERY_S = 10.0   # how often to repeat the "not a bear" message

# Only these drone states may follow a bear or use a deterrent. An e-stop, manual
# control, returning home or landing always wins.
ACTIVE_STATES = ('PATROL', 'FOLLOW', 'TAKEOFF COMPLETE')
STEERING_STATE = 'FOLLOW: following'
DETERRENT_KIND = {'Sound': 'sound', 'Spray & Sound': 'spray'}
PERIMETER = {ThreatLevel.GREEN: 1, ThreatLevel.YELLOW: 2,
             ThreatLevel.RED: 3, ThreatLevel.EMERGENCY: 3}


def q_rot(q, v):
    """Rotate the point v by the quaternion q."""
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2*(y*vz - z*vy), 2*(z*vx - x*vz), 2*(x*vy - y*vx)
    return (vx + w*tx + y*tz - z*ty, vy + w*ty + z*tx - x*tz, vz + w*tz + x*ty - y*tx)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BearResponseNode(Node):
    def __init__(self):
        super().__init__('bear_response_node')
        # Camera geometry, matching urdf_parrot.
        self.image_w = self.declare_parameter('image_width', 720).value
        self.image_h = self.declare_parameter('image_height', 480).value
        hfov = self.declare_parameter('horizontal_fov', 2.0944).value
        self.cam_offset = tuple(self.declare_parameter('camera_offset', [0.255, 0.018, 0.263]).value)
        self.cam_pitch = self.declare_parameter('camera_pitch', 0.785398).value
        self.ground_z = self.declare_parameter('ground_z', 0.0).value
        self.focal = (self.image_w / 2.0) / math.tan(hfov / 2.0)

        self.odom = deque()
        self.fixes = deque(maxlen=FIXES_KEPT)
        self.track = deque()       # (time, x, y) of recent steadied positions
        self.bear = None           # (x, y) on the map
        self.last_seen = None
        self.zone = None
        self.candidate = None      # a new zone waiting to be confirmed
        self.candidate_count = 0
        self.candidate_at = 0.0
        self.deterrent = None      # what the bears were last told
        self.drone_state = ''
        self.last_not_bear = None
        self.clear_ahead = math.inf

        self.marker_pub = self.create_publisher(MarkerArray, '/bears/detections', 10)
        self.perimeter_pub = self.create_publisher(UInt8, '/perimeter/status', 10)
        self.alert_pub = self.create_publisher(String, '/perimeter/alert', 10)
        self.deterrent_status_pub = self.create_publisher(Bool, '/deterrent/status', 10)
        self.deterrent_pub = self.create_publisher(String, '/bear_deterrent', 10)
        self.follow_pub = self.create_publisher(Bool, '/drone/follow', 10)
        self.cmd_pub = self.create_publisher(Twist, '/parrot1/cmd_vel', 10)
        self.not_bear_pub = self.create_publisher(String, '/perception/not_bear', 10)

        self.create_subscription(Odometry, '/parrot1/odometry', self.on_odom, 20)
        self.create_subscription(Detection2DArray, '/parrot1/bear_detections', self.on_bears, 10)
        self.create_subscription(Detection2DArray, '/parrot1/nonbear_detections', self.on_not_bear, 10)
        self.create_subscription(String, '/drone/status', self.on_status, 10)
        self.create_subscription(Empty, '/drone/emergency_stop', self.on_estop, 10)
        self.create_subscription(Empty, '/drone/return_to_home', self.on_return_home, 10)
        self.create_subscription(LaserScan, '/parrot1/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)
        self.create_timer(0.2, self.steer)

    # ------------------------------------------------------------ inputs
    def on_odom(self, msg):
        t = stamp_s(msg.header.stamp)
        self.odom.append((t, msg))
        while self.odom and t - self.odom[0][0] > ODOM_BUFFER_S:
            self.odom.popleft()

    def on_scan(self, msg):
        ahead = [r for i, r in enumerate(msg.ranges)
                 if abs(msg.angle_min + i*msg.angle_increment) < AHEAD_RAD
                 and msg.range_min < r < msg.range_max]
        self.clear_ahead = min(ahead) if ahead else math.inf

    def on_status(self, msg):
        self.drone_state = msg.data

    def on_estop(self, _msg):
        self.stop_response()
        self.alert('E-STOP: drone descending, all responses stopped')

    def on_return_home(self, _msg):
        self.stop_response()
        self.alert('RETURN TO HOME: drone flying back to its start point')

    def on_not_bear(self, msg):
        """Report a person once in a while, and do nothing else about them."""
        if not msg.detections or not self.odom:
            return
        now = self.odom[-1][0]
        if self.last_not_bear is not None and now - self.last_not_bear < NOT_BEAR_EVERY_S:
            return
        self.last_not_bear = now
        text = 'Person detected: not a bear, continuing patrol'
        self.not_bear_pub.publish(String(data=text))
        self.alert(text)

    def on_bears(self, msg):
        if not msg.detections:
            return
        best = max(msg.detections, key=lambda d: d.results[0].hypothesis.score if d.results else 0.0)
        odom = self.pose_at(stamp_s(msg.header.stamp))
        if odom is None:
            return
        # The bottom-centre of the box is where the bear's feet touch the ground.
        hit = self.ground_point(best.bbox.center.position.x,
                                best.bbox.center.position.y + best.bbox.size_y / 2.0, odom)
        if hit is None:
            return
        self.fixes.append(hit)
        self.bear = (statistics.median(f[0] for f in self.fixes),
                     statistics.median(f[1] for f in self.fixes))
        t = stamp_s(msg.header.stamp)
        self.track.append((t, *hit))
        while t - self.track[0][0] > TRACK_WINDOW_S:
            self.track.popleft()
        self.update_zone(classify_threat(distance_to_border_m(*self.bear)), t)

    # ------------------------------------------------------------ position
    def pose_at(self, t):
        if not self.odom:
            return None
        st, msg = min(self.odom, key=lambda s: abs(s[0] - t))
        return msg if abs(st - t) <= ODOM_BUFFER_S else None

    def ground_point(self, u, v, odom):
        """World (x, y) where the camera ray through pixel (u, v) meets the ground."""
        # Gazebo camera: looks along +x, image right is -y, image down is -z.
        ray = (1.0, -(u - self.image_w / 2.0) / self.focal, -(v - self.image_h / 2.0) / self.focal)
        c, s = math.cos(self.cam_pitch), math.sin(self.cam_pitch)
        ray = (c*ray[0] + s*ray[2], ray[1], -s*ray[0] + c*ray[2])     # the camera tilts down
        p, o = odom.pose.pose.position, odom.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        dx, dy, dz = q_rot(q, ray)
        ox, oy, oz = q_rot(q, self.cam_offset)
        cx, cy, cz = p.x + ox, p.y + oy, p.z + oz
        if dz >= -1e-6:
            return None                  # looking at or above the horizon
        k = (self.ground_z - cz) / dz
        x, y = cx + k*dx, cy + k*dy
        return (x, y) if math.hypot(x - cx, y - cy) <= MAX_RANGE_M else None

    # ------------------------------------------------------------ decisions
    def update_zone(self, zone, t):
        if zone == self.zone:
            self.candidate = None
        else:
            if zone != self.candidate or t - self.candidate_at > CONFIRM_WITHIN_S:
                self.candidate, self.candidate_count = zone, 0
            self.candidate_at = t
            self.candidate_count += 1
            if self.candidate_count < FIXES_TO_CHANGE:
                return
            self.zone, self.candidate = zone, None
            self.last_seen = t
            self.publish_marker()
            self.on_zone_changed()
            return
        self.last_seen = t
        self.publish_marker()

    def on_zone_changed(self):
        decision = decision_for_threat(self.zone)
        x, y = self.bear
        self.perimeter_pub.publish(UInt8(data=PERIMETER[self.zone]))
        if self.zone == ThreatLevel.EMERGENCY:
            # The GUI opens its full-screen alert for messages starting BREACH.
            self.alert(f'BREACH:{distance_to_border_m(x, y):.0f}')
        else:
            self.alert(f'Bear in {decision.zone_name} at ({x:.0f}, {y:.0f}): {decision.gui_alert}')
        self.apply_deterrent()

    def may_respond(self):
        return self.drone_state.startswith(ACTIVE_STATES)

    def apply_deterrent(self):
        """Tell the bears about a new deterrent, and the GUI whether one is on."""
        if self.zone is None or not self.may_respond():
            return
        kind = DETERRENT_KIND.get(decision_for_threat(self.zone).deterrent)
        if kind == self.deterrent:
            return
        self.deterrent = kind
        self.deterrent_status_pub.publish(Bool(data=kind is not None))
        if kind is not None:
            # YOLO can't tell bears apart, so every bear in the test hears it. The
            # scenarios only have one bear out.
            self.deterrent_pub.publish(String(data=f'all {kind}'))
            self.alert(f'Deterrent deployed: {kind}')
        else:
            self.alert('Deterrent stopped')

    def steer(self):
        """While following, turn to face the bear and close to FOLLOW_GAP_M.

        Steering directly, not through Nav2, keeps the bear in the camera. There is
        no path planning here, so the laser stops the drone before it hits anything.
        """
        if not self.drone_state.startswith(STEERING_STATE) or not self.track or not self.odom:
            return
        pose = self.odom[-1][1].pose.pose
        vx, vy = self.bear_velocity()
        # Between sightings, assume the bear keeps walking the same way.
        ahead = min(self.odom[-1][0] - self.last_seen, PREDICT_FOR_S)
        # Steer from the latest sighting: the steadied position used for zones lags behind.
        _, last_x, last_y = self.track[-1]
        dx = last_x + vx*ahead - pose.position.x
        dy = last_y + vy*ahead - pose.position.y
        o = pose.orientation
        yaw = math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z))
        err = (math.atan2(dy, dx) - yaw + math.pi) % (2*math.pi) - math.pi
        cmd = Twist()
        cmd.angular.z = max(-MAX_TURN, min(MAX_TURN, 1.2*err))
        seen_recently = self.odom[-1][0] - self.last_seen < SEARCH_AFTER_S
        if abs(err) < FACING_RAD and seen_recently and self.clear_ahead > STOP_DISTANCE_M:
            dist = math.hypot(dx, dy)
            near_green = NEAR_GREEN_M <= distance_to_border_m(*self.bear)
            gap = NEAR_GREEN_GAP_M if near_green else FOLLOW_GAP_M
            # Match the bear's speed away from the drone, then close the remaining gap.
            walking_away = (vx*dx + vy*dy) / dist if dist > 1e-3 else 0.0
            cmd.linear.x = max(0.0, min(MAX_SPEED, walking_away + 1.0*(dist - gap)))
        self.cmd_pub.publish(cmd)

    def bear_velocity(self):
        """The bear's (vx, vy) in m/s from recent sightings, or zero if too few."""
        if len(self.track) < 2 or self.track[-1][0] - self.track[0][0] < 1.0:
            return 0.0, 0.0
        t0, x0, y0 = self.track[0]
        t1, x1, y1 = self.track[-1]
        return (x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0)

    def stop_following(self):
        self.follow_pub.publish(Bool(data=False))

    def stop_response(self):
        """Used when the operator takes over, or the bear has gone."""
        self.stop_following()
        if self.deterrent is not None:
            self.deterrent = None
            self.deterrent_status_pub.publish(Bool(data=False))

    def tick(self):
        """Once a second: keep the GUI current, and follow or drop the bear."""
        if self.zone is None:
            self.perimeter_pub.publish(UInt8(data=1))       # nothing seen, so safe
            return
        now = self.odom[-1][0] if self.odom else None
        if now is not None and now - self.last_seen > LOST_AFTER_S:
            self.alert('Bear lost from view: returning to patrol')
            self.forget_bear()
            return
        self.perimeter_pub.publish(UInt8(data=PERIMETER[self.zone]))
        self.apply_deterrent()
        if self.may_respond() and self.zone != ThreatLevel.GREEN:
            self.follow_pub.publish(Bool(data=True))
        else:
            self.stop_following()

    def forget_bear(self):
        self.stop_response()
        marker = Marker()
        marker.ns, marker.id, marker.action = 'bear', 0, Marker.DELETE
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.bear = self.zone = self.last_seen = None
        self.candidate = None
        self.fixes.clear()
        self.track.clear()

    # ------------------------------------------------------------ outputs
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns, marker.id, marker.action = 'bear', 0, Marker.ADD
        marker.pose.position.x, marker.pose.position.y = self.bear
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def alert(self, text):
        self.alert_pub.publish(String(data=text))
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = BearResponseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
