"""
Represents a patrol path for the parrot: an object holding a list of
waypoints, independent of ROS. A BasicNavigator (or any rclpy node) is only
needed later, when converting these waypoints into PoseStamped messages to
send to Nav2 (PoseStamped needs a live clock, which the path object itself
doesn't have).

x = -75 to 75, y = -40 to +235 (world metres), z = target altitude in metres.
"""

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped

Waypoint = Tuple[float, float, float]


class PatrolPath:
    """A named, ordered list of (x, y, z) waypoints for a patrol route.

    This does not talk to ROS 2; it purely manages the route.
    Looping / back-and-forth is handled by patrol_node.py, not here.
    """

    def __init__(self, frame_id: str = 'parrot1_map',
                 waypoints: Optional[List[Waypoint]] = None):
        self.frame_id = frame_id
        self.waypoints: List[Waypoint] = []

        if waypoints is not None:
            self.set_path(waypoints)
        else:
            self.generate_path()

    def generate_path(self) -> bool:
        """Stretch goal: generate a path from the yellowstone environment.

        Placeholder default route until then.
        """
        self.set_path([
            (25.0, -15.0, 0.0),
            (30.0, 20.0, 0.0),
            (24.0, 68.0, 0.0),
            (-18.0, 66.0, 0.0),
            (-30.0, -15.0, 0.0),
        ])
        return True

    def set_path(self, points: List[Waypoint]) -> None:
        """Replace the current waypoint list with `points` ((x, y, z) tuples)."""
        if not points:
            raise ValueError('set_path() requires at least one waypoint.')
        self.waypoints = [(float(x), float(y), float(z)) for x, y, z in points]

    def get_path(self) -> List[Waypoint]:
        """Return the current list of (x, y, z) waypoints."""
        return self.waypoints

    def to_pose_stamped_list(self, navigator) -> List[PoseStamped]:
        """Convert the stored waypoints into PoseStamped messages.

        Each pose faces the NEXT waypoint, so Nav2 doesn't spin the drone to
        face +x at every stop. The last pose keeps the previous heading.
        """
        poses = []
        n = len(self.waypoints)
        yaw = 0.0
        for i, (x, y, z) in enumerate(self.waypoints):
            if i + 1 < n:
                nx, ny, _ = self.waypoints[i + 1]
                if (nx, ny) != (x, y):
                    yaw = math.atan2(ny - y, nx - x)
            poses.append(self._make_pose(navigator, x, y, z, yaw))
        return poses

    def _make_pose(self, navigator, x: float, y: float, z: float = 0.0,
                   yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z   # Nav2 ignores this; altitude_controller uses it
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose