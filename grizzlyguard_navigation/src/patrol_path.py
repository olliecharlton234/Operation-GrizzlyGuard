"""
Represents a patrol path for the parrot: an object holding a list of
waypoints, independent of ROS. A separate BasicNavigator instance is only
needed later, when converting these waypoints into PoseStamped messages to
send to Nav2 (PoseStamped needs a live clock, which the path object itself
doesn't have).

x = -75 to 75 and y = -40 to +235, z is just 0
"""

class PatrolPath:
    """A named, ordered list of (x, y, z) waypoints for a patrol route.
    this does not manage any instructions with ROS2 and purely manages the route."""

    def __init__(self, frame_id: str = 'parrot1_map', waypoints: Optional[List[Tuple[float, float, float]]] = None, mode):

        self.frame_id = frame_id
        self.waypoints: List[Tuple[float, float, float]] = []

        if waypoints is not None:
            self.set_path(waypoints)
        else:
            self.generate_path()

    def generate_path(self) -> bool:
        """A Stretch goal, will be used to generate an arbitrary path from the yellowstone environment. this will be updated later, 
        however this is not necessary for the current moment
        """
        default_points = [
            (2.0, 2.0, 0.0),
            (25.0, -25.0, 0.0),
            (50.0, -40.0, 0.0),
            (50.0, -10.0, 0.0),
            (-20, 100, 0.0),
            (-30, 150, 0.0),
        ]

        
        self.set_path(default_points)
        return True


    def set_path(self, points: List[Tuple[float, float, float]]) -> None:
        """Replace the current waypoint list with `points`.

        Each point is an (x, y, z) tuple. Validation is intentionally
        minimal -- this is a good place to add checks later (e.g. rejecting
        points outside the known map bounds).
        """
        if not points:
            raise ValueError('set_path() requires at least one waypoint.')
        self.waypoints = list(points)

    def get_path(self) -> List[Tuple[float, float, float]]:
        """Return the current list of (x, y, z) waypoints."""
        return self.waypoints

    def to_pose_stamped_list(self, navigator) -> List[PoseStamped]:
        """Convert the stored waypoints into PoseStamped messages.

        This needs a live `navigator` (or any rclpy node) purely to read the
        current clock for the message timestamp -- the path itself has no
        ROS dependency until this point.
        """
        poses = []
        for x, y, z in self.waypoints:
            poses.append(self._make_pose(navigator, x, y, z))
        return poses

    def _make_pose(self, navigator, x: float, y: float, z: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

if __name__ == '__main__':
    main()