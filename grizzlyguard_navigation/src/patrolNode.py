"""Drive the parrot around a looping PatrolPath using Nav2's waypoint follower."""
 
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
 
from patrol_path import PatrolPath
 
 
def main():
    rclpy.init()
    navigator = BasicNavigator(namespace='parrot1')
    navigator.waitUntilNav2Active()
 
    # No waypoints passed in -> generate_path() supplies the default route.
    # Pass waypoints=[(x, y, z), ...] here instead to patrol a custom route.
    path = PatrolPath(frame_id='parrot1_map')
    poses = path.to_pose_stamped_list(navigator)
 
    while rclpy.ok():
        navigator.followWaypoints(poses)
        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.5)
 
 
if __name__ == '__main__':
    main()