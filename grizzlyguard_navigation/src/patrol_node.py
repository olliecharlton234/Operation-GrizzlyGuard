#!/usr/bin/env python3
"""Drive the parrot along a patrol route chosen from patrol_routes.yaml."""
import os
from enum import Enum

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Float64

from patrol_path import PatrolPath
from route_loader import load_config, map_waypoints, route_mode, route_names

PACKAGE = 'grizzlyguard_navigation'

'''
from decision making, threat zones are integrated
Actual sim-space zone table to be used for the world geometry:
  - GREEN:     radius > 40.0 m
  - YELLOW:    15.0 < radius <= 40.0 m
  - RED:       10.0 < radius < 15.0 m
  - EMERGENCY: radius <= 10.0 m

'''

# set an enum so that different routes can be mapped for demonstration purposes
class PatrolRoute(Enum):
    """Names must match the keys under `routes:` in patrol_routes.yaml."""
    SHORT = 'short_route'
    LONG = 'long_route'
    GREEN = 'green_zone'
    YELLOW = 'yellow_zone'
    RED = 'red_zone'
    EMERGENCY = 'emergency_zone'
    APPROACH = 'zone_approach'
 
def update_gui(current_waypoint, current_location):
    pass


def main():
    rclpy.init()
    navigator = BasicNavigator(namespace='parrot1')
 
    default_cfg = os.path.join(
        get_package_share_directory('grizzlyguard_navigation'), 'config', 'patrol_routes.yaml')
    route = navigator.declare_parameter('route', PatrolRoute.SHORT.value).value
    cfg_path = navigator.declare_parameter('routes_file', default_cfg).value
 
    cfg = load_config(cfg_path)
    if route not in route_names(cfg):
        navigator.get_logger().error(
            f"Unknown route '{route}'. Available: {', '.join(route_names(cfg))}")
        rclpy.shutdown()
        return
 
    waypoints = map_waypoints(cfg, route)
    mode = route_mode(cfg, route)
    navigator.get_logger().info(
        f"Route '{route}' ({mode}, {len(waypoints)} waypoints): "
        f"{cfg['routes'][route].get('description', '')}")
 
    # SLAM Toolbox replaces AMCL here, so don't wait for a localiser.
    # Wait for Nav2's navigator and the waypoint follower instead.
    navigator._waitForNodeToActivate('bt_navigator')
    navigator.get_logger().info('Waiting for waypoint follower...')
    navigator.follow_waypoints_client.wait_for_server()
    navigator.get_logger().info('Nav2 ready - starting patrol.')
 
    forward = True
    while rclpy.ok():
        pts = waypoints if forward else list(reversed(waypoints))
        if mode == 'loop':
            pts = pts + [pts[0]]          # close the loop back to the start
 
        path = PatrolPath(frame_id=cfg['frame_id'], waypoints=pts)
        poses = path.to_pose_stamped_list(navigator)
 
        navigator.followWaypoints(poses)
        while not navigator.isTaskComplete():
            fb = navigator.getFeedback()
            if fb is not None:
                update_gui(fb.current_waypoint, None)
            rclpy.spin_once(navigator, timeout_sec=0.5)
 
        if mode == 'once':
            break
        if mode == 'back_and_fwd':
            forward = not forward
 
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()