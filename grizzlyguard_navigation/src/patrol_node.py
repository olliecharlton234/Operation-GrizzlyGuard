#!/usr/bin/env python3
"""Drive the parrot along a patrol route chosen from patrol_routes.yaml.

Threat zones (from decision making), sim-space radius to nearest bear:
  GREEN > 40 m | YELLOW 15-40 m | RED 10-15 m | EMERGENCY <= 10 m

Manual override: the GUI publishes /manual_mode (latched Bool). When it goes
true the current Nav2 task is cancelled and the node waits. When it goes false
the patrol resumes from the waypoint it was heading to.
"""
import os
from enum import Enum

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from patrol_path import PatrolPath
from route_loader import load_config, map_waypoints, route_mode, route_names

PACKAGE = 'grizzlyguard_navigation'

# Must match the GUI's publisher (transient_local) or the latched value is missed.
LATCHED = QoSProfile(depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class PatrolRoute(Enum):
    """Names must match the keys under `routes:` in patrol_routes.yaml."""
    SHORT = 'short_route'
    LONG = 'long_route'
    GREEN = 'green_zone'
    YELLOW = 'yellow_zone'
    RED = 'red_zone'
    EMERGENCY = 'emergency_zone'
    APPROACH = 'zone_approach'


def main():
    rclpy.init()
    navigator = BasicNavigator(namespace='parrot1')

    # ---- ROS interfaces (need `navigator`, so they live here) ----
    alt_pub = navigator.create_publisher(Float64, 'target_altitude', 10)    # /parrot1/target_altitude
    wp_pub = navigator.create_publisher(String, 'current_waypoint', 10)     # /parrot1/current_waypoint
    status_pub = navigator.create_publisher(String, '/drone/status', 10)    # GUI activity label

    state = {'manual': False}
    navigator.create_subscription(
        Bool, '/manual_mode', lambda m: state.update(manual=m.data), LATCHED)

    def send_altitude(z):
        alt_pub.publish(Float64(data=float(z)))

    def update_gui(text):
        status_pub.publish(String(data=text))

    # ---- Route config ----
    default_cfg = os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'patrol_routes.yaml')
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

    # SLAM Toolbox replaces AMCL, so wait for bt_navigator + waypoint follower.
    update_gui('WAITING FOR NAV2')
    navigator._waitForNodeToActivate('bt_navigator')
    navigator.follow_waypoints_client.wait_for_server()
    navigator.get_logger().info('Nav2 ready - starting patrol.')

    forward = True
    resume = 0   # index into the full leg (pts) to restart from after manual

    while rclpy.ok():
        # ---- Hold while the operator is flying ----
        if state['manual']:
            update_gui('MANUAL')
            navigator.get_logger().info('Manual mode - patrol paused.')
            while state['manual'] and rclpy.ok():
                rclpy.spin_once(navigator, timeout_sec=0.2)
            navigator.get_logger().info(f'Manual released - resuming at waypoint {resume + 1}.')

        # ---- Build this leg ----
        pts = waypoints if forward else list(reversed(waypoints))
        if mode == 'loop':
            pts = pts + [pts[0]]          # close the loop back to the start
        leg = pts[resume:]

        poses = PatrolPath(frame_id=cfg['frame_id'], waypoints=leg).to_pose_stamped_list(navigator)
        send_altitude(leg[0][2])
        navigator.followWaypoints(poses)

        # ---- Fly it, watching for manual override ----
        reached = 0
        interrupted = False
        while not navigator.isTaskComplete():
            if state['manual']:
                navigator.cancelTask()
                interrupted = True
                break
            fb = navigator.getFeedback()
            if fb is not None:
                reached = min(fb.current_waypoint, len(leg) - 1)
                send_altitude(leg[reached][2])
                wp_pub.publish(String(data=str(resume + reached)))
                update_gui(f'PATROL {route}: waypoint {resume + reached + 1}/{len(pts)}')
            rclpy.spin_once(navigator, timeout_sec=0.5)

        if interrupted:
            resume += reached             # index in the FULL leg, not the trimmed one
            continue                      # top of loop waits for manual to end

        # ---- Leg finished normally ----
        resume = 0
        if mode == 'once':
            update_gui(f'IDLE: {route} complete')
            break
        if mode == 'back_and_fwd':
            forward = not forward

    rclpy.shutdown()


if __name__ == '__main__':
    main()