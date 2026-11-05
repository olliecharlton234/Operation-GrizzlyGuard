#!/usr/bin/env python3
"""Drive the parrot along a patrol route and obey the GUI's flight commands.

    ros2 run grizzlyguard_navigation patrol_node.py --ros-args \
        -p route:=short_route -p use_sim_time:=true

Threat zones (from decision making), sim-space radius to nearest bear:
  GREEN > 40 m | YELLOW 15-40 m | RED 10-15 m | EMERGENCY <= 10 m

GUI interface (all topics are absolute):
  SUB /manual_mode            std_msgs/Bool  (latched) true = operator flying, go quiet
  SUB /drone/takeoff          std_msgs/Empty climb to cruise altitude, then patrol
  SUB /drone/land             std_msgs/Empty stop and descend in place
  SUB /drone/return_to_home   std_msgs/Empty stop patrol, fly to the spawn point, hover
  SUB /drone/emergency_stop   std_msgs/Empty highest priority: cancel everything, descend in place
  PUB /drone/status           std_msgs/String current mode / activity for the GUI

In MANUAL mode the GUI flies takeoff/land itself (it owns /cmd_vel). This node
only records them for when manual mode ends:
  last button land    -> stay IDLE (landed) until Takeoff is pressed
  last button takeoff -> climb to cruise and rejoin the patrol route

Priority when several requests arrive together:
  EMERGENCY > MANUAL > LAND > RETURN_HOME > TAKEOFF

x/y go to Nav2. z goes to /parrot1/target_altitude for altitude_controller.
Landing needs altitude_controller's min_altitude below land_altitude, e.g.
  ros2 run grizzlyguard_navigation altitude_controller --ros-args \
      -r __ns:=/parrot1 -p min_altitude:=0.2
"""
import os
import time
from enum import Enum

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64, String

from patrol_path import PatrolPath
from route_loader import load_config, map_waypoints, route_mode, route_names

PACKAGE = 'grizzlyguard_navigation'

# Must match the GUI's /manual_mode publisher (transient_local) or the
# latched value published before this node started is missed.
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


class Mode(Enum):
    IDLE = 'IDLE'                 # hovering, waiting for takeoff / a command
    TAKEOFF = 'TAKEOFF'
    PATROL = 'PATROL'
    MANUAL = 'MANUAL'
    RETURN_HOME = 'RETURN_HOME'
    LANDING = 'LANDING'
    EMERGENCY = 'EMERGENCY'


def main():
    rclpy.init()
    nav = BasicNavigator(namespace='parrot1')
    log = nav.get_logger()

    # ---------------- Parameters ----------------
    default_cfg = os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'patrol_routes.yaml')
    route = nav.declare_parameter('route', PatrolRoute.SHORT.value).value
    cfg_path = nav.declare_parameter('routes_file', default_cfg).value
    cruise_z = float(nav.declare_parameter('cruise_altitude', 2.0).value)
    land_z = float(nav.declare_parameter('land_altitude', 0.3).value)
    autostart = bool(nav.declare_parameter('autostart', True).value)
    alt_timeout = float(nav.declare_parameter('altitude_timeout', 30.0).value)
    # false = if takeoff can't confirm altitude (e.g. 2D plugin, no z odometry),
    # patrol anyway in 2D instead of stopping in IDLE.
    strict_alt = bool(nav.declare_parameter('strict_altitude', True).value)

    # ---------------- ROS interfaces ----------------
    alt_pub = nav.create_publisher(Float64, 'target_altitude', 10)       # /parrot1/target_altitude
    wp_pub = nav.create_publisher(String, 'current_waypoint', 10)        # /parrot1/current_waypoint
    status_pub = nav.create_publisher(String, '/drone/status', 10)

    # Requests from the GUI. 'manual' is a state; the rest are one-shot flags.
    req = {'manual': False, 'takeoff': False, 'land': False,
           'rth': False, 'estop': False}
    alt = {'reached': False}

    def flag(name):
        def cb(_msg):
            log.info(f'GUI request: {name}')
            req[name] = True
        return cb

    nav.create_subscription(Bool, '/manual_mode',
                            lambda m: req.update(manual=m.data), LATCHED)
    nav.create_subscription(Empty, '/drone/takeoff', flag('takeoff'), 10)
    nav.create_subscription(Empty, '/drone/land', flag('land'), 10)
    nav.create_subscription(Empty, '/drone/return_to_home', flag('rth'), 10)
    nav.create_subscription(Empty, '/drone/emergency_stop', flag('estop'), 10)
    nav.create_subscription(Bool, 'altitude_reached',                    # /parrot1/altitude_reached
                            lambda m: alt.update(reached=m.data), 10)

    last_status = {'text': None}

    def status(text):
        status_pub.publish(String(data=text))
        if text != last_status['text']:
            log.info(f'STATUS: {text}')
            last_status['text'] = text

    def send_altitude(z):
        alt_pub.publish(Float64(data=float(z)))

    def spin(t=0.2):
        rclpy.spin_once(nav, timeout_sec=t)

    def interrupted():
        """True if a request should stop whatever is currently running."""
        return req['estop'] or req['manual'] or req['land'] or req['rth']

    def wait_for_altitude(z, label):
        """Send a target altitude and wait for altitude_controller to reach it.
        Returns 'done', 'timeout' or 'interrupted'."""
        send_altitude(z)
        start = time.monotonic()
        while rclpy.ok():
            if label != 'EMERGENCY' and interrupted():
                return 'interrupted'
            elapsed = time.monotonic() - start
            # Ignore the first second: altitude_reached may still describe the old target.
            if elapsed > 1.0 and alt['reached']:
                return 'done'
            if elapsed > alt_timeout:
                return 'timeout'
            send_altitude(z)
            status(f'{label}: climbing/descending to {z:.1f} m')
            spin(0.2)
        return 'interrupted'

    # ---------------- Route config ----------------
    cfg = load_config(cfg_path)
    if route not in route_names(cfg):
        log.error(f"Unknown route '{route}'. Available: {', '.join(route_names(cfg))}")
        rclpy.shutdown()
        return

    # A waypoint z of 0 (default_z: 0.0 in the yaml) would fly into the ground
    # once min_altitude is lowered for landing, so fall back to cruise height.
    waypoints = [(p[0], p[1], p[2] if len(p) > 2 and p[2] > 0.0 else cruise_z)
                 for p in map_waypoints(cfg, route)]
    route_type = route_mode(cfg, route)
    log.info(f"Route '{route}' ({route_type}, {len(waypoints)} waypoints): "
             f"{cfg['routes'][route].get('description', '')}")

    # Home = spawn point. parrot1_map's origin IS the spawn, so home is (0, 0).
    def home_pose():
        p = PoseStamped()
        p.header.frame_id = cfg['frame_id']
        p.header.stamp = nav.get_clock().now().to_msg()
        p.pose.orientation.w = 1.0
        return p

    # SLAM Toolbox replaces AMCL, so wait for bt_navigator + waypoint follower.
    status('WAITING FOR NAV2')
    nav._waitForNodeToActivate('bt_navigator')
    nav.follow_waypoints_client.wait_for_server()
    log.info('Nav2 ready.')

    # ---------------- State machine ----------------
    mode = Mode.TAKEOFF if autostart else Mode.IDLE
    before_manual = mode
    forward = True
    resume = 0          # index into the current leg to restart the patrol from

    while rclpy.ok():
        # ---- Requests, in priority order ----
        if req['estop']:
            req['estop'] = False
            req['land'] = req['rth'] = req['takeoff'] = False
            mode = Mode.EMERGENCY
        elif req['manual']:
            if mode != Mode.MANUAL:
                before_manual = mode
                mode = Mode.MANUAL
            # In manual mode the GUI flies takeoff/land itself. Consume the
            # requests here so they don't run again when manual mode ends,
            # and leave altitude_controller a target that matches what the
            # operator did (it resumes from this target on release).
            if req['land']:
                req['land'] = False
                send_altitude(land_z)
                before_manual = Mode.IDLE     # stay down after manual ends
                log.info('Manual land: will stay IDLE on release')
            if req['takeoff']:
                req['takeoff'] = False
                send_altitude(cruise_z)
                # Same as an automatic takeoff: rejoin the route on release,
                # from the waypoint it was heading to when it was interrupted.
                if before_manual in (Mode.IDLE, Mode.LANDING, Mode.TAKEOFF):
                    before_manual = Mode.PATROL
                log.info('Manual takeoff: will rejoin the patrol route on release')
        elif mode == Mode.MANUAL:
            # Operator released control: resume what we were doing.
            log.info(f'Manual released - resuming {before_manual.value}')
            mode = before_manual
        elif req['land']:
            req['land'] = False
            mode = Mode.LANDING
        elif req['rth']:
            req['rth'] = False
            mode = Mode.RETURN_HOME
        elif req['takeoff']:
            req['takeoff'] = False
            if mode == Mode.IDLE:
                mode = Mode.TAKEOFF
            else:
                log.info(f'Takeoff ignored: already {mode.value}')

        # ---- Run the current mode ----
        if mode == Mode.MANUAL:
            # Publish once on entry only, so the GUI's own manual takeoff/land
            # messages on /drone/status aren't overwritten every 0.2 s.
            if last_status['text'] != 'MANUAL':
                status('MANUAL')
            spin(0.2)

        elif mode == Mode.IDLE:
            status('IDLE: hovering, waiting for takeoff')
            spin(0.2)

        elif mode == Mode.EMERGENCY:
            # Nav2 was already cancelled where the request interrupted it.
            # altitude_controller holds x/y once Nav2 goes quiet; descend in place.
            nav.cancelTask()
            result = wait_for_altitude(land_z, 'EMERGENCY')
            status('EMERGENCY: landed' if result == 'done'
                   else f'EMERGENCY: descent {result}')
            if req['manual']:
                before_manual = Mode.IDLE   # don't resume patrol after an e-stop
            mode = Mode.IDLE

        elif mode == Mode.LANDING:
            result = wait_for_altitude(land_z, 'LANDING')
            if result == 'done':
                status('LANDED')
                mode = Mode.IDLE
            elif result == 'timeout':
                status('LANDING FAILED: altitude not reached')
                mode = Mode.IDLE
            # 'interrupted' -> loop round and handle the new request

        elif mode == Mode.TAKEOFF:
            result = wait_for_altitude(cruise_z, 'TAKEOFF')
            if result == 'done':
                status('TAKEOFF COMPLETE')
                mode = Mode.PATROL
            elif result == 'timeout':
                if strict_alt:
                    status('TAKEOFF FAILED: altitude not reached')
                    mode = Mode.IDLE
                else:
                    status('TAKEOFF: altitude not confirmed - patrolling anyway')
                    mode = Mode.PATROL

        elif mode == Mode.RETURN_HOME:
            send_altitude(cruise_z)

            # The patrol's followWaypoints was just cancelled. waypoint_follower
            # cancels its own NavigateToPose asynchronously; a new goal sent
            # straight away gets caught by that cancel and ends as CANCELED within
            # a fraction of a second. Give Nav2 a moment to go idle first, and
            # retry if it still happens.
            settle_end = time.monotonic() + 2.0
            while time.monotonic() < settle_end and not interrupted():
                status('RETURN_HOME: waiting for Nav2 to stop the patrol')
                spin(0.1)

            outcome = None            # 'success', 'failed:<reason>' or 'interrupted'
            for attempt in range(1, 4):
                if interrupted() and not req['rth']:
                    outcome = 'interrupted'
                    break
                if not nav.goToPose(home_pose()):
                    outcome = 'failed:goal rejected by Nav2'
                    break
                started = time.monotonic()
                stopped = False
                while not nav.isTaskComplete():
                    if req['estop'] or req['manual'] or req['land']:
                        nav.cancelTask()
                        stopped = True
                        break
                    status('RETURN_HOME: flying to spawn')
                    spin(0.2)
                if stopped:
                    outcome = 'interrupted'
                    break
                result = nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    outcome = 'success'
                    break
                quick = time.monotonic() - started < 2.0
                log.warn(f'RETURN_HOME attempt {attempt}: {result.name}'
                         f'{" after <2 s" if quick else ""}')
                if result == TaskResult.CANCELED and quick and attempt < 3:
                    for _ in range(10):       # ~1 s, then try again
                        spin(0.1)
                    continue
                outcome = f'failed:{result.name}'
                break
            req['rth'] = False

            if outcome == 'interrupted':
                continue                      # top of loop handles the new request
            if outcome == 'success':
                status('HOME: hovering at spawn')
            else:
                status(f'RETURN_HOME FAILED ({outcome.split(":", 1)[-1]})')
            resume, forward = 0, True         # next patrol starts from the beginning
            mode = Mode.IDLE

        elif mode == Mode.PATROL:
            pts = waypoints if forward else list(reversed(waypoints))
            if route_type == 'loop':
                pts = pts + [pts[0]]          # close the loop back to the start
            leg = pts[resume:] or pts

            poses = PatrolPath(frame_id=cfg['frame_id'],
                               waypoints=leg).to_pose_stamped_list(nav)
            send_altitude(leg[0][2])
            nav.followWaypoints(poses)

            reached = 0
            stopped = False
            while not nav.isTaskComplete():
                if interrupted():
                    nav.cancelTask()
                    stopped = True
                    break
                fb = nav.getFeedback()
                if fb is not None:
                    reached = min(fb.current_waypoint, len(leg) - 1)
                    send_altitude(leg[reached][2])
                    wp_pub.publish(String(data=str(resume + reached)))
                    status(f'PATROL {route}: waypoint {resume + reached + 1}/{len(pts)}')
                spin(0.5)

            if stopped:
                resume += reached             # index in the FULL leg
                continue                      # top of loop handles the request

            # Leg finished normally
            resume = 0
            if route_type == 'once':
                status(f'IDLE: {route} complete')
                mode = Mode.IDLE
            elif route_type == 'back_and_fwd':
                forward = not forward

    rclpy.shutdown()


if __name__ == '__main__':
    main()