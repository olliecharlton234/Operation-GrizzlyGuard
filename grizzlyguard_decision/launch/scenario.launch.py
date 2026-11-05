#!/usr/bin/env python3
"""Start the whole system for one test scenario with one command.

  ros2 launch grizzlyguard_decision scenario.launch.py scenario:=yellow
  ros2 launch grizzlyguard_decision scenario.launch.py scenario:=red gui:=true
  ros2 launch grizzlyguard_decision scenario.launch.py scenario:=red viewer:=false

scenario: no_bear, green, yellow, red, emergency, not_a_bear, or none for all four
          bears walking normally. The e-stop, return-home and manual scenarios run
          on top of any of these, using the GUI buttons.

bear_custom.onnx must be in grizzlyguard_perception/models/ before building.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The trained model only knows bears and sometimes boxes people as one, so the standard
# model also looks for class "person". bear_response_node ignores bear boxes on people.
# Without this file the system still runs, but with no person check.
PERSON_MODEL = 'yolo26n.onnx'


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    scenario = get('scenario')
    sim_time = {'use_sim_time': True}
    bringup = get_package_share_directory('41068_ignition_bringup')
    perception = get_package_share_directory('grizzlyguard_perception')

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', '41068_ignition_parrot.launch.py')),
        launch_arguments={'slam': 'true', 'nav2': 'true', 'rviz': get('rviz'),
                          'world': 'yellowstone'}.items())]

    # Started after the drone has spawned.
    delayed = [
        Node(package='grizzlyguard_navigation', executable='altitude_controller',
             name='altitude_controller', namespace='parrot1', output='screen',
             remappings=[('cmd_vel_nav', 'cmd_vel'), ('cmd_vel', 'cmd_vel_gz')],
             parameters=[sim_time, {'min_altitude': 0.2}]),
        # Wait for Nav2 first. Started earlier, patrol_node's status check can go unanswered
        # and it then sits on WAITING FOR NAV2 forever.
        ExecuteProcess(
            cmd=['bash', '-c',
                 'until timeout 5 ros2 lifecycle get /parrot1/bt_navigator 2>/dev/null | grep -q active; '
                 'do sleep 2; done; '
                 f'exec ros2 run grizzlyguard_navigation patrol_node.py --ros-args '
                 f'-p route:={get("route")} -p use_sim_time:=true'],
            output='screen'),
        Node(package='grizzlyguard_navigation', executable='world_pose_publisher.py',
             name='world_pose_publisher', output='screen', parameters=[sim_time]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'bear_yolo.launch.py')),
            launch_arguments={'model_path': get('model')}.items()),
        Node(package='grizzlyguard_decision', executable='bear_response_node.py',
             name='bear_response_node', output='screen', parameters=[sim_time]),
    ]
    delayed.append(Node(
        package='grizzlyguard_perception', executable='bear_yolo_node',
        name='person_yolo_node', namespace='parrot1', output='screen',
        remappings=[('bear_detections', 'nonbear_detections'),
                    ('bear_detected', 'nonbear_detected'),
                    ('bear_detections/image', 'nonbear_detections/image')],
        parameters=[os.path.join(perception, 'config', 'bear_yolo.yaml'), sim_time,
                    {'model_path': PERSON_MODEL, 'target_class': 'person'}]))
    if get('viewer').lower() == 'true':
        # Shows the drone's camera with YOLO's boxes drawn on.
        delayed.append(Node(package='rqt_image_view', executable='rqt_image_view', name='yolo_viewer',
                            arguments=['/parrot1/bear_detections/image'], output='log'))
    if get('gui').lower() == 'true':
        # The GUI is plain CMake, not a ROS package, so its program stays in the build folder.
        workspace = os.path.dirname(os.path.dirname(get_package_prefix('grizzlyguard_decision')))
        program = os.path.join(workspace, 'build', 'grizzlyguard_gui', 'BearDroneGUI')
        if not os.path.exists(program):
            raise RuntimeError(f'{program} not found: run colcon build --packages-select grizzlyguard_gui')
        delayed.append(ExecuteProcess(cmd=[program], output='screen'))
    # The bears go first: they start at their world positions until the script moves
    # or hides them, and YOLO must not react to that. Run directly, not as a Node,
    # because the script has its own arguments and rejects ROS ones.
    start = float(get('start_delay'))
    actions.append(TimerAction(period=start, actions=[ExecuteProcess(
        cmd=['ros2', 'run', '41068_ignition_bringup', 'bear_gait_demo.py', '--duration', '7200']
        + ([] if scenario == 'none' else ['--scenario', scenario]),
        output='screen')]))
    actions.append(TimerAction(period=start + 6.0, actions=delayed))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='none',
                              description='no_bear, green, yellow, red, emergency, not_a_bear or none'),
        DeclareLaunchArgument('route', default_value='short_route', description='Patrol route'),
        DeclareLaunchArgument('model', default_value='bear_custom.onnx', description='YOLO bear model'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz'),
        DeclareLaunchArgument('gui', default_value='false', description='Open the team GUI'),
        DeclareLaunchArgument('viewer', default_value='true', description="Show YOLO's boxes"),
        DeclareLaunchArgument('start_delay', default_value='15.0',
                              description='Seconds to let the drone spawn first'),
        OpaqueFunction(function=_setup),
    ])
