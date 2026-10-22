"""Launch the RGB YOLO bear detector for one drone.

ros2 launch grizzlyguard_perception bear_yolo.launch.py
ros2 launch grizzlyguard_perception bear_yolo.launch.py model_path:=/abs/path/bear_custom.onnx
ros2 launch grizzlyguard_perception bear_yolo.launch.py namespace:=parrot1 use_sim_time:=true

Thermal detection is not launched here; RGB detection runs without any thermal input.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
    }
    model_path = LaunchConfiguration('model_path').perform(context)
    if model_path:
        overrides['model_path'] = model_path
    return [Node(
        package='grizzlyguard_perception',
        executable='bear_yolo_node',
        name='bear_yolo_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file'), overrides],
        output='screen',
    )]


def generate_launch_description():
    share = get_package_share_directory('grizzlyguard_perception')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='parrot1',
            description='Drone namespace; the camera is <namespace>/camera/image'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'bear_yolo.yaml'),
            description='Detector parameter file'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Override model (absolute path or file in the package models/ folder)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock'),
        OpaqueFunction(function=_node),
    ])