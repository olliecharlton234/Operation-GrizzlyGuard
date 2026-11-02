from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    camera_frame = LaunchConfiguration('camera_frame')
    world_frame = LaunchConfiguration('world_frame')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='parrot1'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('world_frame', default_value='map'),
        Node(
            package='grizzlyguard_decision',
            executable='nav_follow_bridge',
            name='nav_follow_bridge',
            namespace=namespace,
            parameters=[
                {'namespace': namespace},
                {'camera_frame': camera_frame},
                {'world_frame': world_frame},
            ],
        ),
    ])
