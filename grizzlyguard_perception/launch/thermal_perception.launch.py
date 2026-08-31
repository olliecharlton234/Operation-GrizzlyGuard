"""Launch the GrizzlyGuard thermal (colour-segmentation) perception node.

This node is a placeholder for a real thermal camera: it treats red/orange
/yellow pixels in a simulated RGB camera image as "warm", and looks for a
sufficiently large warm blob to flag as a possible bear heat signature.

By default this subscribes to the Parrot drone's simulated camera
(/parrot1/camera/image), which is bridged from Ignition Gazebo by
41068_ignition_bringup. Start the simulation first, e.g.:

    ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py

then, in a second terminal:

    ros2 launch grizzlyguard_perception thermal_perception.launch.py

See grizzlyguard_perception/README.md for the full list of published topics,
parameters, and known limitations.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('grizzlyguard_perception')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/parrot1/camera/image',
        description=(
            'Input sensor_msgs/Image topic to run detection on '
            '(e.g. the Parrot drone camera bridged by 41068_ignition_bringup).'
        ),
    )
    ld.add_action(image_topic_arg)

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_path, 'config', 'thermal_perception_params.yaml']
        ),
        description='YAML file with HSV thresholds and detection parameters.',
    )
    ld.add_action(params_file_arg)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    )
    ld.add_action(use_sim_time_arg)

    ld.add_action(Node(
        package='grizzlyguard_perception',
        executable='thermal_perception_node',
        name='thermal_perception_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    ))

    return ld
