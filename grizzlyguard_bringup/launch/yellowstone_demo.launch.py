from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    slam = LaunchConfiguration('slam')
    nav2 = LaunchConfiguration('nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    route = LaunchConfiguration('route')
    min_altitude = LaunchConfiguration('min_altitude')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='yellowstone',
            choices=['simple_trees', 'large_demo', 'yellowstone'],
            description='World to launch in the Ignition simulator.'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Launch SLAM for the Parrot drone.'
        ),
        DeclareLaunchArgument(
            'nav2',
            default_value='true',
            description='Launch Nav2 for the Parrot drone.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for the Parrot drone.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='parrot1',
            description='Robot namespace used by the detection and patrol nodes.'
        ),
        DeclareLaunchArgument(
            'route',
            default_value='long_route',
            description='Patrol route name from patrol_routes.yaml.'
        ),
        DeclareLaunchArgument(
            'min_altitude',
            default_value='0.2',
            description='Minimum altitude for the altitude controller.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'launch',
                '41068_ignition_parrot.launch.py',
            ])),
            launch_arguments={
                'world': world,
                'slam': slam,
                'nav2': nav2,
                'rviz': rviz,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('grizzlyguard_perception'),
                'launch',
                'bear_yolo.launch.py',
            ])),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        Node(
            package='grizzlyguard_navigation',
            executable='altitude_controller',
            namespace=namespace,
            parameters=[{'min_altitude': min_altitude, 'use_sim_time': True}],
            remappings=[
                ('cmd_vel_nav', 'cmd_vel'),
                ('cmd_vel', 'cmd_vel_gz'),
            ],
            output='screen',
        ),

        Node(
            package='grizzlyguard_navigation',
            executable='world_pose_publisher.py',
            namespace=namespace,
            parameters=[{
                'spawn_x': 0.0,
                'spawn_y': 0.0,
                'spawn_z': 0.0,
                'spawn_yaw': 0.0,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        Node(
            package='grizzlyguard_decision',
            executable='bear_perception_bridge.py',
            name='bear_perception_bridge',
            namespace=namespace,
            output='screen',
        ),

        Node(
            package='grizzlyguard_decision',
            executable='nav_follow_bridge',
            name='nav_follow_bridge',
            namespace=namespace,
            output='screen',
        ),

        Node(
            package='grizzlyguard_navigation',
            executable='patrol_node.py',
            name='patrol_node',
            namespace=namespace,
            parameters=[{
                'route': route,
                'use_sim_time': True,
                'strict_altitude': False,
            }],
            output='screen',
        ),
    ])
