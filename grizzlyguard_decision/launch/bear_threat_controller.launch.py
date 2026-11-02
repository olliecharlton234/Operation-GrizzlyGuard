#!/usr/bin/env python3
"""Launch the real Gazebo bear-threat controller for the Parrot drone."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    world_name = DeclareLaunchArgument(
        'world_name',
        default_value='yellowstone',
        description='Gazebo world to use for the real sim',
    )
    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='parrot1',
        description='ROS namespace for the drone',
    )
    bear_model_name = DeclareLaunchArgument(
        'bear_model_name',
        default_value='bear_meadow',
        description='Gazebo model name for the bear',
    )
    border_x = DeclareLaunchArgument(
        'border_x',
        default_value='0.0',
        description='Border X coordinate between forest and civilian zones',
    )
    loop_period = DeclareLaunchArgument(
        'loop_period',
        default_value='1.0',
        description='Controller update period in seconds',
    )
    follow_offset_m = DeclareLaunchArgument(
        'follow_offset_m',
        default_value='2.0',
        description='Distance offset to keep behind the bear while following',
    )

    ld.add_action(world_name)
    ld.add_action(robot_namespace)
    ld.add_action(bear_model_name)
    ld.add_action(border_x)
    ld.add_action(loop_period)
    ld.add_action(follow_offset_m)

    ld.add_action(Node(
        package='grizzlyguard_decision',
        executable='real_bear_controller',
        name='real_bear_controller',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('world_name'),
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'bear_model_name': LaunchConfiguration('bear_model_name'),
            'border_x': LaunchConfiguration('border_x'),
            'loop_period': LaunchConfiguration('loop_period'),
            'follow_offset_m': LaunchConfiguration('follow_offset_m'),
        }],
    ))

    return ld
