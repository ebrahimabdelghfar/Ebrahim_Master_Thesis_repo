#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('joy_to_ackermann'),
            'config',
            'params.yaml'
        ]),
        description='Path to config file for joy_to_ackermann node'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joy_to_ackermann_node = Node(
        package='joy_to_ackermann',
        executable='joy_to_ackermann_node',
        name='joy_to_ackermann_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        joy_node,
        joy_to_ackermann_node
    ])
