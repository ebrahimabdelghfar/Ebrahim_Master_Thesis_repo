"""Launch file for estimation_benchmark node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('estimation_benchmark')
    default_config = os.path.join(pkg_dir, 'config', 'benchmark_config.yaml')

    # ── Launch arguments ──
    args = [
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='Odometry topic'),
        DeclareLaunchArgument('ackermann_cmd_topic', default_value='/drive',
                              description='Ackermann command topic'),
        DeclareLaunchArgument('training_complete_topic',
                              default_value='/sysid/training_complete',
                              description='Training completion signal topic'),
        DeclareLaunchArgument('prediction_steps', default_value='[1, 5, 10]',
                              description='List of prediction horizons'),
        DeclareLaunchArgument('dt', default_value='0.02',
                              description='Sampling time [s]'),
        DeclareLaunchArgument('enable_tire_force_benchmark',
                              default_value='false',
                              description='Enable tire force benchmarking'),
        DeclareLaunchArgument('tire_forces_topic', default_value='/tire_forces',
                              description='CarMaker tire forces topic'),
        DeclareLaunchArgument('log_interval', default_value='100',
                              description='Samples between logged summaries'),
        DeclareLaunchArgument('csv_output_path', default_value='',
                              description='Optional CSV output path'),
        DeclareLaunchArgument('min_velocity', default_value='0.5',
                              description='Min v_x for benchmarking [m/s]'),
    ]

    node = Node(
        package='estimation_benchmark',
        executable='estimation_benchmark_node',
        name='estimation_benchmark_node',
        output='screen',
        parameters=[
            default_config,
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'ackermann_cmd_topic': LaunchConfiguration('ackermann_cmd_topic'),
                'training_complete_topic': LaunchConfiguration('training_complete_topic'),
                'dt': LaunchConfiguration('dt'),
                'enable_tire_force_benchmark': LaunchConfiguration('enable_tire_force_benchmark'),
                'tire_forces_topic': LaunchConfiguration('tire_forces_topic'),
                'log_interval': LaunchConfiguration('log_interval'),
                'csv_output_path': LaunchConfiguration('csv_output_path'),
                'min_velocity': LaunchConfiguration('min_velocity'),
            },
        ],
    )

    return LaunchDescription(args + [node])
