import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('adaptive_controller_benchmark'),
        'config',
        'benchmark_config.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description=(
            'Path to a ROS2 parameters YAML (node key: adaptive_controller_benchmark_node) '
            "holding all benchmark parameters. Defaults to the package's own "
            'config/benchmark_config.yaml.'
        ),
    )

    # Convenience CLI overrides for topics/output paths most commonly tweaked when
    # running alongside adaptive_stack.launch.py - everything else (log_interval,
    # plot_max_points, ...) is configured via config_file directly.
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/odom')
    waypoint_topic_arg = DeclareLaunchArgument('waypoint_topic', default_value='/raceline_waypoints')
    drive_topic_arg = DeclareLaunchArgument('drive_topic', default_value='/drive')
    pp_drive_topic_arg = DeclareLaunchArgument('pp_drive_topic', default_value='pp/drive_cmd')
    mpc_drive_topic_arg = DeclareLaunchArgument('mpc_drive_topic', default_value='mpc/drive_cmd')
    manager_state_topic_arg = DeclareLaunchArgument('manager_state_topic', default_value='manager/state')
    csv_output_path_arg = DeclareLaunchArgument(
        'csv_output_path', default_value='',
        description='Optional CSV output file path for raw per-tick logging')
    plot_output_dir_arg = DeclareLaunchArgument(
        'plot_output_dir', default_value='',
        description='If set, directory to export academic benchmark PNG plots to on shutdown')

    node = Node(
        package='adaptive_controller_benchmark',
        executable='adaptive_controller_benchmark_node',
        name='adaptive_controller_benchmark_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                'drive_topic': LaunchConfiguration('drive_topic'),
                'pp_drive_topic': LaunchConfiguration('pp_drive_topic'),
                'mpc_drive_topic': LaunchConfiguration('mpc_drive_topic'),
                'manager_state_topic': LaunchConfiguration('manager_state_topic'),
                'csv_output_path': LaunchConfiguration('csv_output_path'),
                'plot_output_dir': LaunchConfiguration('plot_output_dir'),
            },
        ],
    )

    return LaunchDescription([
        config_file_arg,
        odom_topic_arg,
        waypoint_topic_arg,
        drive_topic_arg,
        pp_drive_topic_arg,
        mpc_drive_topic_arg,
        manager_state_topic_arg,
        csv_output_path_arg,
        plot_output_dir_arg,
        node,
    ])
