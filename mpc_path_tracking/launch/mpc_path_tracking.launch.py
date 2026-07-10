import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mpc_path_tracking')
    default_config = os.path.join(pkg_share, 'config', 'mpc_path_tracking.yaml')

    # Launch-argument defaults are read from the YAML file itself (not
    # hardcoded) so editing config/mpc_path_tracking.yaml actually takes
    # effect without also having to pass a matching `name:=value` CLI
    # override. Without this, a fixed hardcoded default would always win
    # over the YAML value, since the override dict below is unconditionally
    # layered on top of the YAML in `parameters=[...]`.
    with open(default_config, 'r') as f:
        params = yaml.safe_load(f)['mpc_path_tracking']['ros__parameters']

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to the mpc_path_tracking parameter YAML file')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value=params['odom_topic'],
        description='Odometry input topic')
    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic', default_value=params['waypoint_topic'],
        description='Raceline/WaypointArray input topic')
    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic', default_value=params['drive_topic'],
        description='AckermannDriveStamped output topic (mux nav channel)')
    horizon_n_arg = DeclareLaunchArgument(
        'horizon_n', default_value=str(params['horizon']['N']),
        description='MPC prediction horizon steps')
    control_rate_hz_arg = DeclareLaunchArgument(
        'control_rate_hz', default_value=str(params['horizon']['control_rate_hz']),
        description='Control loop rate [Hz]')

    # Load the YAML file first for full defaults, then apply the dict of
    # LaunchConfiguration overrides second: for matching keys, later entries
    # in `parameters=[...]` win in launch_ros.Node. This actually wires the
    # declared launch arguments into the node, unlike pure_pursuit's launch
    # file, which declares several LaunchConfiguration args but never wires
    # any of them into its Node's parameters. Since the launch-argument
    # defaults above are themselves read from the same YAML file, this
    # override is a no-op unless a CLI `name:=value` is actually passed.
    mpc_node = Node(
        package='mpc_path_tracking',
        executable='mpc_node',
        name='mpc_path_tracking',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                'drive_topic': LaunchConfiguration('drive_topic'),
                'horizon.N': LaunchConfiguration('horizon_n'),
                'horizon.control_rate_hz': LaunchConfiguration('control_rate_hz'),
            },
        ],
    )

    return LaunchDescription([
        config_file_arg,
        odom_topic_arg,
        waypoint_topic_arg,
        drive_topic_arg,
        horizon_n_arg,
        control_rate_hz_arg,
        mpc_node,
    ])
