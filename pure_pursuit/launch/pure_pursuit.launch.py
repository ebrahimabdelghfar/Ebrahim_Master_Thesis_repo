import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('pure_pursuit')
    default_config = os.path.join(pkg_share, 'config', 'pure_pursuit.yaml')

    with open(default_config, 'r') as f:
        params = yaml.safe_load(f)['pure_pursuit']['ros__parameters']

    # Declare launch arguments
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='driving_style2.csv',
        description='Name of the waypoint CSV file'
    )

    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value=str(params['lookahead_distance']),
        description='Lookahead distance for pure pursuit'
    )

    velocity_arg = DeclareLaunchArgument(
        'desired_velocity',
        default_value='3.0',
        description='Desired velocity in m/s'
    )

    use_fixed_reference_speed_arg = DeclareLaunchArgument(
        'use_fixed_reference_speed',
        default_value=str(params.get('use_fixed_reference_speed', False)),
        description='True: ignore the per-waypoint vx_mps and track '
                    'fixed_reference_speed everywhere. False: follow the '
                    "raceline's own speed profile."
    )

    fixed_reference_speed_arg = DeclareLaunchArgument(
        'fixed_reference_speed',
        default_value=str(params.get('fixed_reference_speed', 3.0)),
        description='Hardcoded reference speed in m/s, used only when '
                    'use_fixed_reference_speed is true'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value=params['odom_topic'],
        description='Odometry topic name'
    )

    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic',
        default_value=params['waypoint_topic'],
        description='Raceline WaypointArray topic name'
    )

    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic',
        default_value=params['drive_topic'],
        description='Drive topic name'
    )

    show_animation_arg = DeclareLaunchArgument(
        'show_animation',
        default_value=str(params['show_animation']),
        description='Whether to show matplotlib animation'
    )

    standalone_mode_arg = DeclareLaunchArgument(
        'standalone_mode',
        default_value=str(params.get('standalone_mode', False)),
        description='True: always publish, ignore Start_Working_pp (solo tuning). '
                    'False: gate publishing behind Start_Working_pp (managed by '
                    'adaptive_controller_manager).'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to the pure_pursuit parameter YAML file')

    # Load the YAML file first for full defaults, then apply the dict of
    # LaunchConfiguration overrides second: for matching keys, later entries
    # in `parameters=[...]` win in launch_ros.Node. This actually wires the
    # declared launch arguments into the node (mirrors
    # mpc_path_tracking.launch.py's pattern) - previously this file declared
    # these arguments but never passed them into the Node.
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node.py',
        name='pure_pursuit',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'use_fixed_reference_speed': LaunchConfiguration(
                    'use_fixed_reference_speed'),
                'fixed_reference_speed': LaunchConfiguration(
                    'fixed_reference_speed'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                'drive_topic': LaunchConfiguration('drive_topic'),
                'show_animation': LaunchConfiguration('show_animation'),
                'standalone_mode': LaunchConfiguration('standalone_mode'),
            },
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        waypoint_file_arg,
        lookahead_arg,
        velocity_arg,
        use_fixed_reference_speed_arg,
        fixed_reference_speed_arg,
        odom_topic_arg,
        waypoint_topic_arg,
        drive_topic_arg,
        show_animation_arg,
        standalone_mode_arg,
        pure_pursuit_node,
    ])
