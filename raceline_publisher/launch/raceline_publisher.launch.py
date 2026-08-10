import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('raceline_publisher')
    default_config = os.path.join(pkg_share, 'config', 'raceline_publisher.yaml')

    with open(default_config, 'r') as f:
        params = yaml.safe_load(f)['raceline_publisher']['ros__parameters']

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to the raceline_publisher parameter YAML file')

    raceline_csv_arg = DeclareLaunchArgument(
        'raceline_csv',
        default_value=params['raceline_csv'],
        description='Full path to the raceline CSV file to publish'
    )
    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic',
        default_value=params['waypoint_topic'],
        description='Topic to publish the WaypointArray/MarkerArray on'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value=params['frame_id'],
        description='Frame id stamped on published messages'
    )
    publish_initial_pose_arg = DeclareLaunchArgument(
        'publish_initial_pose',
        default_value=str(params['publish_initial_pose']),
        description='Whether to publish the first waypoint as /initialpose'
    )

    # Load the YAML file first for full defaults, then apply the dict of
    # LaunchConfiguration overrides second: for matching keys, later entries
    # in `parameters=[...]` win in launch_ros.Node (mirrors pure_pursuit's
    # launch file pattern).
    raceline_publisher_node = Node(
        package='raceline_publisher',
        executable='raceline_publisher_node',
        name='raceline_publisher',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'raceline_csv': LaunchConfiguration('raceline_csv'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_initial_pose': LaunchConfiguration('publish_initial_pose'),
            },
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        raceline_csv_arg,
        waypoint_topic_arg,
        frame_id_arg,
        publish_initial_pose_arg,
        raceline_publisher_node,
    ])
