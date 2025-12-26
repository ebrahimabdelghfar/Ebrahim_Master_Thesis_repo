import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('pure_pursuit')
    
    # Declare launch arguments
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='driving_style2.csv',
        description='Name of the waypoint CSV file'
    )
    
    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='1.5',
        description='Lookahead distance for pure pursuit'
    )
    
    velocity_arg = DeclareLaunchArgument(
        'desired_velocity',
        default_value='2.0',
        description='Desired velocity in m/s'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic name'
    )
    
    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic',
        default_value='/drive',
        description='Drive topic name'
    )
    
    show_animation_arg = DeclareLaunchArgument(
        'show_animation',
        default_value='False',
        description='Whether to show matplotlib animation'
    )

    # Pure pursuit node
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node.py',
        name='pure_pursuit',
        parameters=[{
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'desired_velocity': LaunchConfiguration('desired_velocity'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'drive_topic': LaunchConfiguration('drive_topic'),
            'show_animation': LaunchConfiguration('show_animation'),
        }],
        output='screen'
    )

    return LaunchDescription([
        waypoint_file_arg,
        lookahead_arg,
        velocity_arg,
        odom_topic_arg,
        drive_topic_arg,
        show_animation_arg,
        pure_pursuit_node,
    ])
