import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic name'
    )
    
    ackermann_topic_arg = DeclareLaunchArgument(
        'ackermann_cmd_topic',
        default_value='/drive',
        description='Ackermann command topic name'
    )
    
    save_lut_name_arg = DeclareLaunchArgument(
        'save_LUT_name',
        default_value='NUCx_on_track_pacejka',
        description='Name for saved LUT file'
    )
    
    plot_model_arg = DeclareLaunchArgument(
        'plot_model',
        default_value='False',
        description='Whether to plot model results'
    )
    
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value='SIM',
        description='Racecar version identifier'
    )

    # On-track system identification node
    sys_id_node = Node(
        package='on_track_sys_id',
        executable='on_track_sys_id.py',
        name='on_track_sys_id',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'ackermann_cmd_topic': LaunchConfiguration('ackermann_cmd_topic'),
            'save_LUT_name': LaunchConfiguration('save_LUT_name'),
            'plot_model': LaunchConfiguration('plot_model'),
            'racecar_version': LaunchConfiguration('racecar_version'),
        }],
        output='screen'
    )

    return LaunchDescription([
        odom_topic_arg,
        ackermann_topic_arg,
        save_lut_name_arg,
        plot_model_arg,
        racecar_version_arg,
        sys_id_node,
    ])
