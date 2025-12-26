import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('f1tenth_simulator')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_share, 'racecar.xacro')
    
    # Declare launch arguments
    racecar_xacro_arg = DeclareLaunchArgument(
        'racecar_xacro',
        default_value=xacro_file,
        description='Full path to the racecar xacro file'
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='racecar',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('racecar_xacro')])
        }],
        output='screen'
    )

    return LaunchDescription([
        racecar_xacro_arg,
        robot_state_publisher_node,
    ])
