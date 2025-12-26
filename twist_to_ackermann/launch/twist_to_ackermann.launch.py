from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('twist_to_ackermann'),
        'config',
        'params.yaml'
        )

    return LaunchDescription([
        Node(
            package='twist_to_ackermann',
            executable='twist_to_ackermann',
            name='twist_to_ackermann',
            parameters=[config]
        )
    ])
