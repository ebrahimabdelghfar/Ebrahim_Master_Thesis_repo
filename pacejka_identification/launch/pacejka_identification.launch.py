import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """All settings come from config/identification_config.yaml."""

    config_file = os.path.join(
        get_package_share_directory('pacejka_identification'),
        'config',
        'identification_config.yaml'
    )

    node = Node(
        package='pacejka_identification',
        executable='identification_node',
        name='pacejka_identification_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([node])
