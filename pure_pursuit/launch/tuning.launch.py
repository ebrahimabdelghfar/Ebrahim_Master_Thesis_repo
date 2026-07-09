import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Custom PyQt GUI for tuning parameters and graphing
    tuning_gui_node = Node(
        package='pure_pursuit',
        executable='tuning_gui.py',
        name='pure_pursuit_tuning_gui',
        output='screen'
    )

    return LaunchDescription([
        tuning_gui_node
    ])
