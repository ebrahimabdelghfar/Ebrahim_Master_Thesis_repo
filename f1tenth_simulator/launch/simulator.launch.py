import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('f1tenth_simulator')
    
    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'levine.yaml'),
        description='Full path to the map yaml file'
    )

    # Load parameters from params.yaml
    params_file = os.path.join(pkg_share, 'params.yaml')

    # Joy node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False
        }],
        output='screen'
    )

    # Lifecycle manager to activate map server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }],
        output='screen'
    )

    # Include racecar model launch
    racecar_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'racecar_model.launch.py')
        )
    )

    # Simulator node
    simulator_node = Node(
        package='f1tenth_simulator',
        executable='simulator',
        name='f1tenth_simulator',
        parameters=[params_file],
        output='screen'
    )

    # Mux controller node
    mux_node = Node(
        package='f1tenth_simulator',
        executable='mux',
        name='mux_controller',
        parameters=[params_file],
        output='screen'
    )

    # Behavior controller node
    behavior_controller_node = Node(
        package='f1tenth_simulator',
        executable='behavior_controller',
        name='behavior_controller',
        parameters=[params_file],
        output='screen'
    )

    # Random walker node
    random_walker_node = Node(
        package='f1tenth_simulator',
        executable='random_walk',
        name='random_walker',
        parameters=[params_file],
        output='screen'
    )

    # Keyboard node - launch in xterm to capture input
    keyboard_node = Node(
        package='f1tenth_simulator',
        executable='keyboard',
        name='keyboard',
        parameters=[params_file],
        output='screen',
        prefix=['xterm -e']
    )

    # RViz2 node
    rviz_config_file = os.path.join(pkg_share, 'launch', 'simulator.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        joy_node,
        map_server_node,
        lifecycle_manager_node,
        racecar_model_launch,
        simulator_node,
        mux_node,
        behavior_controller_node,
        random_walker_node,
        keyboard_node,
        rviz_node,
    ])
