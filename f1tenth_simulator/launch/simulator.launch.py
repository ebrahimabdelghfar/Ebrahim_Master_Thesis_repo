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
    
    # Load parameters from sim.yaml
    params_file = os.path.join(pkg_share, 'config', 'sim.yaml')
    
    # Extract map_name to determine map path
    map_name = 'YasMarina'
    waypoint_topic = '/raceline_waypoints'
    try:
        import yaml
        with open(params_file, 'r') as f:
            sim_config = yaml.safe_load(f)
            map_name = sim_config['/**']['ros__parameters'].get('map_name', 'YasMarina')
            waypoint_topic = sim_config['/**']['ros__parameters'].get('waypoint_topic', '/raceline_waypoints')
    except Exception as e:
        print(f"Warning: Could not read map_name from {params_file}, defaulting to {map_name}. Error: {e}")

    # Resolve racetracks_dir
    # Usually pkg_share is in <workspace>/install/f1tenth_simulator/share/f1tenth_simulator
    # We navigate up to find the workspace root
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    racetracks_dir = os.path.join(workspace_root, 'f1tenth_racetracks')
    
    default_map_path = os.path.join(racetracks_dir, map_name, f"{map_name}_map.yaml")
    if not os.path.exists(default_map_path):
        # Fallback to local maps if not found
        default_map_path = os.path.join(pkg_share, 'maps', 'levine.yaml')

    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the map yaml file'
    )

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

    # Track publisher node to initialize car pose and publish waypoints
    track_publisher_node = Node(
        package='f1tenth_simulator',
        executable='track_publisher.py',
        name='track_publisher',
        parameters=[{
            'map_name': map_name,
            'racetracks_dir': racetracks_dir,
            'waypoint_topic': waypoint_topic
        }],
        output='screen'
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
        track_publisher_node,
    ])
