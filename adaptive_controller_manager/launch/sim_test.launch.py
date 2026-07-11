import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Test-only bringup for exercising the full adaptive-controller stack in
# simulation (enhanced_controller_plan.md's "Offline Simulation Testing").
#
# f1tenth_simulator/launch/simulator.launch.py also starts its own `mux`
# node, which by default targets /drive too - running it alongside
# adaptive_controller_manager would give /drive two independent,
# uncoordinated publishers. So this launches the simulator's components
# directly (map_server + lifecycle_manager + racecar_model + simulator +
# track_publisher.py) instead of including simulator.launch.py wholesale,
# excluding mux, behavior_controller, random_walker, keyboard (manual/
# random-walk driving nodes, not needed for autonomous-stack testing) and
# joy_node/rviz2 (harmless to skip for a lean test launch).


def generate_launch_description():
    sim_share = get_package_share_directory('f1tenth_simulator')
    manager_share = get_package_share_directory('adaptive_controller_manager')

    sim_params_file = os.path.join(sim_share, 'config', 'sim.yaml')

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='YasMarina',
        description='Racetrack name under f1tenth_racetracks/ to load')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic shared by the simulator and the adaptive stack')
    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic', default_value='/raceline_waypoints',
        description='Raceline WaypointArray topic shared by the simulator and the adaptive stack')

    # workspace_root/f1tenth_racetracks, same layout simulator.launch.py
    # assumes (pkg_share is .../install/f1tenth_simulator/share/f1tenth_simulator).
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(sim_share))))
    racetracks_dir = os.path.join(workspace_root, 'f1tenth_racetracks')

    # Resolved at launch time (map_name isn't known until then): produces
    # <racetracks_dir>/<map_name>/<map_name>_map.yaml, matching
    # simulator.launch.py's default_map_path pattern.
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            racetracks_dir,
            LaunchConfiguration('map_name'),
            [LaunchConfiguration('map_name'), '_map.yaml'],
        ]),
        description='Full path to the map yaml file (auto-derived from map_name if not set)')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False,
        }],
        output='screen')

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
        }],
        output='screen')

    racecar_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'racecar_model.launch.py')))

    simulator_node = Node(
        package='f1tenth_simulator',
        executable='simulator',
        name='f1tenth_simulator',
        parameters=[sim_params_file],
        output='screen')

    track_publisher_node = Node(
        package='f1tenth_simulator',
        executable='track_publisher.py',
        name='track_publisher',
        parameters=[{
            'map_name': LaunchConfiguration('map_name'),
            'racetracks_dir': racetracks_dir,
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
        }],
        output='screen')

    adaptive_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manager_share, 'launch', 'adaptive_stack.launch.py')),
        launch_arguments={
            'odom_topic': LaunchConfiguration('odom_topic'),
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
        }.items())

    return LaunchDescription([
        map_name_arg,
        odom_topic_arg,
        waypoint_topic_arg,
        map_arg,
        map_server_node,
        lifecycle_manager_node,
        racecar_model_launch,
        simulator_node,
        track_publisher_node,
        adaptive_stack_launch,
    ])
