import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manager_share = get_package_share_directory('adaptive_controller_manager')
    pp_share = get_package_share_directory('pure_pursuit')
    mpc_share = get_package_share_directory('mpc_path_tracking')
    sysid_share = get_package_share_directory('on_track_sys_id')

    manager_config = os.path.join(manager_share, 'config', 'adaptive_controller_manager.yaml')
    pp_config = os.path.join(pp_share, 'config', 'pure_pursuit.yaml')
    mpc_config = os.path.join(mpc_share, 'config', 'mpc_path_tracking.yaml')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic shared by pure_pursuit, mpc_path_tracking and the manager')
    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic', default_value='/raceline_waypoints',
        description='Raceline WaypointArray topic shared by all three nodes')
    benchmark_update_params_enable_arg = DeclareLaunchArgument(
        'benchmark_update_params_enable', default_value='false',
        description=(
            'If true, forward every accepted sysid/update_params submission to '
            'tire_force_benchmark via benchmark_update_params_service (best-effort, '
            'independent of the PP/MPC arming FSM)'))
    benchmark_update_params_service_arg = DeclareLaunchArgument(
        'benchmark_update_params_service', default_value='benchmark/update_params',
        description='IdentifiedParam service name tire_force_benchmark listens on')

    # standalone_mode:=false is passed explicitly (not left to each package's
    # own YAML default) so a leftover standalone_mode:true from a solo-tuning
    # run never silently survives into a managed launch and defeats the
    # arbiter.
    #
    # config_file and drive_topic are ALSO passed explicitly here (each
    # package's own path/value, not left to that package's launch file's own
    # DeclareLaunchArgument default) - caught via simulation testing: both
    # pure_pursuit.launch.py and mpc_path_tracking.launch.py declare
    # identically-named 'config_file'/'drive_topic' launch arguments, and
    # ROS2 launch's DeclareLaunchArgument is idempotent (first declaration in
    # the shared launch context wins) - so without this, mpc_path_tracking's
    # own declaration silently became a no-op and it inherited
    # pure_pursuit's 'pp/drive_cmd' and pure_pursuit.yaml's config_file
    # instead of its own, since pure_pursuit's include runs first.
    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pp_share, 'launch', 'pure_pursuit.launch.py')),
        launch_arguments={
            'odom_topic': LaunchConfiguration('odom_topic'),
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            'config_file': pp_config,
            'drive_topic': 'pp/drive_cmd',
            'standalone_mode': 'false',
        }.items())

    mpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mpc_share, 'launch', 'mpc_path_tracking.launch.py')),
        launch_arguments={
            'odom_topic': LaunchConfiguration('odom_topic'),
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            'config_file': mpc_config,
            'drive_topic': 'mpc/drive_cmd',
            'standalone_mode': 'false',
        }.items())

    # on_track_sys_id observes /odom and /drive (the manager's arbitrated
    # output) regardless of which controller is currently active - it
    # doesn't need Start_Working_*/standalone_mode gating like the two
    # controllers do, so this include only threads odom_topic through
    # (ackermann_cmd_topic already defaults to '/drive', matching the
    # manager's sole-writer output topic).
    sys_id_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sysid_share, 'launch', 'sys_id.launch.py')),
        launch_arguments={
            'odom_topic': LaunchConfiguration('odom_topic'),
        }.items())

    manager_node = Node(
        package='adaptive_controller_manager',
        executable='adaptive_controller_manager_node',
        name='adaptive_controller_manager',
        parameters=[
            manager_config,
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                'benchmark_update_params_enable': LaunchConfiguration('benchmark_update_params_enable'),
                'benchmark_update_params_service': LaunchConfiguration('benchmark_update_params_service'),
            },
        ],
        output='screen')

    return LaunchDescription([
        odom_topic_arg,
        waypoint_topic_arg,
        benchmark_update_params_enable_arg,
        benchmark_update_params_service_arg,
        pure_pursuit_launch,
        mpc_launch,
        # sys_id_launch,
        manager_node,
    ])
