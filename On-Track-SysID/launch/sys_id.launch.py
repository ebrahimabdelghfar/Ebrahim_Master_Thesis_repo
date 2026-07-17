import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_tire_force_benchmark(context, *args, **kwargs):
    # racecar_version is only known at launch time (a LaunchConfiguration),
    # so the *_pacejka.txt path has to be resolved here rather than with
    # launch substitutions directly.
    #
    # Deliberately NOT passed as tire_force_benchmark's model_file: that
    # file is the *previous* identification run's C_Pf/C_Pr, and model_file
    # would make tire_force_benchmark start scoring Fy/state estimates
    # against that stale model immediately, before on_track_sys_id's first
    # identification cycle even finishes - contaminating the whole-run
    # metrics/plots with pre-identification samples benchmarked against the
    # wrong model. Only the static vehicle constants (m/I_z/l_f/l_r/l_wb -
    # unaffected by identification) are read from it and passed through;
    # c_pf/c_pr are left unset so tire_force_benchmark's own
    # have_identified_params gate keeps benchmarking held off until
    # on_track_sys_id's first accepted identification is actually forwarded
    # live via identified_params_service (requires
    # benchmark_update_params_enable:=true).
    racecar_version = LaunchConfiguration('racecar_version').perform(context)
    sysid_share = get_package_share_directory('on_track_sys_id')
    model_file = os.path.join(
        sysid_share, 'models', racecar_version, f'{racecar_version}_pacejka.txt')

    vehicle_constants = {'m': '0.0', 'I_z': '0.0', 'l_f': '0.0', 'l_r': '0.0', 'l_wb': '0.0'}
    try:
        with open(model_file, 'r') as f:
            model_data = yaml.safe_load(f)
        for key in vehicle_constants:
            if key in model_data:
                vehicle_constants[key] = str(model_data[key])
    except (OSError, yaml.YAMLError):
        pass  # tire_force_benchmark logs its own warning and disables state benchmarking

    benchmark_share = get_package_share_directory('tire_force_benchmark')

    tire_force_benchmark_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(benchmark_share, 'launch', 'tire_force_benchmark.launch.py')),
        launch_arguments={
            'odom_topic': LaunchConfiguration('odom_topic'),
            'ackermann_cmd_topic': LaunchConfiguration('ackermann_cmd_topic'),
            'identified_params_service': LaunchConfiguration('benchmark_update_params_service'),
            'csv_output_path': LaunchConfiguration('tire_force_benchmark_csv_output_path'),
            'plot_output_dir': LaunchConfiguration('tire_force_benchmark_plot_output_dir'),
            # Safe to reuse as nominal_model_file (unlike model_file above): this
            # only feeds the identified-vs-nominal comparison plot, never the
            # live benchmark model - see tire_force_benchmark_node.py's
            # _load_nominal_model_if_available.
            'nominal_model_file': model_file,
            **vehicle_constants,
        }.items())

    return [tire_force_benchmark_launch]


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

    reidentification_interval_arg = DeclareLaunchArgument(
        'reidentification_interval_s',
        default_value='30.0',
        description='Seconds between periodic re-identification cycles once RUNNING'
    )

    # Mirrors adaptive_controller_manager's benchmark_update_params_enable/service:
    # if enabled, on_track_sys_id forwards every accepted identification directly
    # to tire_force_benchmark (best-effort, fire-and-forget), independent of
    # whether adaptive_controller_manager is in the loop.
    benchmark_update_params_enable_arg = DeclareLaunchArgument(
        'benchmark_update_params_enable', default_value='false',
        description=(
            'If true, forward every accepted identification cycle to tire_force_benchmark '
            'via benchmark_update_params_service'))
    benchmark_update_params_service_arg = DeclareLaunchArgument(
        'benchmark_update_params_service', default_value='benchmark/update_params',
        description='IdentifiedParam service name tire_force_benchmark listens on')

    # Distinct from benchmark_update_params_enable above - this gates whether
    # tire_force_benchmark itself is launched alongside on_track_sys_id at all
    # (see launch_tire_force_benchmark), mirroring adaptive_stack.launch.py's
    # enable_controller_benchmark/controller_benchmark_plot_output_dir pattern
    # for adaptive_controller_benchmark.
    enable_tire_force_benchmark_arg = DeclareLaunchArgument(
        'enable_tire_force_benchmark', default_value='false',
        description=(
            'If true, also launch tire_force_benchmark alongside on_track_sys_id to '
            'benchmark the identified Pacejka model against ground truth. Off by default. '
            'Also set benchmark_update_params_enable:=true - otherwise tire_force_benchmark '
            'never receives a model (c_pf/c_pr are deliberately not pre-seeded from a stale '
            'prior model_file, see launch_tire_force_benchmark) and just waits forever.'))
    tire_force_benchmark_plot_output_dir_arg = DeclareLaunchArgument(
        'tire_force_benchmark_plot_output_dir', default_value='',
        description='If set (and enable_tire_force_benchmark:=true), directory to export '
                    'academic benchmark PNG plots to on shutdown')
    tire_force_benchmark_csv_output_path_arg = DeclareLaunchArgument(
        'tire_force_benchmark_csv_output_path', default_value='',
        description='If set (and enable_tire_force_benchmark:=true), CSV file path for '
                    'raw per-tick benchmark logging')

    tire_force_benchmark_launch_action = OpaqueFunction(
        function=launch_tire_force_benchmark,
        condition=IfCondition(LaunchConfiguration('enable_tire_force_benchmark')))

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
            'reidentification_interval_s': LaunchConfiguration('reidentification_interval_s'),
            'benchmark_update_params_enable': LaunchConfiguration('benchmark_update_params_enable'),
            'benchmark_update_params_service': LaunchConfiguration('benchmark_update_params_service'),
        }],
        # output='log' alone isn't enough: rclpy's get_logger() writes to
        # stderr (verified directly - plain shell redirection showed 0 bytes
        # on stdout, everything on stderr), and `ros2 launch`'s own console
        # aggregation still surfaces those INFO lines regardless of a
        # node's output= setting. Silencing at the source instead:
        # --log-level warn drops this node's INFO-level chatter (per-tick
        # progress bars, "waiting for car to move", etc.) before it reaches
        # any sink, so it never interleaves with pure_pursuit/
        # mpc_path_tracking/adaptive_controller_manager's own console
        # output. WARN/ERROR (real problems) still come through normally.
        # The heavier per-training-iteration summaries are separately
        # written via plain print() (see helpers/train_model.py), which IS
        # correctly routed to this node's own log file by output='log'.
        arguments=['--ros-args', '--log-level', 'warn'],
        output='log'
    )

    return LaunchDescription([
        odom_topic_arg,
        ackermann_topic_arg,
        save_lut_name_arg,
        plot_model_arg,
        racecar_version_arg,
        reidentification_interval_arg,
        benchmark_update_params_enable_arg,
        benchmark_update_params_service_arg,
        enable_tire_force_benchmark_arg,
        tire_force_benchmark_plot_output_dir_arg,
        tire_force_benchmark_csv_output_path_arg,
        sys_id_node,
        tire_force_benchmark_launch_action,
    ])
