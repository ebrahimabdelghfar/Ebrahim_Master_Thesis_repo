import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('tire_force_benchmark'),
        'config',
        'benchmark_config.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description=(
            'Path to a ROS2 parameters YAML (node key: tire_force_benchmark_node) holding all '
            'benchmark parameters, including the required min_fz_threshold. Defaults to the '
            "package's own config/benchmark_config.yaml."
        ),
    )

    # Convenience CLI overrides for the most commonly-tweaked parameters, applied on top of
    # config_file (their defaults mirror the shipped config, so omitting them is a no-op).
    # Everything else (min_fz_threshold, state-benchmarking topics/model constants,
    # plot_max_points, ...) is configured via config_file - edit it directly, or pass extra
    # `-p key:=value` to `ros2 run` if you need a one-off override outside this launch file.
    benchmark_mode_arg = DeclareLaunchArgument(
        'benchmark_mode',
        default_value='internal_pacejka',
        description='Benchmark mode: internal_pacejka or external_topic',
    )
    tire_forces_topic_arg = DeclareLaunchArgument(
        'tire_forces_topic',
        default_value='/sim/feedback/tire_forces',
        description='Ground-truth per-wheel tire telemetry topic (sim_manager_msgs/TireForces)',
    )
    estimated_fy_topic_arg = DeclareLaunchArgument(
        'estimated_fy_topic',
        default_value='/estimated_tire_force_fy',
        description='External estimated Fy topic (Float64MultiArray: [FL, FR, RL, RR])',
    )
    external_prediction_lead_samples_arg = DeclareLaunchArgument(
        'external_prediction_lead_samples',
        default_value='1',
        description='In external_topic mode, compare estimate[k] with ground truth[k+lead] by sample index',
    )
    external_max_queue_size_arg = DeclareLaunchArgument(
        'external_max_queue_size',
        default_value='2000',
        description='Maximum buffered samples for robust external queue alignment',
    )
    model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value='',
        description='Path to a vehicle model yaml/txt with C_Pf/C_Pr (+ optionally m/I_z/l_f/l_r/l_wb)',
    )
    identified_params_service_arg = DeclareLaunchArgument(
        'identified_params_service',
        default_value='benchmark/update_params',
        description=(
            'IdentifiedParam service name this node listens on for live c_pf/c_pr updates - '
            'must match the identification source\'s forwarding service (e.g. '
            'adaptive_controller_manager\'s or on_track_sys_id\'s benchmark_update_params_service).'
        ),
    )
    # Vehicle constants as standalone overrides (independent of model_file) - lets a
    # caller provide the static m/I_z/l_f/l_r/l_wb needed for state benchmarking
    # up front, while deliberately leaving c_pf/c_pr unset so internal_pacejka/state
    # benchmarking only begins once a live model arrives via
    # identified_params_service, instead of scoring against a stale model_file
    # snapshot from before the current identification run.
    m_arg = DeclareLaunchArgument('m', default_value='0.0', description='Vehicle mass [kg]')
    i_z_arg = DeclareLaunchArgument('I_z', default_value='0.0', description='Yaw moment of inertia [kg.m^2]')
    l_f_arg = DeclareLaunchArgument('l_f', default_value='0.0', description='CoG-to-front-axle distance [m]')
    l_r_arg = DeclareLaunchArgument('l_r', default_value='0.0', description='CoG-to-rear-axle distance [m]')
    l_wb_arg = DeclareLaunchArgument('l_wb', default_value='0.0', description='Wheelbase [m]')
    nominal_model_file_arg = DeclareLaunchArgument(
        'nominal_model_file',
        default_value='',
        description=(
            'Path to a reference/prior vehicle model yaml/txt with C_Pf/C_Pr, used only for '
            'the pacejka_identified_vs_nominal.png comparison plot - never seeds the live '
            'benchmark model (unlike model_file above).'
        ),
    )
    log_interval_arg = DeclareLaunchArgument(
        'log_interval',
        default_value='200',
        description='How many valid samples between summary logs',
    )
    csv_output_path_arg = DeclareLaunchArgument(
        'csv_output_path',
        default_value='',
        description='Optional CSV output file path',
    )
    plot_output_dir_arg = DeclareLaunchArgument(
        'plot_output_dir',
        default_value='',
        description='If set, directory to export academic benchmark PNG plots to on shutdown',
    )

    node = Node(
        package='tire_force_benchmark',
        executable='tire_force_benchmark_node',
        name='tire_force_benchmark_node',
        output='screen',
        # Plot export runs on shutdown and takes ~10-15 s. launch's defaults
        # (5 s to SIGTERM, 10 s more to SIGKILL) killed the process mid-export,
        # so the last figures were never written.
        sigterm_timeout='90.0',
        sigkill_timeout='90.0',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'benchmark_mode': LaunchConfiguration('benchmark_mode'),
                'tire_forces_topic': LaunchConfiguration('tire_forces_topic'),
                'estimated_fy_topic': LaunchConfiguration('estimated_fy_topic'),
                'external_prediction_lead_samples': LaunchConfiguration('external_prediction_lead_samples'),
                'external_max_queue_size': LaunchConfiguration('external_max_queue_size'),
                'model_file': LaunchConfiguration('model_file'),
                'identified_params_service': LaunchConfiguration('identified_params_service'),
                'm': LaunchConfiguration('m'),
                'I_z': LaunchConfiguration('I_z'),
                'l_f': LaunchConfiguration('l_f'),
                'l_r': LaunchConfiguration('l_r'),
                'l_wb': LaunchConfiguration('l_wb'),
                'nominal_model_file': LaunchConfiguration('nominal_model_file'),
                'log_interval': LaunchConfiguration('log_interval'),
                'csv_output_path': LaunchConfiguration('csv_output_path'),
                'plot_output_dir': LaunchConfiguration('plot_output_dir'),
            },
        ],
    )

    return LaunchDescription([
        config_file_arg,
        benchmark_mode_arg,
        tire_forces_topic_arg,
        estimated_fy_topic_arg,
        external_prediction_lead_samples_arg,
        external_max_queue_size_arg,
        model_file_arg,
        identified_params_service_arg,
        m_arg,
        i_z_arg,
        l_f_arg,
        l_r_arg,
        l_wb_arg,
        nominal_model_file_arg,
        log_interval_arg,
        csv_output_path_arg,
        plot_output_dir_arg,
        node,
    ])
