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
        default_value='/tire_forces',
        description='IPG tire forces topic (hellocm_msgs/TireForcesArray)',
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
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'benchmark_mode': LaunchConfiguration('benchmark_mode'),
                'tire_forces_topic': LaunchConfiguration('tire_forces_topic'),
                'estimated_fy_topic': LaunchConfiguration('estimated_fy_topic'),
                'external_prediction_lead_samples': LaunchConfiguration('external_prediction_lead_samples'),
                'external_max_queue_size': LaunchConfiguration('external_max_queue_size'),
                'model_file': LaunchConfiguration('model_file'),
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
        log_interval_arg,
        csv_output_path_arg,
        plot_output_dir_arg,
        node,
    ])
