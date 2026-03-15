from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
        description='Path to Pacejka model yaml/txt with C_Pf and C_Pr arrays',
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

    node = Node(
        package='tire_force_benchmark',
        executable='tire_force_benchmark_node',
        name='tire_force_benchmark_node',
        output='screen',
        parameters=[{
            'benchmark_mode': LaunchConfiguration('benchmark_mode'),
            'tire_forces_topic': LaunchConfiguration('tire_forces_topic'),
            'estimated_fy_topic': LaunchConfiguration('estimated_fy_topic'),
            'external_prediction_lead_samples': LaunchConfiguration('external_prediction_lead_samples'),
            'external_max_queue_size': LaunchConfiguration('external_max_queue_size'),
            'model_file': LaunchConfiguration('model_file'),
            'log_interval': LaunchConfiguration('log_interval'),
            'csv_output_path': LaunchConfiguration('csv_output_path'),
        }],
    )

    return LaunchDescription([
        benchmark_mode_arg,
        tire_forces_topic_arg,
        estimated_fy_topic_arg,
        external_prediction_lead_samples_arg,
        external_max_queue_size_arg,
        model_file_arg,
        log_interval_arg,
        csv_output_path_arg,
        node,
    ])
