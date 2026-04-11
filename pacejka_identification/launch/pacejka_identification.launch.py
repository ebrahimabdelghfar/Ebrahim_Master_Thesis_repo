import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    topic_arg = DeclareLaunchArgument(
        'tire_forces_topic', default_value='/carmaker/tire_forces',
        description='Topic publishing hellocm_msgs/TireForcesArray',
    )
    duration_arg = DeclareLaunchArgument(
        'duration_seconds', default_value='60',
        description='Data collection duration (seconds)',
    )
    min_vel_arg = DeclareLaunchArgument(
        'min_velocity', default_value='0.5',
        description='Min belt_velocity to accept a sample (m/s)',
    )
    method_arg = DeclareLaunchArgument(
        'method', default_value='adaptive_de_trust_region',
        description='Identification method: trust_region | differential_evolution | dual',
    )
    formulas_arg = DeclareLaunchArgument(
        'formulas',
        default_value="['lateral_fy', 'longitudinal_fx', 'self_aligning_mz']",
        description='Pacejka formulas to identify',
    )
    grouping_arg = DeclareLaunchArgument(
        'axle_grouping', default_value='per_wheel',
        description='per_wheel | per_axle | combined',
    )
    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value='',
        description='CSV output path (empty = auto-generate)',
    )
    yaml_path_arg = DeclareLaunchArgument(
        'yaml_path', default_value='',
        description='YAML output path (empty = auto-generate)',
    )

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
        parameters=[
            config_file,
            {
                'tire_forces_topic': LaunchConfiguration('tire_forces_topic'),
                'data_collection.duration_seconds': LaunchConfiguration('duration_seconds'),
                'data_collection.min_velocity': LaunchConfiguration('min_velocity'),
                'identification.method': LaunchConfiguration('method'),
                'identification.axle_grouping': LaunchConfiguration('axle_grouping'),
                'output.csv_path': LaunchConfiguration('csv_path'),
                'output.yaml_path': LaunchConfiguration('yaml_path'),
            }
        ],
    )

    return LaunchDescription([
        topic_arg,
        duration_arg,
        min_vel_arg,
        method_arg,
        formulas_arg,
        grouping_arg,
        csv_path_arg,
        yaml_path_arg,
        node,
    ])
