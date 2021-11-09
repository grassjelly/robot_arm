from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("pick_n_place_server"), "config", "config.yaml"]
    )

    default_map = PathJoinSubstitution(
        [FindPackageShare("linorobot2_navigation"), "maps", "playground.yaml"]
    )

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('sim')
    lifecycle_nodes = ['map_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=config_path,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Path to map file'),

        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Use sim clock'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
    ])