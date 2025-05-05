import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    localization_config_dir = os.path.join(
        get_package_share_directory('localization_server'), 'config'
    )
    map_server_config_dir = os.path.join(
        get_package_share_directory('map_server'), 'config'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Map YAML file to load'
    )

    use_sim_time = PythonExpression([
        '"true" if "', LaunchConfiguration('map_file'), '" == \'warehouse_map_sim.yaml\' else "false"'
    ])

    amcl_config_file = PythonExpression([
        '"', localization_config_dir, '/amcl_config_sim.yaml" if "', LaunchConfiguration('map_file'), '" == \'warehouse_map_sim.yaml\' else "', localization_config_dir, '/amcl_config_real.yaml"'
    ])

    return LaunchDescription([
        map_file_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': PathJoinSubstitution([
                    map_server_config_dir,
                    LaunchConfiguration('map_file')
                ])}
            ]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                "-d", "/home/user/ros2_ws/src/warehouse_project/localization_server/rviz/localization_map.rviz"
            ],
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        )
    ])
