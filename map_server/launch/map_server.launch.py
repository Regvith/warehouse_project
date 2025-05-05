import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_map_path = os.path.join(
        get_package_share_directory("map_server"),
        "config"
    )

    # Declare 'map_file' launch argument
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="warehouse_map_sim.yaml",
        description="Map YAML file to load"
    )

    use_sim_time = PythonExpression([
        '"true" if "', LaunchConfiguration('map_file'), '" == "warehouse_map_sim.yaml" else "false"'
    ])

    # Create nodes
    map_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": PathJoinSubstitution([
                config_map_path,
                LaunchConfiguration("map_file")
            ])}
        ]
    )

    life_cycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="manager_mapper_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server_node"]}
        ]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "/home/user/ros2_ws/src/warehouse_project/map_server/rviz/map_display.rviz"],
        parameters=[
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([
        map_file_arg,
        map_node,
        life_cycle_node,
        rviz2_node
    ])
