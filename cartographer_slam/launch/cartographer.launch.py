import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    cart_config_path = os.path.join(
        get_package_share_directory("cartographer_slam"),
        "config"
    )

    # Declare the 'use_sim_time' launch argument
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true"
    )

    # Node when using SIMULATION (cartographer_sim.lua)
    cartographer_node_sim = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node_sim",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory", cart_config_path,
            "-configuration_basename", "cartographer_sim.lua"
        ],
        condition=IfCondition(LaunchConfiguration("use_sim_time"))
    )

    # Node when using REAL WORLD (cartographer_real.lua)
    cartographer_node_real = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node_real",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory", cart_config_path,
            "-configuration_basename", "cartographer_real.lua"
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim_time"))
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-resolution", "0.05",
            "-publish_period_sec", "1.0"
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "/home/user/ros2_ws/src/warehouse_project/cartographer_slam/rviz/mapping.rviz"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node_sim,
        cartographer_node_real,
        occupancy_grid_node,
        rviz_node
    ])
