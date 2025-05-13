import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    path_planner_config_dir = os.path.join(
        get_package_share_directory('path_planner_server'), 'config')

    # Declare 'use_sim_time'
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time if True'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Predefine YAML paths
    controller_sim_yaml = os.path.join(path_planner_config_dir, 'controller_sim.yaml')
    controller_real_yaml = os.path.join(path_planner_config_dir, 'controller_real.yaml')
    planner_sim_yaml = os.path.join(path_planner_config_dir, 'planner_sim.yaml')
    planner_real_yaml = os.path.join(path_planner_config_dir, 'planner_real.yaml')
    bt_navigator_sim_yaml = os.path.join(path_planner_config_dir, 'bt_navigator_sim.yaml')
    bt_navigator_real_yaml = os.path.join(path_planner_config_dir, 'bt_navigator_real.yaml')
    recoveries_sim_yaml = os.path.join(path_planner_config_dir, 'recoveries_sim.yaml')
    recoveries_real_yaml = os.path.join(path_planner_config_dir, 'recoveries_real.yaml')

    return LaunchDescription([
        use_sim_time_arg,

        # Controller Server (Simulation)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            condition=IfCondition(use_sim_time),
            parameters=[controller_sim_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
        ),

        # Controller Server (Real Robot)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            condition=UnlessCondition(use_sim_time),
            parameters=[controller_real_yaml, {'use_sim_time': use_sim_time}]
            # No remappings
        ),

        # Planner Server (Simulation)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            condition=IfCondition(use_sim_time),
            parameters=[planner_sim_yaml, {'use_sim_time': use_sim_time}]
        ),

        # Planner Server (Real Robot)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            condition=UnlessCondition(use_sim_time),
            parameters=[planner_real_yaml, {'use_sim_time': use_sim_time}]
        ),

        # Behavior Server (Simulation)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            condition=IfCondition(use_sim_time),
            parameters=[recoveries_sim_yaml, {'use_sim_time': use_sim_time}]
        ),

        # Behavior Server (Real Robot)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            condition=UnlessCondition(use_sim_time),
            parameters=[recoveries_real_yaml, {'use_sim_time': use_sim_time}]
        ),

        # BT Navigator (Simulation)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            condition=IfCondition(use_sim_time),
            parameters=[bt_navigator_sim_yaml, {'use_sim_time': use_sim_time}]
        ),

        # BT Navigator (Real Robot)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            condition=UnlessCondition(use_sim_time),
            parameters=[bt_navigator_real_yaml, {'use_sim_time': use_sim_time}]
        ),

        # RViz (always launched)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_node",
            arguments=["-d", "/home/user/ros2_ws/src/warehouse_project/path_planner_server/rviz/pathplanning.rviz"],
            output="screen"
        ),

        # Lifecycle Manager (always launched)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{
                'autostart': True,
                'use_sim_time': use_sim_time
            }, {
                'node_names': ['planner_server',
                               'controller_server',
                               'recoveries_server',
                               'bt_navigator']
            }]
        )
    ])
