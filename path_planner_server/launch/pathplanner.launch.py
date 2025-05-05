import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    path_planner_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')

    # Declare 'use_sim_time'
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time if True'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Dynamic config files
    controller_yaml = PythonExpression([
        '"', path_planner_config_dir, '/controller_sim.yaml" if ', use_sim_time, ' == "True" else "', path_planner_config_dir, '/controller_real.yaml"'
    ])
    planner_yaml = PythonExpression([
        '"', path_planner_config_dir, '/planner_sim.yaml" if ', use_sim_time, ' == "True" else "', path_planner_config_dir, '/planner_real.yaml"'
    ])
    bt_navigator_yaml = PythonExpression([
        '"', path_planner_config_dir, '/bt_navigator_sim.yaml" if ', use_sim_time, ' == "True" else "', path_planner_config_dir, '/bt_navigator_real.yaml"'
    ])
    recoveries_yaml = PythonExpression([
        '"', path_planner_config_dir, '/recoveries_sim.yaml" if ', use_sim_time, ' == "True" else "', path_planner_config_dir, '/recoveries_real.yaml"'
    ])

    return LaunchDescription([
        use_sim_time_arg,

        # Controller server with remap (only for simulation)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            condition=IfCondition(use_sim_time),
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
            ]
        ),

        # Controller server without remap (only for real robot)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            condition=UnlessCondition(use_sim_time),
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}]
            # No remappings
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}]
        ),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[recoveries_yaml, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time}]
        ),

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
