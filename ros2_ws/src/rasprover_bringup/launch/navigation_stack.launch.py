from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    localization_mode = LaunchConfiguration('localization_mode')
    start_localization = LaunchConfiguration('start_localization')
    start_navigation = LaunchConfiguration('start_navigation')

    nav_params = [params_file, {'use_sim_time': use_sim_time}]
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/nav2_navigation.yaml',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'localization_mode',
            default_value='amcl',
            description='Localization source: amcl or slam.',
        ),
        DeclareLaunchArgument(
            'start_localization',
            default_value='true',
            description='Whether to start map_server/amcl.',
        ),
        DeclareLaunchArgument(
            'start_navigation',
            default_value='true',
            description='Whether to start planner/controller/behaviors/bt navigator.',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
            remappings=remappings,
            condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl' and '", start_localization, "' == 'true'"])),
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=nav_params,
            remappings=remappings,
            condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl' and '", start_localization, "' == 'true'"])),
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=nav_params,
            remappings=remappings,
            condition=IfCondition(start_navigation),
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=nav_params,
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            condition=IfCondition(start_navigation),
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=nav_params,
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            condition=IfCondition(start_navigation),
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=nav_params,
            remappings=remappings,
            condition=IfCondition(start_navigation),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': ['map_server', 'amcl']}],
            condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl' and '", start_localization, "' == 'true'"])),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator'],
                }
            ],
            condition=IfCondition(start_navigation),
        ),
        Node(
            package='rasprover_control',
            executable='nav_cmd_vel_bridge_node',
            name='nav_cmd_vel_bridge_node',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_nav',
                'output_topic': '/cv/control_intent',
                'max_linear': 0.08,
                'max_angular': 0.08,
                'deadband_angular': 0.03,
            }],
        ),
    ])
