from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    joy_topic = LaunchConfiguration('joy_topic')
    with_local_joy = LaunchConfiguration('with_local_joy')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('with_local_joy', default_value='true'),
        Node(
            package='rasprover_base',
            executable='robot_base_node',
            name='robot_base_node',
            output='screen',
            parameters=[{'serial_port': serial_port}],
        ),
        Node(
            package='rasprover_localization',
            executable='slam_sensor_bridge_node',
            name='slam_sensor_bridge_node',
            output='screen',
        ),
        Node(
            package='rasprover_control',
            executable='command_mux_node',
            name='command_mux_node',
            output='screen',
        ),
        Node(
            package='rasprover_control',
            executable='local_joy_node',
            name='local_joy_node',
            output='screen',
            condition=IfCondition(with_local_joy),
        ),
        Node(
            package='rasprover_control',
            executable='joystick_bridge_node',
            name='joystick_bridge_node',
            output='screen',
            parameters=[{'joy_topic': joy_topic}],
        ),
    ])
