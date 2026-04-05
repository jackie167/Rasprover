from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    web_port = LaunchConfiguration('web_port')
    joy_topic = LaunchConfiguration('joy_topic')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('web_port', default_value='5050'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
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
            package='rasprover_mux',
            executable='command_mux_node',
            name='command_mux_node',
            output='screen',
        ),
        Node(
            package='rasprover_mux',
            executable='joystick_teleop_node',
            name='joystick_teleop_node',
            output='screen',
            parameters=[{'joy_topic': joy_topic}],
        ),
        Node(
            package='rasprover_web',
            executable='web_bridge_node',
            name='web_bridge_node',
            output='screen',
            parameters=[{'port': web_port}],
        ),
    ])
