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
            parameters=[{
                'serial_port': serial_port,
                'left_drive_scale': 1.000,
                'right_drive_scale': 0.983,
                'feedback_wheel_separation_m': 0.52,
                'feedback_wheel_yaw_scale': 2.80,
                'swap_feedback_wheels': False,
                'straight_controller_enabled': False,
                'straight_controller_forward_only': True,
                'straight_controller_linear_min': 0.10,
                'straight_controller_angular_window': 0.05,
                'straight_controller_heading_gain': 0.90,
                'straight_controller_integral_gain': 0.12,
                'straight_controller_wheel_balance_gain': 0.80,
                'straight_controller_gyro_gain': 0.20,
                'straight_controller_integral_limit': 0.30,
                'straight_controller_max_correction': 0.12,
            }],
        ),
        Node(
            package='rasprover_sensors',
            executable='slam_sensor_bridge_node',
            name='slam_sensor_bridge_node',
            output='screen',
            parameters=[{
                'wheel_separation_m': 0.52,
                'wheel_yaw_scale': 2.80,
                'left_odom_scale': 1.000,
                'right_odom_scale': 1.000,
            }],
        ),
        Node(
            package='rasprover_control',
            executable='command_mux_node',
            name='command_mux_node',
            output='screen',
        ),
        Node(
            package='rasprover_control',
            executable='joystick_bridge_node',
            name='joystick_bridge_node',
            output='screen',
            parameters=[{'joy_topic': joy_topic}],
        ),
        Node(
            package='rasprover_ui',
            executable='web_bridge_node',
            name='web_bridge_node',
            output='screen',
            parameters=[{'port': web_port}],
        ),
    ])
