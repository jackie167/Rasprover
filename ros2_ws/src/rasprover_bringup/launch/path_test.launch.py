from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    joy_topic = LaunchConfiguration('joy_topic')
    ekf_config = LaunchConfiguration('ekf_config')
    odom_topic = LaunchConfiguration('odom_topic')
    path_topic = LaunchConfiguration('path_topic')
    wheel_path_topic = LaunchConfiguration('wheel_path_topic')
    web_port = LaunchConfiguration('web_port')
    with_web = LaunchConfiguration('with_web')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('web_port', default_value='5050'),
        DeclareLaunchArgument('with_web', default_value='false'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/ekf_wheel_imu.yaml',
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('path_topic', default_value='/odom_path'),
        DeclareLaunchArgument('wheel_path_topic', default_value='/wheel_odom_path'),
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
                'left_odom_scale': 0.990,
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
            executable='local_joy_node',
            name='local_joy_node',
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
            condition=IfCondition(with_web),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
        Node(
            package='rasprover_slam',
            executable='odometry_path_node',
            name='odometry_path_node',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'path_topic': path_topic,
                'min_translation': 0.005,
                'min_rotation': 0.01,
            }],
        ),
        Node(
            package='rasprover_slam',
            executable='odometry_path_node',
            name='wheel_odometry_path_node',
            output='screen',
            parameters=[{
                'odom_topic': '/wheel/odometry',
                'path_topic': wheel_path_topic,
                'min_translation': 0.005,
                'min_rotation': 0.01,
            }],
        ),
    ])
