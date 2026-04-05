from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    ekf_config = LaunchConfiguration('ekf_config')
    odom_topic = LaunchConfiguration('odom_topic')
    path_topic = LaunchConfiguration('path_topic')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/ekf_wheel_imu.yaml',
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('path_topic', default_value='/odom_path'),
        Node(
            package='rasprover_base',
            executable='robot_base_node',
            name='robot_base_node',
            output='screen',
            parameters=[{'serial_port': serial_port}],
        ),
        Node(
            package='rasprover_sensors',
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
        ),
        Node(
            package='rasprover_control',
            executable='joystick_bridge_node',
            name='joystick_bridge_node',
            output='screen',
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
            parameters=[{'odom_topic': odom_topic, 'path_topic': path_topic}],
        ),
    ])
