from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    joy_topic = LaunchConfiguration('joy_topic')
    web_port = LaunchConfiguration('web_port')
    with_local_joy = LaunchConfiguration('with_local_joy')
    with_cv = LaunchConfiguration('with_cv')
    ekf_config = LaunchConfiguration('ekf_config')
    slam_params = LaunchConfiguration('slam_params')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('web_port', default_value='5050'),
        DeclareLaunchArgument('with_local_joy', default_value='true'),
        DeclareLaunchArgument('with_cv', default_value='true'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/ekf_wheel_imu.yaml',
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/slam_toolbox_online_async.yaml',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
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
        Node(
            package='rasprover_ui',
            executable='web_bridge_node',
            name='web_bridge_node',
            output='screen',
            parameters=[{'port': web_port}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'scan_topic': scan_topic}],
        ),
        Node(
            package='rasprover_cv',
            executable='cv_node',
            name='cv_node',
            output='screen',
            condition=IfCondition(with_cv),
            parameters=[{'port': 5051}],
        ),
    ])
