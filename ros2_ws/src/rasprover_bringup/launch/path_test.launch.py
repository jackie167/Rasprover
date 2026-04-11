from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from rasprover_bringup.launch_builders import base_node
from rasprover_bringup.launch_builders import command_mux_node
from rasprover_bringup.launch_builders import ekf_node
from rasprover_bringup.launch_builders import joystick_bridge_node
from rasprover_bringup.launch_builders import local_joy_node
from rasprover_bringup.launch_builders import odometry_path_node
from rasprover_bringup.launch_builders import sensor_bridge_node
from rasprover_bringup.launch_builders import web_bridge_node
from rasprover_bringup.runtime_config import default_serial_port


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
        DeclareLaunchArgument('serial_port', default_value=default_serial_port()),
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
        base_node(serial_port),
        sensor_bridge_node('slam'),
        command_mux_node(),
        local_joy_node(),
        joystick_bridge_node(joy_topic),
        web_bridge_node(web_port, IfCondition(with_web)),
        ekf_node(ekf_config),
        odometry_path_node('odometry_path_node', odom_topic, path_topic),
        odometry_path_node('wheel_odometry_path_node', '/wheel/odometry', wheel_path_topic),
    ])
