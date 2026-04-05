from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = LaunchConfiguration('ekf_config')
    with_bridge = LaunchConfiguration('with_bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_localization/config/ekf_wheel_imu.yaml',
        ),
        DeclareLaunchArgument(
            'with_bridge',
            default_value='true',
        ),
        Node(
            package='rasprover_localization',
            executable='slam_sensor_bridge_node',
            name='slam_sensor_bridge_node',
            output='screen',
            condition=IfCondition(with_bridge),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
