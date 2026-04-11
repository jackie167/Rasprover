from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from rasprover_bringup.launch_builders import ekf_node
from rasprover_bringup.launch_builders import sensor_bridge_node


def generate_launch_description():
    ekf_config = LaunchConfiguration('ekf_config')
    with_bridge = LaunchConfiguration('with_bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/ekf_wheel_imu.yaml',
        ),
        DeclareLaunchArgument(
            'with_bridge',
            default_value='true',
        ),
        sensor_bridge_node('slam', IfCondition(with_bridge)),
        ekf_node(ekf_config),
    ])
