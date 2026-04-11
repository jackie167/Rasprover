from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from rasprover_bringup.launch_builders import ekf_node
from rasprover_bringup.launch_builders import slam_toolbox_node


def generate_launch_description():
    ekf_config = LaunchConfiguration('ekf_config')
    slam_params = LaunchConfiguration('slam_params')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/ekf_wheel_imu.yaml',
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/slam_toolbox_online_async.yaml',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
        ),
        ekf_node(ekf_config),
        slam_toolbox_node(slam_params, scan_topic),
    ])
