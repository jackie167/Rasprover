from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = LaunchConfiguration('ekf_config')
    slam_params = LaunchConfiguration('slam_params')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_localization/config/ekf_wheel_imu.yaml',
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_localization/config/slam_toolbox_online_async.yaml',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
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
            parameters=[
                slam_params,
                {
                    'scan_topic': scan_topic,
                },
            ],
        ),
    ])
