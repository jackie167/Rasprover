from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/frontier_explorer.yaml',
        ),
        Node(
            package='rasprover_slam',
            executable='frontier_explorer_node',
            name='frontier_explorer_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
