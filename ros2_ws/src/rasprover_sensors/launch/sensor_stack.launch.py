from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    scan_topic = LaunchConfiguration('scan_topic')
    laser_frame = LaunchConfiguration('laser_frame')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        Node(
            package='rasprover_sensors',
            executable='lidar_scan_node',
            name='lidar_scan_node',
            output='screen',
            parameters=[{
                'port': lidar_port,
                'scan_topic': scan_topic,
                'frame_id': laser_frame,
            }],
        ),
    ])
