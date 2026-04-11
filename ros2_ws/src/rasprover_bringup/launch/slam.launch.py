from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rasprover_bringup.launch_builders import base_node
from rasprover_bringup.launch_builders import sensor_bridge_node
from rasprover_bringup.launch_builders import slam_toolbox_node
from rasprover_bringup.runtime_config import default_lidar_port

from rasprover_bringup.runtime_config import default_serial_port


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    slam_params = LaunchConfiguration('slam_params')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=default_serial_port()),
        DeclareLaunchArgument('lidar_port', default_value=default_lidar_port()),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value=''),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('laser_x', default_value='0.04'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument(
            'slam_params',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/slam_toolbox_online_async.yaml',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        base_node(serial_port),
        sensor_bridge_node('slam'),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 460800,
                'frame_id': laser_frame,
                'inverted': False,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf',
            output='screen',
            arguments=[
                '--x', laser_x,
                '--y', laser_y,
                '--z', laser_z,
                '--roll', laser_roll,
                '--pitch', laser_pitch,
                '--yaw', laser_yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', laser_frame,
            ],
        ),
        slam_toolbox_node(slam_params, scan_topic),
    ])
