from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rasprover_bringup.launch_builders import base_node
from rasprover_bringup.launch_builders import command_mux_node
from rasprover_bringup.launch_builders import joystick_bridge_node
from rasprover_bringup.launch_builders import local_joy_node
from rasprover_bringup.launch_builders import sensor_bridge_node
from rasprover_bringup.launch_builders import simple_odom_filter_node
from rasprover_bringup.launch_builders import web_bridge_node
from rasprover_bringup.runtime_config import default_lidar_port
from rasprover_bringup.runtime_config import default_serial_port


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    joy_topic = LaunchConfiguration('joy_topic')
    web_port = LaunchConfiguration('web_port')
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
    odom_filter_config = LaunchConfiguration('odom_filter_config')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=default_serial_port()),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('web_port', default_value='5050'),
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
            'odom_filter_config',
            default_value='/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/simple_odom_filter.yaml',
        ),
        base_node(serial_port),
        sensor_bridge_node('slam', extra_params={'publish_tf': False}),
        simple_odom_filter_node(odom_filter_config),
        command_mux_node(),
        local_joy_node(),
        joystick_bridge_node(joy_topic),
        web_bridge_node(web_port),
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
    ])
