from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    ekf_config = LaunchConfiguration('ekf_config')
    slam_params = LaunchConfiguration('slam_params')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('lidar_port', default_value=''),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('laser_x', default_value='0.04'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='3.141592653589793'),
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
            parameters=[{
                'serial_port': serial_port,
                'left_drive_scale': 1.000,
                'right_drive_scale': 0.983,
                'feedback_wheel_separation_m': 0.52,
                'feedback_wheel_yaw_scale': 2.80,
                'swap_feedback_wheels': False,
                'straight_controller_enabled': False,
                'straight_controller_forward_only': True,
                'straight_controller_linear_min': 0.10,
                'straight_controller_angular_window': 0.05,
                'straight_controller_heading_gain': 0.90,
                'straight_controller_integral_gain': 0.12,
                'straight_controller_wheel_balance_gain': 0.80,
                'straight_controller_gyro_gain': 0.20,
                'straight_controller_integral_limit': 0.30,
                'straight_controller_max_correction': 0.12,
            }],
        ),
        Node(
            package='rasprover_sensors',
            executable='slam_sensor_bridge_node',
            name='slam_sensor_bridge_node',
            output='screen',
            parameters=[{
                'publish_tf': False,
                'linear_odom_scale': 0.976,
                'left_odom_scale': 0.990,
                'right_odom_scale': 1.000,
            }],
        ),
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
                'angle_compensate': True,
                'scan_mode': 'Standard',
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
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'scan_topic': scan_topic}],
        ),
    ])
