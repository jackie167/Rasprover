from launch.conditions import IfCondition
from launch_ros.actions import Node

from .runtime_config import nav_cmd_vel_bridge_params
from .runtime_config import robot_base_params
from .runtime_config import sensor_bridge_params


def base_node(serial_port):
    return Node(
        package='rasprover_base',
        executable='robot_base_node',
        name='robot_base_node',
        output='screen',
        parameters=[
            robot_base_params(),
            {'serial_port': serial_port},
        ],
    )


def sensor_bridge_node(profile, condition=None, extra_params=None):
    parameters = [sensor_bridge_params(profile)]
    if extra_params:
        parameters.append(extra_params)
    return Node(
        package='rasprover_sensors',
        executable='slam_sensor_bridge_node',
        name='slam_sensor_bridge_node',
        output='screen',
        parameters=parameters,
        condition=condition,
    )


def command_mux_node():
    return Node(
        package='rasprover_control',
        executable='command_mux_node',
        name='command_mux_node',
        output='screen',
    )


def local_joy_node(enabled=None):
    kwargs = {}
    if enabled is not None:
        kwargs['condition'] = IfCondition(enabled)
    return Node(
        package='rasprover_control',
        executable='local_joy_node',
        name='local_joy_node',
        output='screen',
        **kwargs,
    )


def joystick_bridge_node(joy_topic):
    return Node(
        package='rasprover_control',
        executable='joystick_bridge_node',
        name='joystick_bridge_node',
        output='screen',
        parameters=[{'joy_topic': joy_topic}],
    )


def web_bridge_node(web_port, condition=None):
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    return Node(
        package='rasprover_ui',
        executable='web_bridge_node',
        name='web_bridge_node',
        output='screen',
        parameters=[{'port': web_port}],
        **kwargs,
    )


def cv_node(enabled):
    return Node(
        package='rasprover_cv',
        executable='cv_node',
        name='cv_node',
        output='screen',
        condition=IfCondition(enabled),
        parameters=[{'port': 5051}],
    )


def ekf_node(ekf_config):
    return Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )


def slam_toolbox_node(slam_params, scan_topic):
    return Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'scan_topic': scan_topic}],
    )


def simple_odom_filter_node(config_path):
    return Node(
        package='rasprover_slam',
        executable='simple_odom_filter_node',
        name='simple_odom_filter_node',
        output='screen',
        parameters=[config_path],
    )


def nav_cmd_vel_bridge_node():
    return Node(
        package='rasprover_control',
        executable='nav_cmd_vel_bridge_node',
        name='nav_cmd_vel_bridge_node',
        output='screen',
        parameters=[nav_cmd_vel_bridge_params()],
    )


def odometry_path_node(name, odom_topic, path_topic):
    return Node(
        package='rasprover_slam',
        executable='odometry_path_node',
        name=name,
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'path_topic': path_topic,
            'min_translation': 0.005,
            'min_rotation': 0.01,
        }],
    )
