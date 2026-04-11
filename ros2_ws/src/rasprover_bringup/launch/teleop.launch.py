from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from rasprover_bringup.launch_builders import base_node
from rasprover_bringup.launch_builders import command_mux_node
from rasprover_bringup.launch_builders import joystick_bridge_node
from rasprover_bringup.launch_builders import local_joy_node
from rasprover_bringup.launch_builders import sensor_bridge_node
from rasprover_bringup.runtime_config import default_serial_port


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    joy_topic = LaunchConfiguration('joy_topic')
    with_local_joy = LaunchConfiguration('with_local_joy')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=default_serial_port()),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('with_local_joy', default_value='true'),
        base_node(serial_port),
        sensor_bridge_node('motion'),
        command_mux_node(),
        local_joy_node(with_local_joy),
        joystick_bridge_node(joy_topic),
    ])
