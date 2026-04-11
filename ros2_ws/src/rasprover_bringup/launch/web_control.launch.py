from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from rasprover_bringup.launch_builders import base_node
from rasprover_bringup.launch_builders import command_mux_node
from rasprover_bringup.launch_builders import sensor_bridge_node
from rasprover_bringup.launch_builders import web_bridge_node
from rasprover_bringup.runtime_config import default_serial_port


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    web_port = LaunchConfiguration('web_port')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=default_serial_port()),
        DeclareLaunchArgument('web_port', default_value='5050'),
        base_node(serial_port),
        sensor_bridge_node('motion'),
        command_mux_node(),
        web_bridge_node(web_port),
    ])
