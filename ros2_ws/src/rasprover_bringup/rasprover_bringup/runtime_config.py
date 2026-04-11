from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def runtime_config_path() -> Path:
    try:
        return Path(get_package_share_directory("rasprover_bringup")) / "config" / "robot_runtime.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / "robot_runtime.yaml"


def load_runtime_config() -> dict:
    with open(runtime_config_path(), "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data


def default_serial_port() -> str:
    return str(load_runtime_config().get("serial_port_default", "/dev/ttyAMA0"))


def default_lidar_port() -> str:
    return str(load_runtime_config().get("lidar_port_default", "/dev/ttyUSB0"))


def robot_base_params() -> dict:
    return deepcopy(load_runtime_config().get("robot_base_node", {}))


def sensor_bridge_params(profile: str) -> dict:
    profiles = load_runtime_config().get("slam_sensor_bridge_node", {})
    if profile not in profiles:
        raise KeyError(f"Unknown slam_sensor_bridge_node profile: {profile}")
    return deepcopy(profiles[profile])


def nav_cmd_vel_bridge_params() -> dict:
    return deepcopy(load_runtime_config().get("nav_cmd_vel_bridge_node", {}))
