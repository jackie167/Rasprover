#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
APP_CONFIG_PATH = REPO_ROOT / "config.yaml"
LEDGER_PATH = REPO_ROOT / "docs" / "configuration" / "robot_parameter_ledger.yaml"
RUNTIME_CONFIG_PATH = (
    REPO_ROOT / "ros2_ws" / "src" / "rasprover_bringup" / "config" / "robot_runtime.yaml"
)
BASELINE_CONFIG_PATH = (
    REPO_ROOT / "ros2_ws" / "src" / "rasprover_bringup" / "config" / "robot_baseline.yaml"
)
LATEST_RECOMMENDATION_PATH = REPO_ROOT / "tools" / "calibration" / "latest_recommendation.yaml"


def load_yaml(path: Path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def main():
    app_config = load_yaml(APP_CONFIG_PATH)
    ledger = load_yaml(LEDGER_PATH)
    baseline_config = load_yaml(BASELINE_CONFIG_PATH)
    runtime_config = load_yaml(RUNTIME_CONFIG_PATH)

    robot_identity = ledger.get("robot_identity", {})
    parameter_ledger = ledger.get("parameter_ledger", [])

    print("== robot identity ==")
    print(f"profile_name: {robot_identity.get('profile_name', 'unknown')}")
    print(f"robot_name: {robot_identity.get('robot_name', 'unknown')}")
    print(f"main_type: {app_config.get('base_config', {}).get('main_type')}")
    print(f"module_type: {app_config.get('base_config', {}).get('module_type')}")
    print()

    print("== canonical config sources ==")
    for key, value in robot_identity.get("authoritative_runtime_sources", {}).items():
        print(f"{key}: {value}")
    print()

    print("== active runtime profile ==")
    print(f"profile_name: {runtime_config.get('profile_name', 'unknown')}")
    print(f"serial_port_default: {runtime_config.get('serial_port_default', 'unknown')}")
    print()

    print("== baseline profile ==")
    print(f"profile_name: {baseline_config.get('profile_name', 'unknown')}")
    print(f"serial_port_default: {baseline_config.get('serial_port_default', 'unknown')}")
    print()

    print("== latest recommendation ==")
    if LATEST_RECOMMENDATION_PATH.exists():
        recommendation = load_yaml(LATEST_RECOMMENDATION_PATH)
        print(f"path: {LATEST_RECOMMENDATION_PATH}")
        print(f"type: {recommendation.get('recommendation_type', 'unknown')}")
        print(f"status: {recommendation.get('status', 'unknown')}")
        print(f"source_csv: {recommendation.get('source_csv', 'unknown')}")
    else:
        print("path: none")
        print("status: no recommendation generated yet")
    print()

    print("== hardware-sensitive parameters ==")
    for item in parameter_ledger:
        name = item.get("name", "unknown")
        current_value = item.get("current_value")
        source = item.get("source_of_truth")
        print(f"{name}: {current_value}")
        print(f"  source: {source}")

    print("")
    print("== baseline vs runtime highlights ==")
    comparisons = [
        ("robot_base_node.right_drive_scale", "right_drive_scale"),
        ("robot_base_node.feedback_wheel_yaw_scale", "feedback_wheel_yaw_scale"),
        ("slam_sensor_bridge_node.slam.linear_odom_scale", "slam.linear_odom_scale"),
        ("slam_sensor_bridge_node.slam.left_odom_scale", "slam.left_odom_scale"),
        ("slam_sensor_bridge_node.motion.wheel_yaw_scale", "motion.wheel_yaw_scale"),
    ]
    for dotted_path, label in comparisons:
        baseline_value = baseline_config
        runtime_value = runtime_config
        for part in dotted_path.split("."):
            baseline_value = baseline_value.get(part, {}) if isinstance(baseline_value, dict) else {}
            runtime_value = runtime_value.get(part, {}) if isinstance(runtime_value, dict) else {}
        print(f"{label}: baseline={baseline_value} runtime={runtime_value}")


if __name__ == "__main__":
    main()
