#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"

for name in path_base path_bridge path_mux path_joy path_joystick path_ekf path_path path_wheel_path; do
  pid_file="$PID_DIR/$name.pid"
  if [ -f "$pid_file" ]; then
    pid="$(cat "$pid_file")"
    kill "$pid" 2>/dev/null || true
    rm -f "$pid_file"
  fi
done

if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  pid_file="$PID_DIR/path_web.pid"
  if [ -f "$pid_file" ]; then
    pid="$(cat "$pid_file")"
    kill "$pid" 2>/dev/null || true
    rm -f "$pid_file"
  fi
fi

sleep 1
pkill -f "/rasprover_base/lib/rasprover_base/robot_base_node" || true
pkill -f "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/command_mux_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/local_joy_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/joystick_bridge_node" || true
if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  pkill -f "/rasprover_ui/lib/rasprover_ui/web_bridge_node" || true
fi
pkill -f "/robot_localization/lib/robot_localization/ekf_node" || true
pkill -f "/rasprover_slam/lib/rasprover_slam/odometry_path_node" || true
rm -f "$PROJECT_DIR/runtime_logs"/ros_path_*.log

echo "ros path test stack stopped"
