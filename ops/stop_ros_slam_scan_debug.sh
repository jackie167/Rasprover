#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
PID_DIR="$PROJECT_DIR/.ros_slam_pids"

pid_file="$PID_DIR/slam_scan_debug_launch.pid"
if [ -f "$pid_file" ]; then
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file"
fi

pkill -f "ros2 launch rasprover_bringup slam_scan_debug.launch.py" || true
pkill -f "/rplidar_ros/lib/rplidar_ros/rplidar_node" || true
pkill -f "/tf2_ros/static_transform_publisher.*base_link.*laser" || true
pkill -f "/rasprover_control/lib/rasprover_control/joystick_bridge_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/local_joy_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/command_mux_node" || true
pkill -f "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" || true
pkill -f "/rasprover_slam/lib/rasprover_slam/simple_odom_filter_node" || true
pkill -f "/rasprover_base/lib/rasprover_base/robot_base_node" || true
if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  pkill -f "/rasprover_ui/lib/rasprover_ui/web_bridge_node" || true
fi

echo "ros slam_scan_debug stack stopped"
