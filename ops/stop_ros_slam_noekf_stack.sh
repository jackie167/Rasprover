#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
PID_DIR="$PROJECT_DIR/.ros_slam_pids"

stop_pid_file() {
  local pid_file="$1"
  [ -f "$pid_file" ] || return 0
  local pid
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file"
}

stop_pid_file "$PID_DIR/slam_noekf_launch.pid"

pkill -f "launch rasprover_bringup slam_noekf.launch.py" || true
pkill -f "/slam_toolbox/lib/slam_toolbox/async_slam_toolbox_node" || true
pkill -f "/tf2_ros/static_transform_publisher.*base_link" || true
pkill -f "/rplidar_ros/lib/rplidar_ros/rplidar_node" || true
pkill -f "/rasprover_ui/lib/rasprover_ui/web_bridge_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/local_joy_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/joystick_bridge_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/command_mux_node" || true
pkill -f "/rasprover_slam/lib/rasprover_slam/simple_odom_filter_node" || true
pkill -f "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" || true
pkill -f "/rasprover_base/lib/rasprover_base/robot_base_node" || true

echo "ros slam_noekf stack stopped"
