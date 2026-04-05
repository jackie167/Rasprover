#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"

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

stop_pid_file "$PID_DIR/slam_slam.pid"
stop_pid_file "$PID_DIR/slam_ekf.pid"
stop_pid_file "$PID_DIR/slam_bridge.pid"
stop_pid_file "$PID_DIR/slam_base.pid"

pkill -f "/slam_toolbox/lib/slam_toolbox/async_slam_toolbox_node" || true
pkill -f "/robot_localization/lib/robot_localization/ekf_node" || true
pkill -f "/rasprover_localization/lib/rasprover_localization/slam_sensor_bridge_node" || true
pkill -f "/rasprover_base/lib/rasprover_base/robot_base_node" || true
pkill -f "ros2 run slam_toolbox async_slam_toolbox_node" || true
pkill -f "ros2 run robot_localization ekf_node" || true
pkill -f "ros2 run rasprover_localization slam_sensor_bridge_node" || true
pkill -f "ros2 run rasprover_base robot_base_node" || true

echo "ros slam stack stopped"
