#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
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

if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  stop_pid_file "$PID_DIR/web.pid"
fi
stop_pid_file "$PID_DIR/joystick.pid"
stop_pid_file "$PID_DIR/joy.pid"
stop_pid_file "$PID_DIR/mux.pid"
stop_pid_file "$PID_DIR/localization.pid"
stop_pid_file "$PID_DIR/cv.pid"
stop_pid_file "$PID_DIR/base.pid"

pkill -f "/rasprover_base/lib/rasprover_base/robot_base_node" || true
pkill -f "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" || true
pkill -f "/rasprover_localization/lib/rasprover_localization/slam_sensor_bridge_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/command_mux_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/local_joy_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/joystick_bridge_node" || true
if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  pkill -f "/rasprover_ui/lib/rasprover_ui/web_bridge_node" || true
fi
pkill -f "/rasprover_cv/lib/rasprover_cv/cv_node" || true
pkill -f "/rasprover_mux/lib/rasprover_mux/command_mux_node" || true
pkill -f "/rasprover_mux/lib/rasprover_mux/local_joy_node" || true
pkill -f "/rasprover_mux/lib/rasprover_mux/joystick_teleop_node" || true
pkill -f "/rasprover_web/lib/rasprover_web/web_bridge_node" || true
pkill -f "ros2 run rasprover_base robot_base_node" || true
pkill -f "ros2 run rasprover_sensors slam_sensor_bridge_node" || true
pkill -f "ros2 run rasprover_localization slam_sensor_bridge_node" || true
pkill -f "ros2 run rasprover_control command_mux_node" || true
pkill -f "ros2 run rasprover_control local_joy_node" || true
pkill -f "ros2 run rasprover_control joystick_bridge_node" || true
if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  pkill -f "ros2 run rasprover_ui web_bridge_node" || true
fi
pkill -f "ros2 run rasprover_cv cv_node" || true
pkill -f "ros2 run rasprover_mux command_mux_node" || true
pkill -f "ros2 run rasprover_mux local_joy_node" || true
pkill -f "ros2 run rasprover_mux joystick_teleop_node" || true
pkill -f "ros2 run rasprover_web web_bridge_node" || true

echo "ros motion stack stopped"
