#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
PID_DIR="$PROJECT_DIR/.ros_slam_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

echo "== slam_scan_debug launch pid =="
pid_file="$PID_DIR/slam_scan_debug_launch.pid"
if [ -f "$pid_file" ]; then
  pid="$(cat "$pid_file")"
  if kill -0 "$pid" 2>/dev/null; then
    echo "launch: $pid"
  else
    echo "launch: stale pid ($pid)"
  fi
else
  echo "launch: not started"
fi
echo

echo "== slam_scan_debug processes =="
pgrep -af "/rasprover_base/lib/rasprover_base/robot_base_node|/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node|/rasprover_slam/lib/rasprover_slam/simple_odom_filter_node|/rasprover_control/lib/rasprover_control/command_mux_node|/rasprover_control/lib/rasprover_control/local_joy_node|/rasprover_control/lib/rasprover_control/joystick_bridge_node|/rasprover_ui/lib/rasprover_ui/web_bridge_node|/rplidar_ros/lib/rplidar_ros/rplidar_node|/tf2_ros/static_transform_publisher" || echo "No slam_scan_debug nodes found"
echo

echo "== slam_scan_debug log =="
log_file="$RUNTIME_LOG_DIR/ros_slam_scan_debug.log"
if [ -f "$log_file" ]; then
  tail -n 80 "$log_file"
else
  echo "Log file not found: $log_file"
fi
