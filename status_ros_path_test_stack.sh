#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

echo "== path test node pids =="
for name in path_base path_bridge path_mux path_joy path_joystick path_web path_ekf path_filter path_path path_wheel_path; do
  pid_file="$PID_DIR/$name.pid"
  if [ -f "$pid_file" ]; then
    pid="$(cat "$pid_file")"
    if kill -0 "$pid" 2>/dev/null; then
      echo "$name: $pid"
    else
      echo "$name: stale pid ($pid)"
    fi
  else
    echo "$name: not started"
  fi
done
echo

if grep -q "ROS_LOCALHOST_ONLY is deprecated but still honored if it is enabled" "$RUNTIME_LOG_DIR"/ros_path_*.log 2>/dev/null; then
  echo "warning: some path test nodes still started with ROS_LOCALHOST_ONLY enabled"
  echo
fi

echo "== path test processes =="
pgrep -af "/rasprover_base/lib/rasprover_base/robot_base_node|/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node|/rasprover_control/lib/rasprover_control/command_mux_node|/rasprover_control/lib/rasprover_control/local_joy_node|/rasprover_control/lib/rasprover_control/joystick_bridge_node|/rasprover_ui/lib/rasprover_ui/web_bridge_node|/robot_localization/lib/robot_localization/ekf_node|rasprover_slam.simple_odom_filter_node|/rasprover_slam/lib/rasprover_slam/odometry_path_node" || echo "No path test stack nodes found"
echo

echo "== path logs =="
for name in base bridge mux joy joystick web ekf filter path wheel_path; do
  log_file="$RUNTIME_LOG_DIR/ros_path_${name}.log"
  echo "-- $name --"
  if [ -f "$log_file" ]; then
    tail -n 10 "$log_file"
  else
    echo "Log file not found: $log_file"
  fi
done
