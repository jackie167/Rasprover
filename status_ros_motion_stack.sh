#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"

echo "== ros node pids =="
for name in base localization mux joy joystick web; do
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

echo "== ros node processes =="
pgrep -af "/rasprover_base/lib/rasprover_base/robot_base_node|/rasprover_localization/lib/rasprover_localization/slam_sensor_bridge_node|/rasprover_mux/lib/rasprover_mux/command_mux_node|/rasprover_mux/lib/rasprover_mux/local_joy_node|/rasprover_mux/lib/rasprover_mux/joystick_teleop_node|/rasprover_web/lib/rasprover_web/web_bridge_node" || echo "No motion stack nodes found"
echo

echo "== serial users =="
lsof /dev/ttyAMA0 2>/dev/null || echo "No serial device currently open"
echo

echo "== node logs =="
for name in base localization mux joy joystick web; do
  log_file="$PROJECT_DIR/ros_motion_${name}.log"
  echo "-- $name --"
  if [ -f "$log_file" ]; then
    tail -n 20 "$log_file"
  else
    echo "Log file not found: $log_file"
  fi
done
