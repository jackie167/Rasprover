#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
PID_DIR="$PROJECT_DIR/.ros_slam_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

echo "== slam_noekf launch pid =="
if [ -f "$PID_DIR/slam_noekf_launch.pid" ]; then
  pid="$(cat "$PID_DIR/slam_noekf_launch.pid")"
  if kill -0 "$pid" 2>/dev/null; then
    echo "launch: $pid"
  else
    echo "launch: stale pid ($pid)"
  fi
else
  echo "launch: not started"
fi
echo

echo "== slam_noekf processes =="
pgrep -af "slam_noekf.launch.py|robot_base_node|slam_sensor_bridge_node|command_mux_node|local_joy_node|joystick_bridge_node|web_bridge_node|rplidar_node|static_transform_publisher|slam_toolbox" || echo "No slam_noekf nodes found"
echo

echo "== slam_noekf log =="
if [ -f "$RUNTIME_LOG_DIR/ros_slam_noekf.log" ]; then
  tail -n 40 "$RUNTIME_LOG_DIR/ros_slam_noekf.log"
else
  echo "Log file not found: $RUNTIME_LOG_DIR/ros_slam_noekf.log"
fi
