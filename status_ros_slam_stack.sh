#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

echo "== slam node pids =="
for name in slam_base slam_bridge slam_mux slam_joy slam_joystick slam_lidar slam_laser_tf slam_filter slam_ekf slam_slam; do
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

echo "== slam node processes =="
pgrep -af "/rasprover_base/lib/rasprover_base/robot_base_node|/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node|/rasprover_control/lib/rasprover_control/command_mux_node|/rasprover_control/lib/rasprover_control/local_joy_node|/rasprover_control/lib/rasprover_control/joystick_bridge_node|/rplidar_ros/lib/rplidar_ros/rplidar_node|/rasprover_sensors/lib/rasprover_sensors/lidar_scan_node|/tf2_ros/static_transform_publisher|/rasprover_localization/lib/rasprover_localization/slam_sensor_bridge_node|/robot_localization/lib/robot_localization/ekf_node|rasprover_slam/rasprover_slam/simple_odom_filter_node.py|/slam_toolbox/lib/slam_toolbox/async_slam_toolbox_node" || echo "No slam stack nodes found"
echo

echo "== serial users =="
lsof /dev/ttyAMA0 2>/dev/null || echo "No serial device currently open"
echo

echo "== slam logs =="
for name in base bridge mux joy joystick lidar laser_tf filter ekf slam; do
  log_file="$RUNTIME_LOG_DIR/ros_slam_${name}.log"
  echo "-- $name --"
  if [ -f "$log_file" ]; then
    tail -n 20 "$log_file"
  else
    echo "Log file not found: $log_file"
  fi
done
