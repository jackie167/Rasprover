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
stop_pid_file "$PID_DIR/slam_filter.pid"
stop_pid_file "$PID_DIR/slam_laser_tf.pid"
stop_pid_file "$PID_DIR/slam_lidar.pid"
stop_pid_file "$PID_DIR/slam_joystick.pid"
stop_pid_file "$PID_DIR/slam_joy.pid"
stop_pid_file "$PID_DIR/slam_mux.pid"
stop_pid_file "$PID_DIR/slam_bridge.pid"
stop_pid_file "$PID_DIR/slam_base.pid"

pkill -f "/slam_toolbox/lib/slam_toolbox/async_slam_toolbox_node" || true
pkill -f "/robot_localization/lib/robot_localization/ekf_node" || true
pkill -f "rasprover_slam/rasprover_slam/simple_odom_filter_node.py" || true
pkill -f "/tf2_ros/static_transform_publisher.*base_link --child-frame-id laser" || true
pkill -f "/rplidar_ros/lib/rplidar_ros/rplidar_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/joystick_bridge_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/local_joy_node" || true
pkill -f "/rasprover_control/lib/rasprover_control/command_mux_node" || true
pkill -f "/rasprover_sensors/lib/rasprover_sensors/lidar_scan_node" || true
pkill -f "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" || true
pkill -f "/rasprover_localization/lib/rasprover_localization/slam_sensor_bridge_node" || true
pkill -f "/rasprover_base/lib/rasprover_base/robot_base_node" || true
pkill -f "ros2 run slam_toolbox async_slam_toolbox_node" || true
pkill -f "ros2 run robot_localization ekf_node" || true
pkill -f "ros2 run tf2_ros static_transform_publisher" || true
pkill -f "ros2 run rplidar_ros rplidar_node" || true
pkill -f "ros2 run rasprover_control joystick_bridge_node" || true
pkill -f "ros2 run rasprover_control local_joy_node" || true
pkill -f "ros2 run rasprover_control command_mux_node" || true
pkill -f "ros2 run rasprover_sensors lidar_scan_node" || true
pkill -f "ros2 run rasprover_sensors slam_sensor_bridge_node" || true
pkill -f "ros2 run rasprover_localization slam_sensor_bridge_node" || true
pkill -f "ros2 run rasprover_base robot_base_node" || true

echo "ros slam stack stopped"
