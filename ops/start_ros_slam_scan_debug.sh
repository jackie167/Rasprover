#!/bin/bash

set -e

source "$(dirname "$0")/_project_env.sh"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_slam_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"
LAUNCH_LOG_FILE="$RUNTIME_LOG_DIR/ros_slam_scan_debug.log"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$SCRIPT_DIR/stop_ros_slam_scan_debug.sh" >/dev/null 2>&1 || true
sleep 4

check_process() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "OK"
  else
    echo "DOWN"
  fi
}

setsid bash -lc "
  export PROJECT_DIR='$PROJECT_DIR'
  export PYTHONPATH='$PROJECT_DIR'
  export ROS_LOG_DIR='$LOG_DIR'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export ROS_STATIC_PEERS='${ROS_STATIC_PEERS}'
  source '$ROS_WS/install/local_setup.bash'
  exec ros2 launch rasprover_bringup slam_scan_debug.launch.py
" > "$LAUNCH_LOG_FILE" 2>&1 < /dev/null &

LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PID_DIR/slam_scan_debug_launch.pid"

echo "ros slam_scan_debug stack started"
echo "launch: $LAUNCH_PID"
echo "log: $LAUNCH_LOG_FILE"

sleep 6

echo "health:"
echo "  robot_base_node: $(check_process '/rasprover_base/lib/rasprover_base/robot_base_node')"
echo "  slam_sensor_bridge_node: $(check_process '/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node')"
echo "  simple_odom_filter_node: $(check_process '/rasprover_slam/lib/rasprover_slam/simple_odom_filter_node|/rasprover_slam/.*/simple_odom_filter_node')"
echo "  command_mux_node: $(check_process '/rasprover_control/lib/rasprover_control/command_mux_node')"
echo "  local_joy_node: $(check_process '/rasprover_control/lib/rasprover_control/local_joy_node')"
echo "  joystick_bridge_node: $(check_process '/rasprover_control/lib/rasprover_control/joystick_bridge_node')"
echo "  web_bridge_node: $(check_process '/rasprover_ui/lib/rasprover_ui/web_bridge_node')"
echo "  static_transform_publisher: $(check_process '/tf2_ros/.*/static_transform_publisher|/tf2_ros/static_transform_publisher')"
echo "  rplidar_node: $(check_process '/rplidar_ros/lib/rplidar_ros/rplidar_node')"
if pgrep -f "/rasprover_ui/lib/rasprover_ui/web_bridge_node" >/dev/null 2>&1; then
  echo "  web_url: http://0.0.0.0:5050"
fi
