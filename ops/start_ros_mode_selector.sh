#!/bin/bash

set -e

source "$(dirname "$0")/_project_env.sh"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_selector_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$SCRIPT_DIR/stop_ros_motion_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_nav_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_path_test_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_scan_debug.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_ekf_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_noekf_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_mode_selector.sh" >/dev/null 2>&1 || true
sleep 2

start_node() {
  local name="$1"
  local exec_path="$2"
  local args="$3"
  local log_file="$RUNTIME_LOG_DIR/ros_selector_${name}.log"
  setsid bash -lc "
    export ROS_LOG_DIR='$LOG_DIR'
    export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
    export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
    export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
    export ROS_STATIC_PEERS='${ROS_STATIC_PEERS}'
    export PYTHONPATH='$PROJECT_DIR'
    export PROJECT_DIR='$PROJECT_DIR'
    source '$ROS_WS/install/setup.bash'
    exec '$exec_path' $args
  " > "$log_file" 2>&1 < /dev/null &
  local pid=$!
  echo "$pid" > "$PID_DIR/$name.pid"
}

start_node \
  "base" \
  "$ROS_WS/install/rasprover_base/lib/rasprover_base/robot_base_node" \
  "--ros-args -p serial_port:=/dev/ttyAMA0 --log-level info"
sleep 2

if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  start_node \
    "web" \
    "$ROS_WS/install/rasprover_ui/lib/rasprover_ui/web_bridge_node" \
    "--ros-args -p port:=5050 -p boot_role:=selector --log-level info"
  sleep 2
fi

echo "ros mode selector started"
echo "pid dir: $PID_DIR"
echo "logs:"
echo "  $RUNTIME_LOG_DIR/ros_selector_base.log"
if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  echo "  $RUNTIME_LOG_DIR/ros_selector_web.log"
fi
