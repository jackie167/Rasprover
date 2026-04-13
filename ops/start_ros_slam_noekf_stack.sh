#!/bin/bash

set -e

source "$(dirname "$0")/_project_env.sh"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_slam_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"
LAUNCH_LOG_FILE="$RUNTIME_LOG_DIR/ros_slam_noekf.log"
START_WEB="true"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$SCRIPT_DIR/stop_ros_motion_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_nav_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_path_test_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_scan_debug.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_ekf_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_slam_noekf_stack.sh" >/dev/null 2>&1 || true
"$SCRIPT_DIR/stop_ros_mode_selector.sh" >/dev/null 2>&1 || true
sleep 4

if [ "${KEEP_WEB_BRIDGE:-false}" = "true" ]; then
  START_WEB="false"
fi

check_process() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "OK"
  else
    echo "DOWN"
  fi
}

run_ros_cli() {
  local args="$1"
  bash -lc "
    export PROJECT_DIR='$PROJECT_DIR'
    export PYTHONPATH='$PROJECT_DIR'
    export ROS_LOG_DIR='$LOG_DIR'
    export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
    export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
    export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
    export ROS_STATIC_PEERS='${ROS_STATIC_PEERS}'
    source '$ROS_WS/install/local_setup.bash'
    ros2 $args
  "
}

slam_toolbox_state() {
  run_ros_cli "lifecycle get /slam_toolbox" 2>/dev/null | awk 'NR==1 {print $1}'
}

wait_for_slam_toolbox_state() {
  local expected="$1"
  local attempts="${2:-20}"
  local delay="${3:-1}"
  local state=""
  local i
  for ((i = 0; i < attempts; i++)); do
    state="$(slam_toolbox_state || true)"
    if [ "$state" = "$expected" ]; then
      return 0
    fi
    sleep "$delay"
  done
  return 1
}

activate_slam_toolbox() {
  local i
  for ((i = 0; i < 20; i++)); do
    run_ros_cli "lifecycle get /slam_toolbox" >/dev/null 2>&1 && break
    sleep 1
  done

  if ! wait_for_slam_toolbox_state "unconfigured" 5 1; then
    :
  fi

  for ((i = 0; i < 5; i++)); do
    run_ros_cli "lifecycle set /slam_toolbox configure" >/dev/null 2>&1 || true
    if wait_for_slam_toolbox_state "inactive" 5 1; then
      break
    fi
  done

  for ((i = 0; i < 5; i++)); do
    run_ros_cli "lifecycle set /slam_toolbox activate" >/dev/null 2>&1 || true
    if wait_for_slam_toolbox_state "active" 5 1; then
      return 0
    fi
  done
  return 1
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
  exec ros2 launch rasprover_bringup slam_noekf.launch.py start_web:=${START_WEB}
" > "$LAUNCH_LOG_FILE" 2>&1 < /dev/null &

LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PID_DIR/slam_noekf_launch.pid"

echo "ros slam_noekf stack started"
echo "launch: $LAUNCH_PID"
echo "log: $LAUNCH_LOG_FILE"

sleep 4
activate_slam_toolbox >/dev/null 2>&1 || true

echo "health:"
echo "  robot_base_node: $(check_process '/rasprover_base/lib/rasprover_base/robot_base_node')"
echo "  slam_sensor_bridge_node: $(check_process '/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node')"
echo "  command_mux_node: $(check_process '/rasprover_control/lib/rasprover_control/command_mux_node')"
echo "  local_joy_node: $(check_process '/rasprover_control/lib/rasprover_control/local_joy_node')"
echo "  joystick_bridge_node: $(check_process '/rasprover_control/lib/rasprover_control/joystick_bridge_node')"
echo "  web_bridge_node: $(check_process '/rasprover_ui/lib/rasprover_ui/web_bridge_node')"
echo "  static_transform_publisher: $(check_process '/tf2_ros/.*/static_transform_publisher|/tf2_ros/static_transform_publisher')"
if [ "$(slam_toolbox_state || true)" = "active" ]; then
  echo "  slam_toolbox: OK"
else
  echo "  slam_toolbox: DOWN"
fi
echo "  rplidar_node: $(check_process '/rplidar_ros/lib/rplidar_ros/rplidar_node')"
if pgrep -f "/rasprover_ui/lib/rasprover_ui/web_bridge_node" >/dev/null 2>&1; then
  echo "  web_url: http://0.0.0.0:5050"
fi
