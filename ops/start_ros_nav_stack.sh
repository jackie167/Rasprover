#!/bin/bash

set -e

source "$(dirname "$0")/_project_env.sh"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_nav_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyUSB0}"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

MAP_FILE="${NAV_MAP_FILE:-}"
LOCALIZATION_MODE="${NAV_LOCALIZATION_MODE:-}"
ENABLE_EXPLORE="${NAV_ENABLE_EXPLORE:-false}"
if [ -z "$MAP_FILE" ]; then
  MAP_FILE="$(ls -t "$PROJECT_DIR"/maps/*.yaml 2>/dev/null | head -n 1 || true)"
fi
if [ -z "$LOCALIZATION_MODE" ]; then
  if [ -n "$MAP_FILE" ]; then
    LOCALIZATION_MODE="amcl"
  else
    LOCALIZATION_MODE="slam"
  fi
fi

"$SCRIPT_DIR/stop_ros_nav_stack.sh" >/dev/null 2>&1 || true
sleep 1

start_node() {
  local name="$1"
  local exec_path="$2"
  local args="$3"
  local log_file="$RUNTIME_LOG_DIR/ros_nav_${name}.log"
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
    if [ -f '$ROS_WS/install/dwb_critics/share/dwb_critics/local_setup.bash' ]; then
      source '$ROS_WS/install/dwb_critics/share/dwb_critics/local_setup.bash'
    fi
    if [ -f '$ROS_WS/install/dwb_plugins/share/dwb_plugins/local_setup.bash' ]; then
      source '$ROS_WS/install/dwb_plugins/share/dwb_plugins/local_setup.bash'
    fi
    if [ -f '$ROS_WS/install/nav2_dwb_controller/share/nav2_dwb_controller/package.bash' ]; then
      source '$ROS_WS/install/nav2_dwb_controller/share/nav2_dwb_controller/package.bash'
    fi
    exec '$exec_path' $args
  " > "$log_file" 2>&1 < /dev/null &
  echo $! > "$PID_DIR/nav_${name}.pid"
}

run_ros_cli() {
  local args="$1"
  bash -lc "
    export ROS_LOG_DIR='$LOG_DIR'
    export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
    export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
    export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
    export ROS_STATIC_PEERS='${ROS_STATIC_PEERS}'
    export PYTHONPATH='$PROJECT_DIR'
    export PROJECT_DIR='$PROJECT_DIR'
    source '$ROS_WS/install/setup.bash'
    if [ -f '$ROS_WS/install/dwb_critics/share/dwb_critics/local_setup.bash' ]; then
      source '$ROS_WS/install/dwb_critics/share/dwb_critics/local_setup.bash'
    fi
    if [ -f '$ROS_WS/install/dwb_plugins/share/dwb_plugins/local_setup.bash' ]; then
      source '$ROS_WS/install/dwb_plugins/share/dwb_plugins/local_setup.bash'
    fi
    if [ -f '$ROS_WS/install/nav2_dwb_controller/share/nav2_dwb_controller/package.bash' ]; then
      source '$ROS_WS/install/nav2_dwb_controller/share/nav2_dwb_controller/package.bash'
    fi
    ros2 $args
  "
}

wait_for_ros_node() {
  local node_name="$1"
  local attempts="${2:-20}"
  local delay="${3:-1}"
  local i
  for ((i=0; i<attempts; i++)); do
    if run_ros_cli "node list" 2>/dev/null | grep -qx "$node_name"; then
      return 0
    fi
    sleep "$delay"
  done
  return 1
}

wait_for_amcl_ready() {
  local timeout_secs="${1:-90}"
  local log_file="$RUNTIME_LOG_DIR/ros_nav_nav_localization.log"
  local i
  for ((i=0; i<timeout_secs; i++)); do
    if [ -f "$log_file" ] && grep -q "Setting pose (" "$log_file" 2>/dev/null; then
      return 0
    fi
    sleep 1
  done
  return 1
}

transition_lifecycle_node() {
  local node_name="$1"
  local transition="$2"
  local attempts="${3:-10}"
  local delay="${4:-1}"
  local i
  for ((i=0; i<attempts; i++)); do
    if run_ros_cli "lifecycle set $node_name $transition" >/dev/null 2>&1; then
      return 0
    fi
    sleep "$delay"
  done
  return 1
}

start_node \
  "base" \
  "$ROS_WS/install/rasprover_base/lib/rasprover_base/robot_base_node" \
  "--ros-args -p serial_port:=/dev/ttyAMA0 -p left_drive_scale:=${LEFT_DRIVE_SCALE:-1.000} -p right_drive_scale:=${RIGHT_DRIVE_SCALE:-0.983} -p feedback_wheel_separation_m:=${FEEDBACK_WHEEL_SEPARATION_M:-0.52} -p feedback_wheel_yaw_scale:=${WHEEL_YAW_SCALE:-2.80} -p swap_feedback_wheels:=${SWAP_FEEDBACK_WHEELS:-false} -p straight_controller_enabled:=${STRAIGHT_CONTROLLER_ENABLED:-false} -p straight_controller_forward_only:=${STRAIGHT_CONTROLLER_FORWARD_ONLY:-true} -p straight_controller_linear_min:=${STRAIGHT_CONTROLLER_LINEAR_MIN:-0.10} -p straight_controller_angular_window:=${STRAIGHT_CONTROLLER_ANGULAR_WINDOW:-0.05} -p straight_controller_heading_gain:=${STRAIGHT_CONTROLLER_HEADING_GAIN:-0.90} -p straight_controller_integral_gain:=${STRAIGHT_CONTROLLER_INTEGRAL_GAIN:-0.12} -p straight_controller_wheel_balance_gain:=${STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN:-0.80} -p straight_controller_gyro_gain:=${STRAIGHT_CONTROLLER_GYRO_GAIN:-0.20} -p straight_controller_integral_limit:=${STRAIGHT_CONTROLLER_INTEGRAL_LIMIT:-0.30} -p straight_controller_max_correction:=${STRAIGHT_CONTROLLER_MAX_CORRECTION:-0.12} --log-level info"
sleep 2
start_node \
  "bridge" \
  "$ROS_WS/install/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" \
  "--ros-args -p wheel_yaw_scale:=${WHEEL_YAW_SCALE:-2.80} -p linear_odom_scale:=${LINEAR_ODOM_SCALE:-0.976} -p left_odom_scale:=${LEFT_ODOM_SCALE:-0.990} -p right_odom_scale:=${RIGHT_ODOM_SCALE:-1.000} -p publish_tf:=false --log-level info"
sleep 1
start_node \
  "filter" \
  "$ROS_WS/install/rasprover_slam/lib/rasprover_slam/simple_odom_filter_node" \
  "--ros-args --params-file '$ROS_WS/src/rasprover_slam/config/simple_odom_filter.yaml' --log-level info"
sleep 1
start_node \
  "mux" \
  "$ROS_WS/install/rasprover_control/lib/rasprover_control/command_mux_node" \
  "--ros-args --log-level info"
sleep 1
start_node \
  "joy" \
  "$ROS_WS/install/rasprover_control/lib/rasprover_control/local_joy_node" \
  "--ros-args --log-level info"
sleep 1
start_node \
  "joystick" \
  "$ROS_WS/install/rasprover_control/lib/rasprover_control/joystick_bridge_node" \
  "--ros-args --log-level info"
sleep 1
start_node \
  "lidar" \
  "$ROS_WS/install/rplidar_ros/lib/rplidar_ros/rplidar_node" \
  "--ros-args -p channel_type:=serial -p serial_port:=${LIDAR_PORT} -p serial_baudrate:=460800 -p frame_id:=laser -p inverted:=false -p angle_compensate:=true -p scan_mode:=Standard --log-level info"
sleep 1
start_node \
  "laser_tf" \
  "ros2" \
  "run tf2_ros static_transform_publisher --x 0.04 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 3.141592653589793 --frame-id base_link --child-frame-id laser"
sleep 1
if [ "$LOCALIZATION_MODE" = "slam" ]; then
start_node \
  "slam" \
  "$ROS_WS/install/slam_toolbox/lib/slam_toolbox/async_slam_toolbox_node" \
  "--ros-args --params-file '$ROS_WS/src/rasprover_slam/config/slam_toolbox_online_async.yaml' --log-level info"
sleep 2
wait_for_ros_node "/slam_toolbox" 20 1 || true
transition_lifecycle_node "/slam_toolbox" "configure" 15 1 || true
sleep 1
transition_lifecycle_node "/slam_toolbox" "activate" 15 1 || true
sleep 2
fi

NAV_LAUNCH_ARGS="launch rasprover_bringup navigation_stack.launch.py params_file:='$ROS_WS/src/rasprover_slam/config/nav2_navigation.yaml' localization_mode:='${LOCALIZATION_MODE}'"
if [ -n "$MAP_FILE" ]; then
  NAV_LAUNCH_ARGS="$NAV_LAUNCH_ARGS map:='${MAP_FILE}'"
fi

if [ "$LOCALIZATION_MODE" = "amcl" ]; then
  start_node \
    "nav_localization" \
    "ros2" \
    "$NAV_LAUNCH_ARGS start_localization:='true' start_navigation:='false'"
  echo "waiting for AMCL initial pose before starting navigation bringup..."
  echo "set initial pose in RViz now if needed"
  if wait_for_amcl_ready 90; then
    echo "AMCL initial pose received, starting navigation nodes"
    start_node \
      "nav_navigation" \
      "ros2" \
      "$NAV_LAUNCH_ARGS start_localization:='false' start_navigation:='true'"
    sleep 2
  else
    echo "warning: timed out waiting for AMCL initial pose; localization-only stack is running"
    echo "after setting initial pose, rerun this script to bring up navigation nodes"
  fi
else
  start_node \
    "nav" \
    "ros2" \
    "$NAV_LAUNCH_ARGS start_localization:='false' start_navigation:='true'"
  sleep 2
fi

if [ "$ENABLE_EXPLORE" = "true" ]; then
start_node \
  "explore" \
  "ros2" \
  "launch rasprover_slam frontier_explorer.launch.py params_file:='$ROS_WS/src/rasprover_slam/config/frontier_explorer.yaml'"
sleep 1
fi

echo "ros nav stack started"
echo "localization_mode: $LOCALIZATION_MODE"
echo "explore: $ENABLE_EXPLORE"
if [ -n "$MAP_FILE" ]; then
  echo "map: $MAP_FILE"
else
  echo "map: online /map from slam_toolbox"
fi
echo "pid dir: $PID_DIR"
echo "logs:"
echo "  $RUNTIME_LOG_DIR/ros_nav_base.log"
echo "  $RUNTIME_LOG_DIR/ros_nav_bridge.log"
echo "  $RUNTIME_LOG_DIR/ros_nav_filter.log"
echo "  $RUNTIME_LOG_DIR/ros_nav_mux.log"
echo "  $RUNTIME_LOG_DIR/ros_nav_joy.log"
echo "  $RUNTIME_LOG_DIR/ros_nav_joystick.log"
echo "  $RUNTIME_LOG_DIR/ros_nav_lidar.log"
echo "  $RUNTIME_LOG_DIR/ros_nav_laser_tf.log"
if [ "$LOCALIZATION_MODE" = "slam" ]; then
  echo "  $RUNTIME_LOG_DIR/ros_nav_slam.log"
fi
echo "  $RUNTIME_LOG_DIR/ros_nav_nav.log"
if [ "$ENABLE_EXPLORE" = "true" ]; then
  echo "  $RUNTIME_LOG_DIR/ros_nav_explore.log"
fi
