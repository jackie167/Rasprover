#!/bin/bash

set -e

source "$(dirname "$0")/_project_env.sh"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"
WITH_CV="${WITH_CV:-false}"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$SCRIPT_DIR/stop_ros_motion_stack.sh" >/dev/null 2>&1 || true
sleep 1

start_node() {
  local name="$1"
  local exec_path="$2"
  local args="$3"
  local log_file="$RUNTIME_LOG_DIR/ros_motion_${name}.log"
  setsid bash -lc "
    export ROS_LOG_DIR='$LOG_DIR'
    export ROS_LOCALHOST_ONLY=1
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
  "--ros-args -p serial_port:=/dev/ttyAMA0 -p left_drive_scale:=${LEFT_DRIVE_SCALE:-1.000} -p right_drive_scale:=${RIGHT_DRIVE_SCALE:-0.983} -p feedback_wheel_separation_m:=${FEEDBACK_WHEEL_SEPARATION_M:-0.52} -p feedback_wheel_yaw_scale:=${WHEEL_YAW_SCALE:-2.80} -p swap_feedback_wheels:=${SWAP_FEEDBACK_WHEELS:-false} -p straight_controller_enabled:=${STRAIGHT_CONTROLLER_ENABLED:-false} -p straight_controller_forward_only:=${STRAIGHT_CONTROLLER_FORWARD_ONLY:-true} -p straight_controller_linear_min:=${STRAIGHT_CONTROLLER_LINEAR_MIN:-0.10} -p straight_controller_angular_window:=${STRAIGHT_CONTROLLER_ANGULAR_WINDOW:-0.05} -p straight_controller_heading_gain:=${STRAIGHT_CONTROLLER_HEADING_GAIN:-0.90} -p straight_controller_integral_gain:=${STRAIGHT_CONTROLLER_INTEGRAL_GAIN:-0.12} -p straight_controller_wheel_balance_gain:=${STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN:-0.80} -p straight_controller_gyro_gain:=${STRAIGHT_CONTROLLER_GYRO_GAIN:-0.20} -p straight_controller_integral_limit:=${STRAIGHT_CONTROLLER_INTEGRAL_LIMIT:-0.30} -p straight_controller_max_correction:=${STRAIGHT_CONTROLLER_MAX_CORRECTION:-0.12} --log-level info"
sleep 2
start_node \
  "localization" \
  "$ROS_WS/install/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" \
  "--ros-args -p wheel_yaw_scale:=${WHEEL_YAW_SCALE:-2.80} -p linear_odom_scale:=${LINEAR_ODOM_SCALE:-0.976} -p left_odom_scale:=${LEFT_ODOM_SCALE:-0.990} -p right_odom_scale:=${RIGHT_ODOM_SCALE:-1.000} --log-level info"
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
  "web" \
  "$ROS_WS/install/rasprover_ui/lib/rasprover_ui/web_bridge_node" \
  "--ros-args -p port:=5050 --log-level info"
sleep 2

if [ "$WITH_CV" = "true" ]; then
  pkill -f "/rasprover_cv/lib/rasprover_cv/cv_node" || true
  sleep 1
  start_node \
    "cv" \
    "$ROS_WS/install/rasprover_cv/lib/rasprover_cv/cv_node" \
    "--ros-args -p port:=5051 --log-level info"
  sleep 1
fi

echo "ros motion stack started"
echo "with cv: $WITH_CV"
echo "pid dir: $PID_DIR"
echo "logs:"
echo "  $RUNTIME_LOG_DIR/ros_motion_base.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_localization.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_mux.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_joy.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_joystick.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_web.log"
if [ "$WITH_CV" = "true" ]; then
  echo "  $RUNTIME_LOG_DIR/ros_motion_cv.log"
fi
for pid_file in "$PID_DIR"/*.pid; do
  [ -f "$pid_file" ] || continue
  node_name="$(basename "$pid_file" .pid)"
  pid="$(cat "$pid_file")"
  echo "  $node_name: $pid"
done
