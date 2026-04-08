#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$PROJECT_DIR/stop_ros_path_test_stack.sh" >/dev/null 2>&1 || true
sleep 1

start_node() {
  local name="$1"
  local exec_path="$2"
  local args="$3"
  local log_file="$RUNTIME_LOG_DIR/ros_path_${name}.log"
  setsid bash -lc "
    export ROS_LOG_DIR='$LOG_DIR'
    export ROS_LOCALHOST_ONLY=\${ROS_LOCALHOST_ONLY:-0}
    export ROS_AUTOMATIC_DISCOVERY_RANGE=\${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
    export PYTHONPATH='$ROS_WS/build/rasprover_slam:$PROJECT_DIR':\${PYTHONPATH}
    export PROJECT_DIR='$PROJECT_DIR'
    source '$ROS_WS/install/setup.bash'
    exec '$exec_path' $args
  " > "$log_file" 2>&1 < /dev/null &
  local pid=$!
  echo "$pid" > "$PID_DIR/path_${name}.pid"
}

start_node \
  "base" \
  "$ROS_WS/install/rasprover_base/lib/rasprover_base/robot_base_node" \
  "--ros-args -p serial_port:=/dev/ttyAMA0 -p left_drive_scale:=${LEFT_DRIVE_SCALE:-1.000} -p right_drive_scale:=${RIGHT_DRIVE_SCALE:-0.983} -p feedback_wheel_separation_m:=${WHEEL_SEPARATION_M:-${FEEDBACK_WHEEL_SEPARATION_M:-0.52}} -p feedback_wheel_yaw_scale:=${WHEEL_YAW_SCALE:-2.80} -p swap_feedback_wheels:=${SWAP_FEEDBACK_WHEELS:-false} -p straight_controller_enabled:=${STRAIGHT_CONTROLLER_ENABLED:-false} -p straight_controller_forward_only:=${STRAIGHT_CONTROLLER_FORWARD_ONLY:-true} -p straight_controller_linear_min:=${STRAIGHT_CONTROLLER_LINEAR_MIN:-0.10} -p straight_controller_angular_window:=${STRAIGHT_CONTROLLER_ANGULAR_WINDOW:-0.05} -p straight_controller_heading_gain:=${STRAIGHT_CONTROLLER_HEADING_GAIN:-0.90} -p straight_controller_integral_gain:=${STRAIGHT_CONTROLLER_INTEGRAL_GAIN:-0.12} -p straight_controller_wheel_balance_gain:=${STRAIGHT_CONTROLLER_WHEEL_BALANCE_GAIN:-0.80} -p straight_controller_gyro_gain:=${STRAIGHT_CONTROLLER_GYRO_GAIN:-0.20} -p straight_controller_integral_limit:=${STRAIGHT_CONTROLLER_INTEGRAL_LIMIT:-0.30} -p straight_controller_max_correction:=${STRAIGHT_CONTROLLER_MAX_CORRECTION:-0.12} --log-level info"
sleep 2
start_node \
  "bridge" \
  "$ROS_WS/install/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" \
  "--ros-args -p wheel_separation_m:=${WHEEL_SEPARATION_M:-${FEEDBACK_WHEEL_SEPARATION_M:-0.52}} -p wheel_yaw_scale:=${WHEEL_YAW_SCALE:-2.80} -p left_odom_scale:=${LEFT_ODOM_SCALE:-0.990} -p right_odom_scale:=${RIGHT_ODOM_SCALE:-1.000} -p publish_tf:=false --log-level info"
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

if [ "${PATH_TEST_WITH_WEB:-0}" = "1" ]; then
  start_node \
    "web" \
    "$ROS_WS/install/rasprover_ui/lib/rasprover_ui/web_bridge_node" \
    "--ros-args -p port:=5050 --log-level info"
  sleep 1
fi

if [ "${PATH_TEST_DISABLE_EKF:-0}" != "1" ]; then
  if [ "${PATH_TEST_FILTER_IMPL:-simple}" = "ekf" ]; then
    start_node \
      "ekf" \
      "$ROS_WS/install/robot_localization/lib/robot_localization/ekf_node" \
      "--ros-args --params-file '$ROS_WS/src/rasprover_slam/config/ekf_wheel_imu.yaml' --log-level info"
    sleep 1
  else
    start_node \
      "filter" \
      "python3" \
      "'$ROS_WS/src/rasprover_slam/rasprover_slam/simple_odom_filter_node.py' --ros-args --params-file '$ROS_WS/src/rasprover_slam/config/simple_odom_filter.yaml' --log-level info"
    sleep 1
  fi
  start_node \
    "path" \
    "$ROS_WS/install/rasprover_slam/lib/rasprover_slam/odometry_path_node" \
    "--ros-args -p odom_topic:=/odometry/filtered -p path_topic:=/odom_path -p min_translation:=0.005 -p min_rotation:=0.01 --log-level info"
  sleep 1
fi
start_node \
  "wheel_path" \
  "$ROS_WS/install/rasprover_slam/lib/rasprover_slam/odometry_path_node" \
  "--ros-args -p odom_topic:=/wheel/odometry -p path_topic:=/wheel_odom_path -p min_translation:=0.005 -p min_rotation:=0.01 --log-level info"
sleep 1

echo "ros path test stack started"
echo "pid dir: $PID_DIR"
echo "logs:"
echo "  $RUNTIME_LOG_DIR/ros_path_base.log"
echo "  $RUNTIME_LOG_DIR/ros_path_bridge.log"
echo "  $RUNTIME_LOG_DIR/ros_path_mux.log"
echo "  $RUNTIME_LOG_DIR/ros_path_joy.log"
echo "  $RUNTIME_LOG_DIR/ros_path_joystick.log"
if [ "${PATH_TEST_WITH_WEB:-0}" = "1" ]; then
  echo "  $RUNTIME_LOG_DIR/ros_path_web.log"
fi
if [ "${PATH_TEST_DISABLE_EKF:-0}" != "1" ]; then
  echo "  $RUNTIME_LOG_DIR/ros_path_ekf.log"
  echo "  $RUNTIME_LOG_DIR/ros_path_path.log"
fi
echo "  $RUNTIME_LOG_DIR/ros_path_wheel_path.log"
if [ "${PATH_TEST_DISABLE_EKF:-0}" = "1" ]; then
  echo "mode: wheel odom only (ekf disabled)"
else
  echo "mode: wheel odom + ${PATH_TEST_FILTER_IMPL:-simple} compare"
fi
