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
    export PYTHONPATH='$PROJECT_DIR'
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
  "--ros-args -p serial_port:=/dev/ttyAMA0 --log-level info"
sleep 2
start_node \
  "bridge" \
  "$ROS_WS/install/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" \
  "--ros-args -p wheel_yaw_scale:=${WHEEL_YAW_SCALE:-1.96} --log-level info"
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
start_node \
  "ekf" \
  "$ROS_WS/install/robot_localization/lib/robot_localization/ekf_node" \
  "--ros-args --params-file '$ROS_WS/src/rasprover_slam/config/ekf_wheel_imu.yaml' --log-level info"
sleep 1
start_node \
  "path" \
  "$ROS_WS/install/rasprover_slam/lib/rasprover_slam/odometry_path_node" \
  "--ros-args -p odom_topic:=/odometry/filtered -p path_topic:=/odom_path -p min_translation:=0.005 -p min_rotation:=0.01 --log-level info"
sleep 1
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
echo "  $RUNTIME_LOG_DIR/ros_path_ekf.log"
echo "  $RUNTIME_LOG_DIR/ros_path_path.log"
echo "  $RUNTIME_LOG_DIR/ros_path_wheel_path.log"
