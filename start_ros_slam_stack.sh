#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"

mkdir -p "$LOG_DIR" "$PID_DIR"

"$PROJECT_DIR/stop_ros_slam_stack.sh" >/dev/null 2>&1 || true
sleep 1

start_node() {
  local name="$1"
  local exec_path="$2"
  local args="$3"
  local log_file="$PROJECT_DIR/ros_slam_${name}.log"
  setsid bash -lc "
    export ROS_LOG_DIR='$LOG_DIR'
    export ROS_LOCALHOST_ONLY=1
    export PYTHONPATH='$PROJECT_DIR'
    export PROJECT_DIR='$PROJECT_DIR'
    source '$ROS_WS/install/setup.bash'
    exec '$exec_path' $args
  " > "$log_file" 2>&1 < /dev/null &
  local pid=$!
  echo "$pid" > "$PID_DIR/slam_${name}.pid"
}

start_node \
  "base" \
  "$ROS_WS/install/rasprover_base/lib/rasprover_base/robot_base_node" \
  "--ros-args -p serial_port:=/dev/ttyAMA0 --log-level info"
sleep 2
start_node \
  "bridge" \
  "$ROS_WS/install/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" \
  "--ros-args --log-level info"
sleep 1
start_node \
  "ekf" \
  "$ROS_WS/install/robot_localization/lib/robot_localization/ekf_node" \
  "--ros-args --params-file '$ROS_WS/src/rasprover_slam/config/ekf_wheel_imu.yaml' --log-level info"
sleep 1
start_node \
  "slam" \
  "$ROS_WS/install/slam_toolbox/lib/slam_toolbox/async_slam_toolbox_node" \
  "--ros-args --params-file '$ROS_WS/src/rasprover_slam/config/slam_toolbox_online_async.yaml' --log-level info"
sleep 1

echo "ros slam stack started"
echo "pid dir: $PID_DIR"
echo "logs:"
echo "  $PROJECT_DIR/ros_slam_base.log"
echo "  $PROJECT_DIR/ros_slam_bridge.log"
echo "  $PROJECT_DIR/ros_slam_ekf.log"
echo "  $PROJECT_DIR/ros_slam_slam.log"
for pid_file in "$PID_DIR"/slam_*.pid; do
  [ -f "$pid_file" ] || continue
  node_name="$(basename "$pid_file" .pid)"
  pid="$(cat "$pid_file")"
  echo "  $node_name: $pid"
done
