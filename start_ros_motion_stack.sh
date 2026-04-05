#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$PROJECT_DIR/stop_ros_motion_stack.sh" >/dev/null 2>&1 || true
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
  "--ros-args -p serial_port:=/dev/ttyAMA0 --log-level info"
sleep 2
start_node \
  "localization" \
  "$ROS_WS/install/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" \
  "--ros-args --log-level info"
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

echo "ros motion stack started"
echo "pid dir: $PID_DIR"
echo "logs:"
echo "  $RUNTIME_LOG_DIR/ros_motion_base.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_localization.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_mux.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_joy.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_joystick.log"
echo "  $RUNTIME_LOG_DIR/ros_motion_web.log"
for pid_file in "$PID_DIR"/*.pid; do
  [ -f "$pid_file" ] || continue
  node_name="$(basename "$pid_file" .pid)"
  pid="$(cat "$pid_file")"
  echo "  $node_name: $pid"
done
