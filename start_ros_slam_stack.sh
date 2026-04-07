#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$PROJECT_DIR/stop_ros_slam_stack.sh" >/dev/null 2>&1 || true
sleep 1

start_node() {
  local name="$1"
  local exec_path="$2"
  local args="$3"
  local log_file="$RUNTIME_LOG_DIR/ros_slam_${name}.log"
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
  echo "$pid" > "$PID_DIR/slam_${name}.pid"
}

run_ros_cli() {
  local args="$1"
  bash -lc "
    export ROS_LOG_DIR='$LOG_DIR'
    export ROS_LOCALHOST_ONLY=\${ROS_LOCALHOST_ONLY:-0}
    export ROS_AUTOMATIC_DISCOVERY_RANGE=\${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}
    export PYTHONPATH='$PROJECT_DIR'
    export PROJECT_DIR='$PROJECT_DIR'
    source '$ROS_WS/install/setup.bash'
    ros2 $args
  "
}

start_node \
  "base" \
  "$ROS_WS/install/rasprover_base/lib/rasprover_base/robot_base_node" \
  "--ros-args -p serial_port:=/dev/ttyAMA0 --log-level info"
sleep 2
start_node \
  "bridge" \
  "$ROS_WS/install/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" \
  "--ros-args -p wheel_yaw_scale:=${WHEEL_YAW_SCALE:-1.96} -p linear_odom_scale:=${LINEAR_ODOM_SCALE:-0.976} -p left_odom_scale:=${LEFT_ODOM_SCALE:-0.980} -p right_odom_scale:=${RIGHT_ODOM_SCALE:-1.000} -p publish_tf:=false --log-level info"
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
  "--ros-args -p channel_type:=serial -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=460800 -p frame_id:=laser -p inverted:=true -p angle_compensate:=true -p scan_mode:=Standard --log-level info"
sleep 1
start_node \
  "laser_tf" \
  "ros2" \
  "run tf2_ros static_transform_publisher --x 0.04 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id laser"
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

run_ros_cli "lifecycle set /slam_toolbox configure" >/dev/null 2>&1 || true
sleep 1
run_ros_cli "lifecycle set /slam_toolbox activate" >/dev/null 2>&1 || true
sleep 1

echo "ros slam stack started"
echo "pid dir: $PID_DIR"
echo "logs:"
echo "  $RUNTIME_LOG_DIR/ros_slam_base.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_bridge.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_mux.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_joy.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_joystick.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_lidar.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_laser_tf.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_ekf.log"
echo "  $RUNTIME_LOG_DIR/ros_slam_slam.log"
for pid_file in "$PID_DIR"/slam_*.pid; do
  [ -f "$pid_file" ] || continue
  node_name="$(basename "$pid_file" .pid)"
  pid="$(cat "$pid_file")"
  echo "  $node_name: $pid"
done
