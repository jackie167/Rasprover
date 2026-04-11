#!/bin/bash

set -u

source "$(dirname "$0")/_project_env.sh"

MODE="${1:-slam_noekf}"
ROS_WS="$PROJECT_DIR/ros2_ws"

case "$MODE" in
  slam_noekf)
    LAUNCH_PATTERN="slam_noekf.launch.py"
    REQUIRED_PROCESSES=(
      "/rasprover_base/lib/rasprover_base/robot_base_node"
      "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node"
      "/rasprover_slam/lib/rasprover_slam/simple_odom_filter_node"
      "/rasprover_control/lib/rasprover_control/command_mux_node"
      "/rasprover_control/lib/rasprover_control/local_joy_node"
      "/rasprover_control/lib/rasprover_control/joystick_bridge_node"
      "/rasprover_ui/lib/rasprover_ui/web_bridge_node"
      "/rplidar_ros/lib/rplidar_ros/rplidar_node"
      "/tf2_ros/.*/static_transform_publisher|/tf2_ros/static_transform_publisher"
    )
    ;;
  slam_ekf)
    LAUNCH_PATTERN="slam_ekf.launch.py"
    REQUIRED_PROCESSES=(
      "/rasprover_base/lib/rasprover_base/robot_base_node"
      "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node"
      "/robot_localization/lib/robot_localization/ekf_node"
      "/rasprover_control/lib/rasprover_control/command_mux_node"
      "/rasprover_control/lib/rasprover_control/local_joy_node"
      "/rasprover_control/lib/rasprover_control/joystick_bridge_node"
      "/rasprover_ui/lib/rasprover_ui/web_bridge_node"
      "/rplidar_ros/lib/rplidar_ros/rplidar_node"
      "/tf2_ros/.*/static_transform_publisher|/tf2_ros/static_transform_publisher"
    )
    ;;
  *)
    echo "Usage: bash ./ops/check_slam_health.sh [slam_noekf|slam_ekf]"
    exit 2
    ;;
esac

run_ros_cli() {
  bash -lc "
    export PROJECT_DIR='$PROJECT_DIR'
    export PYTHONPATH='$PROJECT_DIR'
    export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
    export ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE}'
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
    export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
    export ROS_STATIC_PEERS='${ROS_STATIC_PEERS}'
    source '$ROS_WS/install/local_setup.bash'
    $1
  "
}

pass() {
  echo "PASS: $1"
}

warn() {
  echo "WARN: $1"
}

fail() {
  echo "FAIL: $1"
}

check_process() {
  local pattern="$1"
  pgrep -f "$pattern" >/dev/null 2>&1
}

echo "== SLAM Health Check =="
echo "mode: $MODE"
echo "domain_id: $ROS_DOMAIN_ID"
echo "rmw: $RMW_IMPLEMENTATION"
echo

if pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1; then
  pass "launch process found for $MODE"
else
  fail "launch process not found for $MODE"
fi

for pattern in "${REQUIRED_PROCESSES[@]}"; do
  if check_process "$pattern"; then
    pass "process present: $pattern"
  else
    fail "process missing: $pattern"
  fi
done

SLAM_STATE="$(run_ros_cli "ros2 lifecycle get /slam_toolbox" 2>/dev/null | awk 'NR==1 {print $1}')"
if [ "$SLAM_STATE" = "active" ]; then
  pass "slam_toolbox lifecycle is active"
else
  fail "slam_toolbox lifecycle is $SLAM_STATE"
fi

SCAN_INFO="$(run_ros_cli "ros2 topic info /scan" 2>/dev/null || true)"
if printf '%s' "$SCAN_INFO" | grep -q "Publisher count: 1"; then
  pass "/scan has a publisher"
else
  fail "/scan publisher missing"
fi

MAP_INFO="$(run_ros_cli "ros2 topic info /map" 2>/dev/null || true)"
if printf '%s' "$MAP_INFO" | grep -q "Publisher count: 1"; then
  pass "/map has a publisher"
else
  fail "/map publisher missing"
fi

ODOM_INFO="$(run_ros_cli "ros2 topic info /wheel/odometry" 2>/dev/null || true)"
if printf '%s' "$ODOM_INFO" | grep -q "Publisher count: 1"; then
  pass "/wheel/odometry has a publisher"
else
  fail "/wheel/odometry publisher missing"
fi

if run_ros_cli "timeout 6s ros2 run tf2_ros tf2_echo odom base_link" 2>/dev/null | grep -q "At time"; then
  pass "TF chain odom -> base_link is available"
else
  fail "TF chain odom -> base_link is not available"
fi

if run_ros_cli "timeout 6s ros2 run tf2_ros tf2_echo base_link laser" 2>/dev/null | grep -q "At time"; then
  pass "TF chain base_link -> laser is available"
else
  fail "TF chain base_link -> laser is not available"
fi

if run_ros_cli "timeout 8s ros2 service call /slam_toolbox/dynamic_map nav_msgs/srv/GetMap '{}'" 2>/dev/null | grep -q "OccupancyGrid"; then
  pass "slam_toolbox dynamic_map service responds"
else
  fail "slam_toolbox dynamic_map service failed"
fi

SCAN_SAMPLE="$(run_ros_cli "timeout 6s ros2 topic echo /scan --once" 2>/dev/null || true)"
if printf '%s' "$SCAN_SAMPLE" | grep -q "frame_id: laser"; then
  pass "/scan sample received"
else
  fail "/scan sample not received"
fi

if run_ros_cli "timeout 6s ros2 topic hz /scan" 2>/dev/null | grep -q "average rate"; then
  pass "/scan is streaming"
else
  warn "could not measure /scan rate"
fi
