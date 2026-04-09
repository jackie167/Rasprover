#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_DIR="$PROJECT_DIR/.ros_nav_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

echo "== nav node pids =="
for name in nav_base nav_bridge nav_filter nav_mux nav_joy nav_joystick nav_lidar nav_laser_tf nav_slam nav_nav nav_nav_localization nav_nav_navigation nav_explore; do
  pid_file="$PID_DIR/$name.pid"
  if [ -f "$pid_file" ]; then
    pid="$(cat "$pid_file")"
    if kill -0 "$pid" 2>/dev/null; then
      echo "$name: $pid"
    else
      echo "$name: stale pid ($pid)"
    fi
  else
    echo "$name: not started"
  fi
done
echo

echo "== nav processes =="
pgrep -af "navigation_stack.launch.py|frontier_explorer.launch.py|frontier_explorer_node|nav2_map_server|nav2_amcl|nav2_controller|nav2_planner|nav2_behaviors|nav2_bt_navigator|nav2_lifecycle_manager|nav_cmd_vel_bridge_node|simple_odom_filter_node|slam_toolbox|rplidar_node|robot_base_node|slam_sensor_bridge_node|command_mux_node|joystick_bridge_node|local_joy_node" || echo "No nav stack nodes found"
echo

echo "== nav logs =="
for name in base bridge filter mux joy joystick lidar laser_tf slam nav nav_localization nav_navigation explore; do
  log_file="$RUNTIME_LOG_DIR/ros_nav_${name}.log"
  echo "-- $name --"
  if [ -f "$log_file" ]; then
    tail -n 20 "$log_file"
  else
    echo "Log file not found: $log_file"
  fi
done
echo

echo "== nav health =="
nav_running=0
if pgrep -f "navigation_stack.launch.py|nav2_map_server|nav2_amcl|nav2_controller|nav2_planner|nav2_behaviors|nav2_bt_navigator|nav2_lifecycle_manager" >/dev/null 2>&1; then
  nav_running=1
fi

if [ "$nav_running" -eq 1 ] && { \
   grep -q "Managed nodes are active\|Server bt_navigator connected with bond\|Server controller_server connected with bond" "$RUNTIME_LOG_DIR/ros_nav_nav_navigation.log" 2>/dev/null || \
   grep -q "Managed nodes are active\|Server bt_navigator connected with bond\|Server controller_server connected with bond" "$RUNTIME_LOG_DIR/ros_nav_nav.log" 2>/dev/null; \
}; then
  echo "nav bringup: OK"
elif [ "$nav_running" -eq 1 ] && grep -q "waiting for AMCL initial pose\|timed out waiting for AMCL initial pose" "$RUNTIME_LOG_DIR/ros_nav_nav_localization.log" 2>/dev/null; then
  echo "nav bringup: waiting for initial pose"
else
  echo "nav bringup: DOWN or missing"
fi
if pgrep -f "/rplidar_ros/lib/rplidar_ros/rplidar_node" >/dev/null 2>&1; then
  echo "lidar: OK"
else
  echo "lidar: DOWN"
fi
if pgrep -f "/rasprover_sensors/lib/rasprover_sensors/slam_sensor_bridge_node" >/dev/null 2>&1; then
  echo "bridge: OK"
else
  echo "bridge: DOWN"
fi
if pgrep -f "simple_odom_filter_node" >/dev/null 2>&1; then
  echo "odom_filter: OK"
else
  echo "odom_filter: DOWN"
fi
if [ -f "$RUNTIME_LOG_DIR/ros_nav_slam.log" ]; then
  if pgrep -f "/slam_toolbox/lib/slam_toolbox/async_slam_toolbox_node" >/dev/null 2>&1; then
    echo "slam_toolbox: OK"
  elif grep -q "Activating" "$RUNTIME_LOG_DIR/ros_nav_slam.log" 2>/dev/null; then
    echo "slam_toolbox: activating"
  else
    echo "slam_toolbox: DOWN"
  fi
fi
if [ -f "$RUNTIME_LOG_DIR/ros_nav_explore.log" ]; then
  if grep -q "frontier_explorer_node" "$RUNTIME_LOG_DIR/ros_nav_explore.log" 2>/dev/null; then
    echo "explorer: OK"
  else
    echo "explorer: check log"
  fi
else
  echo "explorer: OFF"
fi
echo "tf_source_expected: simple_odom_filter"
