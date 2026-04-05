#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$PROJECT_DIR/ros2_ws"
LOG_DIR="$PROJECT_DIR/.roslog"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"
CV_LOG_FILE="$RUNTIME_LOG_DIR/ros_cv.log"

mkdir -p "$LOG_DIR" "$PID_DIR" "$RUNTIME_LOG_DIR"

"$PROJECT_DIR/start_ros_motion_stack.sh"

pkill -f "/rasprover_cv/lib/rasprover_cv/cv_node" || true
sleep 1

setsid bash -lc "
  export ROS_LOG_DIR='$LOG_DIR'
  export ROS_LOCALHOST_ONLY=1
  export PYTHONPATH='$PROJECT_DIR'
  export PROJECT_DIR='$PROJECT_DIR'
  source '$ROS_WS/install/setup.bash'
  exec '$ROS_WS/install/rasprover_cv/lib/rasprover_cv/cv_node' --ros-args -p port:=5051 --log-level info
" > "$CV_LOG_FILE" 2>&1 < /dev/null &

CV_PID=$!
echo "$CV_PID" > "$PID_DIR/cv.pid"

echo "ros full stack started"
echo "cv: $CV_PID"
echo "cv log: $CV_LOG_FILE"
