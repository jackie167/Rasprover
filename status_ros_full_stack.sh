#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"
RUNTIME_LOG_DIR="$PROJECT_DIR/runtime_logs"

"$PROJECT_DIR/status_ros_motion_stack.sh"
echo
echo "== cv node =="
if [ -f "$PID_DIR/cv.pid" ]; then
  CV_PID="$(cat "$PID_DIR/cv.pid")"
  if kill -0 "$CV_PID" 2>/dev/null; then
    echo "cv: $CV_PID"
  else
    echo "cv: stale pid ($CV_PID)"
  fi
else
  echo "cv: not started"
fi

pgrep -af "/rasprover_cv/lib/rasprover_cv/cv_node" || true
echo
echo "-- cv --"
if [ -f "$RUNTIME_LOG_DIR/ros_cv.log" ]; then
  tail -n 20 "$RUNTIME_LOG_DIR/ros_cv.log"
else
  echo "Log file not found: $RUNTIME_LOG_DIR/ros_cv.log"
fi
