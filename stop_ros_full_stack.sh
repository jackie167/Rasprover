#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_DIR="$PROJECT_DIR/.ros_motion_pids"

if [ -f "$PID_DIR/cv.pid" ]; then
  CV_PID="$(cat "$PID_DIR/cv.pid" 2>/dev/null || true)"
  if [ -n "$CV_PID" ] && kill -0 "$CV_PID" 2>/dev/null; then
    kill "$CV_PID" 2>/dev/null || true
    sleep 1
    kill -9 "$CV_PID" 2>/dev/null || true
  fi
  rm -f "$PID_DIR/cv.pid"
fi

pkill -f "/rasprover_cv/lib/rasprover_cv/cv_node" || true

"$PROJECT_DIR/stop_ros_motion_stack.sh"
