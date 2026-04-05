#!/bin/bash

set -e

# Root operator helper to restart the primary ROS runtime.
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[ros] restarting full stack"
"$PROJECT_DIR/stop_ros_full_stack.sh"
sleep 1
"$PROJECT_DIR/start_ros_full_stack.sh"
