#!/bin/bash

set -e

source "$(dirname "$0")/_project_env.sh"

"$SCRIPT_DIR/stop_ros_slam_stack.sh"
sleep 1
"$SCRIPT_DIR/start_ros_slam_stack.sh"
