#!/bin/bash

set -e

source "$(dirname "$0")/_project_env.sh"

bash "$SCRIPT_DIR/stop_ros_nav_stack.sh"
sleep 1
bash "$SCRIPT_DIR/start_ros_nav_stack.sh"
sleep 1
bash "$SCRIPT_DIR/status_ros_nav_stack.sh"
