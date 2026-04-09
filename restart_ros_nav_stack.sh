#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

bash "$PROJECT_DIR/stop_ros_nav_stack.sh"
sleep 1
bash "$PROJECT_DIR/start_ros_nav_stack.sh"
sleep 1
bash "$PROJECT_DIR/status_ros_nav_stack.sh"
