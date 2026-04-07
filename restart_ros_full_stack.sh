#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

"$PROJECT_DIR/stop_ros_full_stack.sh"
sleep 1
"$PROJECT_DIR/start_ros_full_stack.sh"
