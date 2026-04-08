#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

cd "$PROJECT_DIR"
bash ./restart_ros_path_test_stack.sh
sleep 2
bash ./status_ros_path_test_stack.sh
