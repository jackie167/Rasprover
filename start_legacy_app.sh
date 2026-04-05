#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[legacy] starting fallback legacy_runtime runtime"
echo "[legacy] preferred runtime is ./start_ros_full_stack.sh"

"$PROJECT_DIR/restart_app.sh"
