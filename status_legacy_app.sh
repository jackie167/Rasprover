#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[legacy] status for fallback legacy_runtime runtime"
echo "[legacy] primary runtime is ./status_ros_full_stack.sh"

"$PROJECT_DIR/status_app.sh"
