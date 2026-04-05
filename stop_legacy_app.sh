#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[legacy] stopping fallback legacy_runtime runtime"

"$PROJECT_DIR/stop_app.sh"
