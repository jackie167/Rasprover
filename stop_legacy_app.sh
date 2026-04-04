#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[legacy] stopping fallback app.py runtime"

"$PROJECT_DIR/stop_app.sh"
