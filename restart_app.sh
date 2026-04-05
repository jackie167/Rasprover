#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYTHON_BIN="$PROJECT_DIR/ugv-env/bin/python"
APP_PATTERN="legacy_runtime.app_main"
LOG_FILE="$HOME/ugv.log"
PYTHONPATH_PREFIX="$PROJECT_DIR"

# Kill only this project's legacy runtime processes, then wait for the port/device user to exit.
pkill -f "$APP_PATTERN" || true
pkill -f "$PROJECT_DIR/app.py" || true
sleep 1

# If anything is still around, force kill it.
pgrep -af "$APP_PATTERN" >/dev/null && pkill -9 -f "$APP_PATTERN" || true
pgrep -af "$PROJECT_DIR/app.py" >/dev/null && pkill -9 -f "$PROJECT_DIR/app.py" || true
sleep 1

nohup env PYTHONPATH="$PYTHONPATH_PREFIX" PROJECT_DIR="$PROJECT_DIR" "$PYTHON_BIN" -m legacy_runtime.app_main > "$LOG_FILE" 2>&1 &

echo "legacy runtime restarted"
echo "log: $LOG_FILE"
pgrep -af "$APP_PATTERN" || true
