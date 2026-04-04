#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYTHON_BIN="$PROJECT_DIR/ugv-env/bin/python"
APP_CMD="$PYTHON_BIN $PROJECT_DIR/app.py"
LOG_FILE="$HOME/ugv.log"
PYTHONPATH_PREFIX="$PROJECT_DIR"

# Kill only this project's app.py processes, then wait for the port/device user to exit.
pkill -f "$PROJECT_DIR/app.py" || true
sleep 1

# If anything is still around, force kill it.
pgrep -af "$PROJECT_DIR/app.py" >/dev/null && pkill -9 -f "$PROJECT_DIR/app.py" || true
sleep 1

nohup env PYTHONPATH="$PYTHONPATH_PREFIX" "$PYTHON_BIN" "$PROJECT_DIR/app.py" > "$LOG_FILE" 2>&1 &

echo "app.py restarted"
echo "log: $LOG_FILE"
pgrep -af "$PROJECT_DIR/app.py" || true
