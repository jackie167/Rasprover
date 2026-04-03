#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_PATH="$PROJECT_DIR/app.py"

pkill -f "$APP_PATH" || true
sleep 1
pgrep -af "$APP_PATH" >/dev/null && pkill -9 -f "$APP_PATH" || true

echo "app.py stopped"
pgrep -af "$APP_PATH" || echo "No app.py process found"
