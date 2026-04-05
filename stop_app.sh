#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_PATTERN="legacy_runtime.app_main"
APP_SHIM="$PROJECT_DIR/app.py"

pkill -f "$APP_PATTERN" || true
pkill -f "$APP_SHIM" || true
sleep 1
pgrep -af "$APP_PATTERN" >/dev/null && pkill -9 -f "$APP_PATTERN" || true
pgrep -af "$APP_SHIM" >/dev/null && pkill -9 -f "$APP_SHIM" || true

echo "legacy runtime stopped"
pgrep -af "$APP_PATTERN" || echo "No legacy runtime process found"
