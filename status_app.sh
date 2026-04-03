#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_PATH="$PROJECT_DIR/app.py"
LOG_FILE="$HOME/ugv.log"

echo "== app.py processes =="
pgrep -af "$APP_PATH" || echo "No app.py process found"
echo

echo "== serial users =="
serial_found=0
for device in /dev/ttyAMA0 /dev/serial0; do
    if [ -e "$device" ]; then
        if lsof "$device" 2>/dev/null; then
            serial_found=1
        fi
    fi
done
if [ "$serial_found" -eq 0 ]; then
    echo "No serial device currently open"
fi
echo

echo "== recent log =="
if [ -f "$LOG_FILE" ]; then
    tail -n 20 "$LOG_FILE"
else
    echo "Log file not found: $LOG_FILE"
fi
