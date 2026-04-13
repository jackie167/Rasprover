#!/bin/bash

source "$(dirname "$0")/_project_env.sh"
PID_DIR="$PROJECT_DIR/.ros_selector_pids"

stop_pid_file() {
  local pid_file="$1"
  [ -f "$pid_file" ] || return 0
  local pid
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file"
}

stop_pid_file "$PID_DIR/base.pid"
if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  stop_pid_file "$PID_DIR/web.pid"
fi

pkill -f "/rasprover_base/lib/rasprover_base/robot_base_node" || true
if [ "${KEEP_WEB_BRIDGE:-false}" != "true" ]; then
  pkill -f "/rasprover_ui/lib/rasprover_ui/web_bridge_node" || true
fi

echo "ros mode selector stopped"
