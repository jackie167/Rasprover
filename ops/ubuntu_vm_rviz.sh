#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"

if [ ! -f "$ROS_DISTRO_SETUP" ]; then
  echo "ROS setup not found: $ROS_DISTRO_SETUP" >&2
  exit 1
fi

source "$ROS_DISTRO_SETUP"
source "$SCRIPT_DIR/_project_env.sh"

ros2 daemon stop >/dev/null 2>&1 || true

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

exec rviz2
