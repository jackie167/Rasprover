#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/ws/ugv_rpi/ros2_ws"
INSTALL_DIR="${ROOT_DIR}/install"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="/home/ws/ugv_rpi/runtime_logs/nav_build_watch_${STAMP}.log"

mkdir -p /home/ws/ugv_rpi/runtime_logs

TARGETS=(
  nav2_behaviors
  nav2_controller
  nav2_planner
  nav2_bt_navigator
  nav2_navfn_planner
  nav2_regulated_pure_pursuit_controller
)

echo "watch log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "started_at: $(date --iso-8601=seconds)" | tee -a "${LOG_FILE}"

while true; do
  {
    echo
    echo "== $(date --iso-8601=seconds) =="
    if ps -ef | grep -E 'colcon|cmake --build|make -j|ninja' | grep -v grep >/dev/null 2>&1; then
      echo "build_process: running"
    else
      echo "build_process: stopped"
    fi

    missing=0
    for pkg in "${TARGETS[@]}"; do
      if [ -d "${INSTALL_DIR}/${pkg}" ] && [ -n "$(find "${INSTALL_DIR}/${pkg}" -mindepth 1 -maxdepth 3 2>/dev/null | head -n 1)" ]; then
        echo "${pkg}: installed"
      else
        echo "${pkg}: missing"
        missing=1
      fi
    done

    if [ "${missing}" -eq 0 ]; then
      echo "status: complete"
      exit 0
    fi
    echo "status: waiting"
  } | tee -a "${LOG_FILE}"

  sleep 300
done
