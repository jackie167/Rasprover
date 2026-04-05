#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/ws/ugv_rpi"
WS_DIR="${PROJECT_DIR}/ros2_ws"
LOG_DIR="${PROJECT_DIR}/build_logs"

mkdir -p "${LOG_DIR}"

usage() {
  cat <<'EOF'
Usage:
  ./safe_colcon_build.sh <package> [<package> ...]

What it does:
  - builds with a single worker
  - lowers CPU and I/O priority
  - writes a timestamped log file
  - warns if ROS stack processes are still running

Example:
  ./safe_colcon_build.sh diagnostic_updater geographic_msgs robot_localization
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

if pgrep -af 'ros2 run|/rasprover_|/robot_localization/|/slam_toolbox/' >/dev/null 2>&1; then
  echo "[warn] ROS processes are still running."
  echo "[warn] To reduce load and serial/log spam, stop them first with:"
  echo "       bash ./stop_ros_full_stack.sh"
  echo
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/colcon_safe_${TIMESTAMP}.log"

echo "[safe_colcon] workspace=${WS_DIR}"
echo "[safe_colcon] packages=$*"
echo "[safe_colcon] log=${LOG_FILE}"
echo "[safe_colcon] mode=sequential parallel_workers=1 makeflags=-j1"

cd "${WS_DIR}"

export MAKEFLAGS="-j1"
export CMAKE_BUILD_PARALLEL_LEVEL=1

CMD=(
  colcon build
  --executor sequential
  --parallel-workers 1
  --event-handlers console_direct+
  --packages-select
)

for pkg in "$@"; do
  CMD+=("${pkg}")
done

{
  echo "[safe_colcon] command=${CMD[*]}"
  nice -n 15 ionice -c3 "${CMD[@]}"
} 2>&1 | tee "${LOG_FILE}"
