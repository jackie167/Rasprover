#!/usr/bin/env bash
set -euo pipefail

# Root convenience wrapper for the low-resource build helper in tools/.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec "${SCRIPT_DIR}/tools/build/safe_colcon_build.sh" "$@"
