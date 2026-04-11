#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export SCRIPT_DIR
export PROJECT_DIR

# Keep ROS discovery/network settings consistent across the robot and the
# remote Ubuntu VM used for RViz. These values intentionally pin the last
# known-good cross-machine setup and ignore any stale shell exports unless an
# explicit *_OVERRIDE variable is provided for a one-off test.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_OVERRIDE:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE_OVERRIDE:-SUBNET}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_OVERRIDE:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_OVERRIDE:-rmw_fastrtps_cpp}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS_OVERRIDE:-192.168.2.11}"
