#!/usr/bin/env bash
set -eo pipefail
WS="${ROKEY_CO3_MULTI_WS:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$WS"
export NAV_ROBOT_ROS_DOMAIN_ID="${NAV_ROBOT_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-}}"
export NAV_ROBOT5_ROS_DOMAIN_ID="${NAV_ROBOT5_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-}}"
# Do not source ROS in Isaac terminal.
exec ./tools/run_multi_integrated_isaac.sh "$@"
