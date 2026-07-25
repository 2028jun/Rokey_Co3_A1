#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="${ISAAC_SIM_ROOT:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release}"
ROS2_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/humble/lib"

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ROS2_LIB}"

exec "${ISAAC_ROOT}/python.sh" "$(dirname "$0")/moveit_bridge_sim.py" "$@"
