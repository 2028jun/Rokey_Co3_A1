#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release}"

export COBOT3_WS="$SCRIPT_DIR"
export MOBILE_DEMO_TASK=plate_rack
export MOBILE_DEMO_ROS_CAMERA=0
export MOBILE_DEMO_ROS_LIDAR=0
export MOBILE_DEMO_AUTORUN=0
export MOBILE_DEMO_STABILITY_STEPS=0
export PYTHONPATH="${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/humble/lib"

exec "$ISAAC_SIM_ROOT/python.sh" \
    "$SCRIPT_DIR/isaacpjt/mobile_manipulator_demo_test.py"
