#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release}"
isaac_python="$isaac_root/python.sh"
bridge_lib="$isaac_root/exts/isaacsim.ros2.bridge/humble/lib"

if [[ ! -x "$isaac_python" ]]; then
  echo "Isaac Python을 찾을 수 없습니다: $isaac_python" >&2
  exit 1
fi

export NAV_MULTI_ROBOT=1
# Keep the full 1280x960 camera image, but publish it at 15 Hz. Depth is not
# consumed by the current hand-safety stack and costs another camera helper.
export NAV_CAMERA_WIDTH="${NAV_CAMERA_WIDTH:-1280}"
export NAV_CAMERA_HEIGHT="${NAV_CAMERA_HEIGHT:-960}"
export NAV_CAMERA_FRAME_SKIP="${NAV_CAMERA_FRAME_SKIP:-3}"
export NAV_CAMERA_DEPTH_ENABLED="${NAV_CAMERA_DEPTH_ENABLED:-0}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$bridge_lib"
cd "$workspace_root"
exec "$isaac_python" nav_robot/isaacpjt/nav_restaurant_demo.py "$@"
