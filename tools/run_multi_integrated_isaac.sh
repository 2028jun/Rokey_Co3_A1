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
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$bridge_lib"
cd "$workspace_root"
exec "$isaac_python" nav_robot/isaacpjt/nav_restaurant_demo.py "$@"
