#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${ISAAC_SIM_ROOT:-}" ]]; then
  echo "ISAAC_SIM_ROOT가 설정되어 있지 않습니다. export ISAAC_SIM_ROOT=/path/to/isaac_sim/isaacsim/_build/linux-x86_64/release 실행 후 다시 시도하세요." >&2
  exit 1
fi
isaac_root="$ISAAC_SIM_ROOT"
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
# Keep the current serving-stable docking window as the no-argument default.
# Environment overrides remain available for controlled tuning.
export NAV_DOCK_XY_TOLERANCE_M="${NAV_DOCK_XY_TOLERANCE_M:-0.025}"
export NAV_DOCK_YAW_TOLERANCE_DEG="${NAV_DOCK_YAW_TOLERANCE_DEG:-1.0}"
export NAV_DOCK_ALIGNED_FORWARD_TOLERANCE_M="${NAV_DOCK_ALIGNED_FORWARD_TOLERANCE_M:-0.045}"
export NAV_DOCK_ALIGNED_LATERAL_TOLERANCE_M="${NAV_DOCK_ALIGNED_LATERAL_TOLERANCE_M:-0.025}"
export NAV_DOCK_FINAL_MAX_SPEED_MPS="${NAV_DOCK_FINAL_MAX_SPEED_MPS:-0.18}"
export NAV_DOCK_FINAL_MIN_SPEED_MPS="${NAV_DOCK_FINAL_MIN_SPEED_MPS:-0.035}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$bridge_lib"
cd "$workspace_root"
exec "$isaac_python" isaacpjt/nav_restaurant_demo.py "$@"
