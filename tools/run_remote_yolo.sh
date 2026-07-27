#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS setup files may read optional variables before defining them. Temporarily
# disable nounset while sourcing, then restore the script's strict mode.
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
  echo "빌드된 워크스페이스가 없습니다: $workspace_root/install/setup.bash" >&2
  exit 1
fi
source "$workspace_root/install/setup.bash"
set -u

if [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]]; then
  echo "ROS_LOCALHOST_ONLY=1이면 다른 컴퓨터의 카메라를 받을 수 없습니다." >&2
  echo "export ROS_LOCALHOST_ONLY=0 후 다시 실행하십시오." >&2
  exit 1
fi

exec ros2 launch serving_robot_manager remote_hand_detection.launch.py "$@"
