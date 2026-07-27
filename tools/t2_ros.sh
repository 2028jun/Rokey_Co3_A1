#!/usr/bin/env bash
set -eo pipefail
WS="${ROKEY_CO3_MULTI_WS:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$WS"
# ROS setup.bash references unbound vars; do not use set -u around source.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
exec ros2 launch serving_robot_manager multi_robot.launch.py "$@"
