#!/usr/bin/env bash
# Engage / release base hold after docking (software servo hold via hold_node).
#
# Terminal A:
#   cd ~/git/Rokey_Co3_A1/base_hold && source /opt/ros/humble/setup.bash
#   colcon build --packages-select base_hold && source install/setup.bash
#   ros2 run base_hold hold_node
#
# Then:
set -euo pipefail

CMD="${1:-on}"

case "$CMD" in
  on|true|1|engage)
    echo "[base_hold] ENGAGE (ARM_HOLD)"
    ros2 service call /base/hold std_srvs/srv/SetBool "{data: true}"
    ;;
  off|false|0|release)
    echo "[base_hold] RELEASE (BASE_MOVING)"
    ros2 service call /base/hold std_srvs/srv/SetBool "{data: false}"
    ;;
  *)
    echo "usage: $0 [on|off]" >&2
    exit 2
    ;;
esac

echo "[base_hold] state:"
timeout 2 ros2 topic echo /base/hold_state --once || true
