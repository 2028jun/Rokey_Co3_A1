#!/usr/bin/env bash
# nav_robot5 — 2륜 로봇 레일 미션 (기본 t1 / t2 / t3)
#
#   source ~/git/Rokey_Co3_A1/tools/aliases.sh
#   또는 source ~/git/Rokey_Co3_A1/nav_robot5/tools/aliases.sh
#
#   터미널1: t1
#   터미널2: t2
#   터미널3: t3 --table-id 2

_NAV_ROBOT5_TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export NAV_ROBOT5_WS="${NAV_ROBOT5_WS:-$(dirname "$_NAV_ROBOT5_TOOLS_DIR")}"
export NAV_ROBOT5_ISAAC_PYTHON="${NAV_ROBOT5_ISAAC_PYTHON:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh}"

# Older workspace scripts may already have t1/t2/t3 aliases in an interactive
# shell. Bash expands those aliases while parsing `t1()`, causing a misleading
# "unexpected token (`" error before the function can replace them.
unalias t1 t2 t3 2>/dev/null || true

_nav5_ws() {
  if [[ ! -f "$NAV_ROBOT5_WS/install/setup.bash" ]]; then
    echo "[nav5] install 없음: cd $NAV_ROBOT5_WS && colcon build --packages-select two_wheel_rails" >&2
    return 1
  fi
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "$NAV_ROBOT5_WS/install/setup.bash"
  cd "$NAV_ROBOT5_WS" || return 1
}

t1() {
  cd "$NAV_ROBOT5_WS" || return 1
  [[ -x "$NAV_ROBOT5_ISAAC_PYTHON" ]] || {
    echo "[t1] Isaac python 없음: $NAV_ROBOT5_ISAAC_PYTHON" >&2
    return 1
  }
  echo "[t1] nav_robot5  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  exec "$NAV_ROBOT5_ISAAC_PYTHON" isaacpjt/restaurant_two_wheel_demo.py
}

t2() {
  _nav5_ws || return 1
  echo "[t2] nav_robot5  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ./tools/kill_nav2.sh
  exec ros2 launch two_wheel_rails nav2.launch.py \
    "map:=$NAV_ROBOT5_WS/maps/restaurant/map.yaml"
}

t3() {
  _nav5_ws || return 1
  (($# == 0)) && set -- --table-id 0
  echo "[t3] nav_robot5  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  rail_mission"
  if ! timeout 4 ros2 topic info /two_wheel/odom_raw 2>/dev/null \
      | grep -q 'Publisher count: 1'; then
    echo "[t3] 중단: nav_robot5 T1이 아닙니다." >&2
    echo "     모든 기존 t1/t2/t3를 Ctrl+C로 끈 뒤, 각 터미널에서 이 aliases.sh를 다시 source하세요." >&2
    return 2
  fi
  if ! timeout 4 ros2 topic info /two_wheel/scan_raw 2>/dev/null \
      | grep -q 'Publisher count: 1'; then
    echo "[t3] 중단: nav_robot5 LiDAR 토픽이 없습니다. T1 시작 완료를 기다리세요." >&2
    return 2
  fi
  if [[ "$(timeout 4 ros2 lifecycle get /bt_navigator 2>/dev/null)" != "active [3]" ]]; then
    echo "[t3] 중단: nav_robot5 Nav2가 active가 아닙니다. 새 aliases로 t2를 다시 실행하세요." >&2
    return 2
  fi
  ros2 run two_wheel_rails rail_mission "$@"
  local rc=$?
  ((rc != 0)) && echo "[t3] 실패 exit $rc" >&2
  return "$rc"
}
