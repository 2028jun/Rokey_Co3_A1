#!/usr/bin/env bash
# 2륜 로봇 레일 미션 진단 하네스 (기본 t1 / t2 / t3) — 실행 순서 문서화는
# docs/TWO_WHEEL_RAILS_DIAGNOSTIC.md 참고. 4-터미널 프로덕션 실행 흐름과는
# 별개의 단일 로봇 진단 도구입니다.
#
#   source <워크스페이스 경로>/tools/aliases.sh
#
#   터미널1: t1
#   터미널2: t2
#   터미널3: t3 --table-id 2

_NAV_ROBOT5_TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PROJECT_WS="${PROJECT_WS:-$(dirname "$_NAV_ROBOT5_TOOLS_DIR")}"
if [[ -z "${ISAAC_SIM_ROOT:-}" ]]; then
  echo "ISAAC_SIM_ROOT가 설정되어 있지 않습니다. export ISAAC_SIM_ROOT=/path/to/isaac_sim/isaacsim/_build/linux-x86_64/release 실행 후 다시 시도하세요." >&2
  return 1 2>/dev/null || exit 1
fi
export NAV_ROBOT5_ISAAC_PYTHON="$ISAAC_SIM_ROOT/python.sh"

# Older workspace scripts may already have t1/t2/t3 aliases in an interactive
# shell. Bash expands those aliases while parsing `t1()`, causing a misleading
# "unexpected token (`" error before the function can replace them.
unalias t1 t2 t3 2>/dev/null || true

_nav5_ws() {
  if [[ ! -f "$PROJECT_WS/install/setup.bash" ]]; then
    echo "[nav5] install 없음: cd $PROJECT_WS && colcon build --packages-select two_wheel_rails" >&2
    return 1
  fi
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "$PROJECT_WS/install/setup.bash"
  cd "$PROJECT_WS" || return 1
}

t1() {
  cd "$PROJECT_WS" || return 1
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
    "map:=$PROJECT_WS/maps/restaurant/map.yaml"
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
