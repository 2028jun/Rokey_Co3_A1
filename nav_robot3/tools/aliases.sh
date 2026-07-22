#!/usr/bin/env bash
# nav_robot3 — Nova Carter 레일 미션 (기본 t1 / t2 / t3)
#
#   source ~/git/Rokey_Co3_A1/tools/aliases.sh
#   또는 source ~/git/Rokey_Co3_A1/nav_robot3/tools/aliases.sh
#
#   터미널1: t1
#   터미널2: t2
#   터미널3: t3 --table-id 2

export NAV_ROBOT3_WS="${NAV_ROBOT3_WS:-$HOME/git/Rokey_Co3_A1/nav_robot3}"
export ROS_DOMAIN_ID=103
export NAV_ROBOT3_ROS_DOMAIN_ID=103
export NAV_ROBOT3_ISAAC_PYTHON="${NAV_ROBOT3_ISAAC_PYTHON:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh}"

_nav3_ws() {
  export ROS_DOMAIN_ID=103
  export NAV_ROBOT3_ROS_DOMAIN_ID=103
  if [[ ! -f "$NAV_ROBOT3_WS/install/setup.bash" ]]; then
    echo "[nav3] install 없음: cd $NAV_ROBOT3_WS && colcon build --packages-select nova_rails" >&2
    return 1
  fi
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "$NAV_ROBOT3_WS/install/setup.bash"
  cd "$NAV_ROBOT3_WS" || return 1
}

t1() {
  export ROS_DOMAIN_ID=103
  export NAV_ROBOT3_ROS_DOMAIN_ID=103
  cd "$NAV_ROBOT3_WS" || return 1
  [[ -x "$NAV_ROBOT3_ISAAC_PYTHON" ]] || {
    echo "[t1] Isaac python 없음: $NAV_ROBOT3_ISAAC_PYTHON" >&2
    return 1
  }
  echo "[t1] nav_robot3  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  exec "$NAV_ROBOT3_ISAAC_PYTHON" isaacpjt/restaurant_nova_demo.py
}

t2() {
  _nav3_ws || return 1
  echo "[t2] nav_robot3  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ./tools/kill_nav2.sh
  exec ros2 launch nova_rails nav2.launch.py \
    "map:=$NAV_ROBOT3_WS/maps/restaurant/map.yaml"
}

t3() {
  _nav3_ws || return 1
  (($# == 0)) && set -- --table-id 0
  echo "[t3] nav_robot3  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  rail_mission"
  if ! ros2 node list 2>/dev/null | grep -q bt_navigator; then
    echo "[t3] 경고: bt_navigator 없음 — t2를 먼저 띄우세요." >&2
  fi
  ros2 run nova_rails rail_mission "$@"
  local rc=$?
  ((rc != 0)) && echo "[t3] 실패 exit $rc" >&2
  return "$rc"
}
