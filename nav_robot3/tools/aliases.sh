#!/usr/bin/env bash
# nav_robot3 — 레일 + Nav2 (nav_robot2와 병행 시 domain 104)
#
#   source ~/git/Rokey_Co3_A1/nav_robot3/tools/aliases.sh
#   t1 | t2 | t3

export NAV_ROBOT3_WS="${NAV_ROBOT3_WS:-$HOME/git/Rokey_Co3_A1/nav_robot3}"
export ROS_DOMAIN_ID=104
export NAV_ROBOT3_ROS_DOMAIN_ID=104
export NAV_ROBOT3_ISAAC_PYTHON="${NAV_ROBOT3_ISAAC_PYTHON:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh}"

_nav3_ws() {
  export ROS_DOMAIN_ID=104
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
  export ROS_DOMAIN_ID=104
  export NAV_ROBOT3_ROS_DOMAIN_ID=104
  cd "$NAV_ROBOT3_WS" || return 1
  [[ -x "$NAV_ROBOT3_ISAAC_PYTHON" ]] || {
    echo "[t1] Isaac python 없음: $NAV_ROBOT3_ISAAC_PYTHON" >&2
    return 1
  }
  echo "[t1] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (nav_robot3)"
  exec "$NAV_ROBOT3_ISAAC_PYTHON" isaacpjt/restaurant_nova_demo.py
}

t2() {
  _nav3_ws || return 1
  echo "[t2] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ./tools/kill_nav2.sh
  exec ros2 launch nova_rails nav2.launch.py \
    "map:=$NAV_ROBOT3_WS/maps/restaurant/map.yaml"
}

t3() {
  _nav3_ws || return 1
  (($# == 0)) && set -- --table-id 0
  echo "[t3] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  레일 미션 (rail_mission)"
  ros2 run nova_rails rail_mission "$@"
  local rc=$?
  ((rc != 0)) && echo "[t3] 실패 exit $rc" >&2
  return "$rc"
}
