#!/usr/bin/env bash
# nav_robot2 — 터미널별 단축 명령 (한 번만 source)
#
#   source ~/git/Rokey_Co3_A1/nav_robot2/tools/aliases.sh
#
#   터미널1: t1
#   터미널2: t2
#   터미널3: t3

export NAV_ROBOT2_WS="${NAV_ROBOT2_WS:-$HOME/git/Rokey_Co3_A1/nav_robot2}"
# bashrc의 101 등을 덮어씀 — Isaac/Nav2/미션이 같은 도메인이어야 함
export ROS_DOMAIN_ID=103
export NAV_ROBOT2_ROS_DOMAIN_ID=103
export NAV_ROBOT2_ISAAC_PYTHON="${NAV_ROBOT2_ISAAC_PYTHON:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh}"

_nav2_ws() {
  export ROS_DOMAIN_ID=103
  if [[ ! -f "$NAV_ROBOT2_WS/install/setup.bash" ]]; then
    echo "[nav2] install 없음. 먼저: cd $NAV_ROBOT2_WS && colcon build --packages-select nova_carter" >&2
    return 1
  fi
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "$NAV_ROBOT2_WS/install/setup.bash"
  cd "$NAV_ROBOT2_WS" || return 1
}

t1() {
  export ROS_DOMAIN_ID=103
  export NAV_ROBOT2_ROS_DOMAIN_ID=103
  cd "$NAV_ROBOT2_WS" || return 1
  if [[ ! -x "$NAV_ROBOT2_ISAAC_PYTHON" ]]; then
    echo "[t1] Isaac python 없음: $NAV_ROBOT2_ISAAC_PYTHON" >&2
    return 1
  fi
  echo "[t1] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  exec "$NAV_ROBOT2_ISAAC_PYTHON" isaacpjt/restaurant_nova_demo.py
}

t2() {
  _nav2_ws || return 1
  echo "[t2] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ./tools/kill_nav2.sh
  exec ros2 launch nova_carter nav2.launch.py \
    "map:=$NAV_ROBOT2_WS/maps/restaurant/map.yaml"
}

t3() {
  _nav2_ws || return 1
  if (($# == 0)); then
    set -- --table-id 0
  fi
  echo "[t3] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  (t1·t2와 같아야 함)"
  if ! ros2 node list 2>/dev/null | grep -q bt_navigator; then
    echo "[t3] 경고: bt_navigator 없음 — t2(Nav2)를 먼저 띄우세요." >&2
  fi
  # exec 쓰지 않음: 실패(exit 1)해도 터미널이 닫히지 않게
  ros2 run nova_carter nav_to_pose "$@"
  local rc=$?
  if ((rc != 0)); then
    echo "[t3] 미션 실패 (exit $rc). 위 로그 확인. 터미널은 유지됩니다." >&2
  fi
  return "$rc"
}
