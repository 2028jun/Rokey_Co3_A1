#!/usr/bin/env bash
# map_generate — one-shot slam_toolbox map generation
#
#   source ~/git/Rokey_Co3_multi/map_generate/tools/aliases.sh
#
#   t1 → Play | t2 | t3 | save_map

_MAP_GEN_TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export MAP_GEN_WS="${MAP_GEN_WS:-$(dirname "$_MAP_GEN_TOOLS_DIR")}"
export ROS_DOMAIN_ID="${MAP_GEN_ROS_DOMAIN_ID:-113}"
export MAP_GEN_ROS_DOMAIN_ID="${MAP_GEN_ROS_DOMAIN_ID:-113}"
export MAP_GEN_ISAAC_PYTHON="${MAP_GEN_ISAAC_PYTHON:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh}"

# nav_robot6 등은 alias가 아니라 function — unalias만으로는 안 바뀜
unalias t1 t2 t3 go_table back_kitchen save_map 2>/dev/null || true
unset -f t1 t2 t3 go_table back_kitchen save_map 2>/dev/null || true
export MAP_GEN_ROS_DOMAIN_ID=113
export ROS_DOMAIN_ID=113

_map_gen_ws() {
  export ROS_DOMAIN_ID="${MAP_GEN_ROS_DOMAIN_ID:-113}"
  export MAP_GEN_ROS_DOMAIN_ID="${MAP_GEN_ROS_DOMAIN_ID:-113}"
  if [[ ! -f "$MAP_GEN_WS/install/setup.bash" ]]; then
    echo "[map_gen] install 없음: cd $MAP_GEN_WS && colcon build --packages-select map_gen" >&2
    return 1
  fi
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "$MAP_GEN_WS/install/setup.bash"
  cd "$MAP_GEN_WS" || return 1
}

t1() {
  export ROS_DOMAIN_ID="${MAP_GEN_ROS_DOMAIN_ID:-113}"
  export MAP_GEN_ROS_DOMAIN_ID="${MAP_GEN_ROS_DOMAIN_ID:-113}"
  cd "$MAP_GEN_WS" || return 1
  [[ -x "$MAP_GEN_ISAAC_PYTHON" ]] || {
    echo "[t1] Isaac python 없음: $MAP_GEN_ISAAC_PYTHON" >&2
    return 1
  }
  echo "[t1] map_generate  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  local kitchen="$MAP_GEN_WS/assets/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd"
  if [[ ! -f "$kitchen" ]]; then
    echo "[t1] 주방 에셋 없음 — ./tools/sync_assets.sh 실행" >&2
    "$MAP_GEN_WS/tools/sync_assets.sh" || return 1
  fi
  "$MAP_GEN_ISAAC_PYTHON" isaacpjt/restaurant_two_wheel_demo.py
}

t2() {
  _map_gen_ws || return 1
  echo "[t2] topic_bridge + RSP + slam_toolbox  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ros2 launch map_gen slam_mapping.launch.py "$@"
}

t3() {
  _map_gen_ws || return 1
  if ! timeout 4 ros2 topic info /two_wheel/odom_raw 2>/dev/null \
      | grep -q 'Publisher count: 1'; then
    echo "[t3] 중단: t1(Isaac)이 아닙니다." >&2
    return 2
  fi
  if ! timeout 4 ros2 topic info /scan 2>/dev/null \
      | grep -q 'Publisher count: [1-9]'; then
    echo "[t3] 경고: /scan 없음 — t2(slam_mapping) 확인. 순찰은 계속 시도합니다." >&2
  fi
  echo "[t3] slam_patrol (1회 커버리지)  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ros2 run map_gen slam_patrol
  local rc=$?
  ((rc != 0)) && echo "[t3] 실패 exit $rc" >&2
  return "$rc"
}

save_map() {
  export ROS_DOMAIN_ID="${MAP_GEN_ROS_DOMAIN_ID:-113}"
  echo "[save_map] maps/restaurant/ (기존 파일 유지, 충돌 시 타임스탬프)  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  "$MAP_GEN_WS/tools/save_slam_map.sh" "$@"
}
