#!/usr/bin/env bash
# nav_robot5 — 2륜 로봇 레일 미션, 2대 운영 (t1 / t2r / t3r)
#
#   source ~/git/Rokey_Co3_A1/tools/aliases.sh
#   또는 source ~/git/Rokey_Co3_A1/nav_robot5/tools/aliases.sh
#
# 개별 주행 검증 (동시 동작 아님 — 로봇1, 로봇2를 각각 확인):
#   터미널1: t1                          (Isaac, robot1+robot2 동시 스폰)
#   터미널2: t2r robot1                  (robot1 Nav2/AMCL)
#   터미널3: t2r robot2 rviz:=false      (robot2 Nav2/AMCL, RViz 창 1개만)
#   터미널4: t3r robot1 --table-id 0     (robot1 주행 확인; robot2는 정지)
#            t3r robot2 --table-id 1    (이후 robot2 주행 확인; robot1은 정지)
#
# 단일 로봇 시절 호환용: t2 / t3 는 각각 t2r robot1 / t3r robot1 의 별칭.

_NAV_ROBOT5_TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export NAV_ROBOT5_WS="${NAV_ROBOT5_WS:-$(dirname "$_NAV_ROBOT5_TOOLS_DIR")}"
export NAV_ROBOT5_ISAAC_PYTHON="${NAV_ROBOT5_ISAAC_PYTHON:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh}"

# Older workspace scripts may already have t1/t2/t3 aliases in an interactive
# shell. Bash expands those aliases while parsing `t1()`, causing a misleading
# "unexpected token (`" error before the function can replace them.
unalias t1 t2 t3 t2r t3r 2>/dev/null || true

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

t2r() {
  # Unlike the old single-robot t2, this does NOT auto-run kill_nav2.sh:
  # that script pkills every Nav2 node by name regardless of namespace, so
  # restarting robot1's stack would also kill robot2's. Run
  # ./tools/kill_nav2.sh manually for a full two-robot reset instead.
  _nav5_ws || return 1
  local robot_id="$1"
  if [[ "$robot_id" != "robot1" && "$robot_id" != "robot2" ]]; then
    echo "[t2r] 사용법: t2r robot1|robot2 [rviz:=false] [추가 launch 인자...]" >&2
    return 2
  fi
  shift
  echo "[t2r] nav_robot5  robot_id=$robot_id  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  exec ros2 launch two_wheel_rails nav2.launch.py \
    "namespace:=$robot_id" \
    "robot_urdf:=$NAV_ROBOT5_WS/src/two_wheel_rails/urdf/two_wheel_robot_${robot_id}.urdf" \
    "map:=$NAV_ROBOT5_WS/maps/restaurant/map.yaml" \
    "$@"
}

t3r() {
  local robot_id="$1"
  if [[ "$robot_id" != "robot1" && "$robot_id" != "robot2" ]]; then
    echo "[t3r] 사용법: t3r robot1|robot2 [--table-id N] [rail_mission 추가 인자...]" >&2
    return 2
  fi
  shift
  _nav5_ws || return 1
  (($# == 0)) && set -- --table-id 0
  echo "[t3r] nav_robot5  robot_id=$robot_id  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  rail_mission"
  if ! timeout 4 ros2 topic info "/${robot_id}/two_wheel/odom_raw" 2>/dev/null \
      | grep -q 'Publisher count: 1'; then
    echo "[t3r] 중단: ${robot_id} T1 스폰이 안 보입니다." >&2
    echo "     t1이 두 로봇을 모두 스폰했는지, 새 aliases로 t2r ${robot_id}를 실행했는지 확인하세요." >&2
    return 2
  fi
  if ! timeout 4 ros2 topic info "/${robot_id}/two_wheel/scan_raw" 2>/dev/null \
      | grep -q 'Publisher count: 1'; then
    echo "[t3r] 중단: ${robot_id} LiDAR 토픽이 없습니다. T1 시작 완료를 기다리세요." >&2
    return 2
  fi
  if [[ "$(timeout 4 ros2 lifecycle get "/${robot_id}/bt_navigator" 2>/dev/null)" != "active [3]" ]]; then
    echo "[t3r] 중단: ${robot_id} Nav2가 active가 아닙니다. 새 aliases로 t2r ${robot_id}를 다시 실행하세요." >&2
    return 2
  fi
  # tf2_ros.TransformListener hardcodes the absolute /tf,/tf_static topics;
  # remap them to relative tf/tf_static so __ns:=/<robot_id> actually routes
  # this process onto that robot's TF stream (same fix nav2_bringup applies
  # to amcl/costmap/etc internally).
  ros2 run two_wheel_rails rail_mission --robot-id "$robot_id" "$@" \
    --ros-args -r "__ns:=/${robot_id}" -r /tf:=tf -r /tf_static:=tf_static
  local rc=$?
  ((rc != 0)) && echo "[t3r] 실패 exit $rc" >&2
  return "$rc"
}

# Single-robot-era compatibility: t2/t3 default to robot1.
t2() { t2r robot1 "$@"; }
t3() { t3r robot1 "$@"; }
