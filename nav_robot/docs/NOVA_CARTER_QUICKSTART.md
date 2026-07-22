# Nova Carter + Occupancy Map + sample_code Nav2

기존 two-wheel 스택과 별도로, **Nova Carter**로 식당 Occupancy Map 주행을 테스트합니다.

| 항목 | 값 |
|------|-----|
| Robot USD | Nucleus `Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd` |
| Domain | `ROS_DOMAIN_ID=103` |
| Isaac topics | `/cmd_vel`, `/chassis/odom`, `/front_2d_lidar/scan` |
| Nav2 topics | `/cmd_vel`, `/odom`, `/scan` (via `nova_carter_topic_bridge`) |
| Frames | `map` → `odom` → `base_link` → `front_2d_lidar` |
| Map | **기존** `maps/restaurant/map.yaml` + `map.pgm` (two-wheel과 동일) |
| RViz | `nav2_nova_carter.rviz` (two-wheel costmap 레이아웃 + `/scan`, sim time stamp) |

---

## 1회 준비

```bash
cd ~/git/Rokey_Co3_A1/nav_robot
bash tools/fetch_nav_assets.sh   # 식당/주방 자산
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=103
```

맵은 이미 있는 `maps/restaurant/map.yaml` / `map.pgm` 을 그대로 씁니다.  
재생성이 필요할 때만 `tools/generate_occupancy_map_isaac.py` 실행 (Isaac와 동시 실행 금지).

---

## 실행 (터미널 3개)

### T1 — Isaac Nova Carter

```bash
cd ~/git/Rokey_Co3_A1/nav_robot
export NAV_ROBOT_ROS_DOMAIN_ID=103
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/nav_nova_carter_demo.py
```

스모크:

```bash
export ROS_DOMAIN_ID=103
source /opt/ros/humble/setup.bash
ros2 topic list | rg 'cmd_vel|chassis/odom|front_2d_lidar'
ros2 topic echo /chassis/odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

### T2 — Nav2 (AMCL + planner + topic bridge)

```bash
cd ~/git/Rokey_Co3_A1/nav_robot
export ROS_DOMAIN_ID=103
source /opt/ros/humble/setup.bash
source install/setup.bash
bash tools/kill_nav2.sh   # 이전 Nav2 정리

ros2 launch nav_robot_bringup nav2_nova_carter.launch.py
# 기본: maps/restaurant/map.yaml + nav2_restaurant.rviz (two-wheel과 동일)
```

명시적으로 맵만 지정할 때:

```bash
ros2 launch nav_robot_bringup nav2_nova_carter.launch.py \
  map:=$PWD/maps/restaurant/map.yaml
```

런치가 `nova_carter_topic_bridge`를 같이 띄웁니다
(`/chassis/odom`→`/odom`, `/front_2d_lidar/scan`→`/scan` BEST_EFFORT).

확인:

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic info /scan -v | rg -A2 'Subscription|Reliability'
```

`Nav2 is ready for use!` 확인. RViz Fixed Frame = **map**.

### T3 — sample_code 스타일 미션

```bash
export ROS_DOMAIN_ID=103
source /opt/ros/humble/setup.bash
source ~/git/Rokey_Co3_A1/nav_robot/install/setup.bash

# 복도 경유 순차 goToPose (nav_to_pose.py 패턴, 기본)
ros2 run nav_robot_missions nova_go_to_pose --ros-args -p table_id:=2

# 직선 단일 goToPose만 원할 때
ros2 run nav_robot_missions nova_go_to_pose --ros-args -p table_id:=2 -p direct:=true

# 경유지 (nav_through_pose.py 패턴)
ros2 run nav_robot_missions nova_through_poses --ros-args -p table_id:=2
```

웨이포인트: `src/nav_robot_missions/config/waypoints.yaml` (`table_0..3`, `kitchen`).
초기 pose는 spawn `(0.21, 5.25, -π/2)`와 맞춤.

---

## 파일 맵

| 파일 | 역할 |
|------|------|
| `isaacpjt/nav_nova_carter_demo.py` | 식당 + Nova Carter 스폰 |
| `tools/generate_occupancy_map_isaac.py` | Occupancy Map (z 0.1~0.62) |
| `config/nav2_params_nova_carter.yaml` | AMCL/costmap/RPP (`/odom`, `/scan`) |
| `launch/nav2_nova_carter.launch.py` | Nav2 + topic bridge |
| `nova_carter_topic_bridge.py` | chassis/odom·scan QoS 브리지 |
| `nova_go_to_pose.py` / `nova_through_poses.py` | sample_code 미션 |

기존 two-wheel (`nav_restaurant_demo.py`, `nav2_restaurant.launch.py`, `/nav_robot/*`)은 그대로 둡니다.

---

## 트러블슈팅

- **Occupancy map이 RViz에서 안 보임** → 보통 `/clock` 없음 + Fixed Frame `map` TF 없음.
  Isaac 로그에 `/clock publisher ready` 확인 후:
  `ros2 topic hz /clock`, `ros2 run tf2_ros tf2_echo map odom`.
  Nav2가 중복이면 `bash tools/kill_nav2.sh` 후 재런치. RViz에서 **2D Pose Estimate**로 kitchen `(0.21, 5.25)` 찍기.
- **제자리 회전 + 남은 거리 고정** → `/odom` Publisher 0 또는 `/scan` 구독 0. bridge/Nav2 재시작. Isaac만 켜진 상태에서는 `/chassis/odom`만 있음이 정상.
- **`Invalid deltaTime 0.000000`** → Isaac 데모가 accel limit을 0으로 완화함. 그래도 심하면 Isaac 재시작.
- **assets root / Nova_Carter_ROS.usd 없음** → Isaac Nucleus 자산 경로 설정 확인
- **Nav2 lifecycle 실패** → `bash tools/kill_nav2.sh` 후 단일 launch
- **AMCL 틀어짐** → RViz 2D Pose Estimate로 kitchen `(0.21, 5.25)` 재설정
- **two-wheel과 토픽 충돌** → Nova Carter와 two-wheel 데모를 동시에 켜지 말 것
