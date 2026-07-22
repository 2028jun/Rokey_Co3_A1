# nav_robot3 — 레일 경로 + Nav2 추종 + 횡오차 검증

`nav_robot2`는 그대로 두고, **고정 레일(`routes.yaml`)** 을 Nav2로 순차 추종하면서 **map pose ↔ 레일 횡오차(CTE)** 로 이탈을 검사합니다. 레일 주행 구간은 Nav2만 사용합니다.

## nav_robot2와 차이

| | nav_robot2 | nav_robot3 |
|---|------------|------------|
| 경로 | waypoint `goToPose` | `routes.yaml` spine/branch/dock |
| 검증 | 도착 오차 위주 | 주행 중 **cross-track** (`rail_check.yaml`) |
| ROS domain | 103 | **104** (동시 실행 시 분리) |
| 패키지 | `nova_carter` | `nova_rails` |

전체 안내: 저장소 루트 [`docs/NOVA_CARTER.md`](../docs/NOVA_CARTER.md)

## 1회 준비

```bash
cd ~/git/Rokey_Co3_A1/nav_robot3
./tools/sync_restaurant_assets.sh

source /opt/ros/humble/setup.bash
colcon build --packages-select nova_rails
source install/setup.bash
```

맵은 `maps/restaurant/map.yaml` + `map.pgm` (nav_robot2와 동일 방식). 재생성:

```bash
export NAV_ROBOT3_ISAAC_PYTHON=/path/to/isaacsim/.../python.sh
"$NAV_ROBOT3_ISAAC_PYTHON" tools/generate_occupancy_map_isaac.py
```

레일 미리보기 (선택):

```bash
PYTHONPATH=src/nova_rails python3 tools/plot_rails_preview.py
```

## 실행 (3 터미널)

**각 터미널에서** aliases를 source (`ROS_DOMAIN_ID=104` 고정):

```bash
source ~/git/Rokey_Co3_A1/nav_robot3/tools/aliases.sh
```

| 터미널 | 명령 | 내용 |
|--------|------|------|
| **1** | `t1` | Isaac `restaurant_nova_demo.py` → **Play** |
| **2** | `t2` | Nav2 + AMCL + RViz |
| **3** | `t3 --table-id 2` | 레일 미션 (`rail_mission`) |

순서: **t1 → Play → t2 → t3**

### T3 예시

```bash
t3 --table-id 0
t3 --table-id 2
t3 --table-id 2 --no-return-kitchen
t3 --list-routes
```

### alias 없이

```bash
export ROS_DOMAIN_ID=104
export NAV_ROBOT3_WS=~/git/Rokey_Co3_A1/nav_robot3
source /opt/ros/humble/setup.bash
source $NAV_ROBOT3_WS/install/setup.bash

# T1
cd $NAV_ROBOT3_WS && $ISAAC_PYTHON isaacpjt/restaurant_nova_demo.py

# T2
cd $NAV_ROBOT3_WS && ./tools/kill_nav2.sh
ros2 launch nova_rails nav2.launch.py map:=$NAV_ROBOT3_WS/maps/restaurant/map.yaml

# T3
ros2 run nova_rails rail_mission --table-id 2
```

## 설정

- `src/nova_rails/config/routes.yaml` — 레일 geometry (`to_*`, `return_*`)
- `src/nova_rails/config/rail_check.yaml` — CTE 한도, 주차 후진(`park_out_open_loop`, `aisle_x_max_m`)

미션 시작 시 `/nova_carter/teleport` + AMCL sync (`nav_bootstrap.sync_spawn`).

**주차 패턴:** `to_*` 전진 도킹 → **`/cmd_vel` 직선 후진**으로 복도(`|x|≤aisle_x_max`) → `return_*` spine 전진 복귀. 복도 미도달 시 `return_*` 미실행(슬롯 Nav2 유턴 방지).

## 패키지 `nova_rails`

| 항목 | 설명 |
|------|------|
| `topic_bridge` | `/chassis/odom`→`/odom`, lidar→`/scan` |
| `nav2.launch.py` | map_server + AMCL + Nav2 + RViz |
| `rail_mission` | `routes.yaml` 레일 + CTE 검증 |

## 검증

```bash
export ROS_DOMAIN_ID=104
ros2 topic hz /clock /scan /odom
ros2 lifecycle get /bt_navigator
```
