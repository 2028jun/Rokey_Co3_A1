# nav_robot5 — 2륜 Ridgeback 레일 주행

**고정 레일(`routes.yaml`)** + 2륜 직접 추종 + **횡오차(CTE)** 검증.
저장소 루트에서 `source tools/aliases.sh` 후 `t1` / `t2` / `t3` 사용 (`ROS_DOMAIN_ID=102`).

## 1회 준비

```bash
cd /home/rokey/cobot3_ws/nav_robot5
./tools/sync_restaurant_assets.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select two_wheel_rails
source install/setup.bash
```

## 실행 (3 터미널)

```bash
source /home/rokey/cobot3_ws/nav_robot5/tools/aliases.sh
```

| 터미널 | 명령 |
|--------|------|
| 1 | `t1` (Isaac이 자동으로 Play) |
| 2 | `t2` |
| 3 | `t3 --table-id 2` |

`t1`: 식당과 2륜 USD, `/clock`, 원시 LiDAR/odom, `/cmd_vel` 구동
`t2`: 원시 토픽 변환, TF, AMCL/Nav2, RViz
`t3`: 원본 레일 미션을 2륜 전진·조향 제어로 수행

### T3 예시

```bash
t3 --table-id 2
t3 --table-id 2 --no-return-kitchen
t3 --list-routes
```

## 설정

- `src/two_wheel_rails/config/routes.yaml` — 레일 (`to_*`, `return_*`)
- `src/two_wheel_rails/config/rail_check.yaml` — CTE, 주차 후진, **도킹 후 yaw 정렬** (`align_heading_at_dock`)

**주차:** 전진 도킹 → `/cmd_vel` 직선 후진 → `return_*` spine 복귀.

## 패키지 `two_wheel_rails`

`topic_bridge`, `nav2.launch.py`, `rail_mission`

기준 구현: `origin/younggi:nav_robot3` (`fb37644`), 로봇 구동부만 2륜 모델로 교체.
