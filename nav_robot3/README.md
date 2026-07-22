# nav_robot3 — Nova Carter 레일 주행 (기본 t1/t2/t3)

**고정 레일(`routes.yaml`)** + Nav2 추종 + **횡오차(CTE)** 검증.  
저장소 루트에서 `source tools/aliases.sh` 후 `t1` / `t2` / `t3` 사용 (`ROS_DOMAIN_ID=103`).

## 1회 준비

```bash
cd ~/git/Rokey_Co3_A1/nav_robot3
./tools/sync_restaurant_assets.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select nova_rails
source install/setup.bash
```

## 실행 (3 터미널)

```bash
source ~/git/Rokey_Co3_A1/tools/aliases.sh
```

| 터미널 | 명령 |
|--------|------|
| 1 | `t1` → **Play** |
| 2 | `t2` |
| 3 | `t3 --table-id 2` |

동일 aliases: `source ~/git/Rokey_Co3_A1/nav_robot3/tools/aliases.sh`

### T3 예시

```bash
t3 --table-id 2
t3 --table-id 2 --no-return-kitchen
t3 --list-routes
```

## 설정

- `src/nova_rails/config/routes.yaml` — 레일 (`to_*`, `return_*`)
- `src/nova_rails/config/rail_check.yaml` — CTE, 주차 후진, **도킹 후 yaw 정렬** (`align_heading_at_dock`)

**주차:** 전진 도킹 → `/cmd_vel` 직선 후진 → `return_*` spine 복귀.

## 패키지 `nova_rails`

`topic_bridge`, `nav2.launch.py`, `rail_mission`

문서: [`docs/NOVA_CARTER.md`](../docs/NOVA_CARTER.md)
