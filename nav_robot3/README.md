# nav_robot3 — 레일 경로 + Nav2 추종 + 횡오차 검증

`nav_robot2`는 그대로 두고, **고정 레일(`routes.yaml`)** 을 Nav2 `NavigateToPose`로 순차 추종하면서 **map pose ↔ 레일 폴리라인 횡오차**로 이탈을 검사합니다. open-loop `cmd_vel` 레일은 사용하지 않습니다.

## nav_robot2와 차이

| | nav_robot2 | nav_robot3 |
|---|------------|------------|
| 경로 | waypoint 직행 `goToPose` | `routes.yaml` spine/branch/dock |
| 검증 | 도착 오차 위주 | 주행 중 **cross-track** (`rail_check.yaml`) |
| ROS domain | 103 | **104** (동시 실행 시 분리) |
| 패키지 | `nova_carter` | `nova_rails` |

## 준비

```bash
cd ~/git/Rokey_Co3_A1/nav_robot3
./tools/sync_restaurant_assets.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select nova_rails
source install/setup.bash
```

레일 미리보기 (선택):

```bash
PYTHONPATH=src/nova_rails python3 tools/plot_rails_preview.py
```

## 실행 (3 터미널)

```bash
source ~/git/Rokey_Co3_A1/nav_robot3/tools/aliases.sh
```

| 터미널 | 명령 |
|--------|------|
| 1 | `t1` → Play |
| 2 | `t2` |
| 3 | `t3 --table-id 2` |

## 설정

- `src/nova_rails/config/routes.yaml` — 레일 geometry
- `src/nova_rails/config/rail_check.yaml` — `max_lateral_error_m` (기본 0.28 m)

미션 시작 시 `/nova_carter/teleport` + AMCL sync (`nav_bootstrap.sync_spawn`).
