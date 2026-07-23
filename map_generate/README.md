# map_generate — 1회 SLAM 맵 생성

식당 USD + Ridgeback 2륜(`two_wheel_serving_robot_v2`)으로 **slam_toolbox** 맵을 한 번 만들고 `maps/restaurant/slam_map.{pgm,yaml}`에 저장합니다. Nav2 / AMCL / line_rails는 포함하지 않습니다.

기술·방법 상세: **[docs/SLAM.md](docs/SLAM.md)**

## 의존성

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
```

## 빌드

```bash
cd ~/git/Rokey_Co3_A1/map_generate
./tools/sync_assets.sh   # nav_robot6 assets 심볼릭 링크
colcon build --packages-select map_gen
source install/setup.bash
```

## 사용 (t1 → t2 → t3 → save_map)

```bash
source ~/git/Rokey_Co3_A1/map_generate/tools/aliases.sh
```

| alias | 역할 |
|-------|------|
| `t1` | Isaac `restaurant_two_wheel_demo.py` (Play) — Domain **113** |
| `t2` | `topic_bridge` + `robot_state_publisher` + `async_slam_toolbox` |
| `t3` | `slam_patrol` — **테이블 뒤(OUTER=±2.55) · 주방/남쪽 문 앞 · 외곽 레인** + look-around |
| `save_map` | `map_saver_cli` → `maps/restaurant/slam_map.pgm` / `.yaml` |

순서는 반드시 **t1(Play) → t2 → t3 → save_map** 입니다. `save_map` 시점에 t2(slam)는 계속 떠 있어야 합니다.

`t3`는 회색(미탐색)을 줄이기 위해 테이블 **도크만이 아니라 후면·측면**, 주방 문(중앙 레인만)·문 앞 스캔, 외곽 벽 레인을 순회합니다. 문틀(±0.85) 웨이포인트는 팔 걸림으로 제거했습니다. 막히면 8초 무진행 시 스킵합니다.

`save_map`은 **기존 맵 파일을 덮어쓰지 않습니다.** `slam_map.*`가 이미 있으면 `slam_map_YYYYMMDD_HHMMSS.*`로 새로 저장합니다. `nav_robot6/maps` 등 다른 워크스페이스 맵은 건드리지 않습니다.

RViz가 필요하면: `t2 rviz:=true`

## 토픽 / 프레임

| 항목 | 값 |
|------|-----|
| Domain | `113` (`MAP_GEN_ROS_DOMAIN_ID`) |
| scan | `/scan` (bridge: `/two_wheel/scan_raw`) |
| odom | `/odom` + TF `odom→ridgeback_base_link` |
| cmd | `/cmd_vel` → Isaac |
| slam | `map`, `odom`, `ridgeback_base_link`, `scan_topic: /scan` |

## 에셋

`assets/`는 `nav_robot6/assets`에 대한 심볼릭 링크입니다 (`tools/sync_assets.sh`). Lightwheel Kitchen을 복사하지 않습니다.
