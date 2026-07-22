# Nova Carter 식당 주행 (younggi 브랜치)

Isaac Sim **Nova Carter** + Occupancy Map + **Nav2/AMCL** 로 레스토랑 맵을 주행하는 코드는 **`nav_robot2`** / **`nav_robot3`** 워크스페이스에 있습니다.  
(`nav_robot` 통합 워크스페이스에도 동일 미션을 올려 두었으며, 아래 **선택 C** 참고.)

| 스택 | 경로 | ROS domain | 용도 |
|------|------|------------|------|
| **A (권장)** | [`nav_robot2/`](../nav_robot2/) | **103** | waypoint `goToPose` — 테이블 도킹·주방 복귀 |
| **B** | [`nav_robot3/`](../nav_robot3/) | **104** | `routes.yaml` 레일 + 횡오차 검증 + 주차 후진 이탈 |
| C | [`nav_robot/`](../nav_robot/) | 103 | two-wheel과 같은 colcon WS, `nav2_nova_carter.launch.py` |

**브랜치:** `git checkout younggi` 후 `nav_robot2`, `nav_robot3` 디렉터리가 보여야 합니다.

---

## 공통 요구사항

- Ubuntu 22.04 + ROS 2 Humble
- Isaac Sim 5.x (ROS2 bridge, `/clock` 발행)
- `ROS_DOMAIN_ID`를 **Isaac / Nav2 / 미션 터미널에서 동일**하게 유지

Isaac Python 경로는 환경에 맞게 설정:

```bash
export ISAAC_PYTHON=/path/to/isaacsim/.../python.sh
# nav_robot2 aliases: NAV_ROBOT2_ISAAC_PYTHON
# nav_robot3 aliases: NAV_ROBOT3_ISAAC_PYTHON
```

대용량 주방 USD는 Git에 없을 수 있습니다. 각 WS에서 한 번:

```bash
./tools/sync_restaurant_assets.sh
```

맵: `maps/restaurant/map.yaml` + `map.pgm` (SLAM 없음, AMCL 고정 맵).

---

## A — nav_robot2 (Nova Carter + Nav2, domain 103)

상세: [`nav_robot2/README.md`](../nav_robot2/README.md)

### 1회 준비

```bash
cd ~/git/Rokey_Co3_A1/nav_robot2
./tools/sync_restaurant_assets.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select nova_carter
source install/setup.bash
```

맵 재생성(선택, Isaac 단독 실행):

```bash
"$ISAAC_PYTHON" tools/generate_occupancy_map_isaac.py
```

### 실행 (터미널 3개)

```bash
source ~/git/Rokey_Co3_A1/nav_robot2/tools/aliases.sh
```

| 순서 | 터미널 | 명령 | 설명 |
|------|--------|------|------|
| 1 | T1 | `t1` | Isaac 식당 + Nova Carter → **Play** |
| 2 | T2 | `t2` | Nav2 + AMCL + RViz (`kill_nav2` 후 launch) |
| 3 | T3 | `t3 --table-id 2` | 테이블 주행 (+ 기본 주방 복귀) |

예:

```bash
t3 --table-id 1
t3 --table-id 2 --no-return-kitchen
t3 --visit-all
```

패키지 `nova_carter`: `topic_bridge`, `nav2.launch.py`, `nav_to_pose`  
원본 예제: `sample_code/nav_to_pose.py`

---

## B — nav_robot3 (레일 미션, domain 104)

`nav_robot2`와 **동시에** 켜려면 domain이 **104**로 분리되어 있습니다.

상세: [`nav_robot3/README.md`](../nav_robot3/README.md)

### 1회 준비

```bash
cd ~/git/Rokey_Co3_A1/nav_robot3
./tools/sync_restaurant_assets.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select nova_rails
source install/setup.bash
```

### 실행

```bash
source ~/git/Rokey_Co3_A1/nav_robot3/tools/aliases.sh
# T1 → Play → T2 → T3
t3 --table-id 2
```

설정: `src/nova_rails/config/routes.yaml`, `rail_check.yaml`

---

## C — nav_robot 워크스페이스 (통합)

상세: [`nav_robot/docs/NOVA_CARTER_QUICKSTART.md`](../nav_robot/docs/NOVA_CARTER_QUICKSTART.md)

```bash
cd ~/git/Rokey_Co3_A1/nav_robot
bash tools/fetch_nav_assets.sh
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=103
```

- T1: `isaacpjt/nav_nova_carter_demo.py`
- T2: `ros2 launch nav_robot_bringup nav2_nova_carter.launch.py`
- T3: `ros2 run nav_robot_missions nova_go_to_pose --ros-args -p table_id:=2`

---

## 빠른 점검

```bash
export ROS_DOMAIN_ID=103   # 또는 104
ros2 topic hz /clock /scan /odom
ros2 lifecycle get /bt_navigator   # active [3]
```

- 코스트맵 전부 핑크 → `map.pgm` occupied; `nav_robot2/README.md` 맵 점검 절 참고
- 로봇 안 움직임 → domain 불일치, Nav2 lifecycle, `topic_bridge` / Isaac Play 확인
