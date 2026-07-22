# nav_robot2 — Nova Carter + Nav2 (deprecated)

**이 워크스페이스는 더 이상 쓰지 않습니다.** 레일 주행은 **`nav_robot3`** + 저장소 루트 `tools/aliases.sh` (`t1`/`t2`/`t3`)를 사용하세요.

저장소 공통 안내: [`docs/NOVA_CARTER.md`](../docs/NOVA_CARTER.md)

---

아래는 레거시 문서입니다.

`nav_robot`와 분리된 워크스페이스입니다. **식당 USD/주방 자산만** 복사해 두었고, Nav2·미션·Isaac 실행은 이 디렉터리 기준으로만 사용합니다. **SLAM 없음** — AMCL + Occupancy Map.

## 1회 준비

```bash
cd ~/git/Rokey_Co3_A1/nav_robot2
./tools/sync_restaurant_assets.sh

# Isaac Sim python (다른 Isaac 인스턴스 종료 후)
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  tools/generate_occupancy_map_isaac.py
```

Occupancy Map Generator bounds (Nova Carter Z 슬랩):

- lower: `(-6.5, -5.5, 0.1)`
- upper: `(6.5, 9.5, 0.62)`
- resolution: `0.05` m → `maps/restaurant/map.pgm`, `map.yaml`

빌드:

```bash
source /opt/ros/humble/setup.bash
cd ~/git/Rokey_Co3_A1/nav_robot2
colcon build --packages-select nova_carter
source install/setup.bash
```

## 실행 (3 터미널)

한 번만 로드 (각 터미널에서 **반드시** source — `t3`만 치면 안 됨):

```bash
source ~/git/Rokey_Co3_A1/nav_robot2/tools/aliases.sh
```

`ROS_DOMAIN_ID`는 aliases가 **항상 103**으로 맞춥니다 (bashrc의 101과 섞이면 t3가 바로 실패하거나 터미널이 꺼질 수 있음).

| 터미널 | 명령 | 내용 |
|--------|------|------|
| **1** | `t1` | Isaac 식당 + Nova Carter |
| **2** | `t2` | Nav2 + AMCL + RViz (`kill_nav2` 후 launch) |
| **3** | `t3` | table 0 도킹 + 주방 복귀 (기본) |

T3 예시:

```bash
t3 --table-id 1
t3 --table-id 2 --no-return-kitchen
t3 --visit-all
```

순서: **t1 → (Play) → t2 → t3**. 매 터미널에서 `ROS_DOMAIN_ID=103`은 aliases가 설정합니다.

<details>
<summary>전체 명령 (alias 없이)</summary>

공통:

```bash
export ROS_DOMAIN_ID=103
export NAV_ROBOT2_WS=~/git/Rokey_Co3_A1/nav_robot2
```

**T1 — Isaac**

```bash
cd $NAV_ROBOT2_WS
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/restaurant_nova_demo.py
```

**T2 — Nav2 + RViz**

```bash
source /opt/ros/humble/setup.bash
source $NAV_ROBOT2_WS/install/setup.bash
export ROS_DOMAIN_ID=103
cd $NAV_ROBOT2_WS
./tools/kill_nav2.sh
ros2 launch nova_carter nav2.launch.py \
  map:=$NAV_ROBOT2_WS/maps/restaurant/map.yaml
```

**T3 — 주행**

```bash
source /opt/ros/humble/setup.bash
source $NAV_ROBOT2_WS/install/setup.bash
export ROS_DOMAIN_ID=103
ros2 run nova_carter nav_to_pose --table-id 0
```

</details>

- 미션 **시작마다** Isaac 주방 텔레포트에 맞춰 AMCL 재동기화 (`[sync]` 로그). RViz가 이전 위치에 멈춰 있으면 T3만 다시 실행하면 됨.
- 도킹: waypoint `x=±1.72`, Nav2 `xy_goal_tolerance=0.12` m (테이블 옆 밀착). `nav2_params` 변경 후 **T2 Nav2 재시작** 필요.

## 패키지 `nova_carter`

| 항목 | 설명 |
|------|------|
| `topic_bridge` | `/chassis/odom`→`/odom`, `/front_2d_lidar/scan`→`/scan`, TF `base_link` |
| `nav2.launch.py` | map_server + AMCL + Nav2 + RViz |
| `nav_to_pose` | [`sample_code/nav_to_pose.py`](sample_code/nav_to_pose.py) 패턴 |

참고: [`sample_code/nav_to_pose.py`](sample_code/nav_to_pose.py)는 원본 예제입니다. 실행은 `ros2 run nova_carter nav_to_pose`를 사용하세요.

## 검증

```bash
ros2 topic hz /clock /scan /odom
ros2 topic echo /plan --once
```

주행 성공 시 터미널에 `목적지 도착 완료` 및 남은 거리 감소가 보입니다.

## RViz 글로벌 코스트맵이 전부 핑크일 때

대부분 **`/map`이 전부 장애물(occupied)** 일 때입니다. 맵 PGM이 검은색만 있으면 global costmap도 전부 비용 100(핑크)으로 보입니다.

```bash
python3 tools/generate_placeholder_map.py   # 즉시 복구용
# 또는 Isaac omap 재생성 (실패 시 stderr에 degenerate 메시지)
.../python.sh tools/generate_occupancy_map_isaac.py
```

맵 교체 후 **T2 Nav2 launch를 재시작**하세요 (`map_server`가 새 `/map`을 다시 올립니다).

검증:

```bash
python3 - <<'PY'
from pathlib import Path
import numpy as np
d = Path("maps/restaurant/map.pgm").read_bytes().partition(b"\n255\n")[2]
u = np.frombuffer(d, np.uint8)
print("free(254):", (u==254).sum(), "occ(0):", (u==0).sum())
PY
```

free 픽셀이 충분히 있어야 합니다 (전부 `occ`이면 안 됨).

## 로봇이 안 움직이거나 local costmap이 안 보일 때

1. **`bt_navigator` 비활성** — launch의 BT XML 경로가 잘못되면 Nav2가 절반만 뜹니다.
   ```bash
   ros2 lifecycle get /bt_navigator    # active [3] 이어야 함
   ros2 lifecycle get /velocity_smoother
   ```
   `inactive`이면 T2를 **재시작** (`colcon build` 후 `source install/setup.bash` 필수).

2. **터미널마다** `export ROS_DOMAIN_ID=103` (쉘 기본값 101이면 Isaac/Nav2가 분리됨).

3. Local costmap 프레임은 **`odom`** 입니다. RViz Fixed Frame `map` + `map→odom` TF(AMCL)가 있어야 겹쳐 보입니다.
