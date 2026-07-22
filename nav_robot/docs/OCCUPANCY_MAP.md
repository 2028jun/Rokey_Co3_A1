# Occupancy Map for nav_robot

1차 개발은 Isaac Sim **Occupancy Map Generator**로 정답 맵을 뽑고 Nav2에 넣습니다.
SLAM은 2차 범위입니다.

## 준비

- Isaac Sim 5.1
- 워크스페이스: `nav_robot/`
- 식당 스테이지: `assets/lightweight_restaurant/lightweight_pizza_restaurant.usda`
- Lightwheel Kitchen 런타임 (대용량, gitignore):

```bash
cd nav_robot/assets/Lightwheel_Kitchen
# 이미 팀 머신에 serving_robot 자산이 있으면:
ln -sfn ../../../serving_robot/assets/Lightwheel_Kitchen/Collected_KitchenRoom Collected_KitchenRoom
```

## Isaac Occupancy Map Generator 절차

### A. GUI

1. Isaac Sim에서 식당 USDA를 연다 (또는 `isaacpjt/nav_restaurant_demo.py`로 씬을 띄운 뒤 로봇만 숨김).
2. Window → Occupancy Map / 확장 `isaacsim.asset.gen.omap` 를 연다.
3. 맵 범위 예 (월드 좌표, **주방 y≈7.4 포함**):
   - lower bound: `(-6.5, -5.5, 0.05)`
   - upper bound: `(6.5, 9.5, 0.35)`
4. Cell size / resolution: `0.05` m (Nav2 params와 동일).
5. Compute 후 **Image** / **YAML** 로 저장:
   - `nav_robot/maps/restaurant/map.pgm`
   - `nav_robot/maps/restaurant/map.yaml`

### B. CLI (권장, Isaac python)

Nav demo와 **동시에** 돌리지 마세요 (Isaac 인스턴스 충돌).

```bash
cd ~/git/Rokey_Co3_A1/nav_robot
# Isaac demo가 꺼진 상태에서
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  tools/generate_occupancy_map_isaac.py
```

`origin`은 Generator/스크립트가 내보낸 값을 **그대로** 쓰고, AMCL/waypoint의 `map` 프레임과 일치하는지 RViz에서 확인합니다.

**중요:** kitchen dock `(0.21, 5.25)` 와 주방 공간(y>5)이 맵 안에 있어야 합니다.
예전 placeholder(y≤6)에서는 주방으로 조금만 들어가도 **맵 밖으로 나가** Nav2가 멈춥니다.


`map.yaml` 예시:

```yaml
image: map.pgm
resolution: 0.05
origin: [-6.5, -5.5, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

## 파일로 맵 미리보기

```bash
xdg-open maps/restaurant/map.pgm
cat maps/restaurant/map.yaml
```

검은 셀 = occupied, 밝은 셀 = free. kitchen·테이블 윤곽이 식당 레이아웃과 비슷한지 확인합니다.

## RViz에서 Map ↔ Laser 정렬 검증

Nav2 + Isaac이 떠 있는 상태에서:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=102
rviz2
```

| Display | Topic / 설정 |
|---------|----------------|
| Fixed Frame | `map` |
| Map | `/map` |
| LaserScan | `/nav_robot/scan` |
| PoseWithCovariance | `/amcl_pose` |
| PoseArray (선택) | `/particle_cloud` |

**통과 기준**

- 레이저 점이 맵 벽·테이블 윤곽에 겹침
- `/amcl_pose`가 kitchen 근처 `(≈0.21, 5.25)`에서 안정
- `config/waypoints.yaml`의 table dock이 free 공간(테이블 **옆**)에 있음

어긋나면 Generator `origin`/bounds와 `map→odom` identity TF를 확인합니다.

waypoint 좌표 (serving dock과 동일):

| id | name | (x, y) |
|----|------|--------|
| 0 | table_0 | (-1.82, -2.20) |
| 1 | table_1 | (1.82, -2.20) |
| 2 | table_2 | (-1.82, 0.70) |
| 3 | table_3 | (1.82, 0.70) |
| 4 | kitchen | (0.21, 5.25) |

AMCL/라이다는 테이블 **이름**을 인식하지 않습니다. 정렬 = 맵 점유 + 레이저 윤곽 + waypoint 좌표가 같은 `map` 프레임에 맞는지입니다.

## 플레이스홀더 맵

Generator를 돌리기 전에 bringup 스모크 테스트용으로:

```bash
python3 tools/generate_placeholder_map.py
```

이 맵은 외벽·대략적인 테이블 점유만 반영합니다. **실제 주행 검증 전에는 Generator 결과로 교체하세요.**

## Nav2에서 로드

```bash
export ROS_DOMAIN_ID=102
ros2 launch nav_robot_bringup nav2_restaurant.launch.py \
  map:=$PWD/maps/restaurant/map.yaml
```
