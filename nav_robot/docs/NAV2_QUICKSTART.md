# 레일 미션 = odom-GT만 제어 (AMCL observe-only)

## 계약 (한 줄)

| 구성요소 | 역할 |
|----------|------|
| `/nav_robot/odom` (Isaac world pose) | **유일한 제어 포즈** |
| `go_to_table` 레일 | yaw hold + odom cross-track + 복도 스냅 |
| AMCL + map (`localization.launch.py`) | 과제용 **관측/RViz만**. `cmd_vel` 개입 금지 |
| `nav2_restaurant.launch.py` | 레일 미션과 **동시 실행 금지** |

시뮬에서 odom은 articulation GT이므로 “위치를 모른다”가 아닙니다.
과거 벽 충돌은 횡오차 무시 도착·복도 미스냅·AMCL/Nav2를 조향에 섞은 결과입니다.

## 일일 실행 (권장)

Nav2 full bringup **끄기**. Isaac만 + 미션.

```bash
# T1 Isaac
cd ~/git/Rokey_Co3_A1/nav_robot
export NAV_ROBOT_ROS_DOMAIN_ID=103
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/nav_restaurant_demo.py

# T2 mission (ROS_DOMAIN_ID=103)
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=103
ros2 run nav_robot_missions go_to_table --ros-args -p table_id:=2
```

복귀는 별도 노드(WIP):

```bash
ros2 run nav_robot_missions return_to_kitchen --ros-args -p table_id:=2
```

시작 로그에 `mode=odom-rail outbound only` 가 있어야 합니다.
`AMCL steer` / `NavigateToPose` 문자열이 **나오면 안 됩니다**.

## (선택) AMCL 관측만

RViz/과제 데모용. 미션은 AMCL을 읽지 않습니다.

```bash
ros2 launch nav_robot_bringup localization.launch.py
```

`nav2_restaurant.launch.py`(planner/controller 포함)는 **실행하지 마세요**.
플래너 `cmd_vel`과 레일이 충돌합니다.

## table_id:=2 성공 판정

한 사이클 로그에서:

| 체크 | 로그 | 허용오차 |
|------|------|----------|
| dock | `outbound done (x,y) ... OK` | `|x+1.82|<0.15`, `|y-0.70|<0.12` |
| aisle | `on aisle (x,y) ... OK` | `|x|<0.12` (북진 전) |
| kitchen | `returned to kitchen (x,y) ... OK` | `|x-0.21|<0.20`, `|y-5.25|<0.20` |

`FAIL` / `timeout` / `branch snap FAIL` / `on aisle FAIL` 이면 실패입니다.

## Nav2 플래너 재시도 (나중)

맵이 free로 유지되고 레일 왕복이 안정된 뒤에만 `nav2_restaurant.launch.py`를 따로 검증합니다.
일일 테이블 미션 경로에는 넣지 않습니다.
