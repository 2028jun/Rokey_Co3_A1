# Isaac 식당 내비게이션 모드

## 계약 (한 줄)

| 구성요소 | 역할 |
|----------|------|
| `/nav_robot/odom` (Isaac world pose) | **유일한 제어 포즈** |
| `/navigation/command` | 정지 + 제자리 pivot + 축 직선 도킹 |
| `map_view.launch.py` | 맵 + identity `map→odom` RViz 관측 |
| `nav2_restaurant.launch.py` | RViz 목표/장애물 회피. 레일 미션과 **동시 실행 금지** |

시뮬에서 odom은 articulation GT이므로 “위치를 모른다”가 아닙니다.
과거 벽 충돌은 횡오차 무시 도착·복도 미스냅·AMCL/Nav2를 조향에 섞은 결과입니다.

## 일일 실행 (통합 주행)

Nav2 full bringup **끄기**. Isaac만 + 미션.

```bash
# T1 Isaac
cd /home/rokey/cobot3_ws/nav_robot
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/nav_restaurant_demo.py

# T2 integrated navigation command (inherits ROS_DOMAIN_ID from ~/.bashrc)
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 service call /navigation/command \
  serving_robot_interfaces/srv/TaskCommand \
  "{command: 2}"
```

`go_to_table`의 독립 rolling-dock 경로는 폐기됐습니다. 테이블 `0..3`은
반드시 Isaac 내장 `/navigation/command`를 통해 이동합니다.
Isaac Python이 custom service type support를 로드하지 못하므로
`direct_nav_server_node`가 `/navigation/command`를 `/navigation/trigger`로
전달합니다. 실제 바퀴 제어는 Isaac 내장 axis route가 수행합니다.

복귀는 별도 노드(WIP):

```bash
ros2 run nav_robot_missions return_to_kitchen --ros-args -p table_id:=2
```

시작 로그에 `direct stage=... pivot` / `axis_y` / `pivot` / `axis_x`가
순서대로 나와야 합니다.

## (선택) 맵/RViz 관측만

Isaac odom이 월드와 정렬되므로 AMCL은 사용하지 않습니다.

```bash
ros2 launch nav_robot_bringup map_view.launch.py
```

## RViz 목표 주행

`nav_server_node`, `go_to_table`, `return_to_kitchen`를 모두 끄고 실행합니다.

```bash
ros2 launch nav_robot_bringup nav2_restaurant.launch.py
rviz2 -d /home/rokey/cobot3_ws/nav_robot/config/restaurant_map.rviz
```

RViz의 **Nav2 Goal**로 목표 위치와 방향을 지정합니다.

## table_id:=2 성공 판정

한 사이클 로그에서:

| 체크 | 로그 | 허용오차 |
|------|------|----------|
| dock | `outbound done (x,y) ... OK` | `|x+1.82|<0.15`, `|y-0.70|<0.12` |
| aisle | `on aisle (x,y) ... OK` | `|x|<0.12` (북진 전) |
| kitchen | `returned to kitchen (x,y) ... OK` | `|x-0.21|<0.20`, `|y-5.25|<0.20` |

`FAIL` / `timeout` / `branch snap FAIL` / `on aisle FAIL` 이면 실패입니다.

Nav2와 레일 모드는 모두 `cmd_vel`을 발행하므로 반드시 하나만 실행합니다.
