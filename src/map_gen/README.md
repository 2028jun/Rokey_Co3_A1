# map_gen — 1회 SLAM 맵 생성 (younggi/map_generate 이식)

식당을 `slam_toolbox`로 한 번 순찰하며 점유맵을 만들어
`nav_robot5/src/two_wheel_rails/maps/restaurant/slam_map.{pgm,yaml}`에 저장합니다.
AMCL / controller_server / bt_navigator는 포함하지 않습니다 — 아직 맵이 없는
상태이므로 로컬라이제이션이 성립하지 않기 때문입니다.

`younggi` 브랜치의 `map_generate` 패키지를 원본으로 하되, 다음을 다르게
가져왔습니다.

- 자체 `topic_bridge.py` / `two_wheel_robot.urdf`를 다시 만들지 않고,
  이미 이 워크스페이스에 있는 `two_wheel_rails`의 `topic_bridge` +
  `robot_state_publisher`(+URDF)를 그대로 재사용합니다. `nav_restaurant_demo.py`가
  이미 `/two_wheel/scan_raw`, `/two_wheel/odom_raw`를 올바른 프레임
  (`ridgeback_base_link`)으로 발행하고 있어서 중복시키면 오히려 어긋날 위험이 있습니다.
- ROS_DOMAIN_ID는 113이 아니라 **102**를 씁니다 (이 프로젝트의 Isaac이
  102로 고정 발행하기 때문 — `nav_restaurant_demo.py`의
  `NAV_ROBOT_ROS_DOMAIN_ID` 기본값과 동일).
- `nav2_collision_monitor`는 **의도적으로 안 씁니다.** `NavBridge`가
  `cmd_vel`이 아니라 `cmd_vel_safe`만 구독하도록 바뀌어 있어서
  (`nav_restaurant_demo.py` 참고) 원래는 그 사이를 이어주려 했지만,
  실제로 붙여보니 stop_zone(좌우 0.60m)이 주방 출입구처럼 좁은 구간에서
  문틀/벽에 계속 걸려 **속도 명령 자체를 통째로 삼켜버려** 로봇이 첫
  waypoint에서 못 움직이는 문제가 있었습니다 (사람이 아니라 정적 지형에
  오탐). 그래서 `slam_patrol.py`가 `cmd_vel_safe`로 바로 발행합니다 —
  맵 생성은 사람 없는(`NAV_CROSSING_PEDESTRIAN=0`) 통제된 1회성 순찰이라
  collision_monitor의 사람 안전 게이트가 애초에 필요 없습니다.

## 절대 하지 말 것

**운영 중인 서빙 미션(`vision_manager_nav2.launch.py`)과 동시에 이 launch를
띄우지 마세요.** 둘 다 `/cmd_vel`, `/odom`, `/scan`, TF를 건드리는데, 이
launch는 AMCL/controller_server 없이 `slam_patrol`이 직접 `/cmd_vel`을
쏘는 구조라 정상적인 Nav2 주행과 충돌합니다. 맵 생성은 별도의 실행 모드로
완전히 분리해서 돌리세요.

## 움직이는 사람은 반드시 꺼야 함

`CrossingPedestrian`은 `/scan`에 잡히는 LiDAR collider를 갖고 있습니다
(사람 감지/정지 로직용으로 일부러 붙인 것). 켜둔 채로 맵을 뜨면 걸어다니는
동선이 그대로 점유 셀로 찍혀서 지도에 유령 장애물이 남습니다. 맵 생성
중에는 **반드시 꺼서** 돌리세요 (`TypingCustomer`는 애초에 collider가 없어서
`/scan`에 안 잡히므로 상관없지만, 순찰 중 방해되지 않도록 같이 꺼두는 걸
권장):

```bash
export NAV_CROSSING_PEDESTRIAN=0
export NAV_TYPING_CUSTOMER=0
```

## 사용 순서

```bash
# 1) Isaac Sim (기존과 동일하게 Play) -- 움직이는 사람 비활성화
cd /home/rokey/cobot3_ws/nav_robot
export NAV_ROBOT_ROS_DOMAIN_ID=102
export NAV_CROSSING_PEDESTRIAN=0
export NAV_TYPING_CUSTOMER=0
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib"
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/nav_restaurant_demo.py
```

```bash
# 2) topic_bridge + robot_state_publisher + slam_toolbox
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=102
ros2 launch map_gen slam_mapping.launch.py rviz:=true
```

```bash
# 3) 순찰 (1회 커버리지 주행, 완료되면 자동 종료)
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=102
ros2 run map_gen slam_patrol
```

```bash
# 4) 맵 저장 (2번은 계속 띄워둔 채로)
bash src/map_gen/tools/save_slam_map.sh
```

## 저장된 맵을 실제 Nav2에 반영하기 (수동, 검증 필수)

`save_slam_map.sh`는 기존 `map.yaml`/`map.pgm`(지금 `nav2.launch.py`가
기본으로 로드하는 맵)을 **절대 덮어쓰지 않습니다.** `slam_map.pgm`/
`slam_map.yaml`로 별도 저장됩니다. 실제로 교체하려면:

1. `nav_robot5/src/two_wheel_rails/maps/restaurant/slam_map.yaml`을
   RViz 등에서 열어 식당 형태가 제대로 나왔는지 확인
2. 문제없으면 `two_wheel_rails/launch/nav2.launch.py`의 `map` 인자(또는
   기본 맵 탐색 로직)를 `slam_map.yaml`로 바꾸거나, 기존 `map.yaml`을
   백업 후 교체
3. **좌표계가 완전히 같다고 가정하지 말 것.** SLAM이 잡은 `map` 원점은
   기존 `map.yaml`의 `origin: [-6.5, -5.5, 0.0]`과 다를 수 있습니다.
   교체 후 반드시 재검증:
   - 주방 위치 / park-out 시작 위치
   - 테이블 1~4 도킹 좌표 (`nav_restaurant_demo.py`의 table route,
     `robot_map.js`의 `this.tables`)
   - AMCL 초기 pose
   - 기존 waypoint/route 전부

## slam_patrol 웨이포인트 관련 주의

`map_gen/slam_patrol.py`의 순찰 경로(`_build_patrol`)는 `younggi` 브랜치
식당 치수 기준으로 튜닝된 값입니다. 남/북 테이블 행의 y좌표(-2.20 / 0.70)는
이 프로젝트의 `robot_map.js`가 쓰는 테이블 좌표와 정확히 일치하지만,
문(도어) 위치나 외곽 레인 값은 다를 수 있습니다. 첫 순찰 실행 시 RViz로
로봇이 벽에 끼거나 웨이포인트에 도달 못 하고 8초 무진행 스킵이 자주
뜨는지 확인하고, 필요하면 `DOCK`/`OUTER`/`FLANK`/`DOOR_Y`/`KIT_Y` 상수를
현재 식당 USD에 맞게 조정하세요.

## 원본 문서

기술 상세는 `younggi` 브랜치의 `map_generate/docs/SLAM.md`를 참고하세요
(이 이식에는 포함하지 않았습니다).
