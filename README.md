# Rokey_Co3_A1 서빙 로봇 및 손 안전 작업공간

## 회사 장비 최종 검증

WSL Ubuntu 22.04에서 ROS 2 Humble 빌드와 단위 테스트 17개는 통과했습니다.
아래 항목은 Isaac Sim 5.1이 설치된 회사 Ubuntu 장비에서 최종 확인해야 합니다.

- HMI의 첫 주문이 `robot1`에 배정되는지
- `robot1`이 복귀하기 전에 넣은 두 번째 주문이 `robot2`에 배정되는지
- 두 로봇이 각각 선택한 테이블까지 이동하고 자기 주방 슬롯으로 복귀하는지
- `/clock`, odom, LiDAR, TF, AMCL과 Nav2 costmap이 두 네임스페이스에서 정상인지
- HMI에서 `robot1` 카메라 영상과 위치가 표시되는지

이번 검증 범위는 주행 전용 모드입니다. 음식 스폰, 로봇팔 서빙, 손 안전 동작은
실행하지 않습니다. 로봇 간 정지/감속 및 교착 방지 로직도 이번 변경 범위에
포함하지 않았습니다.

### 1. 최신 코드 및 빌드

회사 장비에서 이 브랜치의 최신 코드를 받은 뒤 빌드합니다.

```bash
cd /home/rokey/cobot3_ws
git switch woduqmulti
git pull --ff-only origin woduqmulti
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 ./tools/colcon_safe.py build \
  --symlink-install \
  --packages-up-to serving_robot_manager serving_hmi two_wheel_rails hand_safety \
  --executor sequential \
  --event-handlers console_start_end+ desktop_notification- status- terminal_title-
source install/setup.bash
```

빌드가 끝나면 선택 사항으로 회귀 테스트를 다시 실행할 수 있습니다.

```bash
colcon test --packages-select serving_robot_manager two_wheel_rails
colcon test-result --verbose
```

합격 기준은 `0 errors`, `0 failures`입니다.

### 2. 터미널별 실행

Isaac 터미널과 ROS 터미널의 환경을 섞지 마십시오. Isaac 터미널에서는
`source /opt/ros/humble/setup.bash`를 실행하지 않습니다.

**터미널 1: Isaac Sim**

```bash
cd /home/rokey/cobot3_ws
./tools/run_multi_integrated_isaac.sh
```

식당과 `/World/NavRobot1`, `/World/NavRobot2`가 모두 로드되고 시뮬레이션이
재생 중인지 확인합니다.

**터미널 2: 멀티로봇 Nav2 및 Manager**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serving_robot_manager multi_robot.launch.py
```

다음 로그가 `robot1`, `robot2`에서 각각 한 번씩 출력될 때까지 주문을 넣지
마십시오.

```text
Navigation is fully initialized and ready!
```

**터미널 3: HMI**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serving_hmi hmi.launch.py
```

브라우저에서 `http://localhost:8000`을 엽니다. HMI 카메라 기본 연결은
`/robot1/camera/color/image_raw`, 위치 연결은 `/robot1/nav_robot/odom`입니다.

### 3. 주문 전 연결 확인

진단용 ROS 터미널을 하나 더 열고 다음 명령을 실행합니다.

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service list | grep -E '/manager/order|/robot[12]/manager/order|/robot[12]/navigation/command'
ros2 topic hz /clock
ros2 topic hz /robot1/nav_robot/odom
ros2 topic hz /robot2/nav_robot/odom
ros2 topic hz /robot1/scan
ros2 topic hz /robot2/scan
```

각 `topic hz`에서 주기가 출력되어야 합니다. 확인 후 `Ctrl+C`로 다음 명령으로
넘어갑니다. 초기화 결과도 확인합니다.

```bash
ros2 topic echo /robot1/navigation/detail --once
ros2 topic echo /robot2/navigation/detail --once
```

두 결과 모두 아래 값을 포함해야 합니다.

```text
state: SUCCEEDED
phase: initialized
```

실제 메시지는 JSON 문자열이므로 한 줄 안에 위 값이 표시될 수 있습니다.

### 4. 주문 두 개 실기 시험

1. HMI에서 메뉴를 하나 이상 담고 원하는 테이블로 첫 주문을 보냅니다.
2. `robot1`이 주방을 떠나 테이블로 이동하는지 확인합니다.
3. `robot1`이 주방으로 복귀하기 전에 다른 테이블로 두 번째 주문을 보냅니다.
4. 두 번째 주문을 `robot2`가 수행하는지 확인합니다.
5. 두 로봇이 테이블 도착 후 각각의 주방 슬롯으로 복귀할 때까지 기다립니다.

상태는 다음 명령으로 확인합니다.

```bash
ros2 topic echo /robot1/system/status
ros2 topic echo /robot2/system/status
```

각 로봇의 정상 상태 순서는 다음과 같습니다.

```text
3 -> 1 -> 6
```

- `3`: 테이블로 이동
- `1`: 주방으로 복귀
- `6`: 주문 완료
- `7`: 실패

`robot1`은 `(-0.90, 5.25)`, `robot2`는 `(0.90, 5.25)` 부근의 자기 주방
슬롯으로 돌아와야 합니다. 테이블 명령은 HMI의 Table 1~4가 내부 명령
`0`~`3`으로 변환됩니다.

### 5. 최종 합격 기준

- 첫 주문은 `robot1`, 복귀 전 두 번째 주문은 `robot2`가 수행함
- 두 로봇 모두 선택한 테이블에 실제로 도착함
- 두 로봇 모두 서로 다른 자기 주방 슬롯으로 복귀함
- 두 로봇의 상태가 `3 -> 1 -> 6`으로 진행하고 `7`이 발생하지 않음
- `/robot1/navigation/detail`, `/robot2/navigation/detail`의 각 주행 완료가
  `state=SUCCEEDED`, `phase=completed`로 확인됨
- HMI에서 `robot1` 카메라와 위치가 갱신됨

### 6. 실패 시 확인

주문 서비스가 없으면 초기화가 끝나지 않은 것입니다.

```bash
ros2 service list | grep -E '/robot[12]/navigation/initialize|/robot[12]/manager/order'
ros2 topic echo /robot1/navigation/detail --once
ros2 topic echo /robot2/navigation/detail --once
```

주문은 접수됐지만 로봇이 움직이지 않거나 실패하면 다음 상태를 확인합니다.

```bash
ros2 topic echo /robot1/navigation/detail
ros2 topic echo /robot2/navigation/detail
ros2 topic info /robot1/two_wheel/mission_command -v
ros2 topic info /robot2/two_wheel/mission_command -v
ros2 topic info /robot1/two_wheel/mission_status -v
ros2 topic info /robot2/two_wheel/mission_status -v
```

카메라만 나오지 않으면 다음 명령으로 Isaac 발행 여부를 확인합니다.

```bash
ros2 topic hz /robot1/camera/color/image_raw
ros2 topic info /robot1/camera/color/image_raw -v
```

HMI의 `active_order_id`와 통합 상태 표시는 아직 전역 값 하나를 사용하므로 두
주문이 동시에 진행될 때 화면의 주문별 상태가 부정확할 수 있습니다. 이는
Fleet Manager의 로봇 배정과 실제 주행 명령에는 영향을 주지 않습니다.

## 작업공간 구조

```text
Rokey_Co3_A1/
├── nav_robot/          # Isaac Sim 독립 실행 브리지(nav_restaurant_demo.py): /scan, /odom, TF,
│                       #   direct_nav 미션 주행(주차 이탈, 주방 복귀, 테이블 도킹),
│                       #   CrossingPedestrian / TypingCustomer 테스트 액터
├── nav_robot5/         # two_wheel_rails ROS 2 패키지: Nav2 스택, nav2_collision_monitor,
│                       #   topic_bridge, routes.yaml(map 좌표계 도킹/중앙 통로 좌표), maps/
├── serving_robot/      # HMI 웹 대시보드 소스(src/serving_hmi) 및 run_hmi.sh
├── src/                # serving_robot_manager, serving_robot_interfaces, map_gen, hand_safety(무시되는 복사본)
├── hand_safety/        # 기준 손 감지 및 안전 ROS 2 패키지
├── .gitignore
└── README.md
```

`src/hand_safety`는 `COLCON_IGNORE`가 포함된 이전 복사본입니다.
루트의 `hand_safety` 패키지만 빌드합니다.

Isaac Sim 설치 경로는 `ISAAC_SIM_ROOT`로 설정할 수 있습니다. 기본값은
`~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release`입니다. Isaac Sim이
다른 위치에 설치되어 있으면 이 환경 변수를 설정하십시오.

각 장비의 `~/.bashrc`에 서로 다른 ROS 도메인을 설정하십시오. 저장소의
스크립트는 이 값을 상속하며 공용 도메인 번호를 하드코딩하지 않습니다.

```bash
export ROS_DOMAIN_ID=<장비별-도메인-번호>
export NAV_ROBOT_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export NAV_ROBOT5_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
```

`~/.bashrc`를 변경한 뒤 새 터미널을 여십시오.

## 멀티로봇 빠른 실행(터미널 3개)

통합 멀티로봇 모드는 Isaac Sim에 완전한 서빙 로봇 두 대를 생성하고,
각 로봇마다 네임스페이스가 분리된 Nav2, 안전, Manager 스택을 실행합니다.
RViz 창 하나에서 두 로봇을 함께 확인할 수 있습니다. Isaac 명령과 ROS
명령은 서로 다른 터미널에서 실행하고, Isaac 터미널에서는 ROS 환경을
`source`하지 마십시오.

최초 실행 전이나 ROS 패키지를 변경한 뒤 한 번 빌드합니다.

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 ./tools/colcon_safe.py build \
  --packages-up-to serving_robot_manager serving_hmi two_wheel_rails hand_safety \
  --executor sequential \
  --event-handlers console_start_end+ desktop_notification- status- terminal_title-
```

**터미널 1: 통합 로봇 두 대를 포함한 Isaac Sim**

```bash
cd /home/rokey/cobot3_ws
./tools/run_multi_integrated_isaac.sh
```

이 스크립트는 `NAV_MULTI_ROBOT=1`을 설정하고 Isaac ROS 2 브리지 라이브러리
경로를 구성한 뒤 `nav_robot/isaacpjt/nav_restaurant_demo.py`를 실행합니다.
식당과 `/World/NavRobot1`, `/World/NavRobot2` 인스턴스가 모두 로드될 때까지
기다리십시오.

**터미널 2: Nav2 스택 두 개, Fleet Manager, 안전 노드 및 RViz**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serving_robot_manager multi_robot.launch.py
```

이 launch는 `/robot1`, `/robot2` worker를 실행하고 RViz 창 하나에 두 로봇의
모델, LiDAR 스캔, 지역/전역 costmap, footprint, 정지/감속 영역을 표시합니다.

주문을 보내기 전에 두 worker에서 다음 로그가 출력될 때까지 기다리십시오.

```text
Navigation is fully initialized and ready!
```

네임스페이스별 Manager는 초기 주방 위치가 확인된 뒤에만 주문 서비스를
노출합니다.

**터미널 3: HMI**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serving_hmi hmi.launch.py
```

브라우저에서 `http://localhost:8000`을 여십시오. HMI 주문은
`/manager/order`로 전송됩니다. Fleet Manager는 첫 주문을 `robot1`에
배정하고, `robot1`이 주행 중일 때 들어온 다음 주문은 유휴 상태인
`robot2`에 배정합니다.

멀티로봇 launch의 기본값은 주행 전용 검증 모드입니다. 각 주문은 음식
스폰이나 로봇팔 서빙 없이 선택한 테이블까지 이동한 뒤 주방으로 복귀합니다.

나중에 전체 서빙 절차를 복원하려면 서빙 단계를 활성화하고, 공용 payload
prim에 대한 접근을 직렬화하십시오.

```bash
ros2 launch serving_robot_manager multi_robot.launch.py \
  navigation_only:=false serialize_shared_payloads:=true
```

HMI 없이 주행 전용 스모크 테스트를 실행하려면 다음 서비스를 호출합니다.

```bash
ros2 service call /robot1/navigation/command \
  serving_robot_interfaces/srv/TaskCommand "{command: 0}"
ros2 service call /robot2/navigation/command \
  serving_robot_interfaces/srv/TaskCommand "{command: 1}"
```

명령 `0`부터 `3`은 테이블을 선택하고, 명령 `4`는 해당 로봇을 주방으로
복귀시킵니다.

## 단일 로봇 통합 테스트(터미널 3개)

**터미널 1: Isaac Sim**

```bash
cd /home/rokey/cobot3_ws/nav_robot
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib"
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/nav_restaurant_demo.py
```

아래 HMI 테스트 버튼을 사용하기 전에 로그에서
`[typing_topic] waiting on /hand_test/type_keyboard ...`가 출력될 때까지
기다리십시오.

**터미널 2: Manager + Nav2 + collision_monitor + hand_safety**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  serving_robot_interfaces serving_robot_manager hand_safety serving_hmi two_wheel_rails
source install/setup.bash
ros2 launch serving_robot_manager vision_manager_nav2.launch.py \
  vision_debug:=true use_sim_time:=true autostart:=true rviz:=true
```

이 launch 하나가 `direct_nav_server_node`, `manager_node`,
`hand_detector_node`, Nav2 스택(AMCL/controller/planner/bt_navigator),
`nav2_collision_monitor`를 실행합니다. `nav2_collision_monitor`는 정지/감속
안전 영역을 적용해 `cmd_vel`을 `cmd_vel_safe`로 전달하며, Isaac의
NavBridge가 실제로 이 토픽을 사용해 주행합니다.

**터미널 3: HMI 웹 대시보드**

```bash
cd /home/rokey/cobot3_ws
./run_hmi.sh
```

브라우저에서 `http://localhost:8000`을 여십시오.

## HMI 테스트 제어

- **사람 스폰/제거** 버튼은 보행 테스트 액터인 `CrossingPedestrian`를
  생성하거나 제거합니다. 이 액터에는 LiDAR로 감지되는 collider가 있으며,
  로봇을 물리적으로 밀지 않도록 contact filtering이 적용됩니다.
- **타이핑 시작** 버튼은 `/hand_test/type_keyboard`를 통해
  `TypingCustomer`의 일회성 타이핑 애니메이션을 실행합니다.
  `hand_safety`의 ROI 침입 감지기가 이 동작을 검사합니다.
- 로봇 지도 패널은 현재 정지/감속 안전 영역을 표시합니다. 이 영역은
  `nav2_params.yaml`의 `nav2_collision_monitor` polygon과
  `nav_restaurant_demo.py`의 `OBSTACLE_STOP_*`/`OBSTACLE_SLOWDOWN_*` 값을
  반영합니다. 값을 조정할 때는 세 위치를 함께 변경하십시오.

## SLAM 지도 생성

`src/map_gen`은 별도 브리지 대신 Isaac의 기존 `/scan`, `/odom`, TF를
재사용하는 일회성 `slam_toolbox` 매핑 모드입니다. 두 실행이 모두
`cmd_vel`을 사용하므로 위 launch와 동시에 실행하지 마십시오.

전체 절차와 생성된 지도를 `nav_robot5/src/two_wheel_rails/maps/`에 적용하는
방법은 [src/map_gen/README.md](src/map_gen/README.md)를 참고하십시오.
