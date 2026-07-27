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

나중에 두 로봇의 전체 서빙을 동시에 활성화하려면 먼저 전역 payload USD
prim을 로봇별 경로로 분리해야 합니다. 분리 완료 전의
`serialize_shared_payloads:=true`는 한 번에 한 로봇만 시험하기 위한 임시
보호책이며 최종 멀티로봇 구조가 아닙니다. 분리와 개별 검증이 끝난 뒤에는
직렬화를 끄고 두 로봇 동시 서빙을 검증합니다.

```bash
ros2 launch serving_robot_manager multi_robot.launch.py \
  navigation_only:=false serialize_shared_payloads:=false
```

### 다른 컴퓨터에서 YOLO 실행

Isaac/Manager 컴퓨터와 YOLO 컴퓨터는 같은 유선 LAN에 연결하고 동일한
`ROS_DOMAIN_ID`를 사용합니다. 저장소 코드에 domain ID를 고정하지 않으므로
두 터미널에서 같은 값을 직접 설정하십시오. 두 컴퓨터 모두
`ROS_LOCALHOST_ONLY=0`이어야 합니다.

Isaac/Manager 컴퓨터에서는 로컬 YOLO 두 개만 비활성화하고 나머지 통합
노드는 그대로 실행합니다.

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0                 # 두 컴퓨터에서 같은 값 사용
export ROS_LOCALHOST_ONLY=0
ros2 launch serving_robot_manager multi_robot.launch.py \
  navigation_only:=false \
  serialize_shared_payloads:=false \
  enable_serving_workers:=true \
  enable_local_hand_detection:=false
```

YOLO 컴퓨터에도 이 저장소와 ROS 2 Humble, CUDA 지원 PyTorch가 필요합니다.
최초 한 번 다음 패키지를 빌드합니다.

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 ./tools/colcon_safe.py build \
  --packages-up-to hand_safety serving_robot_manager \
  --executor sequential \
  --event-handlers console_start_end+ desktop_notification- status- terminal_title-
```

그다음 YOLO 컴퓨터에서 두 로봇 감지기를 실행합니다.

```bash
cd /home/rokey/cobot3_ws
export ROS_DOMAIN_ID=0                 # Isaac/Manager 컴퓨터와 같은 값
export ROS_LOCALHOST_ONLY=0
./tools/run_remote_yolo.sh
```

외부 감지기는 기존 토픽 계약을 변경하지 않습니다.

- 입력: `/robot1/camera/color/image_raw`, `/robot2/camera/color/image_raw`
- 서빙 상태 입력: `/robot1/serving_robot/table_arrived`,
  `/robot2/serving_robot/table_arrived`
- 안전 출력: `/robot1/hand_safety/intrusion`,
  `/robot2/hand_safety/intrusion`

연결 확인은 양쪽 컴퓨터에서 다음과 같이 합니다.

```bash
ros2 topic hz /robot1/camera/color/image_raw
ros2 topic hz /robot2/camera/color/image_raw
ros2 node list | grep hand_detector
ros2 topic echo /robot1/hand_safety/intrusion
```

카메라 해상도는 `1280x960`으로 유지되고 기본 발행률은 로봇당 15 Hz입니다.
무압축 RGB 두 스트림은 이론상 약 111 MB/s이므로 Wi-Fi보다 1 GbE 이상의
유선 네트워크를 권장합니다. 패킷 손실이 있으면 해상도를 낮추지 말고 먼저
ROS image transport 압축 또는 2.5 GbE를 적용하십시오.

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

## 2대 주행 성공 후 전체 서빙 모드 전환

이 절은 멀티로봇 주행 전용 검증이 성공한 뒤 음식 스폰, 로봇팔 서빙,
손 침입 안전 기능을 다시 활성화할 때 사용하는 월요일 작업 체크리스트입니다.
전체 서빙 모드는 launch를 다시 시작해야 적용됩니다. 다만 현재 전역 payload
prim을 로봇별로 분리하기 전에는 두 로봇 전체 서빙을 동시에 실행하면 안 됩니다.

먼저 피자, 음료, 식기, 접시 랙과 배달 완료 보관 경로를 로봇별
`payload_root` 아래로 분리합니다. 권장 구조는 다음과 같습니다.

```text
/World/RobotPayloads/robot1/ServingDish
/World/RobotPayloads/robot1/ServingDrinks
/World/RobotPayloads/robot1/ServingCutlery
/World/RobotPayloads/robot1/ServingPlateRack
/World/RobotPayloads/robot2/ServingDish
/World/RobotPayloads/robot2/ServingDrinks
/World/RobotPayloads/robot2/ServingCutlery
/World/RobotPayloads/robot2/ServingPlateRack
/World/Delivered/robot1/...
/World/Delivered/robot2/...
```

`nav_restaurant_demo.py`에서 각 `NavBridge`의 로봇 이름으로 `payload_root`를
결정하고, 이 값을 피자, 음료, 식기, 접시 랙의 생성, 검색, 유효성 검사,
로봇팔 pick/place 및 archive 코드 전체에 전달해야 합니다. 경로 문자열 일부만
바꾸면 남은 전역 경로가 다른 로봇의 payload를 삭제하거나 집을 수 있으므로
아래 경로를 사용하는 모든 코드를 함께 변경합니다.

```text
/World/ServingDish
/World/PizzaBoardBail
/World/PizzaBoardBailHinge
/World/PizzaBoardGripBearing
/World/PizzaBoardGripBlock
/World/ServingDrinks
/World/ServingCutlery
/World/ServingPlateRack
/World/Delivered
```

분리 완료 후 실행 중인 주행 전용 launch를 `Ctrl+C`로 종료하고 다음과 같이
전체 멀티로봇 서빙을 실행합니다.

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serving_robot_manager multi_robot.launch.py \
  navigation_only:=false \
  serialize_shared_payloads:=false \
  enable_serving_workers:=true
```

`navigation_only:=false`는 음식 스폰, 로봇팔, 손 안전 단계를 활성화합니다.
`serialize_shared_payloads:=false`는 로봇별 prim 분리와 개별 서빙 검증을 모두
통과한 뒤에만 사용합니다. 분리 작업 전에는 아래 명령으로 한 번에 한 로봇만
통합 시험할 수 있습니다. 이 모드는 구조적 해결이 아니라 충돌 방지용 임시
시험 모드입니다.

```bash
ros2 launch serving_robot_manager multi_robot.launch.py \
  navigation_only:=false \
  serialize_shared_payloads:=true \
  enable_serving_workers:=true
```

### 전환 전에 알아둘 동작 차이

- 주행 전용 상태 순서 `3 -> 1 -> 6` 대신 일반적으로
  `2 -> 3 -> 4 -> 1 -> 6` 순서로 진행합니다.
- `2`는 음식 스폰, `3`은 테이블 이동, `4`는 로봇팔 서빙, `5`는 손 침입
  일시정지, `1`은 주방 복귀, `6`은 완료, `7`은 실패입니다.
- 초기 주방 위치가 불확실하면 `2` 앞에 `1`이 한 번 나타날 수 있습니다.
- prim 분리 전 임시 직렬화 시험에서는 첫 로봇이 완료되기 전 두 번째 HMI
  주문이 대기열에 저장되지 않고 거부되어 HMI에서 취소로 표시될 수 있습니다.
  이 단계에서는 반드시 주문을 한 건씩 완료한 뒤 다음 주문을 넣습니다.
- prim 분리 후에는 `serialize_shared_payloads:=false`에서 robot1과 robot2가
  서로 다른 payload만 생성, 조작, 삭제, archive하는지 먼저 확인한 뒤 연속
  주문으로 동시 서빙을 시험합니다.
- Fleet Manager는 유휴 후보 중 항상 `robot1`을 먼저 고릅니다. `robot1`이
  완료된 뒤 HMI로 다음 주문을 보내면 다시 `robot1`이 선택될 수 있으므로,
  `robot2` 전체 서빙 검증은 아래의 `/robot2/manager/order` 직접 호출을
  사용합니다.

### 1. 전체 서빙 노드와 연결 확인

전체 서빙 launch 실행 후 다음 노드가 로봇별로 존재하는지 확인합니다.

```bash
ros2 node list | grep -E '/robot[12]/(manager|isaac_subsystem_adapter|hand_detector|navigation_subsystem)'
```

다음 서비스와 토픽이 모두 보여야 합니다.

```bash
ros2 service list | grep -E '/robot[12]/(manager/order|navigation/command|food_spawn/command|arm/command)'
ros2 topic list | grep -E '/robot[12]/(food_spawn/status|arm/status|hand_safety/intrusion|serving_robot/table_arrived|camera/color/image_raw)'
```

`isaac_subsystem_adapter`는 ROS 2 Humble의 `TaskCommand` 서비스를 Isaac의
`Int32` trigger/status 토픽으로 변환합니다. 다음 두 노드 정보를 확인해
`arm/command`, `food_spawn/command` 서버가 중복 생성되지 않았는지 봅니다.

```bash
ros2 node info /robot1/isaac_subsystem_adapter
ros2 node info /robot1/nav_robot_isaac_bridge_robot1
ros2 node info /robot2/isaac_subsystem_adapter
ros2 node info /robot2/nav_robot_isaac_bridge_robot2
```

정상 구성에서는 adapter가 `/robotN/arm/command`와
`/robotN/food_spawn/command`를 제공합니다. Isaac Python이
`serving_robot_interfaces`까지 불러와 같은 서비스를 직접 제공하면 서버가
중복될 수 있습니다. 이 경우 어느 서버가 응답할지 보장할 수 없으므로 Isaac
터미널의 `PYTHONPATH`와 ROS 환경 혼용부터 정리해야 합니다.

카메라와 손 안전 heartbeat도 주문 전에 확인합니다.

```bash
ros2 topic hz /robot1/camera/color/image_raw
ros2 topic hz /robot2/camera/color/image_raw
ros2 topic echo /robot1/hand_safety/intrusion --once
ros2 topic echo /robot2/hand_safety/intrusion --once
```

카메라 프레임이 없으면 테이블 도착 후 Manager가 첫 손 안전 샘플을 기다리며
상태 `5`에서 진행하지 못합니다. CUDA, YOLO 모델 경로, 카메라 토픽과
`hand_detector` 로그를 먼저 해결하십시오.

### 2. robot1 최소 서빙 시험

처음에는 한 종류와 한 개만 주문합니다. 피자 1개 또는 음료 1개처럼 가장
단순한 주문으로 스폰과 로봇팔의 기본 경로를 확인하십시오. HMI를 사용하거나
다음 서비스를 직접 호출할 수 있습니다.

```bash
ros2 service call /robot1/manager/order \
  serving_robot_interfaces/srv/OrderRequest \
  "{table_id: 0, pizza1_count: 1, pizza2_count: 0, pizza3_count: 0, drink_count: 0, cutlery_count: 0, plate_count: 0}"
```

별도 진단 터미널에서 상태를 관찰합니다.

```bash
ros2 topic echo /robot1/system/status
ros2 topic echo /robot1/food_spawn/status
ros2 topic echo /robot1/arm/status
ros2 topic echo /robot1/navigation/detail
```

다음을 모두 확인해야 합격입니다.

- 음식 스폰 상태가 `1 -> 2`로 진행하고 필요한 USD prim이 실제로 보임
- 테이블 이동이 `SUCCEEDED/completed`로 끝남
- 테이블 도착 시 `/robot1/serving_robot/table_arrived`가 `true`가 됨
- 손이 없을 때 `/robot1/hand_safety/intrusion`이 `false`로 새로 발행됨
- 로봇팔 상태가 `1 -> 2`로 진행하고 음식이 테이블에 놓임
- 로봇팔이 stow 자세로 돌아가고 트레이가 완전히 수납됨
- 로봇이 자기 주방 슬롯으로 복귀하고 시스템 상태가 `6`이 됨

### 3. robot2 개별 서빙 시험

공용 Fleet 서비스 대신 robot2 Manager를 직접 호출하여 robot2의 카메라,
로봇팔 좌표계, payload 접근과 주방 복귀를 독립적으로 확인합니다. robot1의
주문이 완전히 끝나고 상태가 `0` 또는 `6`인 상태에서 실행하십시오. prim 분리
전에는 반드시 `serialize_shared_payloads:=true`로 실행해야 합니다.

```bash
ros2 service call /robot2/manager/order \
  serving_robot_interfaces/srv/OrderRequest \
  "{table_id: 1, pizza1_count: 1, pizza2_count: 0, pizza3_count: 0, drink_count: 0, cutlery_count: 0, plate_count: 0}"
```

robot2에서도 `2 -> 3 -> 4 -> 1 -> 6` 상태, 실제 음식 배치, 팔 stow,
트레이 수납, `(0.90, 5.25)` 주방 슬롯 복귀를 확인합니다. 분리 후에는 모든
생성 prim이 `/World/RobotPayloads/robot2` 아래에만 생기고 robot1 경로에
변경이 없는지 확인합니다. 물체가 robot1 트레이 위치에 생성되거나 robot2 팔의
도달 좌표가 어긋나면 동시 서빙 시험으로 넘어가지 않습니다.

### 4. 메뉴 기능을 단계적으로 확대

피자 1개 성공 직후 복합 주문으로 넘어가지 말고 아래 순서로 한 항목씩
추가합니다. 각 시험은 앞 주문이 상태 `6`으로 완전히 끝난 뒤 시작합니다.

1. 피자 종류별 1개: `pizza1`, `pizza2`, `pizza3`
2. 음료 1개, 음료 2개
3. 식기 리필 1세트
4. 접시 랙 1개부터 4개
5. 피자 1개 + 음료 1개
6. 피자 + 음료 2개 + 식기
7. 여러 트립으로 분할되는 주문

현재 Manager는 한 트립에 음료를 최대 4개까지 허용하지만 Isaac 로봇팔은
코드상 검증된 `soda1`, `soda2` 두 개만 지원하며 음료 3개 이상이면 실패합니다.
이 제한을 수정하고 별도 검증하기 전까지 실제 서빙 주문의 음료는 최대 2개로
제한하십시오.

스폰 adapter는 `/food_spawn/status=2`를 최대 20초 동안 기다립니다. 무거운
payload 생성이 20초를 넘으면 Isaac에서 계속 처리 중이어도 Manager에는 명령
실패로 전달됩니다. 실제 측정 시간이 20초에 근접하면 adapter timeout을
파라미터화하거나 acceptance-only 방식으로 바꾸고, 완료 판단은 Manager의
status 구독 한 곳에서만 담당하도록 정리해야 합니다.

### 5. 손 안전 정지 및 재개 시험

기본 서빙이 성공한 뒤에만 손 침입 시험을 진행합니다.

1. 로봇이 테이블에 도착하고 상태 `4`에서 팔이 움직이는 것을 확인합니다.
2. 카메라 ROI 안에 손을 넣고 5개 연속 프레임 이상 유지합니다.
3. 시스템 상태가 `5`, arm pause 명령이 `99`가 되는지 확인합니다.
4. 손을 뺀 뒤 intrusion이 `false`로 바뀌고 resume 명령 `98`로 재개되는지
   확인합니다.
5. 재개된 동작이 중복 스폰이나 처음부터 재시작 없이 완료되는지 확인합니다.

```bash
ros2 topic echo /robot1/serving_robot/table_arrived
ros2 topic echo /robot1/hand_safety/intrusion
ros2 topic echo /robot1/arm/status
ros2 topic echo /robot1/system/status
```

손 감지 토픽은 서빙 중 2초 이상 끊기면 fail-safe로 상태 `7`이 됩니다.
두 카메라를 동시에 처리할 때 GPU 과부하나 프레임 지연이 생기는지도 확인하고,
필요하면 추론 주기와 입력 크기를 실제 측정값으로 조정합니다.

### 6. 실패 복구와 재시험

상태 `7`이 되면 즉시 새 주문을 넣지 말고 Isaac에서 팔, 트레이, payload와
로봇 주행이 실제로 정지했는지 확인합니다. 정지 확인 후 fault를 초기화합니다.

```bash
ros2 service call /manager/reset_fault std_srvs/srv/Trigger "{}"
```

Fleet reset은 실패한 로봇 Manager에 reset을 전달합니다. 계속 거부되면 해당
로봇의 상태를 직접 확인합니다.

```bash
ros2 topic echo /robot1/system/status --once
ros2 topic echo /robot1/arm/status --once
ros2 topic echo /robot1/food_spawn/status --once
ros2 topic echo /robot1/navigation/status --once
```

팔 또는 스폰 상태가 아직 `1`이면 reset이 거부되는 것이 정상입니다. USD prim,
tensor handle, 트레이 또는 로봇팔 상태가 꼬였으면 같은 프로세스에서 주문만
재시도하지 말고 ROS launch와 Isaac Sim을 모두 종료한 뒤 깨끗한 stage에서
재실행하십시오.

### 7. 실제 운영 전 남은 설계 작업

- 최우선 작업은 전역 `/World/Serving*`, 피자 보드 부속 prim과
  `/World/Delivered`를 로봇별 `payload_root`로 분리하는 것입니다. 생성과
  로봇팔 pick/place뿐 아니라 삭제, 재사용, 유효성 검사, 실패 정리와 archive
  경로까지 같은 root를 사용해야 합니다.
- prim 분리 완료 후 `serialize_shared_payloads:=false`로 두 로봇의 연속 주문을
  실행하여 payload 경로와 로봇팔 조작 대상이 완전히 독립적인지 검증합니다.
  검증이 끝나면 `serialize_shared_payloads` 임시 보호책을 제거할 수 있습니다.
- prim 분리 전 임시 직렬화 중 들어온 주문을 현재는 거부하므로, 그 단계에서는
  주문을 한 건씩 시험합니다. 운영용 대기열이 필요한지는 prim 분리 후 실제
  Fleet 처리량과 HMI 동작을 기준으로 별도 결정합니다.
- HMI의 `active_order_id`와 `/system/status`는 전역 단일 값이므로 로봇별 주문
  상태, 카메라 선택, 장애 복구 상태를 정확히 표시하도록 확장해야 합니다.
- HMI 카메라와 지도는 기본적으로 robot1만 표시합니다. robot2 선택 또는
  두 화면 동시 표시 기능이 필요합니다.
- 음료 3~4개에 대한 Manager와 Isaac 로봇팔의 수량 제한을 일치시켜야 합니다.
- food spawn의 20초 adapter timeout과 Manager의 60초 timeout 책임을 한 곳으로
  통합해야 합니다.
- robot2의 스폰 위치, 로봇팔 기준 좌표, 테이블 배치가 robot1 전용 전역 USD
  경로에 의존하지 않는지 실기 확인 후 로봇별 자산 구조를 결정해야 합니다.
- 로봇 간 충돌 방지와 교착 회피는 별도 담당 작업입니다. 단순 상호 정지 감지를
  이 체크리스트 작업에 임의로 추가하지 않습니다.
