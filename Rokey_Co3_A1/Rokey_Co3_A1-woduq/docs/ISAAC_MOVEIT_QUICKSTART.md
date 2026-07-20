# Isaac Sim 5.1 + M0609 + MoveIt 2 빠른 실행

## 구성

이 통합은 다음 세 계층으로 나뉜다.

1. Isaac Sim OmniGraph
   - `/isaac_joint_states` 발행
   - `/isaac_joint_commands` 구독
   - `/clock` 발행
2. `m0609_isaac_control/trajectory_bridge`
   - Isaac joint state를 표준 `/joint_states`로 중계
   - `/isaac_arm_controller/follow_joint_trajectory` 제공
   - MoveIt trajectory를 100 Hz joint command로 보간
3. `m0609_rg2_moveit`
   - M0609 planning, collision checking, trajectory execution

현재 첫 통합은 M0609 여섯 관절만 제어한다. RG2는 `rg2_tcp`와 보수적인 고정
충돌 프록시로 planning model에 포함된다. 실제 그리퍼 개폐는 이후 별도 controller로
추가한다.

## 빌드

ROS 2 Humble과 MoveIt 2가 설치된 터미널에서 실행한다.

현재 PC에 MoveIt 2가 없다면 관리자 권한으로 먼저 설치한다.

```bash
sudo apt-get install -y ros-humble-moveit ros-humble-control-msgs ros-humble-xacro
```

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=101
colcon build --symlink-install \
  --packages-select m0609_isaac_description m0609_isaac_control m0609_rg2_moveit
source install/setup.bash
```

## 실행

터미널 1: Isaac Sim 전용 Python으로 시뮬레이터를 실행한다.

```bash
export ROS_DOMAIN_ID=101
/home/rokey/cobot3_ws/isaacpjt/M0609/run_moveit_bridge.sh
```

헤드리스 브리지 smoke test는 제한된 step 이후 자동 종료한다.

```bash
ISAAC_HEADLESS=1 ISAAC_TEST_STEPS=10 \
  /home/rokey/cobot3_ws/isaacpjt/M0609/run_moveit_bridge.sh
```

터미널 2: MoveIt과 trajectory bridge를 실행한다.

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=101
ros2 launch m0609_rg2_moveit moveit.launch.py
```

터미널 3: 먼저 계획만 검사한다.

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=101
ros2 run m0609_isaac_control moveit_joint_test --plan-only
```

계획이 성공하면 실제 Isaac 관절 실행을 검사한다.

```bash
ros2 run m0609_isaac_control moveit_joint_test
```

사용자 지정 목표 여섯 관절은 degree 단위다.

```bash
ros2 run m0609_isaac_control moveit_joint_test --target-deg 10 -15 65 0 70 5
```

## 확인해야 할 인터페이스

```bash
ros2 topic hz /clock
ros2 topic hz /isaac_joint_states
ros2 topic echo /joint_states --once
ros2 action list | grep follow_joint_trajectory
ros2 action info /isaac_arm_controller/follow_joint_trajectory
ros2 action info /move_action
```

## 현재 제한사항

- MoveIt model의 RG2는 고정 충돌 프록시이며 개폐하지 않는다.
- `world -> base_link`는 첫 단독 팔 테스트용 static TF다. Ridgeback/Nav2를 붙일 때
  이 노드를 끄고 모바일 베이스 TF를 사용해야 한다.
- Planning model의 RG2 TCP 오프셋은 기존 실기 프로젝트 값 `0.231066 m`를 사용한다.
  Isaac USD의 실제 TCP와 TF를 측정해 최종 검증해야 한다.
- 손 회피, 피자 attach, 수평 orientation constraint는 controller 통신 시험 이후 단계다.
- 시뮬레이션을 pause하면 `/clock`이 멈추므로 MoveIt timeout이 발생할 수 있다.
