# ws_cobot2_pjt MoveIt 2 이식 기록 및 부족분

## 출처

- 원본: https://github.com/kimjihoon001/ws_cobot2_pjt
- 브랜치: `main`
- 기준 커밋: `4c25e1705fcff371d0ed6ab2c75440524222440d`
- 원본 경로: `src/cobot2_ws/m0609_rg2_moveit`
- 가져온 경로: `src/m0609_rg2_moveit`

`m0609_rg2_moveit` 패키지를 기준본으로 가져온 뒤 Isaac Sim 첫 통신 시험에 맞게
수정했다. 최종 실행 방법은 `ISAAC_MOVEIT_QUICKSTART.md`를 따른다.

## 가져온 구성

- M0609 `manipulator` SRDF 그룹
- `rg2_tcp` end-effector
- KDL kinematics 설정
- OMPL/RRTConnect 설정
- M0609 joint limits
- `FollowJointTrajectory` controller 매핑
- `move_group`, RViz launch 및 RViz 설정

## 그대로 가져오지 않은 구성

원본 `m0609_rg2_bringup`은 아래 기능이 한 패키지에 결합되어 있어 현재 프로젝트의
Isaac Sim 5.1 + Ridgeback 구조에 바로 사용할 수 없다.

- 실제 DSR hardware와 DRCF Docker emulator
- `dsr_controller2` ros2_control
- 실제 OnRobot Modbus driver
- RealSense D435 및 카메라 장착 브라켓
- 고정형 `world -> base_link` static TF
- 컨베이어/공구 데모 Planning Scene

특히 현재 모바일 베이스에서는 `world -> base_link`를 static TF로 발행하면 Nav2의
`map -> odom -> base_link` 트리와 충돌한다. 따라서 bringup은 Isaac Sim 전용으로
새로 분리해야 한다.

## 1차 Isaac 통합에서 완료한 작업

### 로봇 description

- `m0609_rg2_bringup` 의존성을 `m0609_isaac_description`으로 교체 완료
- 기존 Isaac Sim M0609 URDF를 설치 가능한 package 구조로 변환 완료
- RG2 고정 충돌 프록시와 `rg2_tcp` 추가 완료
- 절대 mesh 경로를 `package://` URI로 변경 완료
- Ridgeback의 실제 장착 링크를 `base_link`와 팔 base 사이에 추가
- SRDF robot name과 실제 URDF robot name 일치

### controller 연결

- 원본 controller 이름은 다음과 같다.
  - `/dsr01/dsr_moveit_controller/follow_joint_trajectory`
  - `/dsr01/rg2_gripper_controller/follow_joint_trajectory`
- `/isaac_arm_controller/follow_joint_trajectory` bridge 구현 완료
- `/isaac_joint_states`를 `/joint_states`로 중계하도록 구현 완료
- MoveIt trajectory를 `/isaac_joint_commands`로 보간하도록 구현 완료
- DSR 실기기 launch와 Isaac Sim launch를 별도 파일로 유지한다.

### SRDF/kinematics 검증

- `manipulator` chain의 `base_link -> rg2_tcp`가 현재 URDF에서 실제로 연결되는지 확인
- RG2 단일 joint KDL group 제거 완료
- SRDF의 카메라/브라켓 invalid link 제거 완료
- 기존 disable-collisions 목록은 새 통합 모델로 MoveIt Setup Assistant에서 재검증
- `FixedBase(world, base_link)` virtual joint는 모바일 베이스 모델에 맞게 교체

### 피자 서빙 안전 설정

- 기본 velocity/acceleration scaling을 `0.20/0.10`으로 하향 완료
- joint acceleration 값이 실제 M0609/Isaac controller 제한과 맞는지 확인해야 한다.
- 피자판 TCP 기준 `OrientationConstraint`는 아직 없다.
- 피자/피자판 attach/detach 및 touch links 설정이 없다.
- 손 collision object 갱신, TTL, filtering, 실행 중 cancel/replan 로직이 없다.
- 근접 동적 회피가 필요하면 MoveIt Servo 또는 별도 정지 감시 계층이 필요하다.

### 모바일 매니퓰레이터 연동

- `map -> odom -> ridgeback base -> arm mount -> M0609 -> rg2_tcp` TF 확정
- Nav2 주행 중 팔 controller interlock과 운반 자세 정의
- 주행 완료 후 MoveIt 동작을 시작하는 상태 머신
- Isaac Sim `/clock`, 전체 노드 `use_sim_time`, `ROS_DOMAIN_ID=102` 검증

## 재사용 가치가 높은 원본 코드

아직 가져오지는 않았지만 다음 코드는 기능 구현 시 선별 이식할 가치가 있다.

- `m0609_rg2_bringup/scripts/moveit_joint_line_demo.py`
  - `/move_action` 직접 호출
  - 현재 joint 검증
  - 속도/가속도 scaling
  - MoveIt error code 진단
- `m0609_rg2_bringup/scripts/hand_avoidance.py`
  - 손 collision object와 경로 계획의 초기 참고 구현
- `m0609_rg2_bringup/scripts/scene_builder.py`
  - Planning Scene 객체 등록 방식 참고

이 스크립트들은 고정 자세, 공구 데모 객체, 기존 frame/controller 이름을 포함하므로
그대로 실행하지 않고 현재 피자 시나리오와 Isaac Sim frame에 맞춰 이식한다.
