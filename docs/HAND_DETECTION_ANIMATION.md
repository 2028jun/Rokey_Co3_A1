# 손 탐지 타이핑 애니메이션 구조

## 목적

Isaac Sim의 테이블 카메라 영상에서 사람이 타이핑할 때 손이 테이블 ROI를
침범하는지 ROS 2로 탐지한다. ROS 통신은 `ROS_DOMAIN_ID=101`을 사용한다.

## 동작 흐름

1. `mobile_manipulator_demo.py`가 식당, 로봇, 고정 테이블 카메라와 Actor SDG
   사람을 생성한다.
2. 사람은 로봇에서 먼 `TableSet_00/Chair_00_Visual` 앞
   `(-3.70, -3.10, 0.0)`, 테이블 방향 `180도`로 배치된다.
3. `TypingTopicController`가 `/hand_test/type_keyboard`
   (`std_msgs/msg/Empty`)를 기다린다.
4. 토픽을 받으면 현재 위치와 회전을 고정한 채 `type_keyboard` AnimationGraph
   상태를 10초 동안 실행한다.
5. 10초 후 `Action=None`으로 복귀하고 저장된 IDLE 위치와 회전을 복원한다.
   타이핑 중 들어온 중복 요청은 무시한다.
6. Isaac Sim은 `/serving_robot/table_camera/color/image_raw`를 발행하고,
   `hand_detector_node`가 YOLO 손 검출과 ROI 침범 판정을 수행한다.
7. `/serving_robot/table_arrived=true`일 때만 추론하며 결과는
   `/hand_detection/image`, `/hand_detection/detections`,
   `/hand_safety/roi_intrusion`으로 발행한다.
8. 별도 보행자는 주방 출구 앞쪽 통로의
   `(-5.40, 3.00, 0.0) <-> (5.40, 3.00, 0.0)` 구간을 X축으로
   계속 왕복한다. 좌우 벽에서 각각 0.60m 안쪽까지 이동하면서 주방에서
   Y축으로 나오는 로봇 경로를 직각으로 가로지르고, 테이블·의자·후방
   화분 영역에는 들어가지 않는다.

## 주요 코드

- `isaacpjt/mobile_manipulator_demo.py`
  - 식당·로봇·카메라 생성
  - Actor SDG 및 `TypingTopicController` 생성과 프레임별 `update()` 호출
  - ROS 카메라 토픽은 유지하지만 Isaac Sim 내부의 별도 카메라 미리보기 창은
    생성하지 않는다.
- `isaacpjt/actor_sdg_test_actor.py`
  - `type_keyboard.skelanim.usd`를 AnimationGraph 커스텀 상태로 등록
  - IDLE 자세 고정, ROS 타이핑 트리거 수신, 10초 실행과 자세 복귀 담당
  - 사전 작성 커맨드 루프, `Sit`, `push_button`, 자동 복귀 `GoTo`를 사용하지
    않아 루트 회전 누적과 바닥 침하를 방지한다.
  - `CrossingPedestrianController`가 두 번째 사람의 X축 왕복 경로를
    유지하고 Y/Z 이탈을 중앙 통로로 복구한다. 끝점에서도 `Walk` 상태를
    유지해 IDLE 전환에 따른 자세 뒤틀림을 피한다.
- `hand_safety/hand_safety/roi_intrusion.py`
  - `origin/woduq`와 같은 정규화 테이블 다각형 ROI를 사용한다.
- `hand_safety/hand_safety/hand_detector_node.py`
  - 카메라 수신, YOLO 추론, ROI 교차 판정, 주석 영상과 상태 토픽 발행 담당

## 실행

전체 터미널별 실행 명령은 `hand_safety/README.md`의
`GPU 시각 검증 워크플로`를 따른다. 타이핑은 다음 명령으로 한 번 실행한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=101
ros2 topic pub --once \
  /hand_test/type_keyboard std_msgs/msg/Empty "{}"
```

기본 타이핑 시간은 10초이며 필요하면 Isaac Sim 실행 전에
`HAND_TEST_TYPING_SECONDS`로 변경할 수 있다.

보행자는 기본 활성화된다. 끄려면 Isaac Sim 실행 전에
`MOBILE_DEMO_CROSSING_PEDESTRIAN=0`을 설정한다. 경로는
`CROSSING_PEDESTRIAN_LEFT_X`, `CROSSING_PEDESTRIAN_RIGHT_X`,
`CROSSING_PEDESTRIAN_Y`로 조정할 수 있다.
