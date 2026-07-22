# hand_safety

ROS 2 Humble에서 `sensor_msgs/msg/Image` RGB 토픽을 구독해 HaGRIDv2
YOLOv10n 손 탐지를 수행합니다. 원본 모델의 제스처 클래스는 모두
`hand` 단일 클래스로 통합하고 class-agnostic NMS를 적용합니다.

## 토픽

- 입력: `/camera/color/image_raw`
  (`sensor_msgs/msg/Image`)
- 박스가 표시된 영상: `/hand_detection/image` (`sensor_msgs/msg/Image`,
  디버그 옵션을 켰을 때만 발행)
- 탐지 결과: `/hand_detection/detections` (`std_msgs/msg/String`, JSON)
- ROI 침입 상태: `/hand_safety/intrusion` (`std_msgs/msg/Bool`)
- 도착 상태 입력: `/serving_robot/table_arrived` (`std_msgs/msg/Bool`)

`/hand_detection/detections`에는 단일 클래스 ID `0`, 클래스 이름
`hand`, confidence, `[x1, y1, x2, y2]` 형식의 bounding box가
포함됩니다.

고정 테이블 ROI는 `hand_safety/roi_intrusion.py` 상단의
`TABLE_ROI_NORMALIZED`에 화면 비율 기준 네 꼭짓점으로 설정합니다.
손 바운딩 박스가 테이블 폴리곤과 조금이라도 겹치면 침입으로 판단하며, ROI와
겹치는 손만 검출 영상과 결과 메시지에 표시합니다.

손 추론은 이동 로봇이 테이블 도킹을 완료하여
`/serving_robot/table_arrived`가 `true`일 때만 실행됩니다. 이동 요청을
받으면 도착 상태가 즉시 `false`로 바뀌고, ROI 침입 출력도 즉시
`false`로 초기화됩니다. 도착 상태를 아직 받지 못한 경우에도 탐지는
비활성 상태를 유지합니다. 시뮬레이터 시작 위치가 도킹 허용오차 안이면
해당 초기 위치도 정상 도착 상태로 발행합니다.

## 빌드

```bash
cd "$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hand_safety
source install/setup.bash
```

손 검출 모델은 패키지와 함께 설치되며 노드가 설치 경로에서 자동으로
찾습니다.

## 실행

```bash
export ROS_DOMAIN_ID=102
ros2 run hand_safety hand_detector_node
```

이 프로젝트의 Isaac Sim과 통신할 때는 두 터미널 모두 같은 도메인을
사용합니다.

```bash
export ROS_DOMAIN_ID=102
```

입력 토픽을 변경하는 예:

```bash
ros2 run hand_safety hand_detector_node \
  --ros-args \
  -p input_topic:=/camera/color/image_raw
```

CUDA가 사용 가능한 환경에서 GPU를 지정하는 예:

```bash
ros2 run hand_safety hand_detector_node \
  --ros-args \
  -p device:=0
```

OpenCV 창을 표시하지 않으려면 다음과 같이 실행합니다.

```bash
ros2 run hand_safety hand_detector_node \
  --ros-args \
  -p show_window:=false
```

기본 실행은 안정성을 위해 OpenCV 창과 1280×960 결과 영상 발행을 모두
끄고, 테이블 ROI에 여백을 더한 영역 한 장만 1280 크기로 추론하여 JSON 및
ROI 상태만 발행합니다. 이 방식은 손을 확대하면서 ROI 밖 로봇 팔을 입력에서
제외하고, Isaac Sim과 같은 GPU에서 네 장의 타일 추론을 동시에 돌리지 않아
GPU 부하와 응답 지연도 줄입니다. 박스 영상을 확인할 때만 다음처럼
일시적으로 활성화합니다.

```bash
ros2 run hand_safety hand_detector_node \
  --ros-args \
  -p publish_annotated_image:=true
```

이 경우 별도 터미널에서 다음 명령으로 영상을 확인합니다.

```bash
ros2 run rqt_image_view rqt_image_view /hand_detection/image
```

Manager와 detector를 항상 함께 실행하려면 워크스페이스 루트에서
`serving_robot_manager`도 같이 빌드한 뒤 통합 launch를 사용합니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=102
source install/setup.bash
ros2 launch serving_robot_manager vision_manager.launch.py
```

박스와 ROI가 표시된 비전 화면을 발행하려면:

```bash
ros2 launch serving_robot_manager vision_manager.launch.py vision_debug:=true
```

## 확인

```bash
ros2 topic info /camera/color/image_raw
ros2 topic hz /camera/color/image_raw
ros2 topic echo /serving_robot/table_arrived
ros2 topic echo /hand_safety/intrusion
ros2 topic echo /hand_detection/detections
ros2 run rqt_image_view rqt_image_view /hand_detection/image
```

입력 토픽은 `sensor_msgs/msg/Image`여야 합니다.
`sensor_msgs/msg/CompressedImage` 토픽은 이 노드에서 직접 구독하지
않습니다.

## 알려진 캘리브레이션 이슈

`vision-test` GPU 검증에서 RG2 그리퍼가 손으로 오탐되어 ROI 침입이
계속 `true`로 나오는 케이스가 확인됐습니다. 카메라 구도별 그리퍼
박스 좌표가 제공되지 않아 임의 제외 마스크는 적용하지 않았습니다.
실제 프레임에서 ROI 다각형 또는 오탐 제외 영역을 다시 캘리브레이션해야
최종 안전 판정을 신뢰할 수 있습니다.
