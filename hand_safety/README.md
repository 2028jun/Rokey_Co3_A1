# hand_safety

ROS 2 Humble에서 `sensor_msgs/msg/Image` RGB 토픽을 구독해 HaGRIDv2
YOLOv10n 손 탐지를 수행합니다. 원본 모델의 제스처 클래스는 모두
`hand` 단일 클래스로 통합하고 class-agnostic NMS를 적용합니다.

## 토픽

- 입력: `/rgb` (`sensor_msgs/msg/Image`)
- 박스가 표시된 영상: `/hand_detection/image` (`sensor_msgs/msg/Image`)
- 탐지 결과: `/hand_detection/detections` (`std_msgs/msg/String`, JSON)
- ROI 침입 상태: `/hand_safety/roi_intrusion` (`std_msgs/msg/Bool`)

`/hand_detection/detections`에는 단일 클래스 ID `0`, 클래스 이름
`hand`, confidence, `[x1, y1, x2, y2]` 형식의 bounding box가
포함됩니다.

고정 테이블 ROI는 `hand_safety/roi_intrusion.py` 상단의
`TABLE_ROI_NORMALIZED`에 화면 비율 기준 네 꼭짓점으로 설정합니다.
손 바운딩 박스가 테이블 폴리곤과 조금이라도 겹치면 침입으로 판단하며, ROI와
겹치는 손만 검출 영상과 결과 메시지에 표시합니다.

## 빌드

```bash
cd "$(git rev-parse --show-toplevel)/cobot3_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hand_safety
source install/setup.bash
```

손 검출 모델은 패키지와 함께 설치되며 노드가 설치 경로에서 자동으로
찾습니다.

## 실행

```bash
ros2 run hand_safety hand_detector_node
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

## 확인

```bash
ros2 topic info /rgb
ros2 topic hz /rgb
ros2 topic echo /hand_detection/detections
ros2 run rqt_image_view rqt_image_view /hand_detection/image
```

입력 `/rgb`는 `sensor_msgs/msg/Image`여야 합니다.
`sensor_msgs/msg/CompressedImage` 토픽은 이 노드에서 직접 구독하지
않습니다.
