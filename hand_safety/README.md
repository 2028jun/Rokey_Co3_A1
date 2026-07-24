# hand_safety

ROS 2 Humble에서 Isaac Sim의 RGB 영상을 받아 YOLOv10n 손 탐지와 고정
테이블 ROI 침입 판정을 수행합니다.

## 화면과 토픽

- Isaac viewport: 시뮬레이션 자체를 보는 창입니다. `MOBILE_DEMO_HEADLESS=0`
  및 유효한 `DISPLAY`가 필요합니다.
- `/hand_detection/image`: 박스와 ROI가 그려진 ROS 영상입니다. 기본으로
  발행되며 `rqt_image_view`로 보는 것을 권장합니다.
- OpenCV 창: `show_window:=true`일 때만 열립니다. X11/Wayland 환경에 따라
  불안정할 수 있어 기본값은 `false`입니다.
- 입력: `/serving_robot/table_camera/color/image_raw`
- JSON 탐지: `/hand_detection/detections`
- ROI 상태: `/hand_safety/roi_intrusion`
- 도착 게이트: `/serving_robot/table_arrived`
- 타이핑 트리거: `/hand_test/type_keyboard` (`std_msgs/msg/Empty`)

손 추론은 `/serving_robot/table_arrived=true`를 받은 뒤에만 시작합니다.
이 독립 테스트 브랜치에는 실제 도착 발행자가 없으므로 아래처럼 수동으로
발행해야 합니다.

## GPU 시각 검증 워크플로

저장소 루트에서 네 터미널을 사용합니다. 모든 터미널에서
`ROS_DOMAIN_ID=101`을 사용합니다.

터미널 1 — Isaac Sim GUI:

```bash
cd "$(git rev-parse --show-toplevel)"
export ROS_DOMAIN_ID=101
export MOBILE_DEMO_HEADLESS=0
export MOBILE_DEMO_ROS_CAMERA=1
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/mobile_manipulator_demo.py
```

터미널 2 — 검출기:

```bash
cd "$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hand_safety
source install/setup.bash
export ROS_DOMAIN_ID=101
ros2 run hand_safety hand_detector_node --ros-args \
  --params-file hand_safety/config/hand_safety.yaml \
  -p publish_annotated_image:=true \
  -p show_window:=true
```

터미널 3 — 도착 게이트:

```bash
source /opt/ros/humble/setup.bash
source "$(git rev-parse --show-toplevel)/install/setup.bash"
export ROS_DOMAIN_ID=101
ros2 topic pub --once --qos-durability transient_local \
  /serving_robot/table_arrived std_msgs/msg/Bool "{data: true}"
```

터미널 4 — 10초 타이핑 트리거:

```bash
source /opt/ros/humble/setup.bash
source "$(git rev-parse --show-toplevel)/install/setup.bash"
export ROS_DOMAIN_ID=101
ros2 topic pub --once \
  /hand_test/type_keyboard std_msgs/msg/Empty "{}"
```

터미널 2의 OpenCV 창에서 박스와 ROI를 확인합니다. `DISPLAY`가 없는
환경에서는 `show_window:=false`로 바꾸고 별도 터미널에서
`ros2 run rqt_image_view rqt_image_view /hand_detection/image`를 사용합니다.

## 확인 명령

```bash
ros2 param get /hand_detector_node publish_annotated_image
ros2 param get /hand_detector_node show_window
ros2 topic info /serving_robot/table_camera/color/image_raw -v
ros2 topic hz /serving_robot/table_camera/color/image_raw
ros2 topic info /hand_detection/image -v
ros2 topic hz /hand_detection/image
ros2 topic echo /hand_detection/detections
ros2 topic echo /hand_safety/roi_intrusion
```

고정 ROI는 `hand_safety/roi_intrusion.py` 상단의
`TABLE_ROI_NORMALIZED`에 정의됩니다. ROI와 겹치는 손만 영상과 JSON에
포함되며 3프레임 연속 확인 후 침입을 발행합니다.
