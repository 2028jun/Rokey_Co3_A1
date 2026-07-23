# Rokey_Co3_A1 - Serving Robot & Hand Safety Workspace

## Workspace Structure

```text
Rokey_Co3_A1/
├── serving_robot/     # Serving Robot ROS 2, Isaac Sim & HMI Web Dashboard
│   ├── isaacpjt/     # Isaac Sim 5.1 Standalone Python Scripts
│   ├── src/          # ROS 2 Workspace Packages (serving_hmi, etc.)
│   └── run_hmi.sh    # HMI Web Dashboard Runner
├── hand_safety/       # Canonical Hand Detection & Safety ROS 2 Package
├── .gitignore
└── README.md
```

`src/hand_safety` is a legacy duplicate and contains `COLCON_IGNORE`.
Build the canonical root-level `hand_safety` package only.

## Vision-integrated ROS startup

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=102
colcon build --symlink-install --packages-select \
  serving_robot_interfaces serving_robot_manager hand_safety
source install/setup.bash
ros2 launch serving_robot_manager vision_manager.launch.py
```

The Isaac Sim process publishes `/camera/color/image_raw`. The detector
publishes `/hand_safety/intrusion`, and Manager gates inference with the
latched `/serving_robot/table_arrived` state.
The same launch starts `direct_nav_server_node`, which owns the ROS Humble
custom services and forwards them to Isaac's built-in axis-route controller.

## Quick Start (HMI Web Dashboard)

```bash
# Run HMI Web Dashboard
./run_hmi.sh
# Open Browser: http://localhost:8000
```

## 🚀 4-Terminal Hand Detection & Integrated Testing Guide

### Terminal 1 — Isaac Sim (손 검출 고정위치 & 장애물 정지 테스트)
> 이 터미널에서는 `/opt/ros/humble/setup.bash`를 source하지 마십시오.

```bash
cd /home/rokey/cobot3_ws/nav_robot

export NAV_ROBOT_ROS_DOMAIN_ID=102
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib"

export MOBILE_DEMO_HAND_TEST=1
export HAND_TEST_FIXED_REACH=1
export HAND_TEST_TARGET_X=-2.75
export HAND_TEST_TARGET_Y=-2.20
export HAND_TEST_TABLE_CLEARANCE_Z=0.06
export MOBILE_DEMO_OBSTACLE_TEST=1

/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/nav_restaurant_demo.py
```

**정상 로그 확인**:
- `[ros] embedded D455 RGB/depth 1280x960 and /clock connected`
- `[hand_test] fixed at table hand-detection target`
- `[vision-safety] dark material bound to all M0609 arm visuals`

---

### Terminal 2 — 주행 어댑터 + Manager + 손 검출

```bash
cd /home/rokey/cobot3_ws

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=102
export ROS_LOCALHOST_ONLY=0

PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select \
  serving_robot_interfaces serving_robot_manager hand_safety serving_hmi
source install/setup.bash

# 검출 화면 디버그 테스트 모드
ros2 launch serving_robot_manager vision_manager.launch.py \
  vision_debug:=true
```

> **주의**: 이 launch 파일이 `direct_nav_server_node`, `manager_node`, `hand_detector_node`를 모두 실행하므로 별도로 또 실행하면 안 됩니다.
> 성능 측정 시에는 `vision_debug:=false`로 실행하십시오.

---

### Terminal 3 — 손 검출 실시간 디버그 화면 (rqt_image_view & 파라미터)

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash

export ROS_DOMAIN_ID=102

ros2 run rqt_image_view rqt_image_view \
  /hand_detection/image
```

**현재 검출 설정 확인 명령**:
```bash
ros2 param get /hand_detector_node confidence
ros2 param get /hand_detector_node image_size
ros2 param get /hand_detector_node half
```
- 정상 설정값: `confidence: 0.70`, `image_size: 1280`, `half: False`

---

### Terminal 4 — 주문 Web HMI 대시보드

```bash
cd /home/rokey/cobot3_ws
export ROS_DOMAIN_ID=102
./run_hmi.sh
```
* 웹 브라우저 접속: `http://localhost:8000`
