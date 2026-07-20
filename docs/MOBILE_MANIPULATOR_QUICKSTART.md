# Ridgeback + 선반 + M0609 통합 로봇

## 구성

- Clearpath Ridgeback R100: 4개 옴니휠
- 2단 음식 트레이: 바닥 기준 0.44 m, 0.64 m
- 상단 장착판: 0.84 m
- M0609 장착 위치: 후방으로 0.22 m 오프셋, 베이스 높이 약 0.895 m
- 테이블 접안 방향: 로봇 `-X` (팔 장착 측)
- 고정 D455: 높이 1.85 m, 측면 붐 장착, RGB/Depth ROS 2 발행
- 전체 URDF는 `ridgeback_base_link`부터 RG2 TCP까지 하나의 트리입니다.

## 빌드와 정적 검사

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash

xacro \
  /home/rokey/cobot3_ws/src/ridgeback_m0609_description/urdf/ridgeback_m0609.urdf.xacro \
  -o /home/rokey/cobot3_ws/src/ridgeback_m0609_description/urdf/ridgeback_m0609.urdf

colcon build --symlink-install --packages-select ridgeback_m0609_description
source /home/rokey/cobot3_ws/install/setup.bash
ros2 run ridgeback_m0609_description validate_description.py
```

## Isaac Sim 실행

발표 또는 다른 GPU 작업이 끝난 뒤에만 실행합니다.

먼저 별도 배포되는 Lightwheel Kitchen 압축파일을 워크스페이스 루트에서
풉니다. 다음 파일이 존재해야 합니다.

```text
assets/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd
```

```bash
cd /home/rokey/cobot3_ws
tar --zstd -xf /path/to/Lightwheel_Kitchen_runtime_cc-by-nc-4.0.tar.zst
```

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=102

/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  /home/rokey/cobot3_ws/isaacpjt/mobile_manipulator_demo.py
```

짧은 바퀴·팔 진단 시퀀스를 추가하려면 다음과 같이 실행합니다.

```bash
MOBILE_DEMO_AUTORUN=1 \
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  /home/rokey/cobot3_ws/isaacpjt/mobile_manipulator_demo.py
```

기본 실행은 로봇을 스폰하고 준비 자세만 설정하며 자동 이동하지 않습니다.

기본값은 테이블 옆 정차 시연을 위해 parking brake가 켜져 있습니다. 자율주행
시험에서는 다음처럼 해제합니다.

```bash
MOBILE_DEMO_PARKED_HOLD=0 \
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  /home/rokey/cobot3_ws/isaacpjt/mobile_manipulator_demo.py
```

고정 카메라 토픽은 다음과 같습니다.

```text
/serving_robot/table_camera/color/image_raw
/serving_robot/table_camera/depth/image_raw
/serving_robot/table_camera/camera_info
```

세부 현황과 다음 작업은 `docs/PROJECT_HANDOFF_2026-07-20.md`를 확인합니다.
