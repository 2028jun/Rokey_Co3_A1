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

기본값은 테이블 이동 서비스 사용을 위해 parking brake가 꺼져 있습니다.
고정된 상태에서 팔만 시험할 때는 다음처럼 parking brake를 켭니다.

```bash
MOBILE_DEMO_PARKED_HOLD=1 \
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  /home/rokey/cobot3_ws/isaacpjt/mobile_manipulator_demo.py
```

## 테이블 이동 서비스

Isaac 실행 중 다른 ROS 2 Humble 터미널에서 테이블 번호 `0..3`을 보냅니다.

```bash
source /opt/ros/humble/setup.bash
python3 isaacpjt/go_to_table_client.py 2
```

서비스 이름과 타입은 다음과 같습니다.

```text
/serving_robot/go_to_table
example_interfaces/srv/AddTwoInts
```

`request.a`가 테이블 번호이고 `request.b`는 사용하지 않습니다. 응답 `sum`이
요청 번호와 같으면 이동 요청이 접수된 것입니다. `-1`은 잘못된 번호,
`-2`는 parking brake가 켜진 상태를 뜻합니다.

이동은 테이블에서 0.65 m 더 떨어진 pre-dock 위치까지 먼저 이동한 뒤,
그 위치에서 최종 도킹 방향으로 회전합니다. 이후 로봇의 `-X` 도킹면이
테이블을 향한 자세를 유지하면서 저속 후진하여 최종 위치에 접근합니다.

고정 카메라 토픽은 다음과 같습니다.

```text
/serving_robot/table_camera/color/image_raw
/serving_robot/table_camera/depth/image_raw
/serving_robot/table_camera/camera_info
```

세부 현황과 다음 작업은 `docs/PROJECT_HANDOFF_2026-07-20.md`를 확인합니다.
