# base_hold — 도킹 후 베이스 휠 고정

테이블 도킹 뒤 로봇팔이 움직일 때, **바퀴를 그냥 coast(토크 0)로 두면** 반력으로 베이스가 밀립니다.  
이 패키지는 팀원이 재사용할 **Hold 모드**를 제공합니다.

상세 알고리즘: **[docs/BASE_HOLD.md](docs/BASE_HOLD.md)**

## 두 가지 고정 방법

| 방법 | 구현 | 역할 |
|------|------|------|
| **Servo hold** | `ros2 run base_hold hold_node` | `/cmd_vel` 차단 + **주기적 Twist(0)** 재발행 |
| **Parking brake** | `isaac_parking_brake.engage/release` | Isaac에서 base↔World **FixedJoint** (기계식 브레이크 대응) |

실기에서는 보통 전자기 브레이크 + 서보 홀드를 같이 씁니다. 시뮬에서는 FixedJoint가 브레이크에 가깝습니다.

## 빌드

```bash
cd ~/git/Rokey_Co3_A1/base_hold
source /opt/ros/humble/setup.bash
colcon build --packages-select base_hold
source install/setup.bash
```

## 사용 (도킹 직후)

```bash
# 터미널 A — 홀드 노드 (Nav/Isaac의 /cmd_vel 앞에 둠)
ros2 run base_hold hold_node
# 기본: /cmd_vel_in -> /cmd_vel , 서비스 /base/hold
```

미션이 테이블에 도착한 뒤:

```bash
# 홀드 ON (팔 작업)
bash examples/call_hold_on_dock.sh on
# 또는
ros2 service call /base/hold std_srvs/srv/SetBool "{data: true}"

# ... 팔 픽/플레이스 ...

# 홀드 OFF (다시 주행)
bash examples/call_hold_on_dock.sh off
```

Isaac에서 물리 고정까지 쓰려면 `examples/isaac_hold_bridge_snippet.py`를 데모에 붙이고 `/base/hold_state`를 구독하세요.

## ROS API

| 이름 | 타입 | 설명 |
|------|------|------|
| `/base/hold` | `std_srvs/SetBool` | `true`=ARM_HOLD, `false`=BASE_MOVING |
| `/base/hold_state` | `std_msgs/Bool` (latched) | 현재 hold |
| `/cmd_vel_in` → `/cmd_vel` | `Twist` | hold 중이면 출력 강제 0 |

파라미터: `input_topic`, `output_topic`, `hold_publish_hz`(기본 20), `start_held`.

## 배선 예

```text
Nav / teleop  --(/cmd_vel_in)-->  hold_node  --(/cmd_vel)-->  Isaac wheel driver
Mission docked --(SetBool)----->  hold_node  --(/base/hold_state)--> Isaac FixedJoint bridge
```

기존 Isaac 데모가 `/cmd_vel`을 직접 구독 중이면, 홀드 노드를 쓰려면 데모 쪽을 `/cmd_vel` 유지하고 **상류 발행만 `/cmd_vel_in`으로** 바꾸거나, `output_topic`/`input_topic`을 맞춰 주세요.

## 범위

- 포함: 공유 알고리즘, ROS 게이트, Isaac FixedJoint 헬퍼/스니펫
- 미포함: nav_robot6 / serving_robot 미션 자동 연결, 실기 CAN 브레이크 드라이버
