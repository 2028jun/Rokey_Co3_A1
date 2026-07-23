# Base Hold 알고리즘

식당 서빙 로봇이 **테이블에 도킹한 뒤** 팔만 움직일 때, 베이스 바퀴를 어떻게 고정할지에 대한 설계와 이 패키지 구현을 정리한다.

## 1. 문제

모바일 베이스 위에서 팔이 가속하면 차체에 **반력·모멘트**가 걸린다.

- 휠 모터를 **coast**(전류 off / 목표속도 0만 한 번 보냄)하면 감속기가 있어도 바퀴가 미세하게 굴러가거나 차체가 밀린다.
- Nav2/`cmd_vel` 파이프라인의 “정지”는 대개 **명령 0**일 뿐, **주차 브레이크가 아니다**.

그래서 도킹 후 조작 구간에는 명시적인 **Hold 모드**가 필요하다.

## 2. 실기에서 쓰는 방법 (요약)

| 방법 | 요지 |
|------|------|
| 전자기/기계 브레이크 | 휠 또는 감속기 power-off brake / 주차 브레이크 engage |
| 서보 홀드 | 속도 0(또는 조인트 위치)을 **닫힌 루프로 계속** 유지 |
| 모드 분리 | `BASE_MOVING` ↔ `ARM_HOLD` — 팔 동작 중 베이스 명령 차단 |
| 기구·궤적 | 급가속 제한, 무게중심 근처 작업 (보조) |

이 패키지는 그중 **서보 홀드(소프트웨어)** 와 시뮬용 **주차 브레이크(FixedJoint)** 를 팀원이 바로 붙일 수 있는 형태로 제공한다.

## 3. 모드 계약

```text
BASE_MOVING  (hold = false)
  - /cmd_vel_in  →  /cmd_vel  패스스루
  - Isaac parking brake OFF

ARM_HOLD     (hold = true)
  - /cmd_vel_in 무시
  - /cmd_vel 에 Twist(0) 를 hold_publish_hz 로 반복 발행  ← 서보 홀드
  - (옵션) Isaac FixedJoint ON                              ← 주차 브레이크
```

미션 시퀀스:

1. 내비게이션으로 테이블 도킹·정렬
2. `/base/hold` ← `true`
3. 팔 픽/플레이스
4. `/base/hold` ← `false`
5. 다음 목표로 이동

## 4. Servo hold (`hold_node`)

**아이디어:** “0을 한 번 보내고 끝”이 아니라, 홀드 동안 **주기적으로 0을 재발행**하고, 그 사이 들어오는 주행 명령을 버린다.

이유:

- Isaac/드라이버 쪽 `CMD_TIMEOUT`이 있어 명령이 끊기면 내부 상태가 느슨해질 수 있음
- 다른 노드가 늦게 `/cmd_vel`을 또 보내도 게이트가 막음
- 실기 서보에 `ω=0` 유지를 계속 넣는 것과 동일한 소프트웨어 계약

구현: [`base_hold/hold_node.py`](../src/base_hold/base_hold/hold_node.py)

- 서비스: `std_srvs/SetBool` `/base/hold`
- 상태: `std_msgs/Bool` `/base/hold_state` (TRANSIENT_LOCAL)
- 토픽: `/cmd_vel_in` → `/cmd_vel` (파라미터로 변경 가능)

## 5. Parking brake (`isaac_parking_brake`)

**아이디어:** 시뮬에서 base 링크를 World에 **FixedJoint**로 고정해, 팔 반력이 휠을 굴리지 못하게 한다.  
실기의 “브레이크 잠김”에 대응하는 **물리 제약**이다.

구현: [`base_hold/isaac_parking_brake.py`](../src/base_hold/base_hold/isaac_parking_brake.py)

- `engage(stage, articulation_path, world_xyz, ...)` — joint 생성/교체
- `release(stage)` — joint 제거
- 기본 prim: `/World/BaseHold/ParkingBrake`

참고 원형: `serving_robot`의 `add_parking_brake()` (스폰 시 1회 ON, 런타임 토글 없음) → 여기서는 **도킹 시점 토글**이 가능하도록 일반화.

Isaac 루프 연동 예: [`examples/isaac_hold_bridge_snippet.py`](../examples/isaac_hold_bridge_snippet.py)  
`/base/hold_state`를 구독해 engage/release를 호출한다.

## 6. 왜 둘을 같이 쓰나

| 계층 | 없으면 |
|------|--------|
| Servo hold만 | 시뮬 velocity-drive 휠은 여전히 외력에 밀릴 수 있음 |
| FixedJoint만 | ROS 쪽 Nav/teleop이 계속 `/cmd_vel`을 보내 드라이버와 싸우거나, 상태 공유가 안 됨 |
| 둘 다 | 명령 게이트 + 물리 고정 — 도킹 조작에 가장 안전 |

실기에서는 FixedJoint 대신 **실제 브레이크 드라이버 engage**를 `/base/hold_state` 콜백에 넣으면 된다. ROS API는 그대로 유지.

## 7. 하지 않는 것

- 팔 궤적 자체로 베이스 반력을 상쇄하는 whole-body 제어
- CAN/펌웨어 브레이크 프로토콜
- nav_robot6 / map_generate 미션에 자동 `hold` 호출 (미션 쪽에서 서비스만 호출하면 됨)

## 8. 빠른 검증

```bash
# 터미널 1
ros2 run base_hold hold_node

# 터미널 2 — 가짜 주행 명령
ros2 topic pub /cmd_vel_in geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10

# 터미널 3 — 출력 확인 후 홀드
ros2 topic echo /cmd_vel
ros2 service call /base/hold std_srvs/srv/SetBool "{data: true}"
# → /cmd_vel 이 0으로 바뀌고, /cmd_vel_in 은 무시됨
```
