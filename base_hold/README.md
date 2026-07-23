# base_hold — 도킹 후 휠 잠금 (Isaac-safe)

**해결할 문제:** 멈춰 있는 상태에서 팔이 움직이면 반력으로 바퀴가 도는 것.  
`cmd_vel=0`만으로는 부족합니다 (정지 *명령*일 뿐 휠이 외력에 잠기지 않음).

**이 패키지 해법:** Isaac에서 휠 조인트를 **고강성 position hold**로 전환 (현재 각도 유지).  
월드/지면 **FixedJoint 고정은 사용하지 않음** (이전에 Isaac 크래시 유발).

상세 알고리즘: **[docs/BASE_HOLD.md](docs/BASE_HOLD.md)**

## 구성

| 층 | 구현 | 역할 |
|----|------|------|
| ROS 게이트 | `hold_node` | `/cmd_vel` 차단 + 0 재발행 (Nav가 끼어들지 않게) |
| Isaac 잠금 | `IsaacWheelHold` | PhysX gains + 휠 position target (`tick()` 매 스텝) |

**둘 다** 켜야 “팔 움직여도 베이스가 안 밀리는” 효과가 납니다.  
홀드 중에는 Isaac 루프에서 **휠에 `joint_velocities`를 넣지 마세요** (position hold와 충돌).

---

## 빌드

```bash
cd ~/git/Rokey_Co3_A1/base_hold
source /opt/ros/humble/setup.bash
colcon build --packages-select base_hold
source install/setup.bash
```

Isaac 쪽 Python은 colcon 설치본 또는 소스 경로:

```bash
export PYTHONPATH=~/git/Rokey_Co3_A1/base_hold/src/base_hold:$PYTHONPATH
```

---

## ROS만 (게이트 검증)

```bash
# 터미널 1
ros2 run base_hold hold_node

# 터미널 2 — 주행 명령 (홀드 전에는 통과)
ros2 topic pub /cmd_vel_in geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10

# 터미널 3
ros2 topic echo /cmd_vel
bash examples/call_hold_on_dock.sh on    # → /cmd_vel 이 0으로 고정
bash examples/call_hold_on_dock.sh off
```

### ROS API

| 이름 | 타입 | 설명 |
|------|------|------|
| `/base/hold` | `std_srvs/SetBool` | `true`=ARM_HOLD, `false`=BASE_MOVING |
| `/base/hold_state` | `std_msgs/Bool` (latched) | Isaac `BaseHoldBridge`가 구독 |
| `/cmd_vel_in` → `/cmd_vel` | `Twist` | hold 시 출력 강제 0 |

파라미터: `input_topic`, `output_topic`, `hold_publish_hz`(기본 20), `start_held`.

---

## Isaac 연동 (실제 잠금)

1. 데모 Isaac 루프에 [`examples/isaac_hold_bridge_snippet.py`](examples/isaac_hold_bridge_snippet.py)의 `BaseHoldBridge`를 붙입니다.
2. `hold_node`와 **같은 `ROS_DOMAIN_ID`** 로 실행합니다.
3. 매 시뮬 스텝:

```python
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]

bridge = BaseHoldBridge(ros_node, stage, articulation, dof_names, WHEEL_JOINTS)

while simulation_app.is_running():
    world.step(render=True)
    if not bridge.held:
        apply_cmd_vel_to_wheels(...)   # velocity drive
    bridge.spin_once()                 # hold 시 engage + tick()
```

4. 도킹 후:

```bash
bash examples/call_hold_on_dock.sh on
# 팔 동작 ...
bash examples/call_hold_on_dock.sh off
```

### 배선

```text
Nav / teleop  --(/cmd_vel_in)-->  hold_node  --(/cmd_vel)-->  Isaac (hold 중 0)
Mission       --(SetBool /base/hold)-->  hold_node  --(/base/hold_state)-->  BaseHoldBridge
                                                                              --> IsaacWheelHold
```

기존 데모가 `/cmd_vel`만 구독하면, Nav 발행을 `/cmd_vel_in`으로 바꾸거나 `hold_node`의 `input_topic`/`output_topic`을 맞춥니다.

---

## Isaac 단독 테스트 (map_generate 로봇/맵)

에셋: `map_generate` 워크스페이스 (`tools/sync_assets.sh`로 nav_robot6 링크).

```bash
cd ~/git/Rokey_Co3_A1/map_generate && bash tools/sync_assets.sh

ISAAC_PY=/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh
cd ~/git/Rokey_Co3_A1/base_hold
```

**Headless (CI/자동 PASS 기준):**

```bash
MAP_GEN_HEADLESS=1 "$ISAAC_PY" examples/test_isaac_wheel_hold.py
# PASS: drift_hold << drift_free (로그에 RESULT ... PASS)
```

**GUI (눈으로 확인):**

```bash
export DISPLAY=:1    # 본인 디스플레이에 맞게
export MAP_GEN_HEADLESS=0
export HOLD_GUI_LOOPS=3   # spin/hold/release 반복 횟수 (기본 3)
"$ISAAC_PY" examples/test_isaac_wheel_hold.py
# 끝나면 창을 닫을 때까지 idle (HOLD_GUI_KEEP_S=60 이면 60초 후 자동 종료)
```

테스트 단계:

1. **WITHOUT hold** — 바퀴가 속도 명령으로 회전  
2. **WITH hold** — 외란 토크, 각도 거의 고정  
3. **AFTER release** — 다시 회전  

---

## 팀원 체크리스트

- [ ] `colcon build` 후 `hold_node` 실행  
- [ ] Isaac 데모에 `BaseHoldBridge` + `bridge.held`일 때 휠 velocity 미적용  
- [ ] 도킹 후 `call_hold_on_dock.sh on` / 팔 작업 / `off`  
- [ ] (선택) `test_isaac_wheel_hold.py` headless PASS  

---

## 범위 밖

- FixedJoint / 지면 핀 (의도적으로 제외)
- 실기 CAN 브레이크 드라이버 (같은 `/base/hold` API에 나중에 연결 가능)
- nav 미션 자동 `hold` 호출 (미션에서 서비스만 호출하면 됨)
