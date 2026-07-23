# Base Hold 알고리즘 (Isaac-safe wheel lock)

## 1. 문제 정의

도킹 후 베이스는 이미 `cmd_vel≈0`으로 “멈춰” 있다.  
그런데 팔이 가속하면 차체에 반력·모멘트가 걸리고, **velocity drive(stiffness≈0) 휠**은 그 토크에 굴러간다.

따라서 필요한 것은 “정지 명령을 한 번 더 보내는 것”이 아니라  
**휠 자유도를 외력에 대해 잠그는 것**이다.

## 2. 쓰지 않는 방법 (위험)

| 방법 | 이유 |
|------|------|
| Base ↔ World **FixedJoint** | 이 레포에서 Isaac 크래시/불안정 사례 |
| 바퀴를 지면에 구속 | 위와 동일 계열 |

## 3. 채택 방법: 휠 조인트 position hold

실기 **서보 홀드 / 전자 브레이크**에 가장 가까운 시뮬 대응:

1. Hold engage 시 각 휠 조인트 **각**을 읽는다.
2. PhysX `set_gains` + USD DriveAPI를
   - `stiffness` ↑ (기본 8000)
   - `damping` ↑ (기본 800)
   - `targetPosition` = 스냅샷 각도
   - `targetVelocity` = 0
   으로 바꾼다.
3. 휠 joint velocity를 0으로 리셋한다.
4. Hold 동안 **매 스텝 `tick()`**으로 position target을 재적용한다.
5. Hold 동안 Isaac 쪽 **cmd_vel→휠 속도 적용을 끈다**
   (`apply_action(joint_velocities=…)`가 Kp를 다시 0으로 만들기 때문).
6. Release 시 원래 velocity drive(`stiffness=0`, damping≈140)로 복구한다.

구현: [`isaac_wheel_hold.py`](../src/base_hold/base_hold/isaac_wheel_hold.py)  
ROS 게이트: [`hold_node.py`](../src/base_hold/base_hold/hold_node.py)  
연동 예: [`isaac_hold_bridge_snippet.py`](../examples/isaac_hold_bridge_snippet.py)

```text
Mission docked
    │
    ▼
/base/hold = true  ──► hold_node: /cmd_vel = 0 (반복)
                   ──► /base/hold_state = true
                           │
                           ▼
                   IsaacWheelHold.engage()
                   (wheel joints: high-stiffness position hold)
                           │
                           ▼
                   Arm moves — reaction torque resisted by wheel PD
                           │
                           ▼
/base/hold = false ──► release velocity drive — nav resumes
```

## 4. 왜 이걸로 팔 반력에 버티나

Velocity drive(`stiffness=0`)는 “목표 각속도”만 맞추려 하고, 외란 토크에 대한 **위치 복원력**이 거의 없다.  
Position hold는 목표 각도에서 벗어나면 `τ ≈ −K(q−q*) − D q̇` 로 되돌리므로, 팔 반력이 휠을 돌리려 해도 서보가 버틴다 (maxForce 한도 내).

## 5. ROS 게이트의 역할

`hold_node`만으로는 물리 잠금이 아니다. 역할은:

- 홀드 중 Nav/teleop `/cmd_vel`이 Isaac에 들어가 **속도 명령을 다시 거는 것**을 차단
- `/base/hold_state`로 Isaac 브리지에 engage/release 신호

## 6. 모드

| 모드 | hold | 휠 Drive | cmd_vel |
|------|------|----------|---------|
| `BASE_MOVING` | false | velocity (K=0) | 패스스루 |
| `ARM_HOLD` | true | position hold (K≫0) | 강제 0 |

## 7. 한계

- `maxForce`를 넘는 충격에는 여전히 밀릴 수 있음 → 필요 시 stiffness/maxForce 파라미터 상향
- 메카넘 롤러 방향 미끄러짐은 조인트 hold만으로 100% 제거되지 않을 수 있음
- 실기에서는 같은 `/base/hold` API 뒤에 **기계 브레이크**를 붙이는 것이 최종형
