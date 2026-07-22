# Nova Carter 식당 주행 (younggi)

**기본 스택:** [`nav_robot3/`](../nav_robot3/) — 레일(`routes.yaml`) + Nav2 + 횡오차 검증.  
`nav_robot2`는 deprecated.

## 터미널별 실행 (t1 / t2 / t3)

**각 터미널에서** 한 번:

```bash
source ~/git/Rokey_Co3_A1/tools/aliases.sh
```

| 터미널 | 명령 |
|--------|------|
| 1 | `t1` → Isaac **Play** |
| 2 | `t2` |
| 3 | `t3 --table-id 2` |

`ROS_DOMAIN_ID=103` (aliases가 설정). 상세: [`nav_robot3/README.md`](../nav_robot3/README.md)

**이슈 보고서 (텍스트):** `docs/NAV_ROBOT_ISSUES_REPORT_2026-07-22.txt`

```bash
cd ~/git/Rokey_Co3_A1/nav_robot3
./tools/sync_restaurant_assets.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select nova_rails
source install/setup.bash
```

상세: [`nav_robot3/README.md`](../nav_robot3/README.md)

---

## 레거시

| 경로 | 비고 |
|------|------|
| [`nav_robot2/`](../nav_robot2/) | `goToPose` 전용 — 사용 안 함 |
| [`nav_robot/`](../nav_robot/) | 통합 colcon, [`NOVA_CARTER_QUICKSTART.md`](../nav_robot/docs/NOVA_CARTER_QUICKSTART.md) |

---

## 빠른 점검

```bash
export ROS_DOMAIN_ID=103
ros2 topic hz /clock /scan /odom
ros2 lifecycle get /bt_navigator
```
