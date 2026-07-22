# Rokey_Co3_A1

## Nova Carter 식당 주행 (기본: nav_robot3)

터미널마다 한 번:

```bash
source ~/git/Rokey_Co3_A1/tools/aliases.sh
```

| 터미널 | 명령 |
|--------|------|
| 1 | `t1` → Isaac **Play** |
| 2 | `t2` |
| 3 | `t3 --table-id 2` |

`ROS_DOMAIN_ID=103` (aliases가 설정). 상세: [`nav_robot3/README.md`](nav_robot3/README.md), [`docs/NOVA_CARTER.md`](docs/NOVA_CARTER.md).

```bash
git checkout younggi
cd nav_robot3 && colcon build --packages-select nova_rails && source install/setup.bash
```

`nav_robot2/`는 레거시(deprecated)입니다.
