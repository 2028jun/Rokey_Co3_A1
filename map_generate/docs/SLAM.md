# map_generate SLAM 기술·방법

이 문서는 `map_generate/` 워크스페이스에서 **점유 그리드 맵을 한 번 생성하는** 데 쓰는 SLAM 기술, 센서 파이프라인, 탐사(coverage) 방법, 그리고 다른 맵 생성 방식과의 차이를 정리한다.

실행 절차(t1→t2→t3→save_map)는 [README.md](../README.md)를 본다.

---

## 1. 목표와 범위

| 항목 | 내용 |
|------|------|
| 목적 | 식당 씬에서 Nav2/AMCL이 쓸 수 있는 2D occupancy map (`slam_map.pgm` / `.yaml`) 생성 |
| 로봇 | Ridgeback 계열 2륜 (`two_wheel_serving_robot_v2`) |
| 시뮬 | NVIDIA Isaac Sim + PhysX |
| SLAM | ROS 2 Humble **slam_toolbox** (online async mapping) |
| 포함하지 않음 | Nav2, AMCL, line_rails 주행 미션 |

맵이 “비는” 회색 영역은 **미탐색(unknown)** 이다. SLAM은 센서가 본 곳만 free/occupied로 채우므로, 탐사 경로가 맵 품질을 좌우한다.

---

## 2. 전체 파이프라인

```text
Isaac (t1)                    ROS map_gen (t2/t3)
─────────────────────         ────────────────────────────────
PhysX raycast LiDAR  ──►  /two_wheel/scan_raw
월드 pose odom       ──►  /two_wheel/odom_raw
/clock, /cmd_vel 구독
                              topic_bridge
                                ├─ /scan  (frame: nav_lidar_link)
                                ├─ /odom  (rebase) + TF odom→base
                                └─ robot_state_publisher (URDF)

                              async_slam_toolbox_node
                                ├─ scan matching + pose graph
                                ├─ loop closure
                                └─ /map (OccupancyGrid)

                              slam_patrol (t3)
                                └─ /cmd_vel ← 월드 좌표 웨이포인트 순회

                              map_saver_cli (save_map)
                                └─ maps/restaurant/slam_map.{pgm,yaml}
```

**핵심 분리**

- **맵 추정**: slam_toolbox가 `/scan` + `/odom`(+TF)으로 `/map`과 `map→odom` TF를 만든다.
- **탐사 주행**: `slam_patrol`은 SLAM 맵이 아니라 **Isaac 월드 절대 pose**(`/two_wheel/odom_raw`)로 웨이포인트를 따라간다. (맵이 아직 없을 때에도 움직일 수 있음)

---

## 3. 사용 기술 스택

### 3.1 slam_toolbox (Online Async SLAM)

- 패키지: `ros-humble-slam-toolbox`
- 노드: `async_slam_toolbox_node`
- 설정: `src/map_gen/config/slam_toolbox.yaml`
- 모드: `mode: mapping` (실시간 매핑)

slam_toolbox는 레이저 기반 **pose-graph SLAM**이다.

1. **Scan matching**  
   연속 스캔을 정합해 상대 운동을 추정하고, 오도메트리와 결합한다.  
   (`use_scan_matching`, `use_scan_barycenter`, `use_response_expansion`)

2. **Pose graph**  
   이동/회전이 일정 이상일 때 노드를 추가한다.  
   (`minimum_travel_distance`, `minimum_travel_heading`, `minimum_time_interval`)

3. **Loop closure**  
   비슷한 장소로 돌아오면 그래프를 닫아 드리프트를 줄인다.  
   (`do_loop_closing`, `loop_search_maximum_distance`)

4. **Occupancy grid**  
   광선 추적(ray casting)으로 셀을 free / occupied / unknown으로 갱신한다.  
   해상도 `0.05 m` (Nav2 맵과 동일 계열).

5. **프레임**

| 프레임 | 역할 |
|--------|------|
| `map` | SLAM이 유지하는 전역 맵 프레임 |
| `odom` | 브리지가 발행하는 연속 odom |
| `ridgeback_base_link` | 로봇 base |
| `nav_lidar_link` | 스캔 `frame_id` (URDF fixed joint) |

`scan_topic: /scan` — Isaac raw가 아니라 **브리지 출력**을 본다.

### 3.2 Isaac PhysX Raycast LiDAR (센서 소스)

Isaac 데모(`isaacpjt/restaurant_two_wheel_demo.py`)가 실제 LiDAR 플러그인 대신 **PhysX `raycast_closest`** 로 2D 스캔을 합성한다.

- 토픽: `/two_wheel/scan_raw` (`sensor_msgs/LaserScan`)
- 대략 사양: 최대 거리 ~12 m, 주기적 샘플링
- 동시에 `/two_wheel/odom_raw`(월드 pose), `/clock`, `/cmd_vel` 구독

시뮬 시간이 ROS `use_sim_time`과 맞도록 `/clock`을 발행한다.

### 3.3 Topic Bridge (센서 정규화)

`map_gen/topic_bridge.py`

| 입력 | 출력 | 비고 |
|------|------|------|
| `/two_wheel/scan_raw` | `/scan` | `header.frame_id` → `nav_lidar_link` |
| `/two_wheel/odom_raw` | `/odom` + TF `odom→ridgeback_base_link` | **첫 샘플 기준 rebase** (상대 odom) |
| `/two_wheel/teleport` | (rebase 리셋) | 텔레포트 후 odom 원점 재설정 |

QoS: `/odom`, `/scan` 발행은 **BEST_EFFORT** (센서 관례).  
순찰은 절대 pose가 필요하므로 **`/two_wheel/odom_raw`(RELIABLE)** 를 구독한다.

### 3.4 robot_state_publisher

URDF(`urdf/two_wheel_robot.urdf`)로 `ridgeback_base_link` → `nav_lidar_link` 등 **고정 TF**를 발행한다. slam_toolbox가 스캔을 base/odom으로 변환할 때 필요하다.

### 3.5 맵 저장 (nav2_map_server)

- 도구: `ros2 run nav2_map_server map_saver_cli`
- 스크립트: `tools/save_slam_map.sh`
- 입력: 라이브 `/map` (`nav_msgs/OccupancyGrid`)
- 출력: PGM(점유) + YAML(origin, resolution, 임계값)

기존 `slam_map.*`가 있으면 **덮어쓰지 않고** `slam_map_YYYYMMDD_HHMMSS.*`로 저장한다.

---

## 4. 탐사(Coverage) 방법 — slam_patrol

파일: `map_gen/slam_patrol.py`

### 4.1 왜 월드 좌표 웨이포인트인가

SLAM 맵이 완성되기 전에는 `/map` 기준 플래너가 없다.  
식당 USD·스폰·테이블 도크 좌표는 이미 알려져 있으므로(**nav_robot6 rails / waypoints와 동일 계열**), 시뮬 월드 좌표로 경로를 짠다.

피드백: `/two_wheel/odom_raw`  
제어: `/cmd_vel` (전진 + 제자리 회전 `wz`)

### 4.2 제어 방식

1. 목표까지 **방위각(bearing)** 으로 정렬 후 전진  
2. 도착 후 세그먼트 목표 yaw 정렬  
3. `look=True` 지점에서 동·서·남·북 **look-around** + 짧은 dwell (스캔 통합 대기)  
4. 타임아웃 웨이포인트는 **스킵**하고 다음으로 (끼임 시 전체 중단 방지)

### 4.3 커버리지 설계 (회색 줄이기)

LiDAR는 **직선 가시선**만 채운다. 복도 중앙만 지나가면 테이블·기둥 **뒤쪽은 unknown(회색)** 으로 남는다.

| 구간 | 의도 |
|------|------|
| 주방 문 앞·문틀 좌우 | 출입구·벽 코너 관측 |
| 주방 내부 일부 | 문 너머 free/occupied |
| 테이블 도크 (±DOCK≈1.60) | 테이블 전면 |
| 테이블 N/S 플랭크 | 측면 |
| 테이블 후면 루프 (±OUTER≈2.55) | **테이블 뒤** 그림자 해소 |
| 외곽 벽 레인 | 벽·기둥 사이 통로 |
| 남쪽 개구부 앞 | 반대쪽 출입 영역 |

상수(`DOCK`, `OUTER`, `FLANK`, `DOOR_Y`, `KIT_Y`)는 씬 치수에 맞춰 `slam_patrol.py` 상단에서 조정한다.

### 4.4 맵 품질과의 관계

| 현상 | 원인 | 대응 |
|------|------|------|
| 테이블 뒤 회색 | 가시선 차단 | OUTER 루프·외곽 레인 |
| 문 주변 회색 | 문 앞에서만 스캔 | 문틀·주방 진입 웨이포인트 |
| 시간만 길고 맵 동일 | 같은 가시선 반복 | **새 관측 각도/위치**가 필요 (속도↓만으로는 부족) |
| 끼임 | OUTER가 너무 큼 | `OUTER` 축소 또는 해당 WP 스킵 |

---

## 5. Occupancy Map Generator와의 차이

같은 레포의 `nav_robot6/maps/restaurant/map.*` 등은 종종 Isaac **Occupancy Map Generator**(또는 동등 스크립트)로 씬 지오메트리를 **직접** 투영한 결과다.

| | slam_toolbox (이 워크스페이스) | Occupancy Generator |
|--|-------------------------------|---------------------|
| 입력 | 로봇이 실제로 “본” 스캔 | USD/충돌 메시 투영 |
| unknown | 미탐사 구간 남음(회색) | 보통 거의 없음 |
| 장점 | 실기/시뮬 공통 파이프라인, 센서 노이즈·가림 반영 | 완전하고 빠른 “정답” 맵 |
| 단점 | 탐사 품질에 민감, 시간 소요 | 센서 SLAM 경험과 다름 |

Nav 주행용으로 Generator 맵을 쓰는 경우도 있고, **센서 SLAM으로 만든 맵**으로 AMCL을 맞추고 싶은 경우가 이 `map_generate`의 목적이다.

---

## 6. 주요 파라미터 요약

`slam_toolbox.yaml` (매핑 밀도)

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| `resolution` | 0.05 | 셀 크기 [m] |
| `max_laser_range` | 12.0 | Isaac LiDAR와 맞춤 |
| `minimum_travel_distance` | 0.08 | 짧은 이동에도 그래프 갱신 |
| `minimum_travel_heading` | 0.08 | 제자리 회전 시에도 갱신 |
| `map_update_interval` | 1.0 | `/map` 발행 주기 [s] |
| `do_loop_closing` | true | 루프 클로저 ON |

순찰

| 상수 | 대략값 | 의미 |
|------|--------|------|
| `LINEAR_SPEED` | 0.26 m/s | 느린 탐사 (스캔 밀도) |
| `DOCK` | 1.60 m | 테이블 전면 접근 |
| `OUTER` | 2.55 m | 테이블 후면 레인 |
| `LOOK_YAWS` | 0, ±π/2, π | 4방향 스캔 |

---

## 7. 산출물

경로: `maps/restaurant/`

| 파일 | 설명 |
|------|------|
| `slam_map.pgm` | Occupancy 이미지 (검정=occupied, 흰=free, 회색=unknown) |
| `slam_map.yaml` | `origin`, `resolution`, 임계값 |
| `slam_map_*.pgm/yaml` | 재저장 시 타임스탬프 백업 |

YAML 예:

```yaml
image: slam_map.pgm
mode: trinary
resolution: 0.05
origin: [..., ..., 0]
occupied_thresh: 0.65
free_thresh: 0.25
```

`origin`은 slam_toolbox가 탐사를 확장하며 잡힌 **맵 좌하단**이므로, 실행마다 조금씩 달라질 수 있다. Nav2에 넣을 때는 해당 YAML의 origin을 그대로 쓰고, AMCL initial pose를 씬 스폰에 맞춘다.

---

## 8. 관련 파일 목록

| 경로 | 역할 |
|------|------|
| `isaacpjt/restaurant_two_wheel_demo.py` | Isaac LiDAR/odom/clock/cmd |
| `src/map_gen/launch/slam_mapping.launch.py` | bridge + RSP + slam_toolbox |
| `src/map_gen/config/slam_toolbox.yaml` | SLAM 파라미터 |
| `src/map_gen/map_gen/topic_bridge.py` | 토픽/TF 브리지 |
| `src/map_gen/map_gen/slam_patrol.py` | 커버리지 순찰 |
| `src/map_gen/urdf/two_wheel_robot.urdf` | LiDAR TF |
| `tools/save_slam_map.sh` | `/map` → PGM/YAML |
| `tools/aliases.sh` | t1 / t2 / t3 / save_map |

---

## 9. 참고

- slam_toolbox: [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- 레포 내 선행 SLAM 실험: `nav_robot` (`slam_mapping.launch.py`, `slam_patrol.py`)
- Occupancy Generator 맵 가이드: `nav_robot/docs/OCCUPANCY_MAP.md`
