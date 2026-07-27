# serving_hmi UI 개편 — 병합 가이드

`main` 기준으로 작업한 내용. 다른 사본/브랜치에 합칠 때 아래만 확인하면 됨.

## 새 파일 (그대로 복사)
- `web_ui/admin.html`
- `web_ui/css/admin.css`, `web_ui/css/kiosk.css`
- `web_ui/js/admin.js`, `web_ui/js/kiosk.js`

## 삭제된 파일 (기존 쪽에서도 지울 것)
- `web_ui/css/styles.css`, `web_ui/js/app.js` — `index.html`이 admin/kiosk로 쪼개지면서 더 이상 안 씀

## 수정된 파일 — 어디를 봐야 하는지

**`web_ui/index.html`**
고객 키오스크 전용으로 전면 재작성됨. 기존 쪽에 메뉴/가격 커스터마이징이 있었다면 새 파일의 `m1`~`m6` 카드 블록에 다시 반영해야 함.

**`web_ui/js/robot_map.js`**
- `updateRobotPose(x, y, yaw)` → `updateRobotPose(robotName, x, y, yaw)`
- `setActiveTargetTable(tableNum)` → `setActiveTargetTable(robotName, tableNum)`
- 이 함수를 호출하는 다른 코드가 있으면 인자 추가 필요.

**`serving_hmi/order_manager.py`**
- `Order`에 `assigned_robot` 필드 추가
- `active_order_id`(단일) → `active_order_ids`(로봇별 dict)로 변경
- 이 필드/속성을 참조하는 다른 코드 있으면 확인.

**`serving_hmi/hmi_backend_node.py`**
- WebSocket `SYSTEM_STATUS` payload 구조 변경:
  - `robot` → `robots.robot1` / `robots.robot2`
  - `active_order_id` → `active_order_ids` (dict)
  - `camera_connected` / `camera_image` (`camera2_*`) → `robots.robotN.camera_connected` / `camera_image`
  - `domain_id`가 최상위 필드로 이동
- `/admin` 라우트 신규 추가 (`admin.html` 서빙)
- robot2용 신규 ROS 토픽 구독 추가: `/robot2/camera/color/image_raw`, `/robot2/nav_robot/odom`, `/robot2/serving_robot/status`, `/robot2/serving_robot/event`, `/robot2/system/status` (robot1은 기존 미네임스페이스 토픽 그대로 유지)
- 이 payload를 파싱하는 다른 클라이언트가 있으면 새 구조에 맞춰 수정 필요.

## 주의
`multi` / `woduqmulti`처럼 이미 로봇 2대를 **다른 방식**(로봇별 별도 HMI 프로세스 + `robot_namespace` launch 파라미터)으로 처리 중인 브랜치에 합칠 경우, `hmi_backend_node.py`의 아키텍처 자체가 다르므로 단순 병합이 아니라 재통합 설계가 필요함.
