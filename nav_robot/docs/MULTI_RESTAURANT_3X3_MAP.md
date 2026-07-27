# Multi-Robot 3x3 Restaurant Map

## 변경 범위

- 기준 식당 공간 12 m x 10 m를 좌우와 남쪽으로 반복해 3x3 식당으로 확장했다.
- 전체 식당 바닥은 36 m x 30 m이며, 주방이 있는 북쪽 영역은 침범하지 않는다.
- 중앙의 기존 테이블 4개는 원래 물리와 충돌 설정을 유지한다.
- 외부 8개 구역에 복제된 테이블 32개는 시각 요소만 사용하며, 충돌 160개를 명시적으로 비활성화한다.
- 반복적으로 보이던 외부 식물은 구역별 0~2개로 다르게 선택해 총 9개만 배치한다.

## 외벽과 주방

- 식당 외곽과 주방 돌출부를 잇는 충돌 벽 8개를 설치했다.
- 식당 외벽 범위는 X=-18~18 m, Y=-25~5 m이다.
- 주방 외곽 벽은 실제 주방 바닥에서 약 8 cm만 떨어진 X=-2.72~2.72 m,
  Y=10.07 m에 배치해 좌우와 뒤쪽을 감싼다.
- 외벽 8개는 짙은 목재 하부, 황동색 띠, 웜 브라운 상부의 3단 마감이다.
- Lightwheel Kitchen 에셋의 기존 벽, 천장, 창문, 벽 충돌체는 비활성화한다.
- 에셋에서 실제 바닥은 `Kitchen_Ground`, 잘못 `Kitchen_Floor`로 명명된 고가 메시가 천장이다.

## 로봇과 검증

- `NAV_MULTI_ROBOT=1`일 때 `/World/NavRobot1`, `/World/NavRobot2` 두 로봇을 생성한다.
- 시작 위치는 각각 `(-0.9, 5.25)`, `(0.9, 5.25)`이다.
- 외부 테이블 32개, 비활성 충돌 160개, 외부 식물 9개를 런타임에 검증한다.

## 관련 파일

- `nav_robot/assets/lightweight_restaurant/lightweight_pizza_restaurant.usda`
- `nav_robot/isaacpjt/nav_restaurant_demo.py`
- `assets/Lightwheel_Kitchen/`

`map.pgm`, `map.yaml` 등 Nav2 점유 지도 파일은 변경하지 않았다.
