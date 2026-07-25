"""서빙 로봇 전체 작업을 조율하는 Manager(컨트롤) 노드.

UI가 /manager/order로 주문(테이블 번호, 피자1~3 개수, 음료 개수, 식기리필, 접시 수)을 보내면,
주문을 트립(피자 최대 1개 + 음료 최대 4개 + 식기리필 최대 1세트) 단위로 나누고 접시 랙은
좌측 전방 2층 트레이에서 별도 트립으로 배정한다. 트립마다 다음
순서를 반복한다:

  주방 도착 확인 -> 음식 스폰(완료 확인까지 대기) -> 테이블 이동
  -> 로봇팔 서빙(ROI 손 침입 시 일시정지·재개) -> 주방 복귀

음식 스폰은 피자 종류(자산이 다름)를 구분해야 해서 로봇팔 명령과 가중치가 다르다. 두 값 다
트립 하나를 정수 하나로 인코딩하며 서로 겹치지 않는다. 식기리필은 주문당 한 번만 필요하므로
여러 트립으로 나뉘어도 첫 번째 트립에만 포함시킨다.

로봇 한 대가 한 번에 한 트립만 수행할 수 있어, 다른 주문을 처리 중일 때 새 주문이 들어오면
거부하지 않고 대기열에 쌓아 뒀다가 현재 주문이 끝나면 이어서 시작한다.

강건성을 위해 다음을 지킨다:
- 서비스 응답의 success는 '명령 수락'만 의미한다. 실제 완료는 각 상태 토픽으로만 판단하며,
  WORKING/MOVING을 먼저 관찰한 뒤에 온 COMPLETED/ARRIVED만 인정한다 (중복·지연 상태 방지).
  스폰도 /food_spawn/status로 완료를 확인한 뒤에만 테이블로 출발한다.
- 모든 비동기 명령에 작업 세대(generation) 번호를 붙여, FAILED 처리나 다음 주문 시작 이후
  뒤늦게 도착한 응답이 새 작업에 영향을 주지 않도록 무시한다.
- Arm/Navigation/스폰의 상태는 해당 서브시스템이 실제로 활성 상태일 때만 작업 판정과
  타임아웃에 사용하고, 그 외 상태에서 온 지연 신호는 현재 작업에 섞지 않는다.
- 손 침입 토픽이 서빙/일시정지 중 일정 시간 끊기면 안전을 확인할 수 없다고 보고 실패
  처리하며, 실패 시 동작 중일 수 있는 Arm/Navigation에 최선 노력으로 정지 명령을 보낸다.
- 상태별 타임아웃은 WORKING/MOVING을 처음 관찰할 때 한 번만 갱신해, 같은 상태를 반복
  발행하는 멈춘 하위 노드가 타임아웃을 무한정 미루지 못하게 한다.
- 타임아웃/거부/FAILED 상태는 자동으로 IDLE로 돌아가지 않고, 작업자가 /manager/reset_fault를
  호출해야 한다. 이때 Arm/Navigation/스폰이 동작 중인 것으로 보이면 초기화를 거부한다.
- 완료(COMPLETED) 상태는 잠깐 유지한 뒤에 다음 단계로 넘어가 UI가 놓치지 않게 한다.
- 대기열에 있던 주문이 실패로 함께 취소되면 /manager/order_cancelled로 테이블 번호를 알린다.
"""

import json
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

from serving_robot_interfaces.srv import OrderRequest, TaskCommand

# 자율주행 command / status (스펙 3장)
NAV_CMD_KITCHEN = 4
NAV_CMD_RESUME = 98
NAV_CMD_PAUSE = 99
NAV_STATUS_MOVING = 1
NAV_STATUS_ARRIVED = 2
NAV_STATUS_FAILED = 3
NAV_LOCATION_KITCHEN = 4
TABLE_ID_MIN, TABLE_ID_MAX = 0, 3

# 로봇팔 command / status (스펙 2장). 로봇팔은 피자 종류를 구분하지 않는다.
# 한 트립(command)은 음료 + 피자*10 + 식기리필*20 + 접시랙*40으로 인코딩한다.
# 접시 개수는 스폰 명령에서만 전달하며 Arm은 같은 랙 동작을 사용한다.
ARM_CMD_RESUME = 98
ARM_CMD_PAUSE = 99
ARM_STATUS_WORKING = 1
ARM_STATUS_COMPLETED = 2
ARM_STATUS_FAILED = 3
ARM_TRIP_DRINK_MAX = 4
ARM_TRIP_PIZZA_MAX = 1
ARM_TRIP_CUTLERY_MAX = 1
ARM_CMD_DRINK_WEIGHT = 1
ARM_CMD_PIZZA_WEIGHT = 10
ARM_CMD_CUTLERY_WEIGHT = 20
ARM_CMD_PLATE_RACK_WEIGHT = 40

# 음식 스폰 command/status: 하위 두 자리는 음료 + 피자종류*10 + 식기*40,
# 백의 자리는 접시 수(1~4)로 인코딩한다. 따라서 피자1+음료1+식기+접시2는
# 251이다. Arm과 달리 피자 종류와 접시 수를 모두 보존해야 한다.
# status는 Arm과 동일한 관례
# (0=IDLE,1=WORKING,2=COMPLETED,3=FAILED)를 스폰 노드 쪽에서 지켜야 한다.
SPAWN_CMD_DRINK_WEIGHT = 1
SPAWN_CMD_PIZZA_TYPE_WEIGHT = 10  # 피자 N번 -> 10*N
SPAWN_CMD_CUTLERY_WEIGHT = 40
SPAWN_CMD_PLATE_COUNT_WEIGHT = 100
SPAWN_STATUS_WORKING = 1
SPAWN_STATUS_COMPLETED = 2
SPAWN_STATUS_FAILED = 3

ITEM_COUNT_MIN, ITEM_COUNT_MAX = 0, 99
CUTLERY_COUNT_MAX = 1  # 식기리필은 개수가 아니라 여부(0/1)이다.
PLATE_COUNT_MAX = 4
MAX_TRIPS_PER_ORDER = 20
MAX_PENDING_ORDERS = 10
COMPLETED_DISPLAY_SEC = 1.5  # 완료 상태를 최소 이 정도는 유지한 뒤 다음 단계로 넘어간다.
NAVIGATION_HANDOFF_DELAY_SEC = 1.0  # 완료 발행 후 Navigation worker가 정리될 시간을 준다.


@dataclass
class Trip:
    """한 번의 주방<->테이블 왕복에서 옮길 항목."""

    pizza_type: Optional[int]  # 1, 2, 3 중 하나 또는 없음(None)
    drink_count: int  # 0~4
    cutlery: bool
    plate_count: int = 0

    def arm_command(self):
        """Arm 노드용 트립 명령 정수를 반환한다."""
        # 로봇팔은 피자 종류를 구분하지 않으므로 있음/없음만 반영한다.
        return (self.drink_count * ARM_CMD_DRINK_WEIGHT
                + (ARM_CMD_PIZZA_WEIGHT if self.pizza_type else 0)
                + (ARM_CMD_CUTLERY_WEIGHT if self.cutlery else 0)
                + (ARM_CMD_PLATE_RACK_WEIGHT if self.plate_count else 0))

    def spawn_command(self):
        """음식 스폰 노드용 트립 명령 정수를 반환한다."""
        # 스폰은 피자 종류를 구분해야 하므로 Arm과 다른 가중치를 쓴다.
        return (self.drink_count * SPAWN_CMD_DRINK_WEIGHT
                + (self.pizza_type * SPAWN_CMD_PIZZA_TYPE_WEIGHT if self.pizza_type else 0)
                + (SPAWN_CMD_CUTLERY_WEIGHT if self.cutlery else 0)
                + self.plate_count * SPAWN_CMD_PLATE_COUNT_WEIGHT)


class _State(IntEnum):
    IDLE = 0
    RETURNING_TO_KITCHEN = 1
    SPAWNING = 2
    MOVING_TO_TABLE = 3
    ARM_SERVING = 4
    ARM_PAUSED = 5
    COMPLETED = 6
    FAILED = 7


# /system/status는 스펙에 값이 정의되어 있지 않아, Manager의 내부 상태를 그대로 발행한다.
class SystemStatus(IntEnum):
    """UI에 발행하는 Manager 시스템 상태."""

    IDLE = 0
    RETURNING_TO_KITCHEN = 1
    SPAWNING = 2
    MOVING_TO_TABLE = 3
    SERVING = 4
    PAUSED_SAFETY = 5
    COMPLETED = 6
    FAILED = 7


_STATE_TO_SYSTEM_STATUS = {
    _State.IDLE: SystemStatus.IDLE,
    _State.RETURNING_TO_KITCHEN: SystemStatus.RETURNING_TO_KITCHEN,
    _State.SPAWNING: SystemStatus.SPAWNING,
    _State.MOVING_TO_TABLE: SystemStatus.MOVING_TO_TABLE,
    _State.ARM_SERVING: SystemStatus.SERVING,
    _State.ARM_PAUSED: SystemStatus.PAUSED_SAFETY,
    _State.COMPLETED: SystemStatus.COMPLETED,
    _State.FAILED: SystemStatus.FAILED,
}


class ManagerNode(Node):
    """주문별 Navigation, Spawn, Arm 작업을 조율하는 ROS 2 노드."""

    def __init__(self):
        """통신 엔드포인트와 작업 상태를 초기화한다."""
        super().__init__('manager_node')

        self._state_timeout_sec = self.declare_parameter('state_timeout_sec', 30.0).value
        self._navigation_timeout_sec = self.declare_parameter(
            'navigation_timeout_sec', 120.0).value
        self._spawn_timeout_sec = self.declare_parameter(
            'spawn_timeout_sec', 60.0).value
        self._arm_timeout_sec = self.declare_parameter(
            'arm_timeout_sec', 600.0).value
        self._safety_cmd_timeout_sec = self.declare_parameter(
            'safety_cmd_timeout_sec', 5.0).value
        self._hand_safety_heartbeat_sec = self.declare_parameter(
            'hand_safety_heartbeat_sec', 2.0).value
        self._navigation_only = bool(
            self.declare_parameter('navigation_only', False).value)
        self._require_navigation_ready = bool(
            self.declare_parameter('require_navigation_ready', False).value)
        timeout_parameters = {
            'state_timeout_sec': self._state_timeout_sec,
            'navigation_timeout_sec': self._navigation_timeout_sec,
            'spawn_timeout_sec': self._spawn_timeout_sec,
            'arm_timeout_sec': self._arm_timeout_sec,
            'safety_cmd_timeout_sec': self._safety_cmd_timeout_sec,
            'hand_safety_heartbeat_sec': self._hand_safety_heartbeat_sec,
        }
        invalid_parameters = [
            name for name, value in timeout_parameters.items()
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0
        ]
        if invalid_parameters:
            raise ValueError(
                f'타임아웃 파라미터는 0보다 큰 숫자여야 합니다: {invalid_parameters}')
        self._state_timeout_sec = float(self._state_timeout_sec)
        self._navigation_timeout_sec = float(self._navigation_timeout_sec)
        self._spawn_timeout_sec = float(self._spawn_timeout_sec)
        self._arm_timeout_sec = float(self._arm_timeout_sec)
        self._safety_cmd_timeout_sec = float(self._safety_cmd_timeout_sec)
        self._hand_safety_heartbeat_sec = float(self._hand_safety_heartbeat_sec)

        self._state = _State.IDLE
        self._table_id = None
        self._serve_queue = []
        self._current_trip = None
        self._pending_orders = []
        # FAILED/새 주문 시작마다 증가한다. 비동기 응답을 보낼 때의 값과 도착했을 때의 값이
        # 다르면 이미 지나간 작업에 대한 응답이므로 무시한다.
        self._task_generation = 0

        self._nav_status = None
        self._nav_location = None
        self._nav_detail_state = None
        self._nav_detail_phase = None
        self._navigation_initialized = False
        self._nav_moving_confirmed = False
        self._nav_command_accepted = False
        self._nav_command_epoch = 0
        self._arm_status = None
        self._arm_working_confirmed = False
        self._arm_command_accepted = False
        self._arm_command_epoch = 0
        self._spawn_status = None
        self._spawn_working_confirmed = False
        self._spawn_command_accepted = False
        self._spawn_command_epoch = 0

        self._hand_intrusion = False
        self._last_hand_intrusion_stamp = None
        self._table_arrived = False
        # 아직 Arm에 명령을 보내지 않은 트립이 손 침입 때문에 대기 중인지 여부.
        # True면 재개 시 98(재개)이 아니라 새 트립 명령을 보내야 한다.
        self._waiting_to_start_trip = False
        # 직전 pause/resume(99/98) 명령의 응답을 기다리는 동안 손 침입 값이 급격히
        # 토글돼도 새 명령을 또 보내지 않고, 응답을 받은 뒤 최신 값으로 재평가한다.
        self._safety_cmd_pending = False
        self._safety_cmd_deadline = None
        self._safety_command_epoch = 0
        self._state_deadline = None
        self._completed_advance_timer = None
        self._navigation_handoff_timer = None
        self._order_service = None

        # All endpoints are relative.  At the root namespace this preserves
        # the legacy API; under /robot1 or /robot2 it creates an independent
        # worker state machine for that robot.
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._arm_client = self.create_client(TaskCommand, 'arm/command')
        self._nav_client = self.create_client(TaskCommand, 'navigation/command')
        self._spawn_client = self.create_client(TaskCommand, 'food_spawn/command')

        self.create_subscription(Int32, 'arm/status', self._on_arm_status, 10)
        self.create_subscription(
            Int32, 'navigation/status', self._on_nav_status, status_qos)
        self.create_subscription(
            Int32, 'navigation/current_location', self._on_nav_location, status_qos)
        self.create_subscription(
            String, 'navigation/detail', self._on_nav_detail, status_qos)
        self.create_subscription(
            Int32, 'food_spawn/status', self._on_spawn_status, 10)
        self.create_subscription(Bool, 'hand_safety/intrusion', self._on_hand_intrusion, 10)
        self.create_subscription(
            Bool, 'serving_robot/emergency_stop', self._on_emergency_stop, 10)

        # 늦게 연결된 UI도 마지막 시스템 상태를 즉시 받을 수 있게 상태 토픽은 latched QoS로
        # 발행한다. 일반 volatile 구독자와도 현재 이후의 발행은 호환된다.
        self._system_status_pub = self.create_publisher(
            Int32, 'system/status', status_qos)
        self._table_arrived_pub = self.create_publisher(
            Bool, 'serving_robot/table_arrived', status_qos)
        self._order_cancelled_pub = self.create_publisher(Int32, 'manager/order_cancelled', 10)
        self.create_service(Trigger, 'manager/reset_fault', self._on_reset_fault)

        self.create_timer(1.0, self._check_timeouts)

        self._publish_system_status()
        self._maybe_enable_order_service()

    # ------------------------------------------------------------------
    # 주문 접수 (UI -> Manager)
    # ------------------------------------------------------------------
    def _maybe_enable_order_service(self):
        if getattr(self, '_order_service', None) is not None:
            return
        if self._require_navigation_ready and not (
                self._navigation_initialized
                and
                self._nav_status == NAV_STATUS_ARRIVED
                and self._nav_location == NAV_LOCATION_KITCHEN):
            return
        self._order_service = self.create_service(
            OrderRequest, 'manager/order', self._on_order_request)
        self.get_logger().info(
            '✅ 주문 서비스 준비 완료: Navigation 초기 위치가 주방으로 확인됐습니다.')

    def _on_order_request(self, request, response):
        self.get_logger().info(
            f'📥 [수신/RECV Service] /manager/order -> Table={request.table_id}, '
            f'Pizza1={request.pizza1_count}, Pizza2={request.pizza2_count}, '
            f'Pizza3={request.pizza3_count}, Drink={request.drink_count}, '
            f'Cutlery={request.cutlery_count}, Plate={request.plate_count}')
        if self._state == _State.FAILED:
            self.get_logger().warn('❌ [FAILED] FAILED 상태입니다. /manager/reset_fault 호출 후 다시 주문해주세요.')
            response.success = False
            return response

        counts = {
            'pizza1_count': request.pizza1_count,
            'pizza2_count': request.pizza2_count,
            'pizza3_count': request.pizza3_count,
            'drink_count': request.drink_count,
        }
        if not (TABLE_ID_MIN <= request.table_id <= TABLE_ID_MAX) or any(
                not (ITEM_COUNT_MIN <= c <= ITEM_COUNT_MAX) for c in counts.values()):
            self.get_logger().warn(f'❌ [REJECT] 잘못된 주문 형식: table={request.table_id}, {counts}')
            response.success = False
            return response
        if not (0 <= request.cutlery_count <= CUTLERY_COUNT_MAX):
            self.get_logger().warn(
                f'❌ [REJECT] 식기리필은 0 또는 1만 가능합니다 (받은 값: {request.cutlery_count}).')
            response.success = False
            return response

        if not (0 <= request.plate_count <= PLATE_COUNT_MAX):
            self.get_logger().warn(
                f'❌ [REJECT] 접시는 0~{PLATE_COUNT_MAX}개만 가능합니다 '
                f'(받은 값: {request.plate_count}).')
            response.success = False
            return response

        if not any(list(counts.values()) + [request.cutlery_count, request.plate_count]):
            self.get_logger().warn('❌ [REJECT] 서빙할 항목이 없는 주문입니다.')
            response.success = False
            return response
        serve_queue = self._build_serve_queue(
            request.pizza1_count, request.pizza2_count, request.pizza3_count,
            request.drink_count, request.cutlery_count, request.plate_count)
        if len(serve_queue) > MAX_TRIPS_PER_ORDER:
            self.get_logger().warn(
                f'❌ [REJECT] 트립 수가 너무 많은 주문입니다 ({len(serve_queue)}건, 최대 {MAX_TRIPS_PER_ORDER}건).')
            response.success = False
            return response

        navigation_ready = self._nav_client.service_is_ready()
        serving_ready = (self._arm_client.service_is_ready()
                         and self._spawn_client.service_is_ready())
        if not navigation_ready or (not self._navigation_only and not serving_ready):
            required = 'Navigation' if self._navigation_only else 'Arm/Navigation/음식 스폰'
            self.get_logger().error(
                f'❌ [REJECT] {required} 서비스가 준비되지 않아 주문을 거부합니다.')
            response.success = False
            return response

        if self._navigation_only:
            # 주행 검증에서는 주문 품목 수와 무관하게 테이블을 한 번 왕복한다.
            serve_queue = [Trip(None, 0, False)]

        if self._state == _State.IDLE:
            response.success = True
            self.get_logger().info(f'✅ [ACCEPTED] 주문 수락 완료: Table={request.table_id}, 총 {len(serve_queue)}회 트립 생성')
            self._start_task(request.table_id, serve_queue)
            return response

        # 로봇은 한 번에 하나만 처리할 수 있어, 다른 테이블을 처리 중이면 거부 대신 대기열에 넣는다.
        if len(self._pending_orders) >= MAX_PENDING_ORDERS:
            self.get_logger().warn(f'❌ [REJECT] 대기열이 가득 찼습니다 (최대 {MAX_PENDING_ORDERS}건).')
            response.success = False
            return response
        self._pending_orders.append((request.table_id, serve_queue))
        self.get_logger().info(
            f'⏳ [QUEUED] 다른 주문 처리 중이라 대기열에 추가: Table={request.table_id}, '
            f'대기열 길이={len(self._pending_orders)}')
        response.success = True
        return response

    def _on_reset_fault(self, request, response):
        if self._state != _State.FAILED:
            response.success = False
            response.message = 'FAILED 상태가 아니어서 초기화하지 않았습니다.'
            return response
        active_subsystems = []
        if not self._navigation_only and self._arm_status == ARM_STATUS_WORKING:
            active_subsystems.append('Arm')
        if self._nav_status == NAV_STATUS_MOVING:
            active_subsystems.append('Navigation')
        if not self._navigation_only and self._spawn_status == SPAWN_STATUS_WORKING:
            active_subsystems.append('Food spawn')
        if active_subsystems:
            response.success = False
            response.message = (
                f'{", ".join(active_subsystems)}이(가) 아직 동작 중인 것으로 보입니다. '
                '실제로 정지했는지 확인한 뒤 다시 시도해주세요.')
            self.get_logger().warn(
                f'하위 노드가 아직 동작 중으로 보여 reset_fault를 거부합니다 '
                f'(arm_status={self._arm_status}, nav_status={self._nav_status}, '
                f'spawn_status={self._spawn_status}).')
            return response
        self.get_logger().info('작업자 확인 후 FAILED 상태를 초기화합니다.')
        self._reset_to_idle()
        response.success = True
        response.message = 'IDLE로 초기화되었습니다.'
        return response

    def _on_emergency_stop(self, msg):
        if not msg.data:
            self.get_logger().info(
                '비상정지 해제 요청 수신; 안전 확인 후 /manager/reset_fault가 필요합니다.')
            return
        if self._state == _State.FAILED:
            return
        self.get_logger().error('HMI 비상정지 요청을 수신했습니다.')
        self._fail()

    @staticmethod
    def _build_serve_queue(
            pizza1_count, pizza2_count, pizza3_count, drink_total,
            cutlery_count, plate_count=0):
        # 피자는 종류별로 한 개씩 나열해 트립을 나눌 때도 어떤 종류인지 잃지 않게 한다.
        pizza_queue = [1] * pizza1_count + [2] * pizza2_count + [3] * pizza3_count
        # 식기리필은 주문당 한 번만 필요하므로 첫 트립에만 붙인다.
        cutlery_remaining = 1 if cutlery_count else 0
        trips = []
        while pizza_queue or drink_total > 0 or cutlery_remaining > 0:
            pizza_type = pizza_queue.pop(0) if pizza_queue else None
            trip_drink = min(ARM_TRIP_DRINK_MAX, drink_total)
            trip_cutlery = cutlery_remaining > 0
            drink_total -= trip_drink
            cutlery_remaining -= int(trip_cutlery)
            trips.append(Trip(pizza_type=pizza_type, drink_count=trip_drink, cutlery=trip_cutlery))
        # 접시 랙은 왼쪽 트레이, 음료/수저는 오른쪽 트레이, 피자는 상판을
        # 사용하므로 첫 트립에 함께 적재할 수 있다.
        if plate_count:
            if trips:
                trips[0].plate_count = plate_count
            else:
                trips.append(Trip(
                    pizza_type=None,
                    drink_count=0,
                    cutlery=False,
                    plate_count=plate_count,
                ))
        return trips

    def _start_task(self, table_id, serve_queue):
        self._clear_order_state()
        self._table_id = table_id
        self._serve_queue = serve_queue
        self.get_logger().info(
            f'주문 시작(gen={self._task_generation}): table={table_id}, serve_queue={serve_queue}')
        self._return_to_kitchen_for_next_trip()

    # ------------------------------------------------------------------
    # 트립 진행 (주방 확인 -> 스폰 -> 이동 -> 서빙 -> 주방 복귀)
    # ------------------------------------------------------------------
    def _return_to_kitchen_for_next_trip(self):
        # 직전 주행에서 주방 도착 위치와 ARRIVED 상태를 모두 확인했다면 중복 command=4를
        # 보내지 않는다. 위치나 상태 중 하나라도 불확실한 Manager 재시작 직후에는 기존처럼
        # 주방 복귀 명령으로 능동 확인한다.
        if (self._nav_location == NAV_LOCATION_KITCHEN
                and self._nav_status == NAV_STATUS_ARRIVED):
            self.get_logger().info(
                '🏠 현재 위치가 주방으로 확인되어 중복 주방 복귀를 생략합니다.')
            self._start_next_trip()
            return
        self._state = _State.RETURNING_TO_KITCHEN
        self._set_state_deadline()
        self._publish_system_status()
        self._call_nav_command(NAV_CMD_KITCHEN)

    def _start_next_trip(self):
        if not self._serve_queue:
            self.get_logger().info(f'테이블 {self._table_id} 주문 전체 완료')
            self._state = _State.COMPLETED
            self._state_deadline = None
            self._publish_system_status()
            # UI가 COMPLETED를 놓치지 않도록 잠깐 유지했다가 다음 단계로 넘어간다.
            self._completed_advance_timer = self.create_timer(
                COMPLETED_DISPLAY_SEC, self._advance_after_completed_delay)
            return
        self._current_trip = self._serve_queue.pop(0)
        if self._navigation_only:
            self.get_logger().info(
                f'🚚 [NAVIGATION ONLY] 테이블 {self._table_id} 왕복 주행 시작')
            self._state = _State.MOVING_TO_TABLE
            self._set_state_deadline()
            self._publish_system_status()
            self._call_nav_command(self._table_id)
            return
        self._state = _State.SPAWNING
        self._set_state_deadline()
        self._publish_system_status()
        self._call_spawn_command(self._current_trip.spawn_command())

    def _advance_after_completed_delay(self):
        # 취소와 콜백 실행이 맞물린 경우 이미 다음 상태로 넘어갔을 수 있다.
        if self._completed_advance_timer is None or self._state != _State.COMPLETED:
            return
        self._completed_advance_timer.cancel()
        self._completed_advance_timer = None
        self._advance_to_next_order()

    def _advance_to_next_order(self):
        if self._pending_orders:
            table_id, serve_queue = self._pending_orders.pop(0)
            self.get_logger().info(f'대기열에서 다음 주문 시작: table={table_id}')
            self._start_task(table_id, serve_queue)
        else:
            self._reset_to_idle()

    # ------------------------------------------------------------------
    # 서비스 호출 (모두 작업 세대를 붙여 지연 응답을 걸러낸다)
    # ------------------------------------------------------------------
    def _call_nav_command(self, command):
        self._nav_status = None
        self._nav_location = None
        self._nav_detail_state = None
        self._nav_detail_phase = None
        self._nav_moving_confirmed = False
        self._nav_command_accepted = False
        self._nav_command_epoch += 1
        command_epoch = self._nav_command_epoch
        expected_state = self._state
        generation = self._task_generation
        request = TaskCommand.Request()
        request.command = command
        cmd_desc = "주방(4)" if command == NAV_CMD_KITCHEN else f"테이블 {command}"
        self.get_logger().info(f'📤 [송신/SEND Service] /navigation/command -> cmd={command} ({cmd_desc})')
        try:
            future = self._nav_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'❌ [FAIL] Navigation 명령 전송 실패: command={command}, {exc}')
            self._fail()
            return
        future.add_done_callback(
            lambda f: self._on_nav_command_response(
                f, command, generation, command_epoch, expected_state))

    def _on_nav_command_response(
            self, future, command, generation, command_epoch, expected_state):
        response, error = _split_future_result(future)
        if (generation != self._task_generation
                or command_epoch != self._nav_command_epoch
                or self._state != expected_state):
            self.get_logger().warn(f'⚠️ 지연된 Navigation 응답 무시: command={command}')
            return
        if error is not None:
            self.get_logger().error(f'❌ Navigation 명령 처리 오류: command={command}, {error}')
            self._fail()
            return
        if not response.success:
            self.get_logger().error(f'❌ Navigation 명령 거부: command={command}')
            self._fail()
            return
        self._nav_command_accepted = True
        self.get_logger().info(f'📥 [수신/RECV Response] Navigation 명령 수락 확인: command={command}')
        self._check_table_arrival()
        self._check_kitchen_arrival()
        self._maybe_enable_order_service()

    def _call_arm_command(self, command):
        self._arm_working_confirmed = False
        self._arm_command_accepted = False
        self._arm_command_epoch += 1
        command_epoch = self._arm_command_epoch
        generation = self._task_generation
        request = TaskCommand.Request()
        request.command = command
        self.get_logger().info(f'📤 [송신/SEND Service] /arm/command -> cmd={command}')
        try:
            future = self._arm_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'❌ [FAIL] Arm 명령 전송 실패: command={command}, {exc}')
            self._fail()
            return
        future.add_done_callback(
            lambda f: self._on_arm_command_response(
                f, command, generation, command_epoch))

    def _on_arm_command_response(self, future, command, generation, command_epoch):
        response, error = _split_future_result(future)
        if (generation != self._task_generation
                or command_epoch != self._arm_command_epoch
                or self._state not in (_State.ARM_SERVING, _State.ARM_PAUSED)):
            self.get_logger().warn(f'⚠️ 지연된 Arm 응답 무시: command={command}')
            return
        if error is not None:
            self.get_logger().error(f'❌ Arm 명령 처리 오류: command={command}, {error}')
            self._fail()
            return
        if not response.success:
            self.get_logger().error(f'❌ Arm 명령 거부: command={command}')
            self._fail()
            return
        self._arm_command_accepted = True
        self.get_logger().info(f'📥 [수신/RECV Response] Arm 명령 수락 확인: command={command}')
        self._check_arm_completed()

    def _call_spawn_command(self, command):
        self._spawn_working_confirmed = False
        self._spawn_command_accepted = False
        self._spawn_command_epoch += 1
        command_epoch = self._spawn_command_epoch
        generation = self._task_generation
        request = TaskCommand.Request()
        request.command = command
        self.get_logger().info(f'📤 [송신/SEND Service] /food_spawn/command -> cmd={command}')
        try:
            future = self._spawn_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'❌ [FAIL] 음식 스폰 명령 전송 실패: command={command}, {exc}')
            self._fail()
            return
        future.add_done_callback(
            lambda f: self._on_spawn_command_response(
                f, command, generation, command_epoch))

    def _on_spawn_command_response(self, future, command, generation, command_epoch):
        response, error = _split_future_result(future)
        if (generation != self._task_generation
                or command_epoch != self._spawn_command_epoch
                or self._state != _State.SPAWNING):
            self.get_logger().warn(f'⚠️ 지연된 음식 스폰 응답 무시: command={command}')
            return
        if error is not None:
            self.get_logger().error(f'❌ 음식 스폰 명령 처리 오류: command={command}, {error}')
            self._fail()
            return
        self._spawn_command_accepted = True
        if not response.success:
            self.get_logger().error(f'❌ 음식 스폰 명령 거부: command={command}')
            self._fail()
            return
        self.get_logger().info(f'📥 [수신/RECV Response] 음식 스폰 명령 수락 확인: command={command}')
        self._check_spawn_completed()

    def _send_safety_command(self, command):
        self._safety_cmd_pending = True
        self._safety_cmd_deadline = (
            self.get_clock().now() + Duration(seconds=self._safety_cmd_timeout_sec))
        self._safety_command_epoch += 1
        command_epoch = self._safety_command_epoch
        generation = self._task_generation
        request = TaskCommand.Request()
        request.command = command
        try:
            future = self._arm_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            self._safety_cmd_pending = False
            self._safety_cmd_deadline = None
            self.get_logger().error(f'Arm 안전 제어 명령 전송 실패: command={command}, {exc}')
            self._fail()
            return
        future.add_done_callback(
            lambda f: self._on_safety_command_response(
                f, command, generation, command_epoch))

    def _on_safety_command_response(self, future, command, generation, command_epoch):
        response, error = _split_future_result(future)
        if (generation != self._task_generation
                or command_epoch != self._safety_command_epoch):
            self.get_logger().warn(f'지연된 Arm 안전 제어 응답 무시: command={command}')
            return
        self._safety_cmd_pending = False
        self._safety_cmd_deadline = None
        if self._state not in (_State.ARM_SERVING, _State.ARM_PAUSED):
            # Pause 응답을 기다리는 사이 트립이 이미 COMPLETED로 넘어간 경우 등.
            self.get_logger().warn(
                f'서빙 단계가 이미 지나가 Arm 안전 제어 응답을 무시합니다: command={command}')
            return
        if error is not None:
            self.get_logger().error(f'Arm 안전 제어 명령 처리 중 오류: command={command}, {error}')
            self._fail()
            return
        if not response.success:
            self.get_logger().error(f'Arm 안전 제어 명령 거부: command={command}')
            self._fail()
            return
        self.get_logger().info(f'Arm 안전 제어 명령 수락: command={command}')
        # 응답을 기다리는 동안 손 침입 값이 바뀌었을 수 있어 최신 값으로 다시 판단한다.
        self._evaluate_hand_safety()

    # ------------------------------------------------------------------
    # 자율주행 상태 처리
    # ------------------------------------------------------------------
    def _on_nav_status(self, msg):
        prev_status = getattr(self, "_nav_status", None)
        self._nav_status = msg.data
        if prev_status != msg.data:
            status_name = {1: "MOVING", 2: "ARRIVED", 3: "FAILED"}.get(msg.data, str(msg.data))
            self.get_logger().info(f'📥 [수신/RECV Topic] /navigation/status = {msg.data} ({status_name})')
        if self._nav_status == NAV_STATUS_MOVING:
            if self._state in (_State.RETURNING_TO_KITCHEN, _State.MOVING_TO_TABLE):
                if not self._nav_moving_confirmed:
                    self._nav_moving_confirmed = True
                    self._set_state_deadline()
            return
        if self._nav_status == NAV_STATUS_FAILED:
            if self._state in (_State.RETURNING_TO_KITCHEN, _State.MOVING_TO_TABLE):
                self._fail()
            else:
                self.get_logger().warn(
                    f'⚠️ {self._state.name} 상태에서 Navigation FAILED 상태를 수신했습니다 (무시).')
            return
        self._check_table_arrival()
        self._check_kitchen_arrival()
        self._maybe_enable_order_service()

    def _on_nav_location(self, msg):
        prev_location = getattr(self, "_nav_location", None)
        self._nav_location = msg.data
        if prev_location != msg.data:
            loc_name = "주방" if msg.data == 4 else f"테이블 {msg.data}"
            self.get_logger().info(f'📥 [수신/RECV Topic] /navigation/current_location = {msg.data} ({loc_name})')
        self._check_table_arrival()
        self._check_kitchen_arrival()
        self._maybe_enable_order_service()

    def _on_nav_detail(self, msg):
        try:
            detail = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self._nav_detail_state = detail.get('state')
        self._nav_detail_phase = detail.get('phase')
        if (self._nav_detail_state == 'SUCCEEDED'
                and self._nav_detail_phase == 'initialized'):
            self._navigation_initialized = True
            self._maybe_enable_order_service()
        self._check_table_arrival()
        self._check_kitchen_arrival()

    def _check_table_arrival(self):
        if (self._state == _State.MOVING_TO_TABLE
                and self._nav_command_accepted
                and self._nav_moving_confirmed
                and self._nav_status == NAV_STATUS_ARRIVED
                and self._nav_location == self._table_id
                and self._nav_detail_state == 'SUCCEEDED'
                and self._nav_detail_phase == 'completed'):
            if self._navigation_only:
                self.get_logger().info(
                    f'🎯 [ARRIVED] 테이블 {self._table_id} 도착 확인 완료 -> 주방 복귀 준비')
                if self._navigation_handoff_timer is None:
                    self._navigation_handoff_timer = self.create_timer(
                        NAVIGATION_HANDOFF_DELAY_SEC,
                        self._finish_navigation_only_table_stop,
                    )
            else:
                self.get_logger().info(
                    f'🎯 [ARRIVED] 테이블 {self._table_id} 도착 확인 완료 -> 서빙 시작')
                self._begin_arm_serving()

    def _finish_navigation_only_table_stop(self):
        timer = self._navigation_handoff_timer
        if timer is None:
            return
        timer.cancel()
        self._navigation_handoff_timer = None
        if self._state != _State.MOVING_TO_TABLE:
            return
        self._return_to_kitchen_for_next_trip()

    def _check_kitchen_arrival(self):
        if (self._state == _State.RETURNING_TO_KITCHEN
                and self._nav_command_accepted
                and self._nav_moving_confirmed
                and self._nav_status == NAV_STATUS_ARRIVED
                and self._nav_location == NAV_LOCATION_KITCHEN
                and self._nav_detail_state == 'SUCCEEDED'
                and self._nav_detail_phase == 'completed'):
            self.get_logger().info('🏠 [ARRIVED] 주방 대기 위치 도착 확인 완료')
            self._start_next_trip()

    # ------------------------------------------------------------------
    # 음식 스폰 상태 처리
    # ------------------------------------------------------------------
    def _on_spawn_status(self, msg):
        status = msg.data
        self._spawn_status = status
        status_name = {1: "WORKING", 2: "COMPLETED", 3: "FAILED"}.get(status, str(status))
        self.get_logger().info(f'📥 [수신/RECV Topic] /food_spawn/status = {status} ({status_name})')
        if status == SPAWN_STATUS_WORKING:
            if self._state == _State.SPAWNING and not self._spawn_working_confirmed:
                self._spawn_working_confirmed = True
                self._set_state_deadline()
            return
        if status == SPAWN_STATUS_FAILED:
            if self._state == _State.SPAWNING:
                self._fail()
            else:
                self.get_logger().warn(
                    f'⚠️ {self._state.name} 상태에서 음식 스폰 FAILED 상태를 수신했습니다 (무시).')
            return
        self._check_spawn_completed()

    def _check_spawn_completed(self):
        if (self._state == _State.SPAWNING
                and self._spawn_status == SPAWN_STATUS_COMPLETED
                and self._spawn_command_accepted
                and self._spawn_working_confirmed):
            self.get_logger().info('🍲 [SPAWN COMPLETE] 음식 스폰 완료 확인 -> 테이블로 출발')
            self._state = _State.MOVING_TO_TABLE
            self._set_state_deadline()
            self._publish_system_status()
            self._call_nav_command(self._table_id)

    # ------------------------------------------------------------------
    # 로봇팔 상태 처리
    # ------------------------------------------------------------------
    def _begin_arm_serving(self):
        if not self._hand_safety_is_fresh():
            self.get_logger().warn(
                '⏳ [SAFETY WAIT] 테이블 도착 신호를 보내고 첫 손 침입 감지 '
                '샘플을 기다립니다.')
            self._waiting_to_start_trip = True
            self._arm_working_confirmed = False
            self._state = _State.ARM_PAUSED
            self._state_deadline = None
            self._publish_system_status()
            return
        if self._hand_intrusion:
            self.get_logger().warn('✋ [HAND INTRUSION] ROI 손 침입 감지 상태 -> 서빙 동작 일시 대기')
            self._waiting_to_start_trip = True
            self._arm_working_confirmed = False
            self._state = _State.ARM_PAUSED
            self._state_deadline = None
            self._publish_system_status()
            return
        self._waiting_to_start_trip = False
        self._state = _State.ARM_SERVING
        self._set_state_deadline()
        self._publish_system_status()
        self._call_arm_command(self._current_trip.arm_command())

    def _on_arm_status(self, msg):
        arm_status = msg.data
        self._arm_status = arm_status
        status_name = {1: "WORKING", 2: "COMPLETED", 3: "FAILED"}.get(arm_status, str(arm_status))
        self.get_logger().info(f'📥 [수신/RECV Topic] /arm/status = {arm_status} ({status_name})')
        if arm_status == ARM_STATUS_WORKING:
            if self._state in (_State.ARM_SERVING, _State.ARM_PAUSED):
                if not self._arm_working_confirmed:
                    self._arm_working_confirmed = True
                    if self._state == _State.ARM_SERVING:
                        self._set_state_deadline()
            return
        if arm_status == ARM_STATUS_FAILED:
            if self._state in (_State.ARM_SERVING, _State.ARM_PAUSED):
                self._fail()
            else:
                self.get_logger().warn(
                    f'⚠️ {self._state.name} 상태에서 Arm FAILED 상태를 수신했습니다 (무시).')
            return
        self._check_arm_completed()

    def _check_arm_completed(self):
        if (self._state in (_State.ARM_SERVING, _State.ARM_PAUSED)
                and self._arm_status == ARM_STATUS_COMPLETED
                and self._arm_command_accepted
                and self._arm_working_confirmed
                and not self._waiting_to_start_trip):
            self.get_logger().info('트립 서빙 완료, 주방 복귀 요청')
            self._return_to_kitchen_for_next_trip()

    # ------------------------------------------------------------------
    # 비전(ROI 손 침입) 처리
    # ------------------------------------------------------------------
    def _on_hand_intrusion(self, msg):
        self._last_hand_intrusion_stamp = self.get_clock().now()
        self._hand_intrusion = msg.data
        self._evaluate_hand_safety()

    def _evaluate_hand_safety(self):
        # 로봇팔이 실제로 서빙 중일 때만 안전 정지/재개를 적용한다 (스펙 4장).
        if self._safety_cmd_pending:
            return
        if self._hand_intrusion and self._state == _State.ARM_SERVING:
            self.get_logger().warn('ROI 손 침입 감지, 로봇팔 일시정지')
            self._state = _State.ARM_PAUSED
            self._state_deadline = None
            self._publish_system_status()
            self._send_safety_command(ARM_CMD_PAUSE)
        elif not self._hand_intrusion and self._state == _State.ARM_PAUSED:
            if self._waiting_to_start_trip:
                self.get_logger().info('손 이탈 확인, 대기 중이던 서빙 시작')
                self._begin_arm_serving()
            else:
                self.get_logger().info('손 이탈 확인, 로봇팔 작업 재개')
                self._state = _State.ARM_SERVING
                self._set_state_deadline()
                self._publish_system_status()
                self._send_safety_command(ARM_CMD_RESUME)

    def _hand_safety_is_fresh(self):
        if self._last_hand_intrusion_stamp is None:
            return False
        age = self.get_clock().now() - self._last_hand_intrusion_stamp
        return age <= Duration(seconds=self._hand_safety_heartbeat_sec)

    # ------------------------------------------------------------------
    # 타임아웃 워치독
    # ------------------------------------------------------------------
    def _set_state_deadline(self):
        if self._state in (_State.RETURNING_TO_KITCHEN, _State.MOVING_TO_TABLE):
            timeout = getattr(
                self, '_navigation_timeout_sec', self._state_timeout_sec)
        elif self._state == _State.SPAWNING:
            timeout = getattr(self, '_spawn_timeout_sec', self._state_timeout_sec)
        elif self._state == _State.ARM_SERVING:
            timeout = getattr(self, '_arm_timeout_sec', self._state_timeout_sec)
        else:
            timeout = self._state_timeout_sec
        self._active_state_timeout_sec = float(timeout)
        self._state_deadline = self.get_clock().now() + Duration(seconds=timeout)

    def _check_timeouts(self):
        now = self.get_clock().now()
        if self._state_deadline is not None and now > self._state_deadline:
            timeout = getattr(
                self, '_active_state_timeout_sec', self._state_timeout_sec)
            self.get_logger().error(
                f'{self._state.name} 상태에서 {timeout}초간 진행 신호가 없어 '
                '실패 처리합니다.')
            self._fail()
            return
        if self._safety_cmd_deadline is not None and now > self._safety_cmd_deadline:
            self.get_logger().error(
                f'안전 제어 명령 응답이 {self._safety_cmd_timeout_sec}초간 없어 실패 처리합니다.')
            self._fail()
            return
        if self._state in (_State.ARM_SERVING, _State.ARM_PAUSED):
            if not self._hand_safety_is_fresh():
                self.get_logger().error(
                    '손 침입 감지 토픽이 끊긴 것으로 보여 안전을 확인할 수 없어 실패 처리합니다.')
                self._fail()
                return

    # ------------------------------------------------------------------
    # 공통
    # ------------------------------------------------------------------
    def _clear_order_state(self):
        self._task_generation += 1
        # 같은 주문 안에서도 명령별 epoch를 증가시켜 이미 등록된 비동기 콜백을 무효화한다.
        self._nav_command_epoch += 1
        self._arm_command_epoch += 1
        self._spawn_command_epoch += 1
        self._safety_command_epoch += 1
        self._table_id = None
        self._serve_queue = []
        self._current_trip = None
        self._waiting_to_start_trip = False
        self._state_deadline = None
        self._safety_cmd_pending = False
        self._safety_cmd_deadline = None
        self._nav_moving_confirmed = False
        self._nav_command_accepted = False
        self._arm_working_confirmed = False
        self._arm_command_accepted = False
        self._spawn_working_confirmed = False
        self._spawn_command_accepted = False
        if self._completed_advance_timer is not None:
            self._completed_advance_timer.cancel()
            self._completed_advance_timer = None
        handoff_timer = getattr(self, '_navigation_handoff_timer', None)
        if handoff_timer is not None:
            handoff_timer.cancel()
            self._navigation_handoff_timer = None

    def _fail(self):
        if self._state == _State.FAILED:
            return
        failed_state = self._state
        self._send_best_effort_stop(failed_state)
        dropped_orders = self._pending_orders
        self._pending_orders = []
        for table_id, _ in dropped_orders:
            self.get_logger().error(f'대기 중이던 테이블 {table_id} 주문을 취소합니다.')
            msg = Int32()
            msg.data = table_id
            self._order_cancelled_pub.publish(msg)
        self.get_logger().error(
            '작업 실패로 처리합니다. /manager/reset_fault 호출 전까지 새 주문을 받지 않습니다.')
        self._clear_order_state()
        self._state = _State.FAILED
        self._publish_system_status()

    def _send_best_effort_stop(self, failed_state):
        """동작 중일 수 있는 하위 노드에 응답을 기다리지 않고 정지 명령을 보낸다."""
        if (not self._navigation_only
                and (failed_state in (_State.ARM_SERVING, _State.ARM_PAUSED)
                     or self._arm_status == ARM_STATUS_WORKING)):
            self._call_emergency_command(self._arm_client, 'Arm', ARM_CMD_PAUSE)
        if (failed_state in (_State.RETURNING_TO_KITCHEN, _State.MOVING_TO_TABLE)
                or self._nav_status == NAV_STATUS_MOVING):
            self._call_emergency_command(
                self._nav_client, 'Navigation', NAV_CMD_PAUSE)

    def _call_emergency_command(self, client, subsystem, command):
        request = TaskCommand.Request()
        request.command = command
        try:
            future = client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'{subsystem} 긴급 정지 명령 전송 실패: command={command}, {exc}')
            return
        future.add_done_callback(
            lambda f: self._on_emergency_command_response(f, subsystem, command))

    def _on_emergency_command_response(self, future, subsystem, command):
        response, error = _split_future_result(future)
        if error is not None:
            self.get_logger().error(
                f'{subsystem} 긴급 정지 명령 처리 오류: command={command}, {error}')
        elif not response.success:
            self.get_logger().error(
                f'{subsystem} 긴급 정지 명령 거부: command={command}')
        else:
            self.get_logger().warn(
                f'{subsystem} 긴급 정지 명령 수락: command={command}')

    def _reset_to_idle(self):
        self._clear_order_state()
        self._state = _State.IDLE
        self._publish_system_status()

    def _publish_system_status(self):
        msg = Int32()
        msg.data = int(_STATE_TO_SYSTEM_STATUS[self._state])
        self.get_logger().info(f'📤 [송신/SEND Topic] /system/status = {msg.data} ({self._state.name})')
        self._system_status_pub.publish(msg)
        self._publish_table_arrived(
            self._state in (_State.ARM_SERVING, _State.ARM_PAUSED))

    def _publish_table_arrived(self, arrived):
        """Gate vision while the robot is docked and the arm may operate."""
        arrived = bool(arrived)
        if not hasattr(self, '_table_arrived_pub'):
            return
        if arrived != self._table_arrived:
            self.get_logger().info(
                f'📤 [송신/SEND Topic] /serving_robot/table_arrived = {arrived}')
        if not arrived and self._table_arrived:
            # A sample from the previous docking window must not authorize the
            # next arm trip before the detector has processed a new image.
            self._last_hand_intrusion_stamp = None
            self._hand_intrusion = False
        self._table_arrived = arrived
        self._table_arrived_pub.publish(Bool(data=arrived))


def _split_future_result(future):
    """future.result()를 항상 소비하면서 (response, error) 튜플로 반환한다."""
    try:
        return future.result(), None
    except Exception as exc:  # noqa: BLE001 - 서비스 통신 실패는 모두 오류로 취급
        return None, exc


def main(args=None):
    """Run the Manager node until shutdown."""
    rclpy.init(args=args)
    node = ManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
