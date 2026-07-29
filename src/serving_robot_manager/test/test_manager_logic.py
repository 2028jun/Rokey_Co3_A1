"""Manager 노드의 상태 전이 안전장치 회귀 테스트."""

import time
from unittest.mock import Mock

from rclpy.time import Time
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from serving_robot_interfaces.srv import TaskCommand
from serving_robot_manager.manager_node import (
    ARM_STATUS_WORKING,
    NAV_STATUS_MOVING,
    SPAWN_STATUS_COMPLETED,
    SPAWN_STATUS_WORKING,
    ManagerNode,
    Trip,
    _State,
)
from serving_robot_manager.fleet_manager_node import FleetManager


class _FinishedFuture:
    def __init__(self, response):
        self._response = response

    def result(self):
        return self._response

    def add_done_callback(self, callback):
        callback(self)


def _bare_manager(state=_State.IDLE):
    manager = ManagerNode.__new__(ManagerNode)
    manager._state = state
    manager._navigation_only = False
    manager._require_navigation_ready = False
    manager._navigation_initialized = False
    manager._nav_detail_state = None
    manager._nav_detail_phase = None
    manager._order_service = object()
    manager._table_id = None
    manager._occupancy_phase = 'clear'
    manager._table_occupancy_pub = Mock()
    manager.get_namespace = Mock(return_value='/robot1')
    manager.get_logger = Mock(return_value=Mock())
    return manager


def _order_request(table_id, preferred_robot=''):
    request = type('OrderRequest', (), {})()
    request.table_id = table_id
    request.pizza1_count = 1
    request.pizza2_count = 0
    request.pizza3_count = 0
    request.drink_count = 0
    request.cutlery_count = 0
    request.plate_count = 0
    request.preferred_robot = preferred_robot
    return request


def test_working_status_does_not_create_idle_deadline():
    """IDLE에서 온 WORKING 신호가 타임아웃을 만들지 않는지 확인한다."""
    manager = _bare_manager()
    manager._arm_status = None
    manager._arm_working_confirmed = False
    manager._set_state_deadline = Mock()

    manager._on_arm_status(Int32(data=ARM_STATUS_WORKING))

    manager._set_state_deadline.assert_not_called()
    assert manager._arm_working_confirmed is False


def test_paused_arm_without_hand_heartbeat_fails():
    """일시정지 중에도 손 감지 하트비트 단절을 실패 처리하는지 확인한다."""
    manager = _bare_manager(_State.ARM_PAUSED)
    manager._state_deadline = None
    manager._safety_cmd_deadline = None
    manager._last_hand_intrusion_stamp = None
    manager._fail = Mock()
    clock = Mock()
    clock.now.return_value = Time(seconds=10)
    manager.get_clock = Mock(return_value=clock)

    manager._check_timeouts()

    manager._fail.assert_called_once_with()


def test_hand_safety_gpu_inference_gap_does_not_fail_delivery():
    """허용 범위의 긴 GPU 추론 중에는 정상 서빙을 실패시키지 않는다."""
    manager = _bare_manager(_State.ARM_SERVING)
    manager._state_deadline = None
    manager._safety_cmd_deadline = None
    manager._last_hand_intrusion_stamp = Time(seconds=10)
    manager._hand_safety_heartbeat_sec = 15.0
    manager._fail = Mock()
    clock = Mock()
    clock.now.return_value = Time(seconds=22)
    manager.get_clock = Mock(return_value=clock)

    manager._check_timeouts()

    manager._fail.assert_not_called()


def test_hand_safety_gap_beyond_gpu_grace_still_fails_closed():
    """추론 허용 시간을 넘긴 실제 토픽 단절은 계속 실패 처리한다."""
    manager = _bare_manager(_State.ARM_SERVING)
    manager._state_deadline = None
    manager._safety_cmd_deadline = None
    manager._last_hand_intrusion_stamp = Time(seconds=10)
    manager._hand_safety_heartbeat_sec = 15.0
    manager._fail = Mock()
    clock = Mock()
    clock.now.return_value = Time(seconds=26)
    manager.get_clock = Mock(return_value=clock)

    manager._check_timeouts()

    manager._fail.assert_called_once_with()


def test_stale_safety_response_cannot_clear_current_pending_command():
    """이전 안전 응답이 현재 pending 명령을 지우지 않는지 확인한다."""
    manager = _bare_manager(_State.ARM_SERVING)
    manager._task_generation = 5
    manager._safety_command_epoch = 8
    manager._safety_cmd_pending = True
    deadline = object()
    manager._safety_cmd_deadline = deadline
    future = _FinishedFuture(TaskCommand.Response(success=True))

    manager._on_safety_command_response(future, 98, 4, 7)

    assert manager._safety_cmd_pending is True
    assert manager._safety_cmd_deadline is deadline


def test_reset_fault_is_rejected_while_food_spawn_is_working():
    """스폰 동작 중 fault reset을 거부하는지 확인한다."""
    manager = _bare_manager(_State.FAILED)
    manager._arm_status = 0
    manager._nav_status = 0
    manager._spawn_status = SPAWN_STATUS_WORKING
    manager._reset_to_idle = Mock()

    response = manager._on_reset_fault(None, Trigger.Response())

    assert response.success is False
    assert 'Food spawn' in response.message
    manager._reset_to_idle.assert_not_called()


def test_spawn_completion_waits_for_command_acceptance():
    """스폰 완료 전 서비스 수락도 반드시 확인하는지 검사한다."""
    manager = _bare_manager(_State.SPAWNING)
    manager._spawn_status = SPAWN_STATUS_COMPLETED
    manager._spawn_working_confirmed = True
    manager._spawn_command_accepted = False
    manager._table_id = 2
    manager._set_state_deadline = Mock()
    manager._publish_system_status = Mock()
    manager._call_nav_command = Mock()

    manager._check_spawn_completed()

    assert manager._state == _State.SPAWNING
    manager._call_nav_command.assert_not_called()

    manager._spawn_command_accepted = True
    manager._check_spawn_completed()

    assert manager._state == _State.MOVING_TO_TABLE
    manager._call_nav_command.assert_called_once_with(2)


def test_new_order_at_confirmed_kitchen_skips_return_navigation():
    """주방 도착 상태가 확인된 새 주문은 command=4를 다시 보내지 않는다."""
    manager = _bare_manager()
    manager._nav_location = 4
    manager._nav_status = 2
    manager._kitchen_arrival_confirmed = True
    manager._serve_queue = [Trip(1, 0, False)]
    manager._call_nav_command = Mock()
    manager._start_next_trip = Mock()

    manager._return_to_kitchen_for_next_trip()

    manager._call_nav_command.assert_not_called()
    manager._start_next_trip.assert_called_once_with()


def test_unknown_kitchen_state_still_requests_return_navigation():
    """위치 또는 상태가 불확실하면 안전하게 기존 주방 복귀를 수행한다."""
    manager = _bare_manager()
    manager._nav_location = 4
    manager._nav_status = None
    manager._set_state_deadline = Mock()
    manager._publish_system_status = Mock()
    manager._call_nav_command = Mock()

    manager._return_to_kitchen_for_next_trip()

    assert manager._state == _State.RETURNING_TO_KITCHEN
    manager._call_nav_command.assert_called_once_with(4)


def test_kitchen_location_alone_does_not_release_queued_order():
    """주방 좌표만 먼저 와도 이전 Navigation 종료 전 주문을 시작하지 않는다."""
    manager = _bare_manager(_State.RETURNING_TO_KITCHEN)
    manager._nav_location = 4
    manager._nav_status = 1
    manager._nav_detail_state = 'PLANNING'
    manager._nav_detail_phase = 'planning'
    manager._nav_command_accepted = True
    manager._nav_moving_confirmed = True
    manager._kitchen_location_since = time.monotonic() - 30.0
    manager._kitchen_arrival_confirmed = False
    manager._publish_table_occupancy = Mock()
    manager._start_next_trip = Mock()

    manager._check_kitchen_arrival()

    assert manager._state == _State.RETURNING_TO_KITCHEN
    assert manager._kitchen_arrival_confirmed is False
    manager._start_next_trip.assert_not_called()

    manager._nav_status = 2
    manager._nav_detail_state = 'SUCCEEDED'
    manager._nav_detail_phase = 'completed'
    manager._check_kitchen_arrival()

    assert manager._kitchen_arrival_confirmed is True
    manager._start_next_trip.assert_called_once_with()


def test_navigation_only_trip_drives_to_table_without_spawn_or_arm():
    """주행 전용 트립은 스폰·팔 없이 테이블 도착 후 즉시 주방으로 복귀한다."""
    manager = _bare_manager()
    manager._navigation_only = True
    manager._serve_queue = [Trip(1, 2, True)]
    manager._table_id = 2
    manager._set_state_deadline = Mock()
    manager._publish_system_status = Mock()
    manager._call_nav_command = Mock()
    handoff_timer = Mock()
    manager._navigation_handoff_timer = None
    manager.create_timer = Mock(return_value=handoff_timer)

    manager._start_next_trip()

    assert manager._state == _State.MOVING_TO_TABLE
    manager._call_nav_command.assert_called_once_with(2)

    manager._nav_command_accepted = True
    manager._nav_moving_confirmed = True
    manager._nav_status = 2
    manager._nav_location = 2
    manager._nav_detail_state = 'SUCCEEDED'
    manager._nav_detail_phase = 'completed'
    manager._return_to_kitchen_for_next_trip = Mock()
    manager._check_table_arrival()

    manager._return_to_kitchen_for_next_trip.assert_not_called()
    manager._finish_navigation_only_table_stop()

    handoff_timer.cancel.assert_called_once_with()
    manager._return_to_kitchen_for_next_trip.assert_called_once_with()


def test_isaac_arrival_alone_does_not_complete_navigation():
    """Isaac 중복 ARRIVED만으로는 subsystem 완료 전이를 시작하지 않는다."""
    manager = _bare_manager(_State.MOVING_TO_TABLE)
    manager._navigation_only = True
    manager._table_id = 1
    manager._nav_command_accepted = True
    manager._nav_moving_confirmed = True
    manager._nav_status = 2
    manager._nav_location = 1
    manager._nav_detail_state = None
    manager._nav_detail_phase = None
    manager._navigation_handoff_timer = None
    manager.create_timer = Mock(return_value=Mock())

    manager._check_table_arrival()
    manager.create_timer.assert_not_called()

    manager._on_nav_detail(String(
        data='{"state": "SUCCEEDED", "phase": "completed"}'))
    manager.create_timer.assert_called_once()


def test_order_service_waits_for_navigation_initialization():
    """멀티 모드 주문 서비스는 주방 초기화 완료 후에만 열린다."""
    manager = _bare_manager()
    manager._require_navigation_ready = True
    manager._order_service = None
    manager._navigation_initialized = False
    manager._nav_status = None
    manager._nav_location = None
    manager._nav_detail_state = None
    manager._nav_detail_phase = None
    manager.create_service = Mock(return_value=object())

    manager._maybe_enable_order_service()
    manager.create_service.assert_not_called()

    manager._nav_location = 4
    manager._nav_status = 2
    manager._maybe_enable_order_service()
    manager.create_service.assert_not_called()

    manager._on_nav_detail(String(
        data='{"state": "SUCCEEDED", "phase": "initialized"}'
    ))

    manager.create_service.assert_called_once()
    assert manager._order_service is not None


def test_fleet_assigns_second_order_to_robot2_while_robot1_is_reserved():
    """첫 주문 직후 들어온 두 번째 주문은 robot1이 아니라 robot2에 배정한다."""
    fleet = FleetManager.__new__(FleetManager)
    fleet._robots = ['robot1', 'robot2']
    fleet._serialize_shared_payloads = False
    fleet._states = {'robot1': 0, 'robot2': 0}
    fleet._reserved = {}
    fleet._table_claims = {}
    fleet._status_pub = Mock()
    fleet.get_logger = Mock(return_value=Mock())
    accepted = type('Response', (), {'success': True})()
    fleet._order_clients = {
        'robot1': Mock(service_is_ready=Mock(return_value=True)),
        'robot2': Mock(service_is_ready=Mock(return_value=True)),
    }
    for client in fleet._order_clients.values():
        client.call_async.return_value = _FinishedFuture(accepted)

    first_response = type('Response', (), {'success': False, 'assigned_robot': ''})()
    second_response = type('Response', (), {'success': False, 'assigned_robot': ''})()
    fleet._on_order(_order_request(0), first_response)
    fleet._on_order(_order_request(1), second_response)

    assert first_response.success is True
    assert second_response.success is True
    assert first_response.assigned_robot == 'robot1'
    assert second_response.assigned_robot == 'robot2'
    fleet._order_clients['robot1'].call_async.assert_called_once()
    fleet._order_clients['robot2'].call_async.assert_called_once()


def test_third_auto_order_retries_to_first_completed_robot():
    """동시 3주문 중 세 번째는 먼저 복귀 완료한 로봇에 배정된다."""
    fleet = FleetManager.__new__(FleetManager)
    fleet._robots = ['robot1', 'robot2']
    fleet._serialize_shared_payloads = False
    fleet._states = {'robot1': 0, 'robot2': 0}
    fleet._reserved = {}
    fleet._table_claims = {}
    fleet._status_pub = Mock()
    fleet.get_logger = Mock(return_value=Mock())
    accepted = type('Response', (), {'success': True})()
    fleet._order_clients = {
        'robot1': Mock(service_is_ready=Mock(return_value=True)),
        'robot2': Mock(service_is_ready=Mock(return_value=True)),
    }
    for client in fleet._order_clients.values():
        client.call_async.return_value = _FinishedFuture(accepted)

    first = type('Response', (), {'success': False, 'assigned_robot': ''})()
    second = type('Response', (), {'success': False, 'assigned_robot': ''})()
    third_initial = type(
        'Response', (), {'success': False, 'assigned_robot': ''}
    )()

    fleet._on_order(_order_request(0), first)
    fleet._on_order(_order_request(1), second)
    assert first.assigned_robot == 'robot1'
    assert second.assigned_robot == 'robot2'

    # Both workers report that their accepted jobs have started. This also
    # clears the short dispatch reservations.
    fleet._on_status('robot1', Int32(data=3))
    fleet._on_status('robot2', Int32(data=3))
    fleet._on_order(_order_request(2), third_initial)
    assert third_initial.success is False
    assert third_initial.assigned_robot == ''

    # robot2 returns first. HMI retries the same pending third request and
    # Fleet must choose robot2 while robot1 remains busy.
    fleet._on_status('robot2', Int32(data=6))
    third_retry = type(
        'Response', (), {'success': False, 'assigned_robot': ''}
    )()
    fleet._on_order(_order_request(2), third_retry)

    assert third_retry.success is True
    assert third_retry.assigned_robot == 'robot2'
    assert fleet._order_clients['robot1'].call_async.call_count == 1
    assert fleet._order_clients['robot2'].call_async.call_count == 2


def test_fleet_assigns_order_to_robot2_while_robot1_is_driving():
    """robot1이 주행 중이면 다음 주문은 남은 유휴 robot2에 배정한다."""
    fleet = FleetManager.__new__(FleetManager)
    fleet._robots = ['robot1', 'robot2']
    fleet._serialize_shared_payloads = False
    fleet._states = {'robot1': 3, 'robot2': 0}
    fleet._reserved = {}
    fleet._table_claims = {}
    fleet.get_logger = Mock(return_value=Mock())
    accepted = type('Response', (), {'success': True})()
    fleet._order_clients = {
        'robot1': Mock(service_is_ready=Mock(return_value=True)),
        'robot2': Mock(service_is_ready=Mock(return_value=True)),
    }
    for client in fleet._order_clients.values():
        client.call_async.return_value = _FinishedFuture(accepted)

    response = type('Response', (), {'success': False, 'assigned_robot': ''})()
    fleet._on_order(_order_request(1), response)

    assert response.success is True
    assert response.assigned_robot == 'robot2'
    fleet._order_clients['robot1'].call_async.assert_not_called()
    fleet._order_clients['robot2'].call_async.assert_called_once()


def test_fleet_respects_preferred_robot():
    """preferred_robot이 지정되면 해당 로봇만 배정한다."""
    fleet = FleetManager.__new__(FleetManager)
    fleet._robots = ['robot1', 'robot2']
    fleet._serialize_shared_payloads = False
    fleet._states = {'robot1': 0, 'robot2': 0}
    fleet._reserved = {}
    fleet._table_claims = {}
    fleet.get_logger = Mock(return_value=Mock())
    accepted = type('Response', (), {'success': True})()
    fleet._order_clients = {
        'robot1': Mock(service_is_ready=Mock(return_value=True)),
        'robot2': Mock(service_is_ready=Mock(return_value=True)),
    }
    for client in fleet._order_clients.values():
        client.call_async.return_value = _FinishedFuture(accepted)

    response = type('Response', (), {'success': False, 'assigned_robot': ''})()
    fleet._on_order(_order_request(0, preferred_robot='robot2'), response)

    assert response.success is True
    assert response.assigned_robot == 'robot2'
    fleet._order_clients['robot1'].call_async.assert_not_called()
    fleet._order_clients['robot2'].call_async.assert_called_once()


def test_fleet_rejects_busy_preferred_robot():
    """지정 로봇이 busy면 주문을 거부한다."""
    fleet = FleetManager.__new__(FleetManager)
    fleet._robots = ['robot1', 'robot2']
    fleet._serialize_shared_payloads = False
    fleet._states = {'robot1': 3, 'robot2': 0}
    fleet._reserved = {}
    fleet._table_claims = {}
    fleet.get_logger = Mock(return_value=Mock())
    fleet._order_clients = {
        'robot1': Mock(service_is_ready=Mock(return_value=True)),
        'robot2': Mock(service_is_ready=Mock(return_value=True)),
    }

    response = type('Response', (), {'success': True, 'assigned_robot': 'x'})()
    fleet._on_order(_order_request(0, preferred_robot='robot1'), response)

    assert response.success is False
    assert response.assigned_robot == ''
    fleet._order_clients['robot1'].call_async.assert_not_called()
    fleet._order_clients['robot2'].call_async.assert_not_called()


def test_path_clearance_detects_crossing_segments():
    from serving_robot_manager.path_yield_coordinator_node import (
        _path_clearance,
        _trim_horizon,
        PathYieldCoordinator,
        RobotIntent,
    )

    a = [(0.0, 5.0), (0.0, 0.0)]
    b = [(0.0, 0.0), (0.0, 5.0)]
    assert _path_clearance(a, b) < 0.1

    far = [(3.0, 5.0), (3.0, 0.0)]
    assert _path_clearance(a, far) > 2.0

    trimmed = _trim_horizon([(0.0, 0.0), (0.0, 10.0)], 2.5)
    assert abs(trimmed[-1][1] - 2.5) < 0.05

    coord = PathYieldCoordinator.__new__(PathYieldCoordinator)
    # Earlier order has higher priority (more positive / less negative).
    coord._intents = {
        "robot1": RobotIntent("robot1", priority=-100.0),
        "robot2": RobotIntent("robot2", priority=-101.0),
    }
    assert coord._choose_yielder("robot1", "robot2") == "robot2"


def test_far_shared_aisle_does_not_path_conflict():
    """멀리 있는 두 로봇은 통로 polyline이 겹쳐도 pause 하지 않는다."""
    from serving_robot_manager.path_yield_coordinator_node import (
        PathYieldCoordinator,
        RobotIntent,
    )
    import time

    coord = PathYieldCoordinator.__new__(PathYieldCoordinator)
    coord._clearance_m = 0.55
    coord._pose_clearance_m = 0.90
    coord._horizon_m = 1.6
    coord._engage_m = 2.0
    coord._intent_stale_sec = 3.0
    now = time.monotonic()
    coord._intents = {
        "robot1": RobotIntent(
            "robot1",
            priority=-100.0,
            active=True,
            phase="approaching",
            table_id=0,
            pose_xy=(0.0, 4.5),
            polyline=[(0.0, 4.5), (0.0, -2.2), (-1.7, -2.2)],
            updated_at=now,
        ),
        "robot2": RobotIntent(
            "robot2",
            priority=-101.0,
            active=True,
            phase="approaching",
            table_id=1,
            pose_xy=(0.0, -1.0),
            polyline=[(0.0, -1.0), (0.0, -2.2), (1.7, -2.2)],
            updated_at=now,
        ),
    }
    assert coord._conflict("robot1", "robot2") is False


def test_near_shared_aisle_path_conflicts():
    from serving_robot_manager.path_yield_coordinator_node import (
        PathYieldCoordinator,
        RobotIntent,
    )
    import time

    coord = PathYieldCoordinator.__new__(PathYieldCoordinator)
    coord._clearance_m = 0.55
    coord._pose_clearance_m = 0.90
    coord._horizon_m = 1.6
    coord._engage_m = 2.0
    coord._intent_stale_sec = 3.0
    now = time.monotonic()
    coord._intents = {
        "robot1": RobotIntent(
            "robot1",
            priority=-100.0,
            active=True,
            phase="approaching",
            table_id=0,
            pose_xy=(0.0, 1.0),
            polyline=[(0.0, 1.0), (0.0, -2.2), (-1.7, -2.2)],
            updated_at=now,
        ),
        "robot2": RobotIntent(
            "robot2",
            priority=-101.0,
            active=True,
            phase="approaching",
            table_id=1,
            pose_xy=(0.0, 0.2),
            polyline=[(0.0, 0.2), (0.0, -2.2), (1.7, -2.2)],
            updated_at=now,
        ),
    }
    assert coord._conflict("robot1", "robot2") is True


def test_stationary_occupy_does_not_path_conflict_other_table():
    """다른 테이블로 가는 로봇은 docked peer의 stale path에 막히지 않는다."""
    from serving_robot_manager.path_yield_coordinator_node import (
        PathYieldCoordinator,
        RobotIntent,
    )
    import time

    coord = PathYieldCoordinator.__new__(PathYieldCoordinator)
    coord._clearance_m = 0.65
    coord._pose_clearance_m = 1.25
    coord._horizon_m = 2.8
    coord._engage_m = 2.0
    coord._intent_stale_sec = 3.0
    now = time.monotonic()
    # robot1 finished approach to table 0 but still advertises old aisle path.
    coord._intents = {
        "robot1": RobotIntent(
            "robot1",
            priority=-100.0,
            active=True,
            phase="occupying",
            table_id=0,
            pose_xy=(-1.7, -2.2),
            polyline=[(0.0, 4.0), (0.0, -2.2), (-1.7, -2.2)],
            updated_at=now,
        ),
        "robot2": RobotIntent(
            "robot2",
            priority=-101.0,
            active=True,
            phase="returning",
            table_id=1,
            pose_xy=(0.4, 1.0),
            polyline=[(0.4, 1.0), (0.4, 4.5)],
            updated_at=now,
        ),
    }
    assert coord._conflict("robot1", "robot2") is False


def test_same_table_occupying_conflicts_with_approaching():
    from serving_robot_manager.path_yield_coordinator_node import (
        PathYieldCoordinator,
        RobotIntent,
    )
    import time

    coord = PathYieldCoordinator.__new__(PathYieldCoordinator)
    coord._clearance_m = 0.65
    coord._pose_clearance_m = 1.25
    coord._horizon_m = 2.8
    coord._intent_stale_sec = 3.0
    now = time.monotonic()
    coord._intents = {
        "robot1": RobotIntent(
            "robot1",
            priority=-100.0,
            active=True,
            phase="occupying",
            table_id=0,
            pose_xy=(-1.7, -2.2),
            updated_at=now,
        ),
        "robot2": RobotIntent(
            "robot2",
            priority=-101.0,
            active=True,
            phase="approaching",
            table_id=0,
            pose_xy=(0.0, 2.0),
            polyline=[(0.0, -2.2), (-1.7, -2.2)],
            updated_at=now,
        ),
    }
    assert coord._conflict("robot1", "robot2") is True
    assert coord._choose_yielder("robot1", "robot2") == "robot2"


def test_different_tables_no_same_table_conflict():
    from serving_robot_manager.path_yield_coordinator_node import (
        PathYieldCoordinator,
        RobotIntent,
    )
    import time

    coord = PathYieldCoordinator.__new__(PathYieldCoordinator)
    coord._clearance_m = 0.65
    coord._pose_clearance_m = 1.25
    coord._horizon_m = 2.8
    coord._intent_stale_sec = 3.0
    now = time.monotonic()
    coord._intents = {
        "robot1": RobotIntent(
            "robot1",
            priority=-100.0,
            active=True,
            phase="occupying",
            table_id=0,
            pose_xy=(-1.7, -2.2),
            updated_at=now,
        ),
        "robot2": RobotIntent(
            "robot2",
            priority=-101.0,
            active=True,
            phase="approaching",
            table_id=2,
            pose_xy=(0.4, 3.0),
            polyline=[(0.4, 0.7), (1.7, 0.7)],
            updated_at=now,
        ),
    }
    assert coord._same_table_conflict(
        coord._intents["robot1"], coord._intents["robot2"]
    ) is False


def test_manager_publishes_occupancy_phases():
    manager = _bare_manager(_State.MOVING_TO_TABLE)
    manager._table_id = 1
    manager._occupancy_phase = 'clear'
    manager._publish_table_occupancy('approaching')
    assert manager._occupancy_phase == 'approaching'
    manager._table_occupancy_pub.publish.assert_called()
    payload = manager._table_occupancy_pub.publish.call_args.args[0].data
    assert '"phase": "approaching"' in payload or '"phase":"approaching"' in payload
    assert '"table_id": 1' in payload or '"table_id":1' in payload

    manager._publish_table_occupancy('clear')
    assert manager._occupancy_phase == 'clear'


def test_fleet_tracks_table_claims_without_blocking_dispatch():
    fleet = FleetManager.__new__(FleetManager)
    fleet._robots = ['robot1', 'robot2']
    fleet._serialize_shared_payloads = False
    fleet._states = {'robot1': 4, 'robot2': 0}
    fleet._reserved = {}
    fleet._table_claims = {}
    fleet.get_logger = Mock(return_value=Mock())
    accepted = type('Response', (), {'success': True})()
    fleet._order_clients = {
        'robot1': Mock(service_is_ready=Mock(return_value=True)),
        'robot2': Mock(service_is_ready=Mock(return_value=True)),
    }
    for client in fleet._order_clients.values():
        client.call_async.return_value = _FinishedFuture(accepted)

    fleet._on_table_occupancy(String(
        data='{"table_id": 0, "robot_id": "robot1", "phase": "serving"}'
    ))
    assert fleet._table_owner(0) == 'robot1'

    response = type('Response', (), {'success': False, 'assigned_robot': ''})()
    fleet._on_order(_order_request(0), response)
    assert response.success is True
    assert response.assigned_robot == 'robot2'
    fleet.get_logger().warning.assert_called()


def test_fleet_expires_stuck_reservation_so_idle_peer_gets_orders():
    """예약이 sticky로 남으면 idle 로봇 주문이 전부 거부되던 회귀를 막는다."""
    fleet = FleetManager.__new__(FleetManager)
    fleet._robots = ['robot1', 'robot2']
    fleet._serialize_shared_payloads = False
    fleet._states = {'robot1': 1, 'robot2': 0}
    # Pretend robot2 was reserved long ago and never cleared.
    fleet._reserved = {'robot2': time.monotonic() - 10.0}
    fleet._table_claims = {}
    fleet.get_logger = Mock(return_value=Mock())
    accepted = type('Response', (), {'success': True})()
    fleet._order_clients = {
        'robot1': Mock(service_is_ready=Mock(return_value=True)),
        'robot2': Mock(service_is_ready=Mock(return_value=True)),
    }
    for client in fleet._order_clients.values():
        client.call_async.return_value = _FinishedFuture(accepted)

    response = type('Response', (), {'success': False, 'assigned_robot': ''})()
    fleet._on_order(_order_request(1, preferred_robot='robot2'), response)
    assert response.success is True
    assert response.assigned_robot == 'robot2'


def test_plate_rack_shares_first_trip_and_encodes_plate_count():
    """접시 랙이 첫 트립에 합쳐지고 스폰 명령에 수량이 보존되는지 확인한다."""
    trips = ManagerNode._build_serve_queue(1, 0, 0, 2, 1, 3)

    assert trips == [Trip(1, 2, True, 3)]
    assert trips[0].arm_command() == 72
    assert trips[0].spawn_command() == 352

    for plate_count in range(1, 5):
        plate_only = ManagerNode._build_serve_queue(
            0, 0, 0, 0, 0, plate_count
        )
        assert plate_only == [Trip(None, 0, False, plate_count)]
        assert plate_only[0].arm_command() == 40
        assert plate_only[0].spawn_command() == 100 * plate_count


def test_all_pizza_types_use_pepperoni_spawn_command():
    """HMI의 피자 3종 모두 현재 구현된 페퍼로니 스폰 코드 10을 쓴다."""
    assert Trip(1, 0, False).spawn_command() == 10
    assert Trip(2, 0, False).spawn_command() == 10
    assert Trip(3, 0, False).spawn_command() == 10

    # 피자 외 항목의 기존 가중치는 그대로 조합된다.
    assert Trip(2, 2, True, 3).spawn_command() == 352


def test_failure_aborts_arm_and_navigation_workers():
    """실패 시 pause가 아니라 abort를 보내 남은 worker를 종료하는지 확인한다."""
    manager = _bare_manager(_State.ARM_SERVING)
    manager._arm_status = ARM_STATUS_WORKING
    manager._nav_status = NAV_STATUS_MOVING
    manager._arm_client = Mock()
    manager._nav_client = Mock()
    manager._arm_client.call_async.return_value = Mock()
    manager._nav_client.call_async.return_value = Mock()

    manager._send_best_effort_stop(_State.ARM_SERVING)

    arm_request = manager._arm_client.call_async.call_args.args[0]
    nav_request = manager._nav_client.call_async.call_args.args[0]
    assert arm_request.command == 97
    assert nav_request.command == 97


def test_hmi_emergency_stop_enters_failed_state():
    """HMI 비상정지는 manager의 안전 실패 처리를 즉시 시작한다."""
    manager = _bare_manager(_State.MOVING_TO_TABLE)
    manager._fail = Mock()

    manager._on_emergency_stop(type('Message', (), {'data': True})())

    manager._fail.assert_called_once_with()


def test_hmi_emergency_stop_topic_is_global_for_both_robot_namespaces():
    """robot1/robot2 Manager가 HMI의 동일한 전역 E-STOP 토픽을 구독한다."""
    from serving_robot_manager.manager_node import EMERGENCY_STOP_TOPIC

    assert EMERGENCY_STOP_TOPIC == '/serving_robot/emergency_stop'


def test_hmi_emergency_stop_release_does_not_resume_automatically():
    """E-STOP 해제 신호만으로 FAILED 작업을 자동 재개하지 않는다."""
    manager = _bare_manager(_State.FAILED)
    manager._fail = Mock()

    manager._on_emergency_stop(type('Message', (), {'data': False})())

    manager._fail.assert_not_called()
    assert manager._state == _State.FAILED


def test_inactive_navigation_moving_does_not_refresh_deadline():
    """IDLE에서 Navigation MOVING이 deadline을 만들지 않는지 확인한다."""
    manager = _bare_manager()
    manager._nav_status = None
    manager._nav_moving_confirmed = False
    manager._set_state_deadline = Mock()

    manager._on_nav_status(Int32(data=NAV_STATUS_MOVING))

    manager._set_state_deadline.assert_not_called()
    assert manager._nav_moving_confirmed is False


def test_normal_single_trip_reaches_completed():
    """정상적인 단일 트립이 COMPLETED까지 도달하는지 확인한다."""
    manager = _bare_manager()
    manager._table_id = None
    manager._serve_queue = []
    manager._current_trip = None
    manager._pending_orders = []
    manager._task_generation = 0
    manager._nav_status = None
    manager._nav_location = None
    manager._nav_moving_confirmed = False
    manager._nav_command_accepted = False
    manager._nav_command_epoch = 0
    manager._arm_status = None
    manager._arm_working_confirmed = False
    manager._arm_command_accepted = False
    manager._arm_command_epoch = 0
    manager._spawn_status = None
    manager._spawn_working_confirmed = False
    manager._spawn_command_accepted = False
    manager._spawn_command_epoch = 0
    manager._hand_intrusion = False
    manager._last_hand_intrusion_stamp = Time(seconds=10)
    manager._hand_safety_heartbeat_sec = 2.0
    manager._waiting_to_start_trip = False
    manager._safety_cmd_pending = False
    manager._safety_cmd_deadline = None
    manager._safety_command_epoch = 0
    manager._state_deadline = None
    manager._state_timeout_sec = 30.0
    manager._completed_advance_timer = None
    manager._publish_system_status = Mock()
    manager.create_timer = Mock(return_value=Mock())
    clock = Mock()
    clock.now.return_value = Time(seconds=10)
    manager.get_clock = Mock(return_value=clock)

    accepted = TaskCommand.Response(success=True)
    manager._nav_client = Mock()
    manager._spawn_client = Mock()
    manager._arm_client = Mock()
    for client in (manager._nav_client, manager._spawn_client, manager._arm_client):
        client.call_async.side_effect = lambda request: _FinishedFuture(accepted)

    manager._start_task(2, [Trip(1, 4, True)])
    manager._on_nav_status(Int32(data=NAV_STATUS_MOVING))
    manager._on_nav_location(Int32(data=4))
    manager._on_nav_status(Int32(data=2))
    manager._on_nav_detail(String(
        data='{"state": "SUCCEEDED", "phase": "completed"}'))
    manager._on_spawn_status(Int32(data=SPAWN_STATUS_WORKING))
    manager._on_spawn_status(Int32(data=SPAWN_STATUS_COMPLETED))
    manager._on_nav_status(Int32(data=NAV_STATUS_MOVING))
    manager._on_nav_location(Int32(data=2))
    manager._on_nav_status(Int32(data=2))
    manager._on_nav_detail(String(
        data='{"state": "SUCCEEDED", "phase": "completed"}'))
    manager._on_arm_status(Int32(data=ARM_STATUS_WORKING))
    manager._on_arm_status(Int32(data=2))
    manager._on_nav_status(Int32(data=NAV_STATUS_MOVING))
    manager._on_nav_location(Int32(data=4))
    manager._on_nav_status(Int32(data=2))
    manager._on_nav_detail(String(
        data='{"state": "SUCCEEDED", "phase": "completed"}'))

    assert manager._state == _State.COMPLETED
