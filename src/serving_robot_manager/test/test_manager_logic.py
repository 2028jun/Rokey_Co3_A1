"""Manager 노드의 상태 전이 안전장치 회귀 테스트."""

from unittest.mock import Mock

from rclpy.time import Time
from std_msgs.msg import Int32
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
    manager.get_logger = Mock(return_value=Mock())
    return manager


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


def test_failure_requests_arm_and_navigation_pause():
    """실패 시 동작 중인 Arm과 Navigation에 pause를 보내는지 확인한다."""
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
    assert arm_request.command == 99
    assert nav_request.command == 99


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
    manager._on_spawn_status(Int32(data=SPAWN_STATUS_WORKING))
    manager._on_spawn_status(Int32(data=SPAWN_STATUS_COMPLETED))
    manager._on_nav_status(Int32(data=NAV_STATUS_MOVING))
    manager._on_nav_location(Int32(data=2))
    manager._on_nav_status(Int32(data=2))
    manager._on_arm_status(Int32(data=ARM_STATUS_WORKING))
    manager._on_arm_status(Int32(data=2))
    manager._on_nav_status(Int32(data=NAV_STATUS_MOVING))
    manager._on_nav_location(Int32(data=4))
    manager._on_nav_status(Int32(data=2))

    assert manager._state == _State.COMPLETED
