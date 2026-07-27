from unittest.mock import Mock

from two_wheel_rails.navigation_subsystem_node import (
    NAV_MOVING,
    NavigationSubsystemNode,
)


def _bare_subsystem():
    subsystem = NavigationSubsystemNode.__new__(NavigationSubsystemNode)
    subsystem._controller = Mock()
    subsystem._controller.drive_distance.return_value = True
    subsystem._publish_status = Mock()
    subsystem.get_logger = Mock(return_value=Mock())
    return subsystem


def test_park_out_is_skipped_away_from_table_without_force():
    subsystem = _bare_subsystem()

    subsystem._park_out_if_needed((0.0, 0.0, 0.0))

    subsystem._controller.drive_distance.assert_not_called()
    subsystem._publish_status.assert_not_called()


def test_park_out_is_forced_after_completed_table_mission():
    subsystem = _bare_subsystem()

    subsystem._park_out_if_needed((0.0, 0.0, 0.0), force=True)

    subsystem._publish_status.assert_called_once_with(
        NAV_MOVING, "PARKING_OUT", "park_out"
    )
    subsystem._controller.drive_distance.assert_called_once_with(
        0.50, -0.20, label="park_out"
    )


def test_table_pose_still_triggers_park_out_without_force():
    subsystem = _bare_subsystem()

    subsystem._park_out_if_needed((-1.77, -2.15, 3.14))

    subsystem._controller.drive_distance.assert_called_once_with(
        0.50, -0.20, label="park_out"
    )
