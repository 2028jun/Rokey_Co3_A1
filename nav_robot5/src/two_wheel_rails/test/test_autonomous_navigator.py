from types import SimpleNamespace
from unittest.mock import Mock, patch

from two_wheel_rails.autonomous_navigator import (
    SimplifiedPathNavigator,
    merge_short_segments,
    simplify_path,
)


def test_straight_path_collapses_to_one_segment():
    points = [(0.0, 0.0), (0.01, 1.0), (-0.01, 2.0), (0.0, 3.0)]
    assert simplify_path(points, 0.05) == [(0.0, 0.0), (0.0, 3.0)]


def test_right_angle_keeps_corner():
    points = [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (1.0, 2.0)]
    result = simplify_path(points, 0.05)
    assert result == [(0.0, 0.0), (0.0, 2.0), (1.0, 2.0)]


def test_short_segments_keep_final_goal():
    points = [(0.0, 0.0), (0.05, 0.0), (1.0, 0.0)]
    assert merge_short_segments(points, 0.25) == [(0.0, 0.0), (1.0, 0.0)]


def _bare_navigator(robot_id="robot2", priority=-100.0):
    navigator = SimplifiedPathNavigator.__new__(SimplifiedPathNavigator)
    navigator._robot_id = robot_id
    navigator._intent_priority = priority
    navigator._peer_intents = {}
    return navigator


def test_table_side_groups_opposite_docks():
    assert SimplifiedPathNavigator._table_side(0) == "west"
    assert SimplifiedPathNavigator._table_side(2) == "west"
    assert SimplifiedPathNavigator._table_side(1) == "east"
    assert SimplifiedPathNavigator._table_side(3) == "east"
    assert SimplifiedPathNavigator._table_side(None) is None


def test_same_side_peer_does_not_block_kitchen_departure():
    navigator = _bare_navigator(robot_id="robot2", priority=-100.0)
    navigator._peer_intents["robot1"] = {
        "active": True,
        "phase": "approaching",
        "table_id": 2,
        "priority": -101.0,
        "pose_xy": (0.0, 2.0),
    }

    assert navigator._opposite_outbound_blocker(0) is None


def test_opposite_side_later_priority_waits_at_kitchen():
    navigator = _bare_navigator(robot_id="robot2", priority=-101.0)
    navigator._peer_intents["robot1"] = {
        "active": True,
        "phase": "approaching",
        "table_id": 0,
        "priority": -100.0,
        "pose_xy": (0.0, 2.0),
    }

    assert navigator._opposite_outbound_blocker(1) == "robot1"


def test_opposite_side_earlier_priority_does_not_wait_at_kitchen():
    navigator = _bare_navigator(robot_id="robot1", priority=-100.0)
    navigator._peer_intents["robot2"] = {
        "active": True,
        "phase": "approaching",
        "table_id": 1,
        "priority": -101.0,
        "pose_xy": (0.0, 2.0),
    }

    assert navigator._opposite_outbound_blocker(0) is None


def test_peer_in_table_bay_releases_opposite_corridor():
    navigator = _bare_navigator(robot_id="robot2", priority=-100.0)
    navigator._peer_intents["robot1"] = {
        "active": True,
        "phase": "approaching",
        "table_id": 0,
        "priority": -101.0,
        "pose_xy": (-1.7, -2.2),
    }

    assert navigator._opposite_outbound_blocker(1) is None


def test_robot_corridor_lanes_are_separated():
    robot1 = _bare_navigator(robot_id="robot1")
    robot2 = _bare_navigator(robot_id="robot2")

    assert robot1._lane_x() == -0.70
    assert robot2._lane_x() == 0.70


def test_fleet_lane_preserves_axis_aligned_route():
    navigator = _bare_navigator(robot_id="robot1")
    points = [(0.0, 5.0), (0.0, -2.2), (-1.17, -2.2)]

    shifted = navigator._apply_fleet_lane(points)

    assert shifted == [
        {"x": 0.0, "y": 5.0},
        {"x": -0.7, "y": 5.0},
        {"x": -0.7, "y": -2.2},
        {"x": -1.17, "y": -2.2},
    ]
    for start, end in zip(shifted, shifted[1:]):
        assert start["x"] == end["x"] or start["y"] == end["y"]


def test_cached_map_pose_uses_tracker_without_tf_lookup():
    navigator = _bare_navigator()
    navigator._tracker = SimpleNamespace(xy=(1.25, -0.75), yaw=0.4)
    navigator._last_map_pose = None

    with patch(
        "two_wheel_rails.autonomous_navigator.resolve_map_xy",
        side_effect=AssertionError("blocking TF lookup must not run"),
    ), patch(
        "two_wheel_rails.autonomous_navigator.resolve_map_yaw",
        side_effect=AssertionError("blocking TF lookup must not run"),
    ):
        assert navigator._cached_map_pose() == (1.25, -0.75, 0.4)


def test_mission_completion_is_handled_before_fleet_heartbeat():
    navigator = _bare_navigator()
    navigator._nav = Mock()
    navigator._last_status = {
        "mission_id": "table_0_test",
        "state": "completed",
        "phase": "completed",
    }
    navigator._intent_mission_id = "table_0_test"
    navigator._publish_fleet_intent = Mock(
        side_effect=AssertionError("heartbeat must not delay completion")
    )

    with patch("two_wheel_rails.autonomous_navigator.rclpy.spin_once"):
        result = navigator._wait_for_mission_completion(
            "table_0_test", 10.0, "table_0"
        )

    assert result == (True, "completed")
    navigator._publish_fleet_intent.assert_not_called()
