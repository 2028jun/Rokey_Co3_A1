from pathlib import Path
import sys


ISAACPJT = Path(__file__).resolve().parents[1] / "isaacpjt"
sys.path.insert(0, str(ISAACPJT))

from fleet_peer_yield import in_narrow_forward_strip, is_later_active_order


def test_only_later_active_order_yields():
    active = {"robot1": True, "robot2": True}
    priorities = {"robot1": -100.0, "robot2": -101.0}

    assert is_later_active_order("robot2", "robot1", active, priorities)
    assert not is_later_active_order("robot1", "robot2", active, priorities)


def test_missing_or_inactive_peer_intent_never_hard_stops():
    assert not is_later_active_order(
        "robot2",
        "robot1",
        {"robot1": False, "robot2": True},
        {"robot1": -100.0, "robot2": -101.0},
    )
    assert not is_later_active_order(
        "robot2",
        "robot1",
        {"robot1": True, "robot2": True},
        {"robot2": -101.0},
    )


def test_equal_priority_tie_breaks_to_robot2():
    active = {"robot1": True, "robot2": True}
    priorities = {"robot1": -100.0, "robot2": -100.0}

    assert is_later_active_order("robot2", "robot1", active, priorities)
    assert not is_later_active_order("robot1", "robot2", active, priorities)


def test_peer_strip_is_narrower_than_docked_robot_offset():
    assert in_narrow_forward_strip(1.10, 0.35, 1.45, 0.35)
    assert not in_narrow_forward_strip(1.10, 0.36, 1.45, 0.35)
    assert not in_narrow_forward_strip(1.10, 0.80, 1.45, 0.35)
    assert not in_narrow_forward_strip(-0.10, 0.0, 1.45, 0.35)
    assert not in_narrow_forward_strip(1.46, 0.0, 1.45, 0.35)
