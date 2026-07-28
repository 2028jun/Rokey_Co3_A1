from pathlib import Path
import math
import sys


ISAACPJT = Path(__file__).resolve().parents[1] / "isaacpjt"
sys.path.insert(0, str(ISAACPJT))

from orthogonal_route_execution import build_axis_stages, parse_route_points


def test_route_points_become_pivot_and_axis_stages():
    points = parse_route_points(
        [
            {"x": 0.0, "y": 5.0},
            {"x": -0.7, "y": 5.0},
            {"x": -0.7, "y": -2.2},
            {"x": -1.17, "y": -2.2},
        ]
    )
    stages = build_axis_stages(points)

    assert [stage["kind"] for stage in stages] == [
        "pivot", "axis_x", "pivot", "axis_y", "pivot", "axis_x"
    ]
    assert stages[1]["value"] == -0.7
    assert stages[1]["yaw"] == math.pi
    assert stages[3]["value"] == -2.2
    assert stages[3]["yaw"] == -math.pi / 2.0
    assert stages[5]["value"] == -1.17
    assert all(
        stage.get("planned_route") is True
        for stage in stages
        if stage["kind"].startswith("axis_")
    )


def test_diagonal_route_is_rejected():
    try:
        build_axis_stages([(0.0, 0.0), (1.0, 1.0)])
    except ValueError as exc:
        assert "diagonal" in str(exc)
    else:
        raise AssertionError("diagonal route must be rejected")


def test_invalid_or_duplicate_route_is_rejected():
    try:
        parse_route_points([{"x": 0.0, "y": 0.0}, {"x": 0.0, "y": 0.0}])
    except ValueError as exc:
        assert "distinct" in str(exc)
    else:
        raise AssertionError("duplicate-only route must be rejected")
