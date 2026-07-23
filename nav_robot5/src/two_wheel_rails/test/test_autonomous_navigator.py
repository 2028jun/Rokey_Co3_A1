from two_wheel_rails.autonomous_navigator import merge_short_segments, simplify_path


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
