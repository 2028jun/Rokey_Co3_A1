"""Tabletop polygon ROI configuration and intrusion checks."""

from __future__ import annotations

import cv2
import numpy as np


# Normalized polygon corners measured from the fixed table camera image.
# Order: far-left, far-right, near-right, near-left.
# Keeping these values normalized makes the ROI independent of resolution.
#
# near-right pulled in from (0.731, 0.629) to (0.586, 0.629): the RG2
# gripper's own visual meshes sit right at the table's near-right corner in
# this camera's frame and were being misdetected by YOLO as two "hand"
# objects, so roi_intrusion read true continuously even at rest (verified
# on hardware via /hand_detection/detections: a consistent false-positive
# "hand" at bbox_xyxy ~[786-831, 556-727] px at 1280x960). Repainting the
# gripper matte black was tried first per a team member's suggestion, but
# on this Isaac Sim build (5.1.0-rc.19) material bindings authored on an
# already-populated/instanced stage don't reach the render even when bound
# on the correct instance root (same class of late-edit-not-reaching-Hydra
# problem as the UsdSkel joint rotation documented in
# hand_intrusion_test_actor.py) -- see GPU_RUN_LOG.txt pass 3 for the full
# investigation. Tightening the ROI instead: at 1280x960, the old
# near-right corner (936, 604) put the polygon's right edge at x=830 by
# y=632, comfortably overlapping the gripper's box (left edge x=786); the
# new corner (750, 604) keeps the right edge at x<=760 through that same
# y-range, clearing the gripper with margin. Confirmed this still covers
# the actual hand-test reach position (YOLO detected the test character's
# hand moving through x~535-684, y~236-489 during a reach, all comfortably
# inside the tightened polygon).
TABLE_ROI_NORMALIZED = (
    (0.379, 0.310),
    (0.547, 0.286),
    (0.586, 0.629),
    (0.438, 0.732),
)


def get_roi_polygon(
    image_width: int, image_height: int
) -> list[tuple[int, int]]:
    """Return the tabletop ROI polygon in image pixel coordinates."""
    max_x = max(0, image_width - 1)
    max_y = max(0, image_height - 1)
    return [
        (
            max(0, min(max_x, round(x_ratio * image_width))),
            max(0, min(max_y, round(y_ratio * image_height))),
        )
        for x_ratio, y_ratio in TABLE_ROI_NORMALIZED
    ]


def box_intrudes_roi(
    box_xyxy: list[int], roi_polygon: list[tuple[int, int]]
) -> bool:
    """Return True when a detection box overlaps the tabletop polygon."""
    box_x1, box_y1, box_x2, box_y2 = box_xyxy
    if box_x2 <= box_x1 or box_y2 <= box_y1:
        return False

    box_polygon = np.asarray(
        [
            (box_x1, box_y1),
            (box_x2, box_y1),
            (box_x2, box_y2),
            (box_x1, box_y2),
        ],
        dtype=np.float32,
    )
    table_polygon = np.asarray(roi_polygon, dtype=np.float32)
    intersection_area, _ = cv2.intersectConvexConvex(
        box_polygon, table_polygon
    )
    return intersection_area > 0.0
