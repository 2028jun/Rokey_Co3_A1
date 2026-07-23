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
# investigation. That version covered the reach position of pass 1-5's
# seat/target (YOLO detected the hand moving through x~535-684, y~236-489).
#
# Pass 6/7 later moved the test actor to a standing position behind the
# customer-side chairs with TABLE_HAND_TARGET on the table's south edge (see
# hand_intrusion_test_actor.py), which moves the reaching hand to a
# completely different part of the frame -- but nobody re-measured this
# polygon against that new position, so roi_intrusion silently never fired
# for two passes (confirmed this pass: 0 ROI-confirmed detections across
# 16 real reach cycles on hardware, even though the wrist lands within
# 1e-16 m of TABLE_HAND_TARGET -- the reach itself was never broken). Directly
# measured the new hand position by projecting the reach-arm glove mesh's
# true (non-orphaned-point) world bounding box through this camera's actual
# intrinsics/extrinsics at full reach: normalized x=[0.164,0.262],
# y=[0.332,0.433] -- nowhere near the old polygon's x=[0.379,0.586] range.
# Replaced with a rectangle around that measurement plus margin for
# detection-box slack; still clear of the gripper false-positive region
# above (x=[0.614,0.649], y=[0.579,0.757]).
TABLE_ROI_NORMALIZED = (
    (0.10, 0.28),
    (0.32, 0.28),
    (0.32, 0.52),
    (0.10, 0.52),
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
