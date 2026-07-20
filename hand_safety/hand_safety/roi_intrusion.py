"""Tabletop polygon ROI configuration and intrusion checks."""

from __future__ import annotations

import cv2
import numpy as np


# Normalized polygon corners measured from the fixed table camera image.
# Order: far-left, far-right, near-right, near-left.
# Keeping these values normalized makes the ROI independent of resolution.
TABLE_ROI_NORMALIZED = (
    (0.379, 0.310),
    (0.547, 0.286),
    (0.731, 0.629),
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
