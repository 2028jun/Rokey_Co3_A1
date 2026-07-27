"""Small JPEG codec helpers shared by the ROS transport nodes."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def encode_jpeg(frame: Any, quality: int = 90) -> bytes:
    """Encode a BGR image as JPEG and return its serialized payload."""
    if not 1 <= quality <= 100:
        raise ValueError("JPEG quality must be between 1 and 100")

    bgr_frame = np.asarray(frame)
    if bgr_frame.ndim != 3 or bgr_frame.shape[2] != 3:
        raise ValueError(
            f"expected HxWx3 BGR frame, got shape={bgr_frame.shape}"
        )
    if bgr_frame.dtype != np.uint8:
        bgr_frame = np.clip(bgr_frame, 0, 255).astype(np.uint8)
    bgr_frame = np.ascontiguousarray(bgr_frame)

    success, encoded = cv2.imencode(
        ".jpg",
        bgr_frame,
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if not success:
        raise RuntimeError("OpenCV failed to encode the JPEG frame")
    return encoded.tobytes()


def decode_jpeg(payload: bytes | bytearray | memoryview) -> Any:
    """Decode a serialized JPEG payload into a BGR uint8 image."""
    encoded = np.frombuffer(payload, dtype=np.uint8)
    if encoded.size == 0:
        raise ValueError("compressed image payload is empty")
    frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("compressed image payload is not a valid JPEG")
    return frame
