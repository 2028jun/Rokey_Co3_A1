"""Unit tests for the custom compressed image transport."""

import cv2
import numpy as np
import pytest

from hand_safety.jpeg_codec import decode_jpeg, encode_jpeg


def test_jpeg_round_trip_preserves_shape_and_useful_detail():
    frame = np.zeros((96, 128, 3), dtype=np.uint8)
    frame[:, :64] = (20, 80, 220)
    cv2.circle(frame, (90, 48), 24, (230, 180, 30), -1)

    payload = encode_jpeg(frame, quality=90)
    decoded = decode_jpeg(payload)

    assert decoded.shape == frame.shape
    assert decoded.dtype == np.uint8
    assert len(payload) < frame.nbytes
    assert np.mean(np.abs(decoded.astype(float) - frame.astype(float))) < 8.0


@pytest.mark.parametrize("quality", [0, 101])
def test_jpeg_quality_must_be_in_range(quality):
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="quality"):
        encode_jpeg(frame, quality=quality)


def test_invalid_jpeg_is_rejected():
    with pytest.raises(ValueError, match="valid JPEG"):
        decode_jpeg(b"not-a-jpeg")
