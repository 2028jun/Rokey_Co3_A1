#!/usr/bin/env python3
"""Save one RGB frame from the serving robot's fixed table camera."""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameCapture(Node):
    def __init__(self, topic, output):
        super().__init__("table_camera_capture")
        self.output = output
        self.done = False
        self.create_subscription(Image, topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg):
        channels = max(1, msg.step // msg.width)
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, channels
        )
        encoding = msg.encoding.lower()
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if not cv2.imwrite(self.output, image):
            raise RuntimeError(f"failed to write {self.output}")
        print(
            f"saved {msg.width}x{msg.height} encoding={msg.encoding} to {self.output}",
            flush=True,
        )
        self.done = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic",
        default="/serving_robot/table_camera/color/image_raw",
    )
    parser.add_argument("--output", default="/tmp/table_camera_rgb.png")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = FrameCapture(args.topic, args.output)
    deadline = time.monotonic() + args.timeout
    while not node.done and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    done = node.done
    node.destroy_node()
    rclpy.shutdown()
    if not done:
        raise SystemExit(f"camera frame timeout after {args.timeout:.1f}s")


if __name__ == "__main__":
    main()
