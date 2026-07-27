#!/usr/bin/env python3
"""Compress a local raw ROS image stream for transport to a remote worker."""

from __future__ import annotations

import time
import traceback

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image

from hand_safety.jpeg_codec import encode_jpeg


CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class JpegCompressorNode(Node):
    """Publish JPEG only when at least one compressed subscriber exists."""

    def __init__(self) -> None:
        super().__init__("jpeg_compressor")
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "output_topic", "/camera/color/image_raw/compressed"
        )
        self.declare_parameter("jpeg_quality", 90)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.jpeg_quality = int(
            self.get_parameter("jpeg_quality").value
        )
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            CompressedImage, output_topic, CAMERA_QOS
        )
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, CAMERA_QOS
        )
        self.encoded_frames = 0
        self.encoded_bytes = 0
        self.stats_started = time.monotonic()
        self.stats_timer = self.create_timer(5.0, self.log_statistics)

        self.get_logger().info(
            f"JPEG transport ready: {input_topic} -> {output_topic}, "
            f"quality={self.jpeg_quality}"
        )

    def image_callback(self, message: Image) -> None:
        # Avoid spending CPU on JPEG encoding until the remote detector is
        # discovered. With no raw subscriber on the remote PC, DDS keeps the
        # large source message on this computer.
        if self.publisher.get_subscription_count() == 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
            payload = encode_jpeg(frame, self.jpeg_quality)
            compressed = CompressedImage()
            compressed.header = message.header
            compressed.format = "jpeg"
            compressed.data = payload
            self.publisher.publish(compressed)
            self.encoded_frames += 1
            self.encoded_bytes += len(payload)
        except (CvBridgeError, ValueError, RuntimeError) as exc:
            self.get_logger().error(
                f"JPEG compression failed: {type(exc).__name__}: {exc}"
            )

    def log_statistics(self) -> None:
        now = time.monotonic()
        elapsed = now - self.stats_started
        if self.encoded_frames == 0 or elapsed <= 0.0:
            return
        rate = self.encoded_frames / elapsed
        megabits_per_second = self.encoded_bytes * 8.0 / elapsed / 1e6
        self.get_logger().info(
            f"JPEG output: {rate:.1f} fps, {megabits_per_second:.1f} Mbps"
        )
        self.encoded_frames = 0
        self.encoded_bytes = 0
        self.stats_started = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: JpegCompressorNode | None = None
    try:
        node = JpegCompressorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            if node is not None:
                node.get_logger().fatal(traceback.format_exc())
            else:
                print(traceback.format_exc())
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
