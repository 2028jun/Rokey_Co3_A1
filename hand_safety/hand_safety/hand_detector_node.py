#!/usr/bin/env python3
"""ROS 2 node for YOLO hand detection on an RGB image topic."""

from __future__ import annotations

import json
import threading
import time
import traceback
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO

from hand_safety.roi_intrusion import box_intrudes_roi, get_roi_polygon


DEFAULT_MODEL_PATH = (
    Path(get_package_share_directory("hand_safety"))
    / "models"
    / "YOLOv10n_hands.pt"
)


class HandDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_detector_node")

        self.declare_parameter(
            "input_topic",
            "/serving_robot/table_camera/color/image_raw",
        )
        self.declare_parameter(
            "output_image_topic", "/hand_detection/image"
        )
        self.declare_parameter(
            "detections_topic", "/hand_detection/detections"
        )
        self.declare_parameter(
            "roi_intrusion_topic", "/hand_safety/roi_intrusion"
        )
        self.declare_parameter(
            "table_arrived_topic", "/serving_robot/table_arrived"
        )
        self.declare_parameter("model_path", str(DEFAULT_MODEL_PATH))
        self.declare_parameter("confidence", 0.40)
        self.declare_parameter("iou", 0.7)
        # A larger inference image preserves more pixels for distant,
        # small hands. Ultralytics keeps the source aspect ratio and pads
        # it internally, so this does not distort the camera image.
        self.declare_parameter("image_size", 1280)
        # The fixed table camera provides enough hand detail at 1280 px.
        # A full-frame pass avoids the fourfold GPU load and memory pressure
        # of tiled inference while Isaac Sim is using the same GPU.
        self.declare_parameter("tiled_inference", False)
        self.declare_parameter("tile_overlap", 0.15)
        self.declare_parameter("roi_inference", True)
        self.declare_parameter("roi_inference_padding", 0.05)
        self.declare_parameter("device", "0")
        self.declare_parameter("process_rate", 30.0)
        self.declare_parameter("confirmation_frames", 3)
        # Defaulted on for the vision GPU test: `ros2 run hand_safety
        # hand_detector_node` alone now opens a live cv2 window and
        # publishes the annotated stream, no --ros-args needed.
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("show_window", True)

        self.input_topic = str(
            self.get_parameter("input_topic").value
        )
        output_image_topic = str(
            self.get_parameter("output_image_topic").value
        )
        detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        roi_intrusion_topic = str(
            self.get_parameter("roi_intrusion_topic").value
        )
        table_arrived_topic = str(
            self.get_parameter("table_arrived_topic").value
        )
        model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser().resolve()
        self.confidence = float(
            self.get_parameter("confidence").value
        )
        self.iou = float(self.get_parameter("iou").value)
        self.image_size = int(
            self.get_parameter("image_size").value
        )
        self.tiled_inference = bool(
            self.get_parameter("tiled_inference").value
        )
        self.tile_overlap = float(
            self.get_parameter("tile_overlap").value
        )
        self.roi_inference = bool(
            self.get_parameter("roi_inference").value
        )
        self.roi_inference_padding = float(
            self.get_parameter("roi_inference_padding").value
        )
        self.device = str(self.get_parameter("device").value)
        process_rate = float(
            self.get_parameter("process_rate").value
        )
        self.confirmation_frames = int(
            self.get_parameter("confirmation_frames").value
        )
        self.publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self.show_window = bool(
            self.get_parameter("show_window").value
        )

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")
        if not 0.0 <= self.iou <= 1.0:
            raise ValueError("iou must be between 0.0 and 1.0")
        if self.image_size <= 0:
            raise ValueError("image_size must be greater than zero")
        if not 0.0 <= self.tile_overlap < 0.5:
            raise ValueError(
                "tile_overlap must be between 0.0 and 0.5"
            )
        if not 0.0 <= self.roi_inference_padding < 0.5:
            raise ValueError(
                "roi_inference_padding must be between 0.0 and 0.5"
            )
        if process_rate <= 0.0:
            raise ValueError("process_rate must be greater than zero")
        if self.confirmation_frames <= 0:
            raise ValueError(
                "confirmation_frames must be greater than zero"
            )
        if self.device != "cpu" and not torch.cuda.is_available():
            raise RuntimeError(
                f"GPU device '{self.device}' was requested, but CUDA is "
                "not available. Check the NVIDIA driver and CUDA-enabled "
                "PyTorch installation, or set device:=cpu."
            )
        if not model_path.is_file() or model_path.stat().st_size == 0:
            raise FileNotFoundError(
                f"model file does not exist or is empty: {model_path}"
            )

        self.get_logger().info(f"Loading model: {model_path}")
        self.model = YOLO(str(model_path))
        self.get_logger().info("Output classes: {0: 'hand'}")
        self.get_logger().info(
            f"CUDA available: {torch.cuda.is_available()}, "
            f"device: {self.device}"
        )
        # Ultralytics creates its predictor and CUDA execution context lazily
        # on the first call. Doing that inside a ROS executor worker can make
        # the node appear unresponsive while camera callbacks are active.
        # Initialize it once here on the main thread before subscriptions and
        # executor workers start.
        warmup_frame = np.zeros((960, 1280, 3), dtype=np.uint8)
        warmup_started = time.perf_counter()
        self.model.predict(
            source=warmup_frame,
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.image_size,
            device=self.device,
            agnostic_nms=True,
            verbose=False,
        )
        self.get_logger().info(
            "Model warm-up complete in "
            f"{time.perf_counter() - warmup_started:.2f}s"
        )

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_message: Image | None = None
        self.frame_sequence = 0
        self.last_processed_sequence = 0
        self.previous_time: float | None = None
        self.smoothed_fps = 0.0
        self.input_size_logged = False
        self.consecutive_detection_frames = 0
        self.warning_times: dict[str, float] = {}
        self.detection_state_lock = threading.Lock()
        self.detection_enabled = False

        image_output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_publisher = self.create_publisher(
            Image, output_image_topic, image_output_qos
        )
        self.detections_publisher = self.create_publisher(
            String, detections_topic, 10
        )
        self.roi_intrusion_publisher = self.create_publisher(
            Bool, roi_intrusion_topic, 10
        )
        # Sensor-data QoS keeps the camera queue short, so the executor gets
        # recent frames instead of building an unbounded image backlog.
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.rgb_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.image_callback_group,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_callback_group = MutuallyExclusiveCallbackGroup()
        self.table_arrived_subscription = self.create_subscription(
            Bool,
            table_arrived_topic,
            self.table_arrived_callback,
            qos_profile=state_qos,
            callback_group=self.state_callback_group,
        )
        self.timer = self.create_timer(
            1.0 / process_rate, self.process_latest_frame
        )

        self.get_logger().info(
            f"Subscribing to RGB images on {self.input_topic}"
        )
        self.get_logger().info(
            f"Publishing JSON detections to {detections_topic} and ROI "
            f"state to {roi_intrusion_topic}"
        )
        if self.publish_annotated_image:
            self.get_logger().info(
                f"Publishing debug images to {output_image_topic}"
            )
        else:
            self.get_logger().info(
                "Annotated image publication is disabled for runtime "
                "stability; set publish_annotated_image:=true for debugging"
            )
        self.get_logger().info(
            f"Hand inference is disabled until {table_arrived_topic} is true"
        )

    def image_callback(self, message: Image) -> None:
        with self.frame_lock:
            self.latest_message = message
            self.frame_sequence += 1

    def table_arrived_callback(self, message: Bool) -> None:
        enabled = bool(message.data)
        with self.detection_state_lock:
            changed = enabled != self.detection_enabled
            self.detection_enabled = enabled
            if not enabled:
                self.consecutive_detection_frames = 0
                # Serialize this clear with frame-result publication so a
                # completed inference cannot overwrite the disabled state.
                if not rclpy.ok():
                    return
                clear_message = String()
                clear_message.data = json.dumps(
                    {
                        "event": "table_arrived_changed",
                        "count": 0,
                        "detection_enabled": False,
                        "table_arrived": False,
                        "confirmed": False,
                        "consecutive_detection_frames": 0,
                        "confirmation_frames": self.confirmation_frames,
                        "roi_intrusion": False,
                        "detections": [],
                    }
                )
                self.detections_publisher.publish(clear_message)
                self.roi_intrusion_publisher.publish(Bool(data=False))
        if changed:
            state = "enabled" if enabled else "disabled"
            self.get_logger().info(
                f"Hand inference {state}: table_arrived={enabled}"
            )

    def process_latest_frame(self) -> None:
        with self.frame_lock:
            message = self.latest_message
            frame_sequence = self.frame_sequence

        if message is None:
            self.warn_throttled(
                "waiting_for_rgb_images",
                f"Waiting for RGB images on {self.input_topic}.",
            )
            return
        if frame_sequence == self.last_processed_sequence:
            return
        self.last_processed_sequence = frame_sequence

        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
            if not self.input_size_logged:
                height, width = frame.shape[:2]
                self.get_logger().info(
                    f"Camera input: {width}x{height}; "
                    f"YOLO inference size: {self.image_size}"
                )
                if max(width, height) < self.image_size:
                    self.get_logger().warning(
                        "The camera image is smaller than the configured "
                        "inference size. Increasing the camera publisher's "
                        "native resolution will improve distant-hand detail "
                        "more than upscaling this image."
                    )
                self.input_size_logged = True

            with self.detection_state_lock:
                detection_enabled = self.detection_enabled

            candidate_detections = (
                self.detect_hands(frame) if detection_enabled else []
            )
            if not rclpy.ok():
                return
            frame_height, frame_width = frame.shape[:2]
            roi_polygon = get_roi_polygon(frame_width, frame_height)
            roi_detections = []
            for detection in candidate_detections:
                if box_intrudes_roi(
                    detection["bbox_xyxy"], roi_polygon
                ):
                    detection["roi_intrusion"] = True
                    roi_detections.append(detection)

            # Recheck the arrival state after GPU inference before confirming
            # and publishing the result.
            with self.detection_state_lock:
                detection_enabled = (
                    detection_enabled and self.detection_enabled
                )
                if detection_enabled and roi_detections:
                    self.consecutive_detection_frames += 1
                else:
                    self.consecutive_detection_frames = 0
                consecutive_detection_frames = (
                    self.consecutive_detection_frames
                )

            is_confirmed = (
                detection_enabled
                and consecutive_detection_frames
                >= self.confirmation_frames
            )
            detections = roi_detections if is_confirmed else []

            intrusion_detected = bool(detections)
            fps = self.update_fps()
            render_debug_image = (
                self.publish_annotated_image or self.show_window
            )
            if render_debug_image:
                for detection in detections:
                    x1, y1, x2, y2 = detection["bbox_xyxy"]
                    self.draw_detection(
                        frame,
                        x1,
                        y1,
                        x2,
                        y2,
                        detection["class_name"],
                        detection["confidence"],
                    )

                self.draw_roi(
                    frame,
                    roi_polygon,
                    intrusion_detected,
                    detection_enabled=detection_enabled,
                )
                if not detection_enabled:
                    confirmation_status = "disabled: robot not arrived"
                elif is_confirmed:
                    confirmation_status = "confirmed"
                else:
                    confirmation_status = (
                        f"checking {consecutive_detection_frames}/"
                        f"{self.confirmation_frames}"
                    )
                cv2.putText(
                    frame,
                    f"FPS: {fps:.1f}  Hands: {len(detections)}  "
                    f"{confirmation_status}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            if self.publish_annotated_image:
                annotated_message = self.bridge.cv2_to_imgmsg(
                    frame, encoding="bgr8"
                )
                annotated_message.header = message.header
                self.image_publisher.publish(annotated_message)

            detection_payload = {
                "stamp": {
                    "sec": message.header.stamp.sec,
                    "nanosec": message.header.stamp.nanosec,
                },
                "frame_id": message.header.frame_id,
                "count": len(detections),
                "detection_enabled": detection_enabled,
                "table_arrived": detection_enabled,
                "confirmed": is_confirmed,
                "consecutive_detection_frames": (
                    consecutive_detection_frames
                ),
                "confirmation_frames": self.confirmation_frames,
                "roi_polygon": [list(point) for point in roi_polygon],
                "roi_intrusion": intrusion_detected,
                "detections": detections,
            }
            with self.detection_state_lock:
                # Recheck at the publication boundary. The callback uses this
                # same lock, guaranteeing that its disabled clear is never
                # followed by a stale positive result from this frame.
                if not self.detection_enabled:
                    detection_payload.update(
                        {
                            "count": 0,
                            "detection_enabled": False,
                            "table_arrived": False,
                            "confirmed": False,
                            "consecutive_detection_frames": 0,
                            "roi_intrusion": False,
                            "detections": [],
                        }
                    )
                    intrusion_detected = False
                detection_message = String()
                detection_message.data = json.dumps(
                    detection_payload, ensure_ascii=False
                )
                self.detections_publisher.publish(detection_message)
                self.roi_intrusion_publisher.publish(
                    Bool(data=intrusion_detected)
                )

            if self.show_window:
                cv2.imshow("ROS 2 Hand Detection", frame)
                cv2.waitKey(1)
        except CvBridgeError as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
        except Exception as exc:
            if not rclpy.ok():
                return
            self.get_logger().error(
                f"Inference failed: {type(exc).__name__}: {exc}\n"
                f"{traceback.format_exc()}"
            )

    def warn_throttled(
        self, key: str, message: str, period: float = 5.0
    ) -> None:
        now = time.monotonic()
        last_time = self.warning_times.get(key)
        if last_time is None or now - last_time >= period:
            self.warning_times[key] = now
            self.get_logger().warning(message)

    def detect_hands(self, frame: Any) -> list[dict[str, Any]]:
        height, width = frame.shape[:2]
        if self.tiled_inference:
            overlap_x = int(width * self.tile_overlap)
            overlap_y = int(height * self.tile_overlap)
            middle_x = width // 2
            middle_y = height // 2
            tile_regions = [
                (0, 0, middle_x + overlap_x, middle_y + overlap_y),
                (
                    middle_x - overlap_x,
                    0,
                    width,
                    middle_y + overlap_y,
                ),
                (
                    0,
                    middle_y - overlap_y,
                    middle_x + overlap_x,
                    height,
                ),
                (
                    middle_x - overlap_x,
                    middle_y - overlap_y,
                    width,
                    height,
                ),
            ]
        elif self.roi_inference:
            roi_points = np.asarray(
                get_roi_polygon(width, height), dtype=np.int32
            )
            roi_x, roi_y, roi_width, roi_height = cv2.boundingRect(
                roi_points
            )
            padding_x = int(width * self.roi_inference_padding)
            padding_y = int(height * self.roi_inference_padding)
            tile_regions = [
                (
                    max(0, roi_x - padding_x),
                    max(0, roi_y - padding_y),
                    min(width, roi_x + roi_width + padding_x),
                    min(height, roi_y + roi_height + padding_y),
                )
            ]
        else:
            tile_regions = [(0, 0, width, height)]

        tiles = [
            frame[y1:y2, x1:x2]
            for x1, y1, x2, y2 in tile_regions
        ]
        results = self.model.predict(
            source=tiles,
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.image_size,
            device=self.device,
            agnostic_nms=True,
            verbose=False,
        )

        global_boxes: list[list[int]] = []
        scores: list[float] = []
        for result, (offset_x, offset_y, _, _) in zip(
            results, tile_regions
        ):
            if result.boxes is None:
                continue
            boxes = result.boxes.xyxy.detach().cpu().numpy()
            confidences = result.boxes.conf.detach().cpu().numpy()
            for box, confidence in zip(boxes, confidences):
                x1, y1, x2, y2 = (int(value) for value in box)
                global_boxes.append(
                    [
                        x1 + offset_x,
                        y1 + offset_y,
                        x2 - x1,
                        y2 - y1,
                    ]
                )
                scores.append(float(confidence))

        if not global_boxes:
            return []

        kept_indices = cv2.dnn.NMSBoxes(
            global_boxes,
            scores,
            self.confidence,
            self.iou,
        )
        detections: list[dict[str, Any]] = []
        for index in kept_indices:
            x, y, box_width, box_height = global_boxes[int(index)]
            detections.append(
                {
                    "class_id": 0,
                    "class_name": "hand",
                    "confidence": scores[int(index)],
                    "bbox_xyxy": [
                        x,
                        y,
                        x + box_width,
                        y + box_height,
                    ],
                }
            )
        return detections

    def update_fps(self) -> float:
        current_time = time.perf_counter()
        if self.previous_time is None:
            self.previous_time = current_time
            return 0.0

        elapsed = current_time - self.previous_time
        self.previous_time = current_time
        instantaneous = 1.0 / elapsed if elapsed > 0.0 else 0.0
        self.smoothed_fps = (
            instantaneous
            if self.smoothed_fps == 0.0
            else 0.9 * self.smoothed_fps + 0.1 * instantaneous
        )
        return self.smoothed_fps

    @staticmethod
    def draw_detection(
        frame: Any,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        name: str,
        confidence: float,
    ) -> None:
        label = f"{name} {confidence:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame,
            label,
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def draw_roi(
        frame: Any,
        roi_polygon: list[tuple[int, int]],
        intrusion_detected: bool,
        detection_enabled: bool = True,
    ) -> None:
        if not detection_enabled:
            color = (128, 128, 128)
            label = "ROI: DISABLED - ROBOT NOT ARRIVED"
        else:
            color = (0, 0, 255) if intrusion_detected else (255, 255, 0)
            label = "ROI: INTRUSION" if intrusion_detected else "ROI: SAFE"
        points = np.asarray(roi_polygon, dtype=np.int32)
        cv2.polylines(frame, [points], True, color, 3, cv2.LINE_AA)
        label_x, label_y = roi_polygon[0]
        cv2.putText(
            frame,
            label,
            (label_x, max(55, label_y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA,
        )

    def destroy_node(self) -> None:
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: HandDetectorNode | None = None
    executor: SingleThreadedExecutor | None = None
    try:
        node = HandDetectorNode()
        # Keep CUDA inference on the main thread. Ultralytics/PyTorch can stall
        # when its predictor is driven from a ROS worker thread while another
        # worker is copying large camera messages.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # SIGTERM/launch shutdown may invalidate the ROS context while the
        # executor is rebuilding its wait set. That is a normal shutdown, not
        # a detector crash.
        if rclpy.ok():
            if node is not None:
                node.get_logger().fatal(traceback.format_exc())
            else:
                print(traceback.format_exc())
            raise
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
