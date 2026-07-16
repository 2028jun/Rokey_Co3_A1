"""PC B용 ROS 2 색상 판별 노드.

/rgb(sensor_msgs/msg/Image)를 구독해 중앙 Pick 영역의 큐브 색상을 판별하고
/cube_color(std_msgs/msg/Int32)로 1=파랑, 2=초록을 발행한다.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


RGB_TOPIC = "/rgb"
COLOR_TOPIC = "/cube_color"

BLUE = 1
GREEN = 2

# 시뮬레이션 카메라용 HSV 범위. 조명에 따라 S/V 하한은 조정할 수 있다.
BLUE_LOWER = np.array([100, 100, 50], dtype=np.uint8)
BLUE_UPPER = np.array([130, 255, 255], dtype=np.uint8)
GREEN_LOWER = np.array([40, 80, 50], dtype=np.uint8)
GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)

MIN_PIXEL_RATIO = 0.01       # ROI의 1% 이상일 때만 색상 후보로 인정
STABLE_FRAME_COUNT = 5       # 연속 5프레임 동일 결과일 때 확정
PUBLISH_INTERVAL_SEC = 0.5   # Isaac의 새 라운드도 받을 수 있도록 반복 발행


class ColorDetector(Node):
    def __init__(self):
        super().__init__("pc_b_color_detector")
        self._bridge = CvBridge()
        self._candidate = None
        self._candidate_count = 0
        self._last_publish_ns = 0

        self._image_sub = self.create_subscription(
            Image,
            RGB_TOPIC,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._color_pub = self.create_publisher(Int32, COLOR_TOPIC, 10)
        self.get_logger().info(f"구독: {RGB_TOPIC}, 발행: {COLOR_TOPIC}")

    def _image_callback(self, msg):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"이미지 변환 실패: {exc}")
            return

        height, width = bgr.shape[:2]

        # 목표 마커나 대기 큐브의 오검출을 줄이기 위해 화면 중앙 60%만 검사한다.
        y0, y1 = int(height * 0.20), int(height * 0.80)
        x0, x1 = int(width * 0.20), int(width * 0.80)
        roi = bgr[y0:y1, x0:x1]
        if roi.size == 0:
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
        green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

        # 작은 노이즈를 제거하고 같은 색 영역을 연결한다.
        kernel = np.ones((5, 5), dtype=np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

        pixel_count = roi.shape[0] * roi.shape[1]
        blue_ratio = cv2.countNonZero(blue_mask) / pixel_count
        green_ratio = cv2.countNonZero(green_mask) / pixel_count

        if blue_ratio >= MIN_PIXEL_RATIO and blue_ratio > green_ratio:
            detected = BLUE
        elif green_ratio >= MIN_PIXEL_RATIO and green_ratio > blue_ratio:
            detected = GREEN
        else:
            self._candidate = None
            self._candidate_count = 0
            return

        # 단일 프레임 노이즈를 피하기 위해 동일 결과가 연속되는지 확인한다.
        if detected == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = detected
            self._candidate_count = 1

        if self._candidate_count < STABLE_FRAME_COUNT:
            return

        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(PUBLISH_INTERVAL_SEC * 1_000_000_000)
        if now_ns - self._last_publish_ns < interval_ns:
            return

        result = Int32()
        result.data = detected
        self._color_pub.publish(result)
        self._last_publish_ns = now_ns

        name = "BLUE" if detected == BLUE else "GREEN"
        self.get_logger().info(
            f"발행: {name} ({detected}), blue={blue_ratio:.3f}, green={green_ratio:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
