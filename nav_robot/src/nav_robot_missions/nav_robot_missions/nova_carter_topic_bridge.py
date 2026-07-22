#!/usr/bin/env python3
"""Bridge Nova_Carter_ROS.usd topics to Nav2-friendly names/QoS/TF.

Uses the same base frame as the two-wheel stack (ridgeback_base_link + base_scan)
so AMCL/costmaps/RViz match nav2_params.yaml / nav2_restaurant.rviz.
"""

from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# Match nav_restaurant_demo.py sensor mounts (two-wheel).
BASE_LINK = "ridgeback_base_link"
LIDAR_FRAME = "base_scan"
_LIDAR_XYZ = (0.40, 0.0, 0.33)


def _reliable(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def _sensor_data(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class NovaCarterTopicBridge(Node):
    def __init__(self) -> None:
        super().__init__("nova_carter_topic_bridge")

        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_sensor_static_tf()

        self._odom_pub = self.create_publisher(Odometry, "/odom", _sensor_data(20))
        self.create_subscription(
            Odometry, "/chassis/odom", self._on_odom, _reliable(20)
        )

        self._scan_pub = self.create_publisher(LaserScan, "/scan", _sensor_data(5))
        self.create_subscription(
            LaserScan, "/front_2d_lidar/scan", self._on_scan, _reliable(5)
        )

        self.get_logger().info(
            f"bridging odom+TF -> {BASE_LINK}, scan -> /scan frame={LIDAR_FRAME}"
        )

    def _publish_sensor_static_tf(self) -> None:
        stamp = self.get_clock().now().to_msg()
        lidar = TransformStamped()
        lidar.header.stamp = stamp
        lidar.header.frame_id = BASE_LINK
        lidar.child_frame_id = LIDAR_FRAME
        lidar.transform.translation.x = _LIDAR_XYZ[0]
        lidar.transform.translation.y = _LIDAR_XYZ[1]
        lidar.transform.translation.z = _LIDAR_XYZ[2]
        lidar.transform.rotation.w = 1.0
        # Isaac Nova USD uses base_link; alias for debugging only.
        alias = TransformStamped()
        alias.header.stamp = stamp
        alias.header.frame_id = BASE_LINK
        alias.child_frame_id = "base_link"
        alias.transform.rotation.w = 1.0
        self._static_tf.sendTransform([lidar, alias])

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = "odom"
        out.child_frame_id = BASE_LINK
        out.pose = msg.pose
        out.twist = msg.twist
        self._odom_pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = BASE_LINK
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(tf)

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.frame_id = LIDAR_FRAME
        self._scan_pub.publish(msg)


def main(argv=None) -> int:
    os.environ.setdefault("ROS_DOMAIN_ID", "103")
    rclpy.init(args=None)
    node = NovaCarterTopicBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
