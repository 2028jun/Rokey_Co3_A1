#!/usr/bin/env python3
"""Bridge Isaac two-wheel raw topics to Nav2 names, QoS and TF."""

from __future__ import annotations

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

BASE_LINK = "ridgeback_base_link"
LIDAR_FRAME = "nav_lidar_link"
TELEPORT_TOPIC = "/two_wheel/teleport"
RAW_ODOM_TOPIC = "/two_wheel/odom_raw"
RAW_SCAN_TOPIC = "/two_wheel/scan_raw"


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _rotate2d(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


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


class TopicBridge(Node):
    def __init__(self) -> None:
        super().__init__("two_wheel_topic_bridge")
        self._tf = TransformBroadcaster(self)
        # Rebase the absolute Isaac pose at startup and after each teleport.
        self._odom_origin: tuple[float, float, float] | None = None
        self._rebase_on_next_odom = False

        self._odom_pub = self.create_publisher(Odometry, "/odom", _sensor_data(20))
        self.create_subscription(
            Odometry, RAW_ODOM_TOPIC, self._on_odom, _reliable(20)
        )

        self._scan_pub = self.create_publisher(LaserScan, "/scan", _reliable(5))
        self.create_subscription(
            LaserScan, RAW_SCAN_TOPIC, self._on_scan, _sensor_data(5)
        )
        self.create_subscription(
            PoseStamped, TELEPORT_TOPIC, self._on_teleport, _reliable(5)
        )

        self.get_logger().info(
            f"bridge: {RAW_ODOM_TOPIC} -> /odom + TF odom->{BASE_LINK}, "
            f"{RAW_SCAN_TOPIC} -> /scan, teleport rebases odom"
        )

    def _on_teleport(self, _msg: PoseStamped) -> None:
        self._rebase_on_next_odom = True
        self.get_logger().info("teleport: will rebase /odom on next chassis/odom")

    def _rebase_odom(self, msg: Odometry) -> Odometry:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        raw_yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
        if self._rebase_on_next_odom or self._odom_origin is None:
            self._odom_origin = (float(p.x), float(p.y), raw_yaw)
            self._rebase_on_next_odom = False

        ox, oy, oyaw = self._odom_origin
        dx = float(p.x) - ox
        dy = float(p.y) - oy
        lx, ly = _rotate2d(dx, dy, -oyaw)
        rel_yaw = raw_yaw - oyaw

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "odom"
        out.child_frame_id = BASE_LINK
        out.pose.pose.position.x = lx
        out.pose.pose.position.y = ly
        out.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _quat_from_yaw(rel_yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        return out

    def _on_odom(self, msg: Odometry) -> None:
        out = self._rebase_odom(msg)
        self._odom_pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = BASE_LINK
        tf.transform.translation.x = out.pose.pose.position.x
        tf.transform.translation.y = out.pose.pose.position.y
        tf.transform.translation.z = out.pose.pose.position.z
        tf.transform.rotation = out.pose.pose.orientation
        self._tf.sendTransform(tf)

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.frame_id = LIDAR_FRAME
        self._scan_pub.publish(msg)


def main() -> None:
    os.environ.setdefault("ROS_DOMAIN_ID", "102")
    rclpy.init()
    node = TopicBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
