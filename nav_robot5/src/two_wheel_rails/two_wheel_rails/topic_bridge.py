#!/usr/bin/env python3
"""Bridge Isaac two-wheel raw topics to Nav2 names, QoS and TF."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# Topic names are relative (no leading slash) so that launching this node
# with namespace=<robot_id> (see nav2.launch.py) auto-prefixes them to
# /<robot_id>/... and matches what the Isaac bridge (restaurant_two_wheel_demo.py)
# publishes for that robot. TF frame_id strings are NOT affected by ROS
# namespace push-down, so they get an explicit robot_id_ prefix instead.
TELEPORT_TOPIC = "two_wheel/teleport"
RAW_ODOM_TOPIC = "two_wheel/odom_raw"
RAW_SCAN_TOPIC = "two_wheel/scan_raw"


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
        self.declare_parameter("robot_id", "robot1")
        robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        # Frame names stay unprefixed: each robot's TF is already isolated by
        # topic (/robot1/tf vs /robot2/tf via nav2_bringup's namespace push),
        # and some nav2-internal calls (e.g. nav2_util::getCurrentPose) fall
        # back to a hardcoded "base_link"-style default rather than reading
        # robot_base_frame, so matching the plain frame name avoids TF lookup
        # failures in bt_navigator regardless of which robot this is.
        self.base_link = "base_link"
        self.lidar_frame = "nav_lidar_link"

        # tf2_ros.TransformBroadcaster hardcodes the absolute "/tf" topic,
        # which would bypass this node's namespace and land on the global
        # /tf instead of /<robot_id>/tf (what nav2_bringup's -r /tf:=tf
        # remap + PushRosNamespace actually listen on). Publish TFMessage
        # on a relative "tf" topic ourselves so namespace push-down applies.
        self._tf_pub = self.create_publisher(TFMessage, "tf", QoSProfile(depth=100))

        self._odom_pub = self.create_publisher(Odometry, "odom", _sensor_data(20))
        self.create_subscription(
            Odometry, RAW_ODOM_TOPIC, self._on_odom, _reliable(20)
        )

        self._scan_pub = self.create_publisher(LaserScan, "scan", _reliable(5))
        self.create_subscription(
            LaserScan, RAW_SCAN_TOPIC, self._on_scan, _sensor_data(5)
        )
        self.create_subscription(
            PoseStamped, TELEPORT_TOPIC, self._on_teleport, _reliable(5)
        )

        self.get_logger().info(
            f"bridge[{robot_id}]: {RAW_ODOM_TOPIC} -> odom + TF odom->{self.base_link}, "
            f"{RAW_SCAN_TOPIC} -> scan, absolute Isaac pose preserved"
        )

    def _on_teleport(self, _msg: PoseStamped) -> None:
        # The next raw odometry sample already contains the teleported Isaac
        # world pose.  Never turn that pose into a new (0, 0, 0) odom origin.
        self.get_logger().info("teleport: preserving absolute Isaac odometry")

    def _absolute_odom(self, msg: Odometry) -> Odometry:
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "odom"
        out.child_frame_id = self.base_link
        out.pose = msg.pose
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        return out

    def _on_odom(self, msg: Odometry) -> None:
        out = self._absolute_odom(msg)
        self._odom_pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = self.base_link
        tf.transform.translation.x = out.pose.pose.position.x
        tf.transform.translation.y = out.pose.pose.position.y
        tf.transform.translation.z = out.pose.pose.position.z
        tf.transform.rotation = out.pose.pose.orientation
        self._tf_pub.publish(TFMessage(transforms=[tf]))

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.frame_id = self.lidar_frame
        self._scan_pub.publish(msg)


def main() -> None:
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
