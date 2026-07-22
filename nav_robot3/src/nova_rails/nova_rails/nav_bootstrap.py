"""Isaac /clock, teleport, AMCL sync (shared by rail missions)."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rosgraph_msgs.msg import Clock
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

TELEPORT_TOPIC = "/nova_carter/teleport"

AMCL_POSE_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

CLOCK_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def make_pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = yaw_to_quat(yaw)
    pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
    pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
    return pose


class AmclPoseTracker:
    def __init__(self, nav: BasicNavigator) -> None:
        self._xy: tuple[float, float] | None = None
        self._sub = nav.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, AMCL_POSE_QOS
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self._xy = (float(p.x), float(p.y))

    @property
    def xy(self) -> tuple[float, float] | None:
        return self._xy


def lookup_map_xy(
    tf_buffer: Buffer, nav: BasicNavigator | None = None
) -> tuple[float, float] | None:
    stamps = []
    if nav is not None:
        now = nav.get_clock().now()
        stamps.extend([now, now - Duration(nanoseconds=200_000_000)])
    stamps.append(Time())
    for stamp in stamps:
        try:
            tf = tf_buffer.lookup_transform(
                "map", "base_link", stamp, timeout=Duration(seconds=0.5)
            )
            return float(tf.transform.translation.x), float(tf.transform.translation.y)
        except Exception:
            continue
    return None


def resolve_map_xy(
    nav: BasicNavigator,
    tf_buffer: Buffer,
    tracker: AmclPoseTracker | None,
) -> tuple[float, float] | None:
    xy = lookup_map_xy(tf_buffer, nav)
    return xy if xy is not None else (tracker.xy if tracker else None)


def wait_for_clock(nav: BasicNavigator, timeout_sec: float = 90.0) -> bool:
    received = {"ok": False}

    def _cb(_msg: Clock) -> None:
        received["ok"] = True

    sub = nav.create_subscription(Clock, "/clock", _cb, CLOCK_QOS)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.1)
            if received["ok"]:
                return True
        return False
    finally:
        nav.destroy_subscription(sub)


def publish_initial_pose(
    nav: BasicNavigator, x: float, y: float, yaw: float, repeats: int = 5
) -> None:
    pose = make_pose(nav, x, y, yaw)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose = pose.pose
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    for _ in range(repeats):
        msg.header.stamp = nav.get_clock().now().to_msg()
        nav.initial_pose_pub.publish(msg)
        rclpy.spin_once(nav, timeout_sec=0.05)
        time.sleep(0.08)


def teleport_to_spawn(nav: BasicNavigator, x: float, y: float, yaw: float) -> None:
    pub = nav.create_publisher(PoseStamped, TELEPORT_TOPIC, 10)
    msg = make_pose(nav, x, y, yaw)
    for _ in range(5):
        msg.header.stamp = nav.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(nav, timeout_sec=0.05)
        time.sleep(0.05)
    nav.destroy_publisher(pub)
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.05)


def reinitialize_amcl(nav: BasicNavigator) -> None:
    client = nav.create_client(Empty, "/reinitialize_global_localization")
    if not client.wait_for_service(timeout_sec=2.0):
        return
    future = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(nav, future, timeout_sec=5.0)


def sync_spawn(
    nav: BasicNavigator,
    tf_buffer: Buffer,
    tracker: AmclPoseTracker,
    x: float,
    y: float,
    yaw: float,
) -> bool:
    print(f"[sync] Isaac 텔레포트 + AMCL → ({x:.2f}, {y:.2f})")
    teleport_to_spawn(nav, x, y, yaw)
    nav.clearAllCostmaps()
    reinitialize_amcl(nav)
    publish_initial_pose(nav, x, y, yaw, repeats=4)
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.1)
        xy = resolve_map_xy(nav, tf_buffer, tracker)
        if xy and math.hypot(xy[0] - x, xy[1] - y) < 0.65:
            print(f"[sync] OK ({xy[0]:.2f}, {xy[1]:.2f})")
            return True
    return False


def prepare_navigator(node_name: str = "rail_navigator") -> tuple[
    BasicNavigator, Buffer, AmclPoseTracker
]:
    nav = BasicNavigator(node_name=node_name)
    if not wait_for_clock(nav):
        raise RuntimeError("/clock 없음 — Isaac Play 및 ROS_DOMAIN_ID 확인")
    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
    TransformListener(tf_buffer, nav)
    tracker = AmclPoseTracker(nav)
    return nav, tf_buffer, tracker
