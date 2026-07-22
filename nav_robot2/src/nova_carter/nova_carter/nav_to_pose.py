import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rosgraph_msgs.msg import Clock
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from std_srvs.srv import Empty

AMCL_POSE_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

# Nav2 goal checker (nav2_params) + AMCL slack for mission success printout.
DOCK_XY_TOL_M = 0.15
TELEPORT_TOPIC = "/nova_carter/teleport"

# Isaac Sim /clock publisher is RELIABLE; must match or no messages are delivered.
CLOCK_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = (
        math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    )
    qy = (
        math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    )
    qz = (
        math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    )
    qw = (
        math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    )
    return [qx, qy, qz, qw]


def get_euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def dist_xy(ax, ay, bx, by) -> float:
    return math.hypot(ax - bx, ay - by)


def print_final_pose(pose_msg):
    if not pose_msg:
        return
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    roll_rad, pitch_rad, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    yaw_deg = math.degrees(yaw_rad)
    print("-" * 50)
    print(f"최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"최종 Yaw: {yaw_rad:.3f} rad ({yaw_deg:.1f}°)")
    print("-" * 50)


def create_pose(navigator, x, y, yaw_rad, frame_id="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = get_quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def load_waypoints():
    share = Path(get_package_share_directory("nova_carter"))
    path = share / "config" / "waypoints.yaml"
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data["destinations"]


def table_ids(destinations):
    return sorted(
        int(k) for k in destinations if isinstance(k, int) and 0 <= int(k) < 4
    )


def lookup_map_xy(tf_buffer: Buffer, nav: BasicNavigator | None = None) -> tuple[float, float] | None:
    stamps = []
    if nav is not None:
        now = nav.get_clock().now()
        stamps.append(now)
        stamps.append(now - Duration(nanoseconds=200_000_000))
    stamps.append(Time())

    for stamp in stamps:
        try:
            tf = tf_buffer.lookup_transform(
                "map",
                "base_link",
                stamp,
                timeout=Duration(seconds=0.5),
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
            )
        except Exception:
            continue
    return None


class AmclPoseTracker:
    """Latched /amcl_pose when map→base_link TF is briefly unavailable."""

    def __init__(self, nav: BasicNavigator) -> None:
        self._xy: tuple[float, float] | None = None
        self._sub = nav.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl,
            AMCL_POSE_QOS,
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self._xy = (float(p.x), float(p.y))

    @property
    def xy(self) -> tuple[float, float] | None:
        return self._xy


def resolve_map_xy(
    nav: BasicNavigator,
    tf_buffer: Buffer,
    tracker: AmclPoseTracker | None,
) -> tuple[float, float] | None:
    xy = lookup_map_xy(tf_buffer, nav)
    if xy is not None:
        return xy
    if tracker is not None:
        return tracker.xy
    return None


def wait_for_clock(nav, timeout_sec: float = 60.0) -> bool:
    """Wait for first /clock before enabling use_sim_time (chicken-and-egg)."""
    received = {"ok": False}

    def _cb(_msg: Clock) -> None:
        received["ok"] = True

    sub = nav.create_subscription(Clock, "/clock", _cb, CLOCK_QOS)
    deadline = time.monotonic() + timeout_sec
    last_log = time.monotonic()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.1)
            if received["ok"]:
                return True
            if time.monotonic() - last_log > 5.0:
                print(
                    "[nav] /clock 수신 대기 중… (T1 Play, ROS_DOMAIN_ID=103)",
                    flush=True,
                )
                last_log = time.monotonic()
        return False
    finally:
        nav.destroy_subscription(sub)


def get_map_xy(
    nav,
    tf_buffer: Buffer,
    tracker: AmclPoseTracker | None = None,
    timeout_sec: float = 30.0,
) -> tuple[float, float]:
    """Current pose from TF, falling back to latched /amcl_pose."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.05)
        xy = resolve_map_xy(nav, tf_buffer, tracker)
        if xy is not None:
            return xy
    raise RuntimeError("map→base_link TF 없음 (AMCL/Isaac /clock 확인)")


def publish_initial_pose(nav, x: float, y: float, yaw: float, repeats: int = 8) -> None:
    pose = create_pose(nav, x, y, yaw)
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
        time.sleep(0.1)


def teleport_to_spawn(nav, spawn: dict, settle_sec: float = 1.2) -> None:
    """Isaac Sim + topic_bridge odom rebase (map coords = world coords)."""
    sx, sy, yaw = float(spawn["x"]), float(spawn["y"]), float(spawn["yaw"])
    z = float(spawn.get("z", 0.0))
    pub = nav.create_publisher(PoseStamped, TELEPORT_TOPIC, 10)
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose = create_pose(nav, sx, sy, yaw).pose
    msg.pose.position.z = z
    for _ in range(5):
        msg.header.stamp = nav.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(nav, timeout_sec=0.05)
        time.sleep(0.05)
    nav.destroy_publisher(pub)
    deadline = time.monotonic() + settle_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.05)
    print(f"[sync] Isaac 텔레포트 → ({sx:.2f}, {sy:.2f})")


def reinitialize_amcl_particles(nav) -> None:
    client = nav.create_client(Empty, "/reinitialize_global_localization")
    if not client.wait_for_service(timeout_sec=2.0):
        return
    future = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(nav, future, timeout_sec=5.0)


def wait_for_amcl_pose(
    nav, tracker: AmclPoseTracker, timeout_sec: float = 20.0
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.1)
        if tracker.xy is not None:
            return True
    return False


def relocalize_at_spawn(
    nav,
    tf_buffer: Buffer,
    tracker: AmclPoseTracker,
    spawn: dict,
    timeout_sec: float = 45.0,
) -> bool:
    """Isaac 주방 텔레포트 후 AMCL/RViz를 스폰 위치에 맞춤."""
    sx, sy, yaw = float(spawn["x"]), float(spawn["y"]), float(spawn["yaw"])
    print(f"[sync] AMCL 초기화 → 주방 ({sx:.2f}, {sy:.2f})")
    teleport_to_spawn(nav, spawn)
    nav.clearAllCostmaps()
    reinitialize_amcl_particles(nav)

    deadline = time.monotonic() + timeout_sec
    last_repub = 0.0
    while time.monotonic() < deadline:
        if time.monotonic() - last_repub > 2.0:
            publish_initial_pose(nav, sx, sy, yaw, repeats=3)
            last_repub = time.monotonic()
        rclpy.spin_once(nav, timeout_sec=0.1)
        xy = resolve_map_xy(nav, tf_buffer, tracker)
        if xy is None:
            continue
        d = dist_xy(xy[0], xy[1], sx, sy)
        if d < 0.65:
            print(f"[sync] 위치 OK: ({xy[0]:.2f}, {xy[1]:.2f})")
            time.sleep(0.5)
            return True
        if time.monotonic() - last_repub > 1.5:
            print(f"[sync] 위치 오차 {d:.2f} m — initialpose 재전송 중...")

    xy = resolve_map_xy(nav, tf_buffer, tracker)
    if xy is not None:
        print(
            f"[sync] TF는 있으나 스폰과 거리 {dist_xy(xy[0], xy[1], sx, sy):.2f} m — 주행 시도",
            file=sys.stderr,
        )
        return True

    print(
        "[sync] 실패: map→base_link TF 없음.\n"
        "  · T1 Isaac **Play** 중인지, 로그 domain=103 인지\n"
        "  · T2 Nav2 실행 중인지\n"
        "  · ros2 topic hz /clock /scan",
        file=sys.stderr,
    )
    return False


def run_go_to_pose(
    nav,
    tf_buffer: Buffer,
    tracker: AmclPoseTracker,
    goal_x,
    goal_y,
    goal_yaw,
    *,
    label: str = "goal",
    min_travel_m: float = 1.5,
    dock_tol_m: float = DOCK_XY_TOL_M,
    require_travel: bool = True,
):
    sx, sy = get_map_xy(nav, tf_buffer, tracker)
    start_dist = dist_xy(sx, sy, goal_x, goal_y)
    print(f"[{label}] 출발 ({sx:.2f}, {sy:.2f}) → 거리 {start_dist:.2f} m")

    if start_dist < 0.35:
        print(f"[{label}] 이미 도킹 반경 안입니다.", file=sys.stderr)
        return True

    nav.cancelTask()
    nav.result_future = None
    nav.feedback = None
    nav.status = None
    time.sleep(0.3)

    goal_pose = create_pose(nav, goal_x, goal_y, goal_yaw)
    if not nav.goToPose(goal_pose):
        print(f"[{label}] goToPose 거부됨", file=sys.stderr)
        return False
    if nav.result_future is None:
        print(f"[{label}] result_future 없음 (이전 goal 잔여?)", file=sys.stderr)
        return False

    last_pose = None
    max_dist_seen = 0.0
    saw_feedback = False
    nav_start = time.monotonic()

    while not nav.isTaskComplete():
        if time.monotonic() - nav_start > 8.0 and not saw_feedback:
            print(
                f"[{label}] Nav2 피드백 없음 — Isaac Play·t2·텔레포트 확인",
                file=sys.stderr,
            )
            nav.cancelTask()
            return False
        feedback = nav.getFeedback()
        if feedback:
            saw_feedback = True
            last_pose = feedback.current_pose
            rem = float(feedback.distance_remaining)
            max_dist_seen = max(max_dist_seen, rem)
            print(f"[{label}] 남은 거리: {rem:.2f} m")
        rclpy.spin_once(nav, timeout_sec=0.05)
        time.sleep(0.2)

    if nav.getResult() != TaskResult.SUCCEEDED:
        print(f"[{label}] 주행 실패", file=sys.stderr)
        return False

    ex, ey = get_map_xy(nav, tf_buffer, tracker, timeout_sec=5.0)
    end_err = dist_xy(ex, ey, goal_x, goal_y)
    traveled = dist_xy(sx, sy, ex, ey)
    print(
        f"[{label}] 종료 ({ex:.2f}, {ey:.2f}) "
        f"도킹오차 {end_err:.2f} m, 이동 {traveled:.2f} m"
    )

    if not saw_feedback and max_dist_seen < 0.5:
        print(f"[{label}] 피드백 없이 완료 (가짜 성공 의심)", file=sys.stderr)
        return False

    if require_travel and traveled < min_travel_m and start_dist > min_travel_m + 0.5:
        print(f"[{label}] 거의 움직이지 않음", file=sys.stderr)
        return False

    if end_err > dock_tol_m:
        print(
            f"[{label}] 도킹 오차 {end_err:.2f} m > {dock_tol_m:.2f} m",
            file=sys.stderr,
        )
        return False

    print(f"[{label}] 도킹 완료 (테이블 옆 밀착 목표)")
    print_final_pose(last_pose)
    return True


def get_dest(destinations, key):
    """YAML keys may be int (0,4) or str ('spawn')."""
    if key in destinations:
        return destinations[key]
    if isinstance(key, str) and key.isdigit():
        alt = int(key)
        if alt in destinations:
            return destinations[alt]
    if isinstance(key, int):
        alt = str(key)
        if alt in destinations:
            return destinations[alt]
    raise KeyError(key)


def dest_dict(destinations, key):
    if key == "kitchen":
        for k in ("kitchen", 4, "4"):
            if k in destinations:
                return destinations[k]
        raise KeyError("kitchen")
    return get_dest(destinations, key)


def main():
    parser = argparse.ArgumentParser(
        description="주방 스폰 → 테이블 밀착 도킹 → (선택) 주방 복귀"
    )
    parser.add_argument("--table-id", type=int, default=0, choices=[0, 1, 2, 3])
    parser.add_argument(
        "--visit-all",
        action="store_true",
        help="table 0..3 순차 도킹 후 주방 복귀",
    )
    parser.add_argument(
        "--return-kitchen",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="단일 테이블 미션 후 주방 복귀 (기본: 켜짐)",
    )
    parser.add_argument(
        "--no-sync-spawn",
        action="store_true",
        help="미션 시작 시 AMCL 주방 재동기화 생략 (연속 주행 시)",
    )
    args = parser.parse_args()

    destinations = load_waypoints()
    spawn = destinations.get("spawn") or get_dest(destinations, 4)
    kitchen = dest_dict(destinations, "kitchen")

    rclpy.init(args=sys.argv)
    nav = BasicNavigator()

    print("[nav] /clock 대기 (Isaac Play 필요)...")
    if not wait_for_clock(nav, timeout_sec=90.0):
        print(
            "[nav] /clock 없음 — T1에서 Play 누른 뒤 domain=103인지 확인하세요.",
            file=sys.stderr,
        )
        rclpy.shutdown()
        raise SystemExit(1)

    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    print("[nav] sim time 활성화 (/clock OK)")

    tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
    TransformListener(tf_buffer, nav)
    pose_tracker = AmclPoseTracker(nav)

    init_pose = create_pose(nav, spawn["x"], spawn["y"], float(spawn["yaw"]))
    nav.initial_pose = init_pose
    publish_initial_pose(nav, spawn["x"], spawn["y"], float(spawn["yaw"]), repeats=3)
    if wait_for_amcl_pose(nav, pose_tracker, timeout_sec=25.0):
        nav.initial_pose_received = True
    nav.waitUntilNav2Active()

    if not args.no_sync_spawn:
        if not relocalize_at_spawn(nav, tf_buffer, pose_tracker, spawn):
            rclpy.shutdown()
            raise SystemExit(1)

    ok = True

    if args.visit_all:
        for tid in table_ids(destinations):
            dest = get_dest(destinations, tid)
            ok = run_go_to_pose(
                nav,
                tf_buffer,
                pose_tracker,
                dest["x"],
                dest["y"],
                float(dest["yaw"]),
                label=dest["name"],
            ) and ok
        print("\n=== 주방 복귀 ===")
        ok = run_go_to_pose(
            nav,
            tf_buffer,
            pose_tracker,
            kitchen["x"],
            kitchen["y"],
            float(kitchen["yaw"]),
            label=kitchen["name"],
            min_travel_m=2.0,
            require_travel=True,
        ) and ok
    else:
        dest = get_dest(destinations, args.table_id)
        ok = run_go_to_pose(
            nav,
            tf_buffer,
            pose_tracker,
            dest["x"],
            dest["y"],
            float(dest["yaw"]),
            label=dest["name"],
        ) and ok
        if ok and args.return_kitchen:
            print("\n=== 주방 복귀 ===")
            ok = run_go_to_pose(
                nav,
                tf_buffer,
                pose_tracker,
                kitchen["x"],
                kitchen["y"],
                float(kitchen["yaw"]),
                label=kitchen["name"],
                min_travel_m=2.0,
            ) and ok

    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
