"""Nav2 aisle approach followed by deterministic table docking.

Nav2 approaches the table's Y coordinate on the clear centreline.  The node
then takes over ``cmd_vel_nav`` within a relaxed position tolerance: finish
the straight table-Y alignment,
rotate in place using ground-truth odom, and drive forward to the final pose.
Publishing to
``cmd_vel_nav`` intentionally keeps Nav2's velocity smoother in the command
chain while avoiding RPP's translation-based progress checker during pivots.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32

from serving_robot_interfaces.srv import TaskCommand


NAV_CMD_KITCHEN = 4
NAV_STATUS_MOVING = 1
NAV_STATUS_ARRIVED = 2
NAV_STATUS_FAILED = 3

# Manager table IDs retain the established restaurant mapping.
TABLE_DOCKS = {
    1: (1.82, -2.20, 0.0),
    2: (-1.82, 0.70, math.pi),
    3: (1.82, 0.70, 0.0),
}
KITCHEN_POSE = (0.00, 5.25, -math.pi / 2.0)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _yaw_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class HybridNavServer(Node):
    PRE_DOCK_X = 0.0
    # Nav2 receives the centreline point at the exact table Y.  Its relaxed
    # position tolerance hands control over before RPP can overshoot and circle
    # back; the odom controller then removes the remaining position error.
    NAV2_APPROACH_CLEARANCE = 0.0
    ROTATE_KP = 1.8
    ROTATE_MAX = 0.65
    ROTATE_MIN = 0.18
    ROTATE_TOLERANCE = math.radians(3.0)
    ROTATE_TIMEOUT = 20.0
    DOCK_MAX_SPEED = 0.12
    DOCK_MIN_SPEED = 0.035
    DOCK_YAW_KP = 1.8
    DOCK_LATERAL_KP = 1.2
    DOCK_MAX_ANGULAR = 0.30
    DOCK_POSITION_TOLERANCE = 0.05
    DOCK_TIMEOUT = 30.0

    def __init__(self):
        super().__init__("hybrid_nav_server_node")
        self._action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # navigation_launch remaps controller output to cmd_vel_nav and the
        # velocity smoother maps its output to the Isaac-facing /cmd_vel.
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._status_pub = self.create_publisher(
            Int32, "/navigation/status", 10
        )
        self._location_pub = self.create_publisher(
            Int32, "/navigation/current_location", 10
        )
        self.create_subscription(
            Odometry, "/nav_robot/odom", self._on_odom, 20
        )
        self.create_service(
            TaskCommand, "/navigation/command", self._on_command
        )
        self._odom_lock = threading.Lock()
        self._latest_odom = None
        self._mission_lock = threading.Lock()
        self._busy = False
        self.get_logger().info(
            "Hybrid navigation ready: Nav2 pre-dock -> odom pivot -> "
            "straight forward dock"
        )

    def _on_odom(self, msg: Odometry):
        with self._odom_lock:
            self._latest_odom = msg

    def _pose(self):
        with self._odom_lock:
            msg = self._latest_odom
        if msg is None:
            return None
        return (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _yaw_from_odom(msg),
        )

    def _publish_cmd(self, vx: float = 0.0, wz: float = 0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _stop(self):
        # Send several zeros so the 20 Hz smoother cannot retain an old sample.
        for _ in range(3):
            self._publish_cmd()
            time.sleep(0.03)

    def _on_command(self, request, response):
        target = int(request.command)
        if target not in (*TABLE_DOCKS.keys(), NAV_CMD_KITCHEN):
            response.success = False
            return response
        with self._mission_lock:
            if self._busy:
                self.get_logger().warning("navigation mission already active")
                response.success = False
                return response
            self._busy = True
        response.success = True
        threading.Thread(
            target=self._run_mission, args=(target,), daemon=True
        ).start()
        return response

    def _wait_future(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    def _nav2_goal(self, x: float, y: float, yaw: float) -> bool:
        if not self._action.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action unavailable")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = _yaw_quaternion(yaw)
        self.get_logger().info(
            f"Nav2 pre-dock goal=({x:.2f},{y:.2f}) travel_yaw="
            f"{math.degrees(yaw):.1f}deg"
        )
        handle = self._wait_future(
            self._action.send_goal_async(goal), timeout=10.0
        )
        if handle is None or not handle.accepted:
            self.get_logger().error("Nav2 pre-dock goal rejected")
            return False
        result = self._wait_future(handle.get_result_async(), timeout=180.0)
        if result is None or int(result.status) != 4:  # STATUS_SUCCEEDED
            status = None if result is None else int(result.status)
            self.get_logger().error(f"Nav2 pre-dock failed status={status}")
            return False
        self._stop()
        return True

    def _rotate_to(self, target_yaw: float) -> bool:
        start = time.monotonic()
        next_log = start
        while rclpy.ok():
            pose = self._pose()
            if pose is None:
                time.sleep(0.05)
                continue
            error = _wrap(target_yaw - pose[2])
            if abs(error) <= self.ROTATE_TOLERANCE:
                self._stop()
                self.get_logger().info(
                    f"pivot complete yaw={math.degrees(pose[2]):.1f}deg"
                )
                return True
            if time.monotonic() - start > self.ROTATE_TIMEOUT:
                self._stop()
                self.get_logger().error(
                    f"pivot timeout yaw={math.degrees(pose[2]):.1f}deg "
                    f"error={math.degrees(error):.1f}deg"
                )
                return False
            wz = max(
                self.ROTATE_MIN,
                min(self.ROTATE_MAX, self.ROTATE_KP * abs(error)),
            )
            wz = math.copysign(wz, error)
            self._publish_cmd(0.0, wz)
            if time.monotonic() >= next_log:
                self.get_logger().info(
                    f"pivot yaw={math.degrees(pose[2]):.1f}deg "
                    f"target={math.degrees(target_yaw):.1f}deg "
                    f"cmd_wz={wz:.2f}"
                )
                next_log = time.monotonic() + 1.0
            time.sleep(0.05)
        return False

    def _drive_straight_to(
        self,
        goal_x: float,
        goal_y: float,
        yaw: float,
        phase_name: str,
    ) -> bool:
        start = time.monotonic()
        next_log = start
        while rclpy.ok():
            pose = self._pose()
            if pose is None:
                time.sleep(0.05)
                continue
            dx, dy = goal_x - pose[0], goal_y - pose[1]
            forward = math.cos(yaw) * dx + math.sin(yaw) * dy
            lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
            distance = math.hypot(dx, dy)
            yaw_error = _wrap(yaw - pose[2])
            if distance <= self.DOCK_POSITION_TOLERANCE:
                self._stop()
                self.get_logger().info(
                    f"{phase_name} complete pose=({pose[0]:.3f},"
                    f"{pose[1]:.3f},"
                    f"{math.degrees(pose[2]):.1f}deg)"
                )
                return True
            if time.monotonic() - start > self.DOCK_TIMEOUT:
                self._stop()
                self.get_logger().error(
                    f"{phase_name} timeout distance={distance:.3f}m"
                )
                return False
            # Never reverse or chase a goal already passed; stop safely.
            if forward <= -self.DOCK_POSITION_TOLERANCE:
                self._stop()
                self.get_logger().error(
                    f"{phase_name} target passed "
                    f"forward_error={forward:.3f}m"
                )
                return False
            vx = min(
                self.DOCK_MAX_SPEED,
                max(self.DOCK_MIN_SPEED, 0.5 * max(0.0, forward)),
            )
            wz = self.DOCK_YAW_KP * yaw_error - self.DOCK_LATERAL_KP * lateral
            wz = max(-self.DOCK_MAX_ANGULAR, min(self.DOCK_MAX_ANGULAR, wz))
            self._publish_cmd(vx, wz)
            if time.monotonic() >= next_log:
                self.get_logger().info(
                    f"{phase_name} forward={forward:.3f}m "
                    f"lateral={lateral:.3f}m "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg "
                    f"cmd=({vx:.2f},{wz:.2f})"
                )
                next_log = time.monotonic() + 1.0
            time.sleep(0.05)
        return False

    def _run_mission(self, target: int):
        self._status_pub.publish(Int32(data=NAV_STATUS_MOVING))
        success = False
        try:
            if target == NAV_CMD_KITCHEN:
                success = self._nav2_goal(*KITCHEN_POSE)
            else:
                goal_x, goal_y, goal_yaw = TABLE_DOCKS[target]
                nav2_y = goal_y + self.NAV2_APPROACH_CLEARANCE
                # Nav2 follows the centreline toward the exact table Y.  Its
                # relaxed position tolerance hands off before an overshoot;
                # odom control finishes Y alignment, then pivots in place.
                if self._nav2_goal(
                    self.PRE_DOCK_X, nav2_y, -math.pi / 2.0
                ):
                    self.get_logger().info(
                        "safe approach position reached; aligning aisle yaw "
                        "with direct pivot"
                    )
                    success = self._rotate_to(-math.pi / 2.0)
                    if success:
                        self.get_logger().info(
                            "aisle yaw aligned; aligning table Y by straight "
                            "centre-aisle motion"
                        )
                        success = self._drive_straight_to(
                            self.PRE_DOCK_X,
                            goal_y,
                            -math.pi / 2.0,
                            "aisle alignment",
                        )
                    if success:
                        self.get_logger().info(
                            "table Y aligned; starting direct pivot"
                        )
                        success = self._rotate_to(goal_yaw)
                    if success:
                        self.get_logger().info(
                            "pivot reached; starting straight final approach"
                        )
                        success = self._drive_straight_to(
                            goal_x, goal_y, goal_yaw, "dock"
                        )
            if success:
                self._location_pub.publish(Int32(data=target))
                self._status_pub.publish(Int32(data=NAV_STATUS_ARRIVED))
            else:
                self._status_pub.publish(Int32(data=NAV_STATUS_FAILED))
        except Exception as exc:
            self.get_logger().exception(f"hybrid navigation failed: {exc}")
            self._status_pub.publish(Int32(data=NAV_STATUS_FAILED))
        finally:
            self._stop()
            with self._mission_lock:
                self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = HybridNavServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
