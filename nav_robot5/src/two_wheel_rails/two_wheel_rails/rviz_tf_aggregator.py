"""Merge isolated robot TF topics into one collision-free RViz TF tree."""

from __future__ import annotations

import copy

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class RvizTfAggregator(Node):
    def __init__(self) -> None:
        super().__init__("rviz_tf_aggregator")
        self.declare_parameter("robot_names", ["robot1", "robot2"])
        self._robots = list(self.get_parameter("robot_names").value)
        self._dynamic_pub = self.create_publisher(TFMessage, "/tf", 100)
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self._map_pub = self.create_publisher(OccupancyGrid, "/rviz/map", static_qos)
        self._scan_publishers = {}
        self._grid_publishers = {}
        self._polygon_publishers = {}
        for robot in self._robots:
            self.create_subscription(
                OccupancyGrid, f"/{robot}/map", self._relay_map, static_qos
            )
            self.create_subscription(
                TFMessage, f"/{robot}/tf",
                lambda msg, name=robot: self._relay(msg, name, False), 100,
            )
            self.create_subscription(
                TFMessage, f"/{robot}/tf_static",
                lambda msg, name=robot: self._relay(msg, name, True), static_qos,
            )
            self._scan_publishers[robot] = self.create_publisher(
                LaserScan, f"/rviz/{robot}/scan", 10
            )
            self.create_subscription(
                LaserScan, f"/{robot}/scan",
                lambda msg, name=robot: self._relay_scan(msg, name), 10,
            )
            for source, suffix in (
                ("local_costmap/costmap", "local_costmap"),
                ("global_costmap/costmap", "global_costmap"),
            ):
                key = (robot, suffix)
                self._grid_publishers[key] = self.create_publisher(
                    OccupancyGrid, f"/rviz/{robot}/{suffix}", static_qos
                )
                self.create_subscription(
                    OccupancyGrid,
                    f"/{robot}/{source}",
                    lambda msg, name=robot, label=suffix: self._relay_grid(
                        msg, name, label
                    ),
                    static_qos,
                )
            for source, suffix in (
                ("local_costmap/published_footprint", "local_footprint"),
                ("global_costmap/published_footprint", "global_footprint"),
                ("polygon_stop", "polygon_stop"),
                ("polygon_slowdown", "polygon_slowdown"),
            ):
                key = (robot, suffix)
                self._polygon_publishers[key] = self.create_publisher(
                    PolygonStamped, f"/rviz/{robot}/{suffix}", 10
                )
                self.create_subscription(
                    PolygonStamped,
                    f"/{robot}/{source}",
                    lambda msg, name=robot, label=suffix: self._relay_polygon(
                        msg, name, label
                    ),
                    10,
                )

        # Collision Monitor only publishes its visualization while active and
        # receiving data.  Publish the configured physical footprint and
        # safety envelopes independently as well, so RViz always shows the
        # actual boundaries used by both robots.
        self._fixed_polygons = {
            "local_footprint": (
                (0.42, 0.35), (0.42, -0.35), (-0.42, -0.35), (-0.42, 0.35)
            ),
            "polygon_stop": (
                (0.75, 0.60), (0.75, -0.60), (-0.55, -0.60), (-0.55, 0.60)
            ),
            "polygon_slowdown": (
                (1.35, 0.85), (1.35, -0.85), (-0.75, -0.85), (-0.75, 0.85)
            ),
        }
        self.create_timer(0.5, self._publish_fixed_polygons)

        # Isaac publishes absolute world poses and topic_bridge intentionally
        # preserves them as odom poses.  Therefore map and each robot's odom
        # frame are coincident for the combined RViz tree.  Publish these
        # links ourselves so both models are visible before AMCL initializes.
        # Any AMCL map->odom transform is filtered in _relay() to avoid two TF
        # authorities publishing the same combined child frame.
        self._publish_absolute_odom_roots()

    def _relay_map(self, msg: OccupancyGrid) -> None:
        out = copy.deepcopy(msg)
        out.header.frame_id = "map"
        self._map_pub.publish(out)

    def _publish_fixed_polygons(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for robot in self._robots:
            for label, coordinates in self._fixed_polygons.items():
                msg = PolygonStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = f"{robot}/ridgeback_base_link"
                msg.polygon.points = [
                    Point32(x=float(x), y=float(y), z=0.03)
                    for x, y in coordinates
                ]
                self._polygon_publishers[(robot, label)].publish(msg)

    def _publish_absolute_odom_roots(self) -> None:
        transforms = []
        stamp = self.get_clock().now().to_msg()
        for robot in self._robots:
            item = TransformStamped()
            item.header.stamp = stamp
            item.header.frame_id = "map"
            item.child_frame_id = f"{robot}/odom"
            item.transform.rotation.w = 1.0
            transforms.append(item)
        self._static_pub.publish(TFMessage(transforms=transforms))
        self.get_logger().info(
            "combined RViz TF roots ready: "
            + ", ".join(f"map->{robot}/odom" for robot in self._robots)
        )

    @staticmethod
    def _frame(frame: str, robot: str) -> str:
        clean = frame.lstrip("/")
        return "map" if clean == "map" else f"{robot}/{clean}"

    def _relay(self, msg: TFMessage, robot: str, static: bool) -> None:
        out = TFMessage()
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent == "map" and child == "odom":
                continue
            item = copy.deepcopy(transform)
            item.header.frame_id = self._frame(item.header.frame_id, robot)
            item.child_frame_id = self._frame(item.child_frame_id, robot)
            out.transforms.append(item)
        if out.transforms:
            (self._static_pub if static else self._dynamic_pub).publish(out)

    def _relay_scan(self, msg: LaserScan, robot: str) -> None:
        out = copy.deepcopy(msg)
        out.header.frame_id = self._frame(out.header.frame_id, robot)
        self._scan_publishers[robot].publish(out)

    def _relay_grid(self, msg: OccupancyGrid, robot: str, label: str) -> None:
        out = copy.deepcopy(msg)
        out.header.frame_id = self._frame(out.header.frame_id, robot)
        self._grid_publishers[(robot, label)].publish(out)

    def _relay_polygon(
        self, msg: PolygonStamped, robot: str, label: str
    ) -> None:
        out = copy.deepcopy(msg)
        out.header.frame_id = self._frame(out.header.frame_id, robot)
        self._polygon_publishers[(robot, label)].publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizTfAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
