"""Continuously register the Isaac demo box in the MoveIt planning scene."""

import os

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


def env_float(name: str, default: float) -> float:
    return float(os.environ.get(name, str(default)))


class DemoObstacle(Node):
    def __init__(self) -> None:
        super().__init__('moveit_demo_obstacle')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('obstacle_id', 'moveit_demo_obstacle')
        self.declare_parameter('x', env_float('DEMO_OBSTACLE_X', 0.50))
        self.declare_parameter('y', env_float('DEMO_OBSTACLE_Y', 0.08))
        self.declare_parameter('z', env_float('DEMO_OBSTACLE_Z', 0.40))
        self.declare_parameter(
            'size_x', env_float('DEMO_OBSTACLE_SIZE_X', 0.10)
        )
        self.declare_parameter(
            'size_y', env_float('DEMO_OBSTACLE_SIZE_Y', 0.30)
        )
        self.declare_parameter(
            'size_z', env_float('DEMO_OBSTACLE_SIZE_Z', 0.24)
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            CollisionObject, '/collision_object', qos
        )
        self._logged = False
        self._timer = self.create_timer(1.0, self._publish)
        self._publish()

    def _publish(self) -> None:
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            float(self.get_parameter('size_x').value),
            float(self.get_parameter('size_y').value),
            float(self.get_parameter('size_z').value),
        ]

        pose = Pose()
        pose.position.x = float(self.get_parameter('x').value)
        pose.position.y = float(self.get_parameter('y').value)
        pose.position.z = float(self.get_parameter('z').value)
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = str(self.get_parameter('frame_id').value)
        collision.id = str(self.get_parameter('obstacle_id').value)
        collision.primitives = [box]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        self._publisher.publish(collision)

        if not self._logged and self._publisher.get_subscription_count() > 0:
            self.get_logger().info(
                f'published {collision.id} in {collision.header.frame_id}: '
                f'position=({pose.position.x:.3f}, {pose.position.y:.3f}, '
                f'{pose.position.z:.3f}), size={box.dimensions}'
            )
            self._logged = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
