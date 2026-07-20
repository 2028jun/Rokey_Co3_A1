#!/usr/bin/env python3
"""Request table 0..3 from the mobile-manipulator Isaac ROS service."""

import argparse
import os
import sys

os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "SERVING_ROBOT_ROS_DOMAIN_ID", "101"
)

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


SERVICE_NAME = "/serving_robot/go_to_table"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a table navigation request to Isaac Sim."
    )
    parser.add_argument(
        "table_id",
        type=int,
        choices=range(4),
        metavar="{0,1,2,3}",
        help="destination table number",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("go_to_table_client")
    client = node.create_client(AddTwoInts, SERVICE_NAME)
    try:
        if not client.wait_for_service(timeout_sec=15.0):
            node.get_logger().error(f"service unavailable: {SERVICE_NAME}")
            return 1

        request = AddTwoInts.Request()
        request.a = args.table_id
        request.b = 0
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            node.get_logger().error("service call timed out or failed")
            return 1

        result = int(future.result().sum)
        if result == -2:
            node.get_logger().error(
                "request rejected: Isaac parking brake is engaged"
            )
            return 2
        if result != args.table_id:
            node.get_logger().error(
                f"request rejected: response code={result}"
            )
            return 2

        node.get_logger().info(
            f"table {args.table_id} navigation request accepted"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
