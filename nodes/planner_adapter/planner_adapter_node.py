#!/usr/bin/env python3
"""Skeleton planner adapter node placeholder."""

import rclpy
from rclpy.node import Node


class PlannerAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_adapter")
        self.get_logger().info("planner_adapter skeleton ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
