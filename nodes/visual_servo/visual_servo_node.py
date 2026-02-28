#!/usr/bin/env python3
"""Skeleton visual servo node placeholder."""

import rclpy
from rclpy.node import Node


class VisualServoNode(Node):
    def __init__(self) -> None:
        super().__init__("visual_servo")
        self.get_logger().info("visual_servo skeleton ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
