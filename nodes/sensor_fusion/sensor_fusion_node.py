#!/usr/bin/env python3
"""Skeleton sensor fusion node placeholder."""

import rclpy
from rclpy.node import Node


class SensorFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_fusion")
        self.get_logger().info("sensor_fusion skeleton ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
