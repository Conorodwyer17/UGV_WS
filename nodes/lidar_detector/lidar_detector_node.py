#!/usr/bin/env python3
"""Skeleton LiDAR detector node placeholder."""

import rclpy
from rclpy.node import Node


class LidarDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_detector")
        self.get_logger().info("lidar_detector skeleton ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
