#!/usr/bin/env python3
"""Skeleton photo capture node placeholder."""

import rclpy
from rclpy.node import Node


class PhotoCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("photo_capture")
        self.get_logger().info("photo_capture skeleton ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
