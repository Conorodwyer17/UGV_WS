#!/usr/bin/env python3
"""
Minimal 3D detector adapter for demo pipeline.

Input:  /darknet_ros_3d/vehicle_bounding_boxes
Output: /demo/detections_3d
"""

import rclpy
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class DemoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("demo_detector")
        self.declare_parameter("input_topic", "/darknet_ros_3d/vehicle_bounding_boxes")
        self.declare_parameter("output_topic", "/demo/detections_3d")
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(BoundingBoxes3d, out_topic, qos)
        self.create_subscription(BoundingBoxes3d, in_topic, self._cb, qos)
        self.get_logger().info(f"Relaying detections {in_topic} -> {out_topic}")

    def _cb(self, msg: BoundingBoxes3d) -> None:
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
