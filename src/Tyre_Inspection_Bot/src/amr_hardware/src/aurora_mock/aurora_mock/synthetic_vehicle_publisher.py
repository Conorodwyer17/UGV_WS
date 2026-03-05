#!/usr/bin/env python3
"""
Publish static synthetic vehicle BoundingBoxes3d for simulation.
Used when use_mock:=true so inspection_manager can start a mission without real perception.
Supports multiple vehicles (vehicle_count > 1) for return-later and sequencing tests.
"""
import rclpy
from rclpy.node import Node
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from std_msgs.msg import Header


class SyntheticVehiclePublisher(Node):
    """Publish one or more static vehicle boxes in slamware_map frame for mock simulation."""

    def __init__(self):
        super().__init__("synthetic_vehicle_publisher")
        self.declare_parameter("output_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("vehicle_x_m", 2.0)
        self.declare_parameter("vehicle_y_m", 0.0)
        self.declare_parameter("vehicle_z_m", 0.3)
        self.declare_parameter("vehicle_extent_m", 1.5)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("vehicle_count", 1)
        self.declare_parameter("vehicle_spacing_m", 2.0)

        topic = self.get_parameter("output_topic").value
        self._pub = self.create_publisher(BoundingBoxes3d, topic, 10)
        period = 1.0 / self.get_parameter("publish_rate_hz").value
        self.create_timer(period, self._publish)
        count = self.get_parameter("vehicle_count").value
        self.get_logger().info(f"Publishing {count} synthetic vehicle(s) to {topic}")

    def _publish(self):
        x0 = self.get_parameter("vehicle_x_m").value
        y0 = self.get_parameter("vehicle_y_m").value
        z = self.get_parameter("vehicle_z_m").value
        ext = self.get_parameter("vehicle_extent_m").value
        count = max(1, int(self.get_parameter("vehicle_count").value))
        spacing = self.get_parameter("vehicle_spacing_m").value
        half = ext / 2.0

        boxes = []
        for i in range(count):
            x = x0 + i * spacing
            y = y0
            box = BoundingBox3d()
            box.object_name = "car"
            box.probability = 0.95
            box.xmin = x - half
            box.xmax = x + half
            box.ymin = y - half
            box.ymax = y + half
            box.zmin = z - 0.3
            box.zmax = z + 0.3
            boxes.append(box)

        msg = BoundingBoxes3d()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "slamware_map"
        msg.bounding_boxes = boxes
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticVehiclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
