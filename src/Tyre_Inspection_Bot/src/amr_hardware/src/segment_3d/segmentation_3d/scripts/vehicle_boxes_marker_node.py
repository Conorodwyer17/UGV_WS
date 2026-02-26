#!/usr/bin/env python3
"""
Subscribe to /aurora_semantic/vehicle_bounding_boxes (BoundingBoxes3d) and publish
visualization_msgs/MarkerArray to /aurora_semantic/vehicle_markers for RViz.
Run with Aurora + aurora_semantic_fusion_node to see vehicle bounding boxes in 3D.
"""
import rclpy
from rclpy.node import Node
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from visualization_msgs.msg import MarkerArray, Marker


class VehicleBoxesMarkerNode(Node):
    def __init__(self):
        super().__init__("vehicle_boxes_marker")
        self.declare_parameter("input_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("marker_topic", "/aurora_semantic/vehicle_markers")
        input_topic = self.get_parameter("input_topic").value
        marker_topic = self.get_parameter("marker_topic").value

        self._sub = self.create_subscription(
            BoundingBoxes3d,
            input_topic,
            self._cb,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.get_logger().info(
            f"Publishing vehicle box markers: {input_topic} -> {marker_topic}"
        )

    def _cb(self, msg: BoundingBoxes3d):
        arr = MarkerArray()
        for i, box in enumerate(msg.bounding_boxes):
            m = Marker()
            m.header = msg.header
            m.ns = "aurora_vehicle_boxes"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            cx = (box.xmin + box.xmax) / 2.0
            cy = (box.ymin + box.ymax) / 2.0
            cz = (box.zmin + box.zmax) / 2.0
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = cz
            m.pose.orientation.w = 1.0
            m.scale.x = max(0.1, box.xmax - box.xmin)
            m.scale.y = max(0.1, box.ymax - box.ymin)
            m.scale.z = max(0.1, box.zmax - box.zmin)
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 0.7
            m.lifetime.sec = 1
            m.lifetime.nanosec = 0
            arr.markers.append(m)
        # Publish even when empty so RViz clears old markers when car leaves view
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleBoxesMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
