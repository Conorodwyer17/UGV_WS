#!/usr/bin/env python3
"""
Minimal tracker for demo pipeline.

Input:  /demo/detections_3d
Output: /demo/tracks_3d
Strategy: nearest-neighbor association by 2D centroid distance.
"""

import math
from dataclasses import dataclass
from typing import Dict

import rclpy
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class TrackState:
    track_id: int
    x: float
    y: float


class DemoTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("demo_tracker")
        self.declare_parameter("input_topic", "/demo/detections_3d")
        self.declare_parameter("output_topic", "/demo/tracks_3d")
        self.declare_parameter("association_max_dist_m", 2.0)
        self.in_topic = str(self.get_parameter("input_topic").value)
        self.out_topic = str(self.get_parameter("output_topic").value)
        self.max_dist = float(self.get_parameter("association_max_dist_m").value)

        self._next_id = 1
        self._tracks: Dict[int, TrackState] = {}
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(BoundingBoxes3d, self.out_topic, qos)
        self.create_subscription(BoundingBoxes3d, self.in_topic, self._cb, qos)
        self.get_logger().info(f"Tracking {self.in_topic} -> {self.out_topic}")

    def _associate(self, x: float, y: float) -> int:
        best_id = -1
        best_d = 1e9
        for track_id, st in self._tracks.items():
            d = math.hypot(st.x - x, st.y - y)
            if d < best_d:
                best_d = d
                best_id = track_id
        if best_id > 0 and best_d <= self.max_dist:
            return best_id
        new_id = self._next_id
        self._next_id += 1
        return new_id

    def _cb(self, msg: BoundingBoxes3d) -> None:
        out = BoundingBoxes3d()
        out.header = msg.header
        for box in msg.bounding_boxes:
            cx = 0.5 * (box.xmin + box.xmax)
            cy = 0.5 * (box.ymin + box.ymax)
            tid = self._associate(cx, cy)
            self._tracks[tid] = TrackState(track_id=tid, x=float(cx), y=float(cy))
            box.object_name = f"{box.object_name}|track_{tid}"
            out.bounding_boxes.append(box)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
