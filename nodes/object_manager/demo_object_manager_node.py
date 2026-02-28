#!/usr/bin/env python3
"""
Minimal object manager demo:
- subscribes to /darknet_ros_3d/vehicle_bounding_boxes
- tracks nearest centroid with persistent IDs
- publishes TF frames object_<id> in map frame
- writes object registry snapshots for demo evidence
"""

import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


@dataclass
class ObjectState:
    object_id: int
    x: float
    y: float
    z: float
    label: str
    updated_at: float


class DemoObjectManager(Node):
    def __init__(self) -> None:
        super().__init__("demo_object_manager")
        self.declare_parameter("input_topic", "/darknet_ros_3d/vehicle_bounding_boxes")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("association_max_dist_m", 2.0)
        self.declare_parameter("snapshot_dir", "research/demos")
        self.declare_parameter("snapshot_period_s", 2.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.max_assoc_dist = float(self.get_parameter("association_max_dist_m").value)
        self.snapshot_dir = str(self.get_parameter("snapshot_dir").value)
        self.snapshot_period = float(self.get_parameter("snapshot_period_s").value)

        self._next_id = 1
        self._objects: Dict[int, ObjectState] = {}
        self._tf = TransformBroadcaster(self)
        self._last_snapshot = 0.0

        os.makedirs(self.snapshot_dir, exist_ok=True)
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(BoundingBoxes3d, self.input_topic, self._cb, qos_sensor)
        self.create_timer(0.2, self._publish_all_transforms)
        self.get_logger().info(f"Demo object manager listening on {self.input_topic}")

    def _cb(self, msg: BoundingBoxes3d) -> None:
        now = time.time()
        self.get_logger().info(f"received {len(msg.bounding_boxes)} vehicle boxes")
        for box in msg.bounding_boxes:
            cx = 0.5 * (box.xmin + box.xmax)
            cy = 0.5 * (box.ymin + box.ymax)
            cz = 0.5 * (box.zmin + box.zmax)
            obj_id = self._associate(cx, cy)
            self._objects[obj_id] = ObjectState(
                object_id=obj_id,
                x=float(cx),
                y=float(cy),
                z=float(cz),
                label=str(box.object_name),
                updated_at=now,
            )
            self._publish_tf(self._objects[obj_id])

        if now - self._last_snapshot >= self.snapshot_period:
            self._write_snapshot()
            self._last_snapshot = now

    def _associate(self, x: float, y: float) -> int:
        best_id = -1
        best_d = 1e9
        for obj_id, state in self._objects.items():
            d = math.hypot(state.x - x, state.y - y)
            if d < best_d:
                best_d = d
                best_id = obj_id
        if best_id > 0 and best_d <= self.max_assoc_dist:
            return best_id
        obj_id = self._next_id
        self._next_id += 1
        return obj_id

    def _publish_tf(self, state: ObjectState) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.global_frame
        t.child_frame_id = f"object_{state.object_id}"
        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = state.z
        t.transform.rotation.w = 1.0
        self._tf.sendTransform(t)

    def _write_snapshot(self) -> None:
        payload = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "objects": [
                {
                    "id": s.object_id,
                    "label": s.label,
                    "position": {"x": s.x, "y": s.y, "z": s.z},
                    "updated_at": s.updated_at,
                }
                for s in sorted(self._objects.values(), key=lambda v: v.object_id)
            ],
        }
        path = os.path.join(self.snapshot_dir, "object_manager_registry_latest.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    def _publish_all_transforms(self) -> None:
        for state in self._objects.values():
            self._publish_tf(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoObjectManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
