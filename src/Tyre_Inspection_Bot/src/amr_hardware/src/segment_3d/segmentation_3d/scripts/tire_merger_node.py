#!/usr/bin/env python3
"""
Tire merger: prefer YOLO tire detections when available; fallback to PCL when YOLO is empty or stale.

Subscribes to:
  - /darknet_ros_3d/tire_bounding_boxes (YOLO)
  - /tire_bounding_boxes_pcl_fallback (PCL)

Publishes to /tire_bounding_boxes_merged.
Staleness: if YOLO has had no messages for N seconds, treat as stale and allow PCL.

Optional EMA smoothing (perception_stability_report): reduces centroid jitter when smoothing_enabled.
"""
import rclpy
from rclpy.node import Node
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d


def _box_centroid(box: BoundingBox3d) -> tuple:
    """Return (x, y, z) centroid of bounding box."""
    x = (box.xmin + box.xmax) / 2.0
    y = (box.ymin + box.ymax) / 2.0
    z = (box.zmin + box.zmax) / 2.0
    return (x, y, z)


def _box_half_extents(box: BoundingBox3d) -> tuple:
    """Return (dx, dy, dz) half-extents."""
    dx = (box.xmax - box.xmin) / 2.0
    dy = (box.ymax - box.ymin) / 2.0
    dz = (box.zmax - box.zmin) / 2.0
    return (dx, dy, dz)


def _box_from_centroid_extents(centroid: tuple, extents: tuple, template: BoundingBox3d) -> BoundingBox3d:
    """Build BoundingBox3d from centroid and half-extents, copying other fields from template."""
    out = BoundingBox3d()
    out.object_name = template.object_name
    out.probability = template.probability
    cx, cy, cz = centroid
    dx, dy, dz = extents
    out.xmin = cx - dx
    out.xmax = cx + dx
    out.ymin = cy - dy
    out.ymax = cy + dy
    out.zmin = cz - dz
    out.zmax = cz + dz
    return out


class TireMergerNode(Node):
    def __init__(self):
        super().__init__("tire_merger")
        self.declare_parameter("yolo_topic", "/darknet_ros_3d/tire_bounding_boxes")
        self.declare_parameter("pcl_fallback_topic", "/tire_bounding_boxes_pcl_fallback")
        self.declare_parameter("output_topic", "/tire_bounding_boxes_merged")
        self.declare_parameter("yolo_stale_s", 2.0)
        self.declare_parameter("log_fallback_usage", True)
        self.declare_parameter("smoothing_enabled", False)
        self.declare_parameter("ema_alpha", 0.3)

        self._yolo_topic = self.get_parameter("yolo_topic").value
        self._pcl_topic = self.get_parameter("pcl_fallback_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._yolo_stale_s = self.get_parameter("yolo_stale_s").value
        self._log_fallback = self.get_parameter("log_fallback_usage").value
        self._smoothing_enabled = bool(self.get_parameter("smoothing_enabled").value)
        v = self.get_parameter("ema_alpha").value
        self._ema_alpha = float(v) if v is not None else 0.3

        self._last_yolo_time = None
        self._latest_yolo = None
        self._latest_pcl = None
        self._last_source_logged = None
        self._ema_state: dict = {}  # index -> (x, y, z) smoothed centroid

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        self._sub_yolo = self.create_subscription(
            BoundingBoxes3d, self._yolo_topic, self._yolo_cb, qos
        )
        self._sub_pcl = self.create_subscription(
            BoundingBoxes3d, self._pcl_topic, self._pcl_cb, qos
        )
        self._pub = self.create_publisher(BoundingBoxes3d, self._output_topic, 10)

        self.get_logger().info(
            f"tire_merger: yolo={self._yolo_topic} pcl={self._pcl_topic} -> {self._output_topic} "
            f"(yolo_stale_s={self._yolo_stale_s}, smoothing={self._smoothing_enabled})"
        )

    def _yolo_cb(self, msg: BoundingBoxes3d):
        self._last_yolo_time = self.get_clock().now()
        self._latest_yolo = msg

    def _pcl_cb(self, msg: BoundingBoxes3d):
        self._latest_pcl = msg

    def _publish_merged(self):
        now = self.get_clock().now()
        yolo_has_boxes = (
            self._latest_yolo is not None
            and len(self._latest_yolo.bounding_boxes) > 0
        )
        yolo_stale = (
            self._last_yolo_time is None
            or (now - self._last_yolo_time).nanoseconds / 1e9 > self._yolo_stale_s
        )

        if yolo_has_boxes and not yolo_stale:
            out = self._latest_yolo
            source = "yolo"
        elif self._latest_pcl is not None and len(self._latest_pcl.bounding_boxes) > 0:
            out = self._latest_pcl
            source = "pcl"
        else:
            out = BoundingBoxes3d()
            out.header.stamp = now.to_msg()
            out.header.frame_id = "slamware_map"
            out.bounding_boxes = []
            source = "empty"

        if self._log_fallback and source != self._last_source_logged:
            self._last_source_logged = source
            if source == "pcl":
                self.get_logger().info(
                    f"tire_merger: using PCL fallback ({len(out.bounding_boxes)} boxes)"
                )
            elif source == "empty":
                self.get_logger().debug("tire_merger: no detections (empty)")

        if self._smoothing_enabled and len(out.bounding_boxes) > 0:
            out = self._apply_ema(out)

        # Publish with consistent frame (inspection_manager expects slamware_map)
        out.header.stamp = now.to_msg()
        if not out.header.frame_id:
            out.header.frame_id = "slamware_map"
        self._pub.publish(out)

    def _apply_ema(self, msg: BoundingBoxes3d) -> BoundingBoxes3d:
        """Apply EMA smoothing to box centroids. Reduces jitter (perception_stability_report)."""
        out = BoundingBoxes3d()
        out.header = msg.header
        out.bounding_boxes = []
        for i, box in enumerate(msg.bounding_boxes):
            cx, cy, cz = _box_centroid(box)
            extents = _box_half_extents(box)
            if i in self._ema_state:
                ox, oy, oz = self._ema_state[i]
                a = self._ema_alpha
                nx = a * cx + (1 - a) * ox
                ny = a * cy + (1 - a) * oy
                nz = a * cz + (1 - a) * oz
                self._ema_state[i] = (nx, ny, nz)
                centroid = (nx, ny, nz)
            else:
                self._ema_state[i] = (cx, cy, cz)
                centroid = (cx, cy, cz)
            out.bounding_boxes.append(_box_from_centroid_extents(centroid, extents, box))
        # Trim state when we have fewer boxes than before
        keys_to_del = [k for k in self._ema_state if k >= len(msg.bounding_boxes)]
        for k in keys_to_del:
            del self._ema_state[k]
        return out

    def _timer_cb(self):
        self._publish_merged()

    def run(self):
        """Run with 10 Hz merge rate."""
        self._timer = self.create_timer(0.1, self._timer_cb)
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = TireMergerNode()
    node._timer = node.create_timer(0.1, node._timer_cb)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
