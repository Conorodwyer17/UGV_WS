#!/usr/bin/env python3
"""
Aurora semantic fusion — vehicle detection from native semantic_segmentation + depth.

COCO80 labels: bicycle(2), car(3), motorcycle(4), bus(6), truck(8).
Resizes semantic (480x640) to depth (224x416) via nearest-neighbor.
Computes mask centroid, samples depth, projects to 3D, transforms to slamware_map.
Publishes BoundingBoxes3d (vehicles only) for inspection_manager.
"""
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
import cv2
import yaml


# COCO80 label IDs for vehicles
VEHICLE_LABEL_IDS = {2, 3, 4, 6, 8}  # bicycle, car, motorcycle, bus, truck
VEHICLE_LABEL_NAMES = {2: "bicycle", 3: "car", 4: "motorcycle", 6: "bus", 8: "truck"}


class AuroraSemanticFusionNode(Node):
    def __init__(self):
        super().__init__("aurora_semantic_fusion")
        self.declare_parameter("semantic_topic", "/slamware_ros_sdk_server_node/semantic_labels")
        self.declare_parameter("depth_topic", "/slamware_ros_sdk_server_node/depth_image_raw")
        self.declare_parameter("output_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("target_frame", "slamware_map")
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 15.0)
        self.declare_parameter("min_mask_area", 100)
        self.declare_parameter("fallback_vehicle_boxes_topic", "")  # When set, use YOLO 3D boxes if semantic not received (e.g. /darknet_ros_3d/bounding_boxes)
        self.declare_parameter("fallback_vehicle_labels", "car,truck,bus,motorcycle,bicycle")  # Comma-separated names to pass through
        self.declare_parameter("semantic_stale_s", 2.0)  # seconds without semantic before treating as stale
        self.declare_parameter("depth_stale_s", 2.0)  # seconds without depth before treating as stale
        self.declare_parameter("intrinsics_file", "")  # Load from aurora_depth_intrinsics.yaml when set
        self.declare_parameter("fx", 180.0)
        self.declare_parameter("fy", 180.0)
        self.declare_parameter("cx", 208.0)
        self.declare_parameter("cy", 112.0)

        # Load intrinsics from file or use params (must match aurora_depth_camera_info for correct geometry)
        intrinsics_file = self.get_parameter("intrinsics_file").value
        if intrinsics_file and os.path.isfile(intrinsics_file):
            with open(intrinsics_file) as f:
                cfg = yaml.safe_load(f)
            c = cfg.get("aurora_depth_camera_info", cfg)
            fx = float(c.get("fx", 180.0))
            fy = float(c.get("fy", 180.0))
            cx = float(c.get("cx", 208.0))
            cy = float(c.get("cy", 112.0))
            self.get_logger().info(f"Loaded intrinsics from {intrinsics_file}: fx={fx:.1f} cx={cx:.1f}")
        else:
            fx = self.get_parameter("fx").value
            fy = self.get_parameter("fy").value
            cx = self.get_parameter("cx").value
            cy = self.get_parameter("cy").value

        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.min_depth = self.get_parameter("min_depth_m").value
        self.max_depth = self.get_parameter("max_depth_m").value
        self.min_mask_area = self.get_parameter("min_mask_area").value
        self.semantic_stale_s = float(self.get_parameter("semantic_stale_s").value)
        self.depth_stale_s = float(self.get_parameter("depth_stale_s").value)
        fallback_topic = self.get_parameter("fallback_vehicle_boxes_topic").value
        fallback_labels_str = self.get_parameter("fallback_vehicle_labels").value
        self._fallback_vehicle_boxes_topic = fallback_topic.strip() if isinstance(fallback_topic, str) else ""
        self._fallback_vehicle_names = {s.strip().lower() for s in str(fallback_labels_str).split(",") if s.strip()}
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

        self.bridge = CvBridge()
        self._semantic = None
        self._depth = None
        self._semantic_header = None
        self._depth_header = None
        self._last_semantic_time = None
        self._last_depth_time = None
        self._last_semantic_stale_log_time = None
        self._last_depth_stale_log_time = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        # Aurora SDK publishes with default QoS (RELIABLE). Use RELIABLE so we receive.
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self._qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self._sub_semantic = self.create_subscription(
            Image, self.semantic_topic, self._cb_semantic, qos_reliable
        )
        self._sub_depth = self.create_subscription(
            Image, self.depth_topic, self._cb_depth, qos_reliable
        )
        self._pub = self.create_publisher(BoundingBoxes3d, self.output_topic, 10)

        self._alignment_logged = False
        self._semantic_received_logged = False
        self._depth_received_logged = False
        self._no_data_warn_time = None  # for delayed "no data received" warning
        self._fallback_sub_created = False
        self._semantic_absent_since = None  # time when we first had depth but no semantic
        self.get_logger().info(
            f"aurora_semantic_fusion: semantic={self.semantic_topic} + depth={self.depth_topic} -> {self.output_topic}"
        )
        if self._fallback_vehicle_boxes_topic:
            self.get_logger().info(
                f"Fallback: if no semantic after 10s, will use vehicle boxes from {self._fallback_vehicle_boxes_topic} (labels: {self._fallback_vehicle_names})"
            )

    def _cb_semantic(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self._semantic = img
            self._semantic_header = msg.header
            self._last_semantic_time = time.time()
            if not self._semantic_received_logged:
                self.get_logger().info(f"Receiving semantic: {msg.width}x{msg.height} encoding={msg.encoding}")
                self._semantic_received_logged = True
        except Exception as e:
            self.get_logger().warn(f"Semantic cb: {e}")

    def _cb_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            elif depth.dtype != np.float32:
                depth = depth.astype(np.float32)
            self._depth = depth
            self._depth_header = msg.header
            self._last_depth_time = time.time()
            if not self._depth_received_logged:
                self.get_logger().info(f"Receiving depth: {msg.width}x{msg.height} encoding={msg.encoding}")
                self._depth_received_logged = True
        except Exception as e:
            self.get_logger().warn(f"Depth cb: {e}")

    def _cb_fallback_vehicle_boxes(self, msg: BoundingBoxes3d):
        """Republish only vehicle-class boxes from YOLO 3D pipeline when semantic is unavailable."""
        now = time.time()
        if self._last_semantic_time is not None:
            age = now - self._last_semantic_time
            if age <= self.semantic_stale_s:
                return  # semantic is fresh; ignore fallback to avoid duplicates
        out = BoundingBoxes3d()
        out.header = msg.header
        for box in msg.bounding_boxes:
            name = (box.object_name or "").strip().lower()
            if name in self._fallback_vehicle_names:
                out.bounding_boxes.append(box)
        self._pub.publish(out)
        if out.bounding_boxes and not getattr(self, "_fallback_publish_logged", False):
            self.get_logger().info(
                f"Fallback: publishing {len(out.bounding_boxes)} vehicle box(es) from {self._fallback_vehicle_boxes_topic}"
            )
            self._fallback_publish_logged = True

    def _run_fusion(self):
        # Warn once if we never receive semantic or depth (Aurora not running or wrong topics)
        now = time.time()
        semantic_age = None if self._last_semantic_time is None else (now - self._last_semantic_time)
        depth_age = None if self._last_depth_time is None else (now - self._last_depth_time)
        semantic_stale = semantic_age is None or semantic_age > self.semantic_stale_s
        depth_stale = depth_age is None or depth_age > self.depth_stale_s
        if semantic_stale and (self._last_semantic_stale_log_time is None or (now - self._last_semantic_stale_log_time) > 5.0):
            semantic_age_msg = f"{semantic_age:.1f}" if semantic_age is not None else "inf"
            self.get_logger().warn(
                f"Semantic stale for {semantic_age_msg}s (threshold {self.semantic_stale_s}s)."
            )
            self._last_semantic_stale_log_time = now
        if depth_stale and (self._last_depth_stale_log_time is None or (now - self._last_depth_stale_log_time) > 5.0):
            depth_age_msg = f"{depth_age:.1f}" if depth_age is not None else "inf"
            self.get_logger().warn(
                f"Depth stale for {depth_age_msg}s (threshold {self.depth_stale_s}s)."
            )
            self._last_depth_stale_log_time = now

        semantic_available = self._semantic is not None and not semantic_stale
        depth_available = self._depth is not None and not depth_stale
        if not semantic_available and depth_available:
            if self._semantic_absent_since is None:
                self._semantic_absent_since = now
            # After 10s without semantic, enable fallback if configured
            if (now - self._semantic_absent_since) >= 10.0 and self._fallback_vehicle_boxes_topic and not self._fallback_sub_created:
                self._sub_fallback = self.create_subscription(
                    BoundingBoxes3d,
                    self._fallback_vehicle_boxes_topic,
                    self._cb_fallback_vehicle_boxes,
                    self._qos_best_effort,
                )
                self._fallback_sub_created = True
                self.get_logger().warn(
                    f"No semantic_labels after 10s. Using fallback: {self._fallback_vehicle_boxes_topic} "
                    "(run segment_3d + ultralytics for YOLO vehicle boxes)."
                )
        else:
            self._semantic_absent_since = None  # we have semantic, no fallback needed

        # Only warn when we have neither semantic nor depth. If we have depth but no semantic, don't repeat every 5s.
        if not semantic_available and not depth_available:
            if self._no_data_warn_time is None:
                self._no_data_warn_time = now
            elif (now - self._no_data_warn_time) > 5.0:
                self.get_logger().warn(
                    "No semantic or depth received after 5s. Is Aurora (slamware_ros_sdk_server_node) running? "
                    "Check: ros2 topic hz /slamware_ros_sdk_server_node/semantic_labels and .../depth_image_raw"
                )
                self._no_data_warn_time = now
        else:
            self._no_data_warn_time = None

        if not semantic_available or not depth_available:
            return
        if self._depth_header is None:
            return

        h_d, w_d = self._depth.shape
        h_s, w_s = self._semantic.shape

        if not self._alignment_logged:
            self.get_logger().info(
                f"Alignment: semantic {w_s}x{h_s} -> depth {w_d}x{h_d}, scale=({w_d/w_s:.3f}, {h_d/h_s:.3f})"
            )
            if (h_d, w_d) != (224, 416):
                self.get_logger().warn(
                    f"Depth resolution {w_d}x{h_d} != expected Aurora 2.11 (416x224). "
                    "Verify depth topic and intrinsics."
                )
            if (h_s, w_s) != (480, 640):
                self.get_logger().warn(
                    f"Semantic resolution {w_s}x{h_s} != expected Aurora 2.11 (640x480). "
                    "Alignment may be incorrect."
                )
            self._alignment_logged = True

        # Resize semantic to depth resolution (nearest-neighbor for label IDs)
        if (h_s, w_s) != (h_d, w_d):
            semantic_resized = cv2.resize(
                self._semantic, (w_d, h_d), interpolation=cv2.INTER_NEAREST
            )
        else:
            semantic_resized = self._semantic

        # semantic_labels topic: mono8, pixel value = COCO80 class ID
        if semantic_resized.ndim == 3:
            semantic_labels = cv2.cvtColor(semantic_resized, cv2.COLOR_BGR2GRAY)
        else:
            semantic_labels = semantic_resized

        depth = self._depth
        boxes = BoundingBoxes3d()
        boxes.header = self._depth_header
        boxes.header.frame_id = self.target_frame
        had_vehicle_mask = False  # True if any label had area >= min_mask_area (for diagnostic when 0 boxes)

        for label_id in VEHICLE_LABEL_IDS:
            mask = (semantic_labels == label_id)
            if not np.any(mask):
                continue
            area = np.sum(mask)
            if area < self.min_mask_area:
                continue
            had_vehicle_mask = True
            ys, xs = np.where(mask)
            cy_px = float(np.mean(ys))
            cx_px = float(np.mean(xs))
            # Sample depth at centroid (with small neighborhood median for robustness)
            r = 2
            y0, y1 = max(0, int(cy_px) - r), min(h_d, int(cy_px) + r + 1)
            x0, x1 = max(0, int(cx_px) - r), min(w_d, int(cx_px) + r + 1)
            roi = depth[y0:y1, x0:x1]
            valid = roi[np.isfinite(roi) & (roi > self.min_depth) & (roi < self.max_depth)]
            if valid.size == 0:
                continue
            z = float(np.median(valid))
            if z <= 0 or not np.isfinite(z):
                continue
            # Unproject to camera frame
            x_cam = (cx_px - self.cx) * z / self.fx
            y_cam = (cy_px - self.cy) * z / self.fy
            z_cam = z
            # Transform to target frame (use latest TF to avoid clock sync issues with Aurora)
            try:
                t = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5),
                )
            except Exception as e:
                self.get_logger().warn(
                    f"TF lookup failed ({self.target_frame} <- {self.camera_frame}): {e}",
                    throttle_duration_sec=5.0,
                )
                continue
            # Apply transform (quaternion to rotation matrix)
            R = np.array([
                [t.transform.rotation.w**2 + t.transform.rotation.x**2 - t.transform.rotation.y**2 - t.transform.rotation.z**2,
                 2*(t.transform.rotation.x*t.transform.rotation.y - t.transform.rotation.w*t.transform.rotation.z),
                 2*(t.transform.rotation.x*t.transform.rotation.z + t.transform.rotation.w*t.transform.rotation.y)],
                [2*(t.transform.rotation.x*t.transform.rotation.y + t.transform.rotation.w*t.transform.rotation.z),
                 t.transform.rotation.w**2 - t.transform.rotation.x**2 + t.transform.rotation.y**2 - t.transform.rotation.z**2,
                 2*(t.transform.rotation.y*t.transform.rotation.z - t.transform.rotation.w*t.transform.rotation.x)],
                [2*(t.transform.rotation.x*t.transform.rotation.z - t.transform.rotation.w*t.transform.rotation.y),
                 2*(t.transform.rotation.y*t.transform.rotation.z + t.transform.rotation.w*t.transform.rotation.x),
                 t.transform.rotation.w**2 - t.transform.rotation.x**2 - t.transform.rotation.y**2 + t.transform.rotation.z**2],
            ])
            p = np.array([x_cam, y_cam, z_cam])
            t_vec = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            p_map = R @ p + t_vec
            # Create minimal BoundingBox3d (center + small extent)
            box = BoundingBox3d()
            box.object_name = VEHICLE_LABEL_NAMES[label_id]
            box.probability = 0.95
            extent = 0.5
            box.xmin = p_map[0] - extent
            box.xmax = p_map[0] + extent
            box.ymin = p_map[1] - extent
            box.ymax = p_map[1] + extent
            box.zmin = p_map[2] - extent
            box.zmax = p_map[2] + extent
            boxes.bounding_boxes.append(box)

        if boxes.bounding_boxes:
            self._pub.publish(boxes)
            self.get_logger().info(
                f"Publishing {len(boxes.bounding_boxes)} vehicle box(es): "
                f"{[b.object_name for b in boxes.bounding_boxes]}",
                throttle_duration_sec=2.0,
            )
        else:
            # Publish empty so the topic exists (ros2 topic echo / list can see it)
            self._pub.publish(boxes)
            if had_vehicle_mask:
                self.get_logger().warn(
                    "Vehicle mask(s) above min_area but 0 boxes (depth invalid or TF lookup failed); check depth topic and camera_depth_optical_frame->slamware_map.",
                    throttle_duration_sec=5.0,
                )


def main(args=None):
    rclpy.init(args=args)
    node = AuroraSemanticFusionNode()
    timer = node.create_timer(0.1, node._run_fusion)  # 10 Hz
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
