#!/usr/bin/env python3
"""
Aurora semantic fusion — vehicle detection from native semantic_segmentation + depth.

COCO80 labels: bicycle(2), car(3), motorcycle(4), bus(6), truck(8).
Resizes semantic (480x640) to depth (224x416) via nearest-neighbor.
Computes mask centroid, samples depth, projects to 3D, transforms to slamware_map.
Publishes BoundingBoxes3d (vehicles only) for inspection_manager.

Phase 1: Uses message_filters.ApproximateTimeSynchronizer to align semantic and depth
by timestamp, reducing fallback_depth usage when centroid depth is invalid.
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
import message_filters


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
        self.declare_parameter("fallback_depth_m", 4.0)  # Use when centroid depth invalid (ensures location saved)
        self.declare_parameter("min_mask_area", 100)
        self.declare_parameter("fallback_vehicle_boxes_topic", "")  # When set, use YOLO 3D boxes if semantic not received (e.g. /darknet_ros_3d/vehicle_bounding_boxes)
        self.declare_parameter("fallback_vehicle_labels", "car,truck,bus,motorcycle,bicycle")  # Comma-separated names to pass through
        self.declare_parameter("semantic_stale_s", 2.0)  # seconds without semantic before treating as stale
        self.declare_parameter("depth_stale_s", 2.0)  # seconds without depth before treating as stale
        self.declare_parameter("intrinsics_file", "")  # Load from aurora_depth_intrinsics.yaml when set
        self.declare_parameter("fx", 180.0)
        self.declare_parameter("fy", 180.0)
        self.declare_parameter("cx", 208.0)
        self.declare_parameter("cy", 112.0)
        self.declare_parameter("sync_slop_s", 0.1)  # message_filters ApproximateTime slop (s); 0.05-0.1 typical
        self.declare_parameter("sync_queue_size", 10)  # message_filters queue depth

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
        self.fallback_depth = self.get_parameter("fallback_depth_m").value
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
        self._last_sync_time = None
        self._last_semantic_stale_log_time = None
        self._last_depth_stale_log_time = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        # Aurora SDK publishes with default QoS (RELIABLE). Use RELIABLE so we receive.
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self._qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        sync_slop = float(self.get_parameter("sync_slop_s").value)
        sync_queue = int(self.get_parameter("sync_queue_size").value)

        self._sub_semantic = message_filters.Subscriber(self, Image, self.semantic_topic, qos_profile=qos_reliable)
        self._sub_depth = message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=qos_reliable)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_semantic, self._sub_depth],
            queue_size=sync_queue,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._cb_sync)

        self._pub = self.create_publisher(BoundingBoxes3d, self.output_topic, 10)

        self._alignment_logged = False
        self._sync_received_logged = False
        self._no_data_warn_time = None  # for delayed "no data received" warning
        self._fallback_sub_created = False
        self._semantic_absent_since = None  # time when we first had depth but no semantic
        self.get_logger().info(
            f"aurora_semantic_fusion: semantic={self.semantic_topic} + depth={self.depth_topic} -> {self.output_topic} "
            f"(sync slop={sync_slop}s, queue={sync_queue})"
        )
        if self._fallback_vehicle_boxes_topic:
            self.get_logger().info(
                f"Fallback: if no semantic after 10s, will use vehicle boxes from {self._fallback_vehicle_boxes_topic} (labels: {self._fallback_vehicle_names})"
            )

    def _cb_sync(self, semantic_msg: Image, depth_msg: Image):
        """Timestamp-synchronized callback: process semantic + depth pair together."""
        self._last_sync_time = time.time()
        if not self._sync_received_logged:
            self.get_logger().info(
                f"Receiving synchronized semantic+depth: semantic {semantic_msg.width}x{semantic_msg.height}, "
                f"depth {depth_msg.width}x{depth_msg.height}"
            )
            self._sync_received_logged = True
        try:
            semantic = self.bridge.imgmsg_to_cv2(semantic_msg, desired_encoding="passthrough")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            elif depth.dtype != np.float32:
                depth = depth.astype(np.float32)
            self._process_fusion_pair(semantic, depth, semantic_msg.header, depth_msg.header)
        except Exception as e:
            self.get_logger().warn(f"Sync cb: {e}")

    def _cb_fallback_vehicle_boxes(self, msg: BoundingBoxes3d):
        """Republish only vehicle-class boxes from YOLO 3D pipeline when sync pairs unavailable."""
        now = time.time()
        if self._last_sync_time is not None:
            age = now - self._last_sync_time
            if age <= self.semantic_stale_s:
                return  # sync pairs are fresh; ignore fallback to avoid duplicates
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
        """Timer: staleness warnings and fallback enablement. Fusion runs in _cb_sync."""
        now = time.time()
        sync_age = None if self._last_sync_time is None else (now - self._last_sync_time)
        sync_stale = sync_age is None or sync_age > max(self.semantic_stale_s, self.depth_stale_s)

        if sync_stale and (self._last_semantic_stale_log_time is None or (now - self._last_semantic_stale_log_time) > 5.0):
            sync_age_msg = f"{sync_age:.1f}s" if sync_age is not None else "no sync yet"
            self.get_logger().warn(
                f"No synchronized semantic+depth for {sync_age_msg} (threshold {max(self.semantic_stale_s, self.depth_stale_s)}s). "
                "Check Aurora topics and sync_slop_s."
            )
            self._last_semantic_stale_log_time = now

        if sync_stale:
            if self._semantic_absent_since is None:
                self._semantic_absent_since = now
            if (now - self._semantic_absent_since) >= 10.0 and self._fallback_vehicle_boxes_topic and not self._fallback_sub_created:
                self._sub_fallback = self.create_subscription(
                    BoundingBoxes3d,
                    self._fallback_vehicle_boxes_topic,
                    self._cb_fallback_vehicle_boxes,
                    self._qos_best_effort,
                )
                self._fallback_sub_created = True
                self.get_logger().warn(
                    f"No sync pairs after 10s. Using fallback: {self._fallback_vehicle_boxes_topic} "
                    "(run segment_3d + ultralytics for YOLO vehicle boxes)."
                )
        else:
            self._semantic_absent_since = None

        if sync_stale:
            if self._no_data_warn_time is None or (now - self._no_data_warn_time) > 5.0:
                self.get_logger().warn(
                    "No synchronized semantic+depth. Is Aurora (slamware_ros_sdk_server_node) running? "
                    "Check: ros2 topic hz /slamware_ros_sdk_server_node/semantic_labels and .../depth_image_raw"
                )
                self._no_data_warn_time = now
        else:
            self._no_data_warn_time = None

    def _process_fusion_pair(self, semantic: np.ndarray, depth: np.ndarray, semantic_header, depth_header):
        """Core fusion: semantic + depth -> vehicle BoundingBoxes3d. Called from sync callback."""
        h_d, w_d = depth.shape
        h_s, w_s = semantic.shape

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
                semantic, (w_d, h_d), interpolation=cv2.INTER_NEAREST
            )
        else:
            semantic_resized = semantic

        # semantic_labels topic: mono8, pixel value = COCO80 class ID
        if semantic_resized.ndim == 3:
            semantic_labels = cv2.cvtColor(semantic_resized, cv2.COLOR_BGR2GRAY)
        else:
            semantic_labels = semantic_resized

        boxes = BoundingBoxes3d()
        boxes.header = depth_header
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
            # Research: use median depth across full mask (more robust than centroid-only)
            # Single centroid pixel can be invalid/noisy; mask median handles occlusion and edge cases
            mask_depths = depth[ys, xs]
            valid = mask_depths[
                np.isfinite(mask_depths)
                & (mask_depths > self.min_depth)
                & (mask_depths < self.max_depth)
            ]
            z = float(np.median(valid)) if valid.size > 0 else 0.0
            # Fallback: if median invalid, try 25th percentile (closer surface, robust to occlusion)
            if (z <= 0 or not np.isfinite(z)) and valid.size > 0:
                z = float(np.percentile(valid, 25))
            # Final fallback: if no valid depth in mask, use fallback_depth (saves location)
            if z <= 0 or not np.isfinite(z):
                z = self.fallback_depth
            if z == self.fallback_depth:
                self.get_logger().info(
                    f"Using fallback_depth={z}m for {VEHICLE_LABEL_NAMES[label_id]} (centroid depth invalid); location saved.",
                    throttle_duration_sec=5.0,
                )
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
