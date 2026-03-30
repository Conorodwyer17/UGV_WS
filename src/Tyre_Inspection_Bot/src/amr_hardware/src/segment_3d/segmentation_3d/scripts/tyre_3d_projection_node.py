#!/usr/bin/env python3
"""
Project 2D wheel/tire segmentations to 3D map positions using Aurora depth + intrinsics.

Subscribes to segmentation_msgs/ObjectsSegment (wheel detections), depth image, and RGB (for
image/depth size alignment). Publishes geometry_msgs/PoseArray in the configured output frame
(typically slamware_map to match inspection_manager world_frame) and RViz markers.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional, Set

import numpy as np
import rclpy
from cv_bridge import CvBridge
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from segmentation_msgs.msg import ObjectsSegment
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray


def _param_bool(node: Node, name: str) -> bool:
    v = node.get_parameter(name).value
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes")
    return bool(v)


def _median_valid_depth(patch: np.ndarray) -> float:
    """Return median of positive finite depth values in patch (meters)."""
    if patch.size == 0:
        return float("nan")
    v = patch[np.isfinite(patch) & (patch > 0.05)]
    if v.size == 0:
        return float("nan")
    return float(np.median(v))


class Tyre3dProjectionNode(Node):
    def __init__(self) -> None:
        super().__init__("tyre_3d_projection")

        self.declare_parameter("objects_segment_topic", "/ultralytics_tire/segmentation/objects_segment")
        self.declare_parameter("depth_topic", "/slamware_ros_sdk_server_node/depth_image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("rgb_topic", "/slamware_ros_sdk_server_node/left_image_raw")
        self.declare_parameter("pose_output_topic", "/tyre_3d_positions")
        self.declare_parameter("markers_topic", "/tyre_markers")
        self.declare_parameter("output_frame", "slamware_map")
        self.declare_parameter("camera_optical_frame", "camera_depth_optical_frame")
        # For "approximate" pairing only: max |stamp_seg − stamp_depth| (seconds).
        self.declare_parameter("sync_slop_s", 2.0)
        # "latest" = pair each ObjectsSegment with the most recent depth (robust to stamp skew / QoS).
        # "approximate" = message_filters ApproximateTime (strict stamp alignment).
        self.declare_parameter("depth_pairing_mode", "latest")
        self.declare_parameter("depth_min_m", 0.3)
        self.declare_parameter("depth_max_m", 3.0)
        self.declare_parameter("patch_half_size", 3)
        self.declare_parameter(
            "wheel_class_names",
            ["tyre", "wheel", "tire", "car_tire", "car_tyre", "car-tire", "car-tyre"],
        )
        # If true, project using latest TF (helps stub/no-odom when sensor stamps diverge from /tf).
        self.declare_parameter("tf_lookup_use_latest", False)
        # Aurora depth is often RELIABLE; BEST_EFFORT subscriber will not connect. Set false for pure sensor_data QoS.
        self.declare_parameter("depth_qos_reliable", True)

        self._bridge = CvBridge()
        self._fx = self._fy = self._cx = self._cy = None
        self._depth_w = self._depth_h = None
        self._rgb_w = self._rgb_h = None
        self._wheel_names: Set[str] = set()
        self._update_wheel_names()
        self._warned_no_intrinsics = False
        self._tf_warn_last = 0.0
        self._last_depth_msg: Optional[Image] = None
        self._dbg_sync_last = 0.0
        self._warned_no_depth_yet = False

        self._output_frame = self.get_parameter("output_frame").value
        self._camera_optical = self.get_parameter("camera_optical_frame").value
        self._patch_half = int(self.get_parameter("patch_half_size").value)

        from tf2_ros import Buffer, TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Ultralytics publishes ObjectsSegment with default RELIABLE / depth 10.
        qos_seg = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        if _param_bool(self, "depth_qos_reliable"):
            qos_depth = rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            )
        else:
            qos_depth = rclpy.qos.qos_profile_sensor_data
        qos_be = rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self._pub_poses = self.create_publisher(PoseArray, self.get_parameter("pose_output_topic").value, 10)
        self._pub_markers = self.create_publisher(MarkerArray, self.get_parameter("markers_topic").value, 10)

        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self._on_cam_info, 10)
        self.create_subscription(Image, self.get_parameter("rgb_topic").value, self._on_rgb, qos_be)

        mode = str(self.get_parameter("depth_pairing_mode").value or "latest").strip().lower()
        self._pairing_mode = mode if mode in ("latest", "approximate") else "latest"

        if self._pairing_mode == "latest":
            self.create_subscription(
                Image,
                self.get_parameter("depth_topic").value,
                self._on_depth_cache,
                qos_depth,
            )
            self.create_subscription(
                ObjectsSegment,
                self.get_parameter("objects_segment_topic").value,
                self._on_segment_with_latest_depth,
                qos_seg,
            )
            self.get_logger().info(
                f"tyre_3d_projection: mode=latest (depth cache + ObjectsSegment) "
                f"{self.get_parameter('objects_segment_topic').value} + "
                f"{self.get_parameter('depth_topic').value} -> {self.get_parameter('pose_output_topic').value} "
                f"frame={self._output_frame}"
            )
        else:
            import message_filters
            from message_filters import ApproximateTimeSynchronizer, Subscriber

            slop = float(self.get_parameter("sync_slop_s").value)
            queue = 8
            seg_sub = Subscriber(
                self,
                ObjectsSegment,
                self.get_parameter("objects_segment_topic").value,
                qos_profile=qos_seg,
            )
            depth_sub = Subscriber(
                self,
                Image,
                self.get_parameter("depth_topic").value,
                qos_profile=qos_depth,
            )
            self._sync = ApproximateTimeSynchronizer([seg_sub, depth_sub], queue, slop)
            self._sync.registerCallback(self._on_sync)

            self.get_logger().info(
                f"tyre_3d_projection: mode=approximate slop={slop}s "
                f"{self.get_parameter('objects_segment_topic').value} + "
                f"{self.get_parameter('depth_topic').value} -> {self.get_parameter('pose_output_topic').value} "
                f"frame={self._output_frame}"
            )

    def _update_wheel_names(self) -> None:
        raw = self.get_parameter("wheel_class_names").value
        if isinstance(raw, list):
            self._wheel_names = {str(x).strip().lower() for x in raw if str(x).strip()}
        else:
            self._wheel_names = {"wheel", "tire", "tyre"}

    def _on_cam_info(self, msg: CameraInfo) -> None:
        if msg.k is not None and len(msg.k) >= 9:
            self._fx = float(msg.k[0])
            self._fy = float(msg.k[4])
            self._cx = float(msg.k[2])
            self._cy = float(msg.k[5])
            self._depth_w = int(msg.width)
            self._depth_h = int(msg.height)

    def _on_rgb(self, msg: Image) -> None:
        self._rgb_w = int(msg.width)
        self._rgb_h = int(msg.height)

    def _on_depth_cache(self, msg: Image) -> None:
        self._last_depth_msg = msg

    def _on_segment_with_latest_depth(self, seg_msg: ObjectsSegment) -> None:
        if self._last_depth_msg is None:
            if not self._warned_no_depth_yet:
                self._warned_no_depth_yet = True
                self.get_logger().warn(
                    "tyre_3d_projection: no depth image received yet; waiting for "
                    f"{self.get_parameter('depth_topic').value}"
                )
            return
        depth_msg = self._last_depth_msg
        slop = float(self.get_parameter("sync_slop_s").value)
        ts = Time.from_msg(seg_msg.header.stamp)
        td = Time.from_msg(depth_msg.header.stamp)
        dt = abs((ts - td).nanoseconds / 1e9)
        if dt > slop:
            self.get_logger().warning(
                f"tyre_3d_projection: |seg−depth| stamp delta {dt:.3f}s > slop {slop}s; projecting anyway (latest mode)",
                throttle_duration_sec=10.0,
            )
        self._on_sync(seg_msg, depth_msg)

    def _on_sync(self, seg_msg: ObjectsSegment, depth_msg: Image) -> None:
        if self._fx is None or self._depth_w is None:
            if not self._warned_no_intrinsics:
                self._warned_no_intrinsics = True
                self.get_logger().warn(
                    "tyre_3d_projection: no CameraInfo yet on "
                    f"{self.get_parameter('camera_info_topic').value} — cannot project; "
                    "check aurora_depth_camera_info_node and /camera/depth/camera_info."
                )
            return
        if seg_msg.objects:
            wall = time.time()
            if wall - self._dbg_sync_last > 3.0:
                self._dbg_sync_last = wall
                ts = Time.from_msg(seg_msg.header.stamp)
                td = Time.from_msg(depth_msg.header.stamp)
                ddt = abs((ts - td).nanoseconds / 1e9)
                self.get_logger().info(
                    f"tyre_3d_projection: mode={self._pairing_mode} |seg−depth|={ddt:.3f}s "
                    f"objects={len(seg_msg.objects)}"
                )
        try:
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"depth cv_bridge: {e}")
            return

        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        dh, dw = depth_m.shape[:2]
        rgb_w = self._rgb_w or dw
        rgb_h = self._rgb_h or dh
        sx = float(dw) / float(rgb_w) if rgb_w > 0 else 1.0
        sy = float(dh) / float(rgb_h) if rgb_h > 0 else 1.0

        dmin = float(self.get_parameter("depth_min_m").value)
        dmax = float(self.get_parameter("depth_max_m").value)
        ph = max(0, self._patch_half)

        poses_out: List[Pose] = []
        markers: List[Marker] = []
        stamp = seg_msg.header.stamp
        t = Time.from_msg(stamp)
        use_latest_tf = _param_bool(self, "tf_lookup_use_latest")

        for idx, obj in enumerate(seg_msg.objects):
            name = (obj.class_name or "").strip().lower()
            if self._wheel_names and name not in self._wheel_names:
                continue
            if not obj.x_indices or not obj.y_indices or len(obj.x_indices) != len(obj.y_indices):
                continue
            cx_rgb = float(np.mean(np.asarray(obj.x_indices, dtype=np.float64)))
            cy_rgb = float(np.mean(np.asarray(obj.y_indices, dtype=np.float64)))
            u = int(round(cx_rgb * sx))
            v = int(round(cy_rgb * sy))
            u = max(ph, min(dw - 1 - ph, u))
            v = max(ph, min(dh - 1 - ph, v))
            patch = depth_m[v - ph : v + ph + 1, u - ph : u + ph + 1]
            z = _median_valid_depth(patch)
            if not math.isfinite(z) or z < dmin or z > dmax:
                continue

            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
            X = (float(u) - cx) * z / fx
            Y = (float(v) - cy) * z / fy
            Z = float(z)

            ps = PointStamped()
            ps.header.frame_id = self._camera_optical
            ps.header.stamp = stamp
            ps.point.x = X
            ps.point.y = Y
            ps.point.z = Z

            try:
                tf_msg = None
                if use_latest_tf:
                    try:
                        tf_msg = self._tf_buffer.lookup_transform(
                            self._output_frame,
                            self._camera_optical,
                            Time(),
                            timeout=Duration(seconds=0.75),
                        )
                    except TransformException:
                        tf_msg = None
                if tf_msg is None:
                    try:
                        tf_msg = self._tf_buffer.lookup_transform(
                            self._output_frame,
                            self._camera_optical,
                            t,
                            timeout=Duration(seconds=0.75),
                        )
                    except TransformException:
                        tf_msg = self._tf_buffer.lookup_transform(
                            self._output_frame,
                            self._camera_optical,
                            Time(),
                            timeout=Duration(seconds=0.75),
                        )
                out_pt = tf2_geometry_msgs.do_transform_point(ps, tf_msg)
            except TransformException as e:
                wall = time.time()
                if wall - self._tf_warn_last > 5.0:
                    self._tf_warn_last = wall
                    self.get_logger().warning(
                        f"TF {self._camera_optical}->{self._output_frame} at sensor time: {e}"
                    )
                continue

            p = Pose()
            p.position.x = out_pt.point.x
            p.position.y = out_pt.point.y
            p.position.z = out_pt.point.z
            p.orientation.w = 1.0
            poses_out.append(p)

            m = Marker()
            m.header.frame_id = self._output_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "tyre_3d"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = p.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.85)
            m.lifetime.sec = 2
            markers.append(m)

        if not poses_out:
            return

        self.get_logger().info(
            f"tyre_3d_projection: publishing {len(poses_out)} pose(s) on {self.get_parameter('pose_output_topic').value}",
            throttle_duration_sec=2.0,
        )

        arr = PoseArray()
        arr.header.frame_id = self._output_frame
        arr.header.stamp = stamp
        arr.poses = poses_out
        self._pub_poses.publish(arr)

        ma = MarkerArray()
        ma.markers = markers
        self._pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = Tyre3dProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
