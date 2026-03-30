#!/usr/bin/env python3
"""
Creates an organized (image-aligned) PointCloud2 from Aurora depth_image_raw.
Pixel (x,y) maps to point at index (y * width + x). Output is PointXYZ only
(matching what the segmentation processor needs). Requires camera_info for
intrinsics (fx, fy, cx, cy).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import struct


class DepthToRegisteredPointCloudNode(Node):
    def __init__(self):
        super().__init__("depth_to_registered_pointcloud")
        self.declare_parameter("depth_topic", "/slamware_ros_sdk_server_node/depth_image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "/segmentation_processor/registered_pointcloud")
        self.declare_parameter("output_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("depth_points_topic", "/camera/depth/points")  # also publish here for Nav2 costmap
        self.declare_parameter("publish_rate_hz", 0.0)  # 0 = publish every frame; else max rate (Hz)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.depth_points_topic = self.get_parameter("depth_points_topic").value

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self._cam_info_width = self._cam_info_height = None
        self.camera_info_received = False
        self._first_frame_logged = False

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.depth_sub = self.create_subscription(Image, self.depth_topic, self._depth_cb, qos_rel)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_cb, 10
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_be)
        self.pub_depth_points = self.create_publisher(PointCloud2, self.depth_points_topic, qos_be) if self.depth_points_topic else None
        _pr = self.get_parameter("publish_rate_hz").value
        if isinstance(_pr, int):
            _pr = float(_pr)
        self._publish_rate_hz = float(_pr if _pr is not None else 0.0)
        self._last_publish_time = None

        self.get_logger().info(
            f"depth_to_registered_pointcloud: depth={self.depth_topic} "
            f"camera_info={self.camera_info_topic} -> {self.output_topic} "
            f"publish_rate_hz={self._publish_rate_hz}"
        )

    def _camera_info_cb(self, msg: CameraInfo):
        if msg.k is not None and len(msg.k) >= 9:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self._cam_info_width = msg.width
            self._cam_info_height = msg.height
            self.camera_info_received = True

    def _depth_cb(self, msg: Image):
        if not self.camera_info_received or self.fx is None:
            return
        if self._publish_rate_hz > 0.0:
            now = self.get_clock().now()
            min_interval_ns = int(1e9 / self._publish_rate_hz)
            if self._last_publish_time is not None:
                dt_ns = (now - self._last_publish_time).nanoseconds
                if dt_ns < min_interval_ns:
                    return
            self._last_publish_time = now

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        h, w = depth.shape
        # Intrinsics mismatch check
        if self._cam_info_width is not None and (w != self._cam_info_width or h != self._cam_info_height):
            self.get_logger().error(
                f"INTRINSICS MISMATCH: depth {w}x{h} != camera_info {self._cam_info_width}x{self._cam_info_height}. "
                "Point cloud geometry will be incorrect. Fix CameraInfo resolution."
            )
        if not self._first_frame_logged:
            self.get_logger().info(
                f"Depth resolution: {w}x{h}, camera_info: {self._cam_info_width}x{self._cam_info_height}"
            )
            self._first_frame_logged = True
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        elif depth.dtype != np.float32:
            depth = depth.astype(np.float32)
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy

        # Organized cloud: row-major, point at (u,v) has index v*width+u
        out = np.full((h * w, 3), np.nan, dtype=np.float32)
        valid = ~np.isnan(depth) & ~np.isinf(depth) & (depth > 0)
        vv, uu = np.where(valid)
        zz = depth[valid].astype(np.float32)
        xx = (uu.astype(np.float32) - cx) * zz / fx
        yy = (vv.astype(np.float32) - cy) * zz / fy
        idx = vv * w + uu
        out[idx, 0] = xx
        out[idx, 1] = yy
        out[idx, 2] = zz

        # Build PointCloud2
        pc_msg = PointCloud2()
        pc_msg.header = msg.header
        pc_msg.header.frame_id = self.output_frame_id
        pc_msg.height = h
        pc_msg.width = w
        pc_msg.is_dense = False
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12  # 3 * float32
        pc_msg.row_step = pc_msg.point_step * w
        pc_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.data = out.tobytes()
        self.pub.publish(pc_msg)
        if self.pub_depth_points is not None:
            self.pub_depth_points.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToRegisteredPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
