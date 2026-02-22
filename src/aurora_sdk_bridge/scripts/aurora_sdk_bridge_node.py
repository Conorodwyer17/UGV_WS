#!/usr/bin/env python3
"""
Aurora SDK Bridge — Depth pipeline for classic Aurora (client-side stereo).

Subscribes to slamware raw stereo images, computes depth via fisheye stereo,
publishes /camera/depth/image, /camera/depth/camera_info, /camera/depth/points,
and /stereo/navigation_permitted for depth_gate.

Uses equidistant (fisheye) calibration. Replace config with checkerboard result
for production. See docs/PHASE1_CALIBRATION_ANALYSIS.md.
"""

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import cv2
import yaml
import os
import time
from std_msgs.msg import Header


class AuroraSdkBridgeNode(Node):
    def __init__(self):
        super().__init__("aurora_sdk_bridge")

        self.declare_parameter("left_image_topic", "/slamware_ros_sdk_server_node/left_image_raw")
        self.declare_parameter("right_image_topic", "/slamware_ros_sdk_server_node/right_image_raw")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("frame_id", "camera_depth_optical_frame")
        self.declare_parameter("min_disparity", 0)
        self.declare_parameter("num_disparities", 64)  # 16*4
        self.declare_parameter("block_size", 5)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("max_stereo_sync_delta_ms", 80.0)  # Phase E

        self.left_topic = self.get_parameter("left_image_topic").value
        self.right_topic = self.get_parameter("right_image_topic").value
        self.calib_file = self.get_parameter("calibration_file").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = self.get_parameter("publish_rate").value

        self.bridge = CvBridge()
        self._left = None
        self._right = None
        self._left_ts = None
        self._rectify_ready = False
        self._map1_left = self._map2_left = None
        self._map1_right = self._map2_right = None
        self._Q = None
        self._P1 = None
        self._stereo = None
        self._K_depth = None

        # Load calibration
        if not self._load_calibration():
            self.get_logger().warn("Calibration failed; depth will be approximate. Run checkerboard calibration.")

        self._depth_pub = self.create_publisher(Image, "/camera/depth/image", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
        self._pc_pub = self.create_publisher(PointCloud2, "/camera/depth/points", 10)
        self._nav_permitted_pub = self.create_publisher(Bool, "/stereo/navigation_permitted", 10)

        qos_be = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub_left = Subscriber(self, Image, self.left_topic, qos_profile=qos_be)
        self._sub_right = Subscriber(self, Image, self.right_topic, qos_profile=qos_be)
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_left, self._sub_right],
            queue_size=10,
            slop=0.1,
        )
        self._sync.registerCallback(self._stereo_cb)

        self._timer = self.create_timer(1.0 / self.publish_rate, self._publish_nav_permitted)
        self._last_depth_valid = False
        self._max_sync_ms = self.get_parameter("max_stereo_sync_delta_ms").value
        self._desync_count = 0
        self._last_metrics_time = 0.0

        self.get_logger().info(
            f"aurora_sdk_bridge: {self.left_topic} + {self.right_topic} -> "
            f"/camera/depth/image, /camera/depth/points, /stereo/navigation_permitted"
        )

    def _load_calibration(self) -> bool:
        """Load equidistant stereo calibration from YAML."""
        if not self.calib_file or not os.path.isfile(self.calib_file):
            pkg = get_package_share_directory("aurora_sdk_bridge")
            path = os.path.join(pkg, "config", "equidistant_calibration.yaml")
            if os.path.isfile(path):
                self.calib_file = path
            else:
                self.get_logger().warn("No calibration file; using Aurora specs placeholder")
                return self._use_placeholder_calibration()

        with open(self.calib_file) as f:
            c = yaml.safe_load(f)
        w = c.get("image_width", 640)
        h = c.get("image_height", 480)
        baseline = c.get("baseline_m", 0.06)
        Kl = np.array(c["left"]["camera_matrix"], dtype=np.float64).reshape(3, 3)
        Kr = np.array(c["right"]["camera_matrix"], dtype=np.float64).reshape(3, 3)
        Dl = np.array(c["left"]["dist_coeffs"], dtype=np.float64)
        Dr = np.array(c["right"]["dist_coeffs"], dtype=np.float64)
        R = np.array(c.get("rotation", np.eye(3).tolist()), dtype=np.float64).reshape(3, 3)
        T = np.array(c.get("translation", [-baseline, 0, 0]), dtype=np.float64)

        try:
            R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
                Kl, Dl, Kr, Dr, (w, h), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY,
                balance=1.0,
                fov_scale=1.0,
            )
        except Exception as e:
            self.get_logger().error(f"fisheye.stereoRectify failed: {e}")
            return self._use_placeholder_calibration()

        self._map1_left, self._map2_left = cv2.fisheye.initUndistortRectifyMap(
            Kl, Dl, R1, P1, (w, h), cv2.CV_32FC1
        )
        self._map1_right, self._map2_right = cv2.fisheye.initUndistortRectifyMap(
            Kr, Dr, R2, P2, (w, h), cv2.CV_32FC1
        )
        self._Q = Q
        self._P1 = P1
        self._K_depth = P1[:3, :3]
        self._rectify_ready = True

        self._stereo = cv2.StereoSGBM_create(
            minDisparity=self.get_parameter("min_disparity").value,
            numDisparities=self.get_parameter("num_disparities").value,
            blockSize=self.get_parameter("block_size").value,
            P1=8 * 3 * self.get_parameter("block_size").value ** 2,
            P2=32 * 3 * self.get_parameter("block_size").value ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.get_logger().info("Calibration loaded successfully")
        return True

    def _use_placeholder_calibration(self) -> bool:
        """Placeholder for Aurora 640x480, 6cm baseline, ~180° FOV."""
        w, h = 640, 480
        f = 255.0  # approximate for wide FOV
        cx, cy = 319.5, 239.5
        Kl = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
        Kr = Kl.copy()
        Dl = Dr = np.zeros(4)
        R = np.eye(3)
        T = np.array([-0.06, 0, 0])
        try:
            R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
                Kl, Dl, Kr, Dr, (w, h), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY,
                balance=1.0,
                fov_scale=1.0,
            )
        except Exception as e:
            self.get_logger().error(f"Placeholder stereoRectify failed: {e}")
            return False
        self._map1_left, self._map2_left = cv2.fisheye.initUndistortRectifyMap(
            Kl, Dl, R1, P1, (w, h), cv2.CV_32FC1
        )
        self._map1_right, self._map2_right = cv2.fisheye.initUndistortRectifyMap(
            Kr, Dr, R2, P2, (w, h), cv2.CV_32FC1
        )
        self._Q = Q
        self._P1 = P1
        self._K_depth = P1[:3, :3]
        self._rectify_ready = True
        self._stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=5,
            P1=600,
            P2=2400,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        return True

    def _stereo_cb(self, left_msg: Image, right_msg: Image):
        if not self._rectify_ready or self._stereo is None:
            return
        lt = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec * 1e-9
        rt = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec * 1e-9
        sync_delta_ms = abs(lt - rt) * 1000
        if sync_delta_ms > self._max_sync_ms:
            self._desync_count += 1
            return
        t0 = time.perf_counter()
        try:
            left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="passthrough")
            right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        if left.ndim == 3:
            left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        if right.ndim == 3:
            right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        rl = cv2.remap(left, self._map1_left, self._map2_left, cv2.INTER_LINEAR)
        rr = cv2.remap(right, self._map1_right, self._map2_right, cv2.INTER_LINEAR)
        disp = self._stereo.compute(rl, rr)
        disp = np.clip(disp, 0, 65535).astype(np.uint16)
        # Disparity to depth: z = (f * baseline) / d
        fx = self._P1[0, 0]
        baseline = 0.06
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = (fx * baseline * 1000.0) / np.where(disp > 0, disp.astype(np.float32) / 16.0, np.nan)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth = np.clip(depth, 0, 10000).astype(np.float32)  # mm
        compute_ms = (time.perf_counter() - t0) * 1000
        valid_ratio = np.sum((depth > 0) & (depth < 5000)) / depth.size
        self._last_depth_valid = valid_ratio > 0.01
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_metrics_time > 30.0 and compute_ms > 75:
            self.get_logger().warn(f"compute_time_ms={compute_ms:.1f} > 75 ms")
            self._last_metrics_time = now
        stamp = left_msg.header.stamp
        # Publish depth image (float32 meters for depth_to_registered_pointcloud)
        depth_m = (depth / 1000.0).astype(np.float32)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
        depth_msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        self._depth_pub.publish(depth_msg)
        # Camera info
        info = CameraInfo()
        info.header = depth_msg.header
        info.height, info.width = depth.shape
        info.k = self._K_depth.flatten().tolist()
        info.p = self._P1[:3, :].flatten().tolist()
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        self._info_pub.publish(info)
        # Point cloud
        self._publish_pointcloud(depth_m, stamp)

    def _publish_pointcloud(self, depth: np.ndarray, stamp):
        if self._K_depth is None:
            return
        fx, fy = self._K_depth[0, 0], self._K_depth[1, 1]
        cx, cy = self._K_depth[0, 2], self._K_depth[1, 2]
        h, w = depth.shape
        uu = np.arange(w, dtype=np.float32)
        vv = np.arange(h, dtype=np.float32)
        u, v = np.meshgrid(uu, vv)
        z = depth
        valid = (z > 0.1) & (z < 10.0) & np.isfinite(z)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pts = np.stack([x[valid], y[valid], z[valid]], axis=-1)
        if pts.size == 0:
            return
        # Optical frame: X right, Y down, Z forward
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.height = 1
        msg.width = pts.shape[0]
        msg.is_dense = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts.astype(np.float32).tobytes()
        self._pc_pub.publish(msg)

    def _publish_nav_permitted(self):
        self._nav_permitted_pub.publish(Bool(data=bool(self._last_depth_valid)))


def main(args=None):
    rclpy.init(args=args)
    node = AuroraSdkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
