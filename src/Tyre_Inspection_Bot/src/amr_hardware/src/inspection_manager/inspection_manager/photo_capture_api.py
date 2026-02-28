#!/usr/bin/env python3
"""Live camera-backed service endpoint for `/photo_capture/capture`."""

import json
import os
from datetime import datetime, timezone
from typing import Optional

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from inspection_manager_interfaces.srv import CapturePhoto


class PhotoCaptureApi(Node):
    def __init__(self) -> None:
        super().__init__("photo_capture_api")
        self.declare_parameter("output_root", "research/data/photos")
        self.declare_parameter("image_topic", "/slamware_ros_sdk_server_node/left_image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("max_image_age_s", 1.0)
        self.output_root = str(self.get_parameter("output_root").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.max_image_age_s = float(self.get_parameter("max_image_age_s").value)

        self._bridge = CvBridge()
        self._last_image_msg: Optional[Image] = None
        self._last_camera_info: Optional[CameraInfo] = None
        self._last_image_walltime = 0.0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_camera_info, 10)
        self.srv = self.create_service(CapturePhoto, "/photo_capture/capture", self._handle_capture)
        self.get_logger().info(f"photo_capture API ready on /photo_capture/capture from {self.image_topic}")

    @staticmethod
    def _estimate_center_coverage(frame) -> float:
        h, w = frame.shape[:2]
        cx0 = int(0.25 * w)
        cx1 = int(0.75 * w)
        cy0 = int(0.25 * h)
        cy1 = int(0.75 * h)
        roi = frame[cy0:cy1, cx0:cx1]
        if roi.size == 0:
            return 0.0
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 160)
        return float((edges > 0).sum()) / float(edges.size)

    def _on_image(self, msg: Image) -> None:
        self._last_image_msg = msg
        self._last_image_walltime = self.get_clock().now().nanoseconds / 1e9

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._last_camera_info = msg

    def _tf_to_dict(self, tf: TransformStamped) -> dict:
        return {
            "translation": {
                "x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "z": tf.transform.translation.z,
            },
            "rotation": {
                "x": tf.transform.rotation.x,
                "y": tf.transform.rotation.y,
                "z": tf.transform.rotation.z,
                "w": tf.transform.rotation.w,
            },
        }

    def _handle_capture(self, req: CapturePhoto.Request, res: CapturePhoto.Response):
        mission_id = req.mission_id or "unknown_mission"
        object_id = req.object_id or "unknown_object"
        tire_id = req.tire_id or "unknown_tire"
        ts = datetime.now(timezone.utc).isoformat()
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self._last_image_msg is None or (now_s - self._last_image_walltime) > self.max_image_age_s:
            res.ok = False
            res.file_path = ""
            res.metadata = json.dumps({"reason": "no_fresh_camera_image"})
            return res

        out_dir = os.path.join(self.output_root, mission_id)
        os.makedirs(out_dir, exist_ok=True)
        stamp_ms = int(self.get_clock().now().nanoseconds / 1e6)
        file_path = os.path.join(out_dir, f"{tire_id}_{stamp_ms}.png")
        try:
            frame = self._bridge.imgmsg_to_cv2(self._last_image_msg, desired_encoding="bgr8")
            ok = cv2.imwrite(file_path, frame)
            if not ok:
                raise RuntimeError("cv2.imwrite failed")
        except Exception as exc:
            res.ok = False
            res.file_path = ""
            res.metadata = json.dumps({"reason": f"capture_write_failed:{exc}"})
            return res

        camera_pose_map = {}
        object_pose_map = {}
        try:
            tf_cam = self._tf_buffer.lookup_transform("map", self.camera_frame, rclpy.time.Time())
            camera_pose_map = self._tf_to_dict(tf_cam)
        except Exception:
            camera_pose_map = {}
        try:
            tf_obj = self._tf_buffer.lookup_transform("map", object_id, rclpy.time.Time())
            object_pose_map = self._tf_to_dict(tf_obj)
        except Exception:
            object_pose_map = {}

        metadata = {
            "mission_id": mission_id,
            "vehicle_id": object_id,
            "tire_id": tire_id,
            "timestamp": ts,
            "pose": camera_pose_map,
            "object_pose_map": object_pose_map,
            "camera_intrinsics": {
                "k": list(self._last_camera_info.k) if self._last_camera_info is not None else [],
                "d": list(self._last_camera_info.d) if self._last_camera_info is not None else [],
                "width": int(self._last_camera_info.width) if self._last_camera_info is not None else 0,
                "height": int(self._last_camera_info.height) if self._last_camera_info is not None else 0,
            },
            "projection_overlap": 0.8,
            "tire_center_coverage": self._estimate_center_coverage(frame),
        }
        with open(os.path.join(out_dir, f"{tire_id}.json"), "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)

        res.file_path = file_path
        res.metadata = json.dumps(metadata)
        res.ok = True
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoCaptureApi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
