#!/usr/bin/env python3
"""
CPU-based tire detection node using ONNX model.
Runs entirely on CPU to avoid CUDA OOM when GPU is saturated by Aurora/depth/costmaps.
Publishes to /ultralytics_tire/segmentation/objects_segment (same topic as GPU node).
Use when use_cpu_inference:=true in launch.
"""
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from segmentation_msgs.msg import ObjectSegment, ObjectsSegment

# Ultralytics YOLO with ONNX model - uses onnxruntime for inference when model is .onnx
from ultralytics import YOLO

TIRE_CLASSES = frozenset({"tire", "tyre", "wheel", "car_tire", "car_tyre", "car-tire", "car-tyre"})


def _is_tire_class(name: str) -> bool:
    return (name or "").strip().lower() in TIRE_CLASSES


class UltralyticsNodeCPU(Node):
    """Tire detection using ONNX model on CPU. Zero GPU memory."""

    def __init__(self):
        super().__init__("ultralytics_tire_cpu")

        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_rgb_topic", "/slamware_ros_sdk_server_node/left_image_raw")
        self.declare_parameter("objects_segment_topic", "/ultralytics_tire/segmentation/objects_segment")
        self.declare_parameter("imgsz", 224)
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("inference_interval_s", 0.2)  # 5 Hz
        self.declare_parameter("interested_class_names", ["wheel"])

        model_path = self.get_parameter("model_path").value
        if not model_path or not os.path.isfile(model_path):
            for cand in [
                os.path.expanduser("~/ugv_ws/src/Tyre_Inspection_Bot/best_fallback.onnx"),
                os.path.expanduser("~/ugv_ws/best_fallback.onnx"),
            ]:
                if os.path.isfile(cand):
                    model_path = cand
                    break
        if not model_path or not os.path.isfile(model_path):
            self.get_logger().error(
                "ONNX model not found. Run: bash scripts/export_onnx.sh"
            )
            raise FileNotFoundError("best_fallback.onnx not found")

        self.get_logger().info(f"Loading ONNX model from {model_path} (CPU only)")
        self.model = YOLO(model_path)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._conf = float(self.get_parameter("conf_threshold").value)
        self._iou = float(self.get_parameter("iou_threshold").value)
        self._interval_s = float(self.get_parameter("inference_interval_s").value)
        interested = self.get_parameter("interested_class_names").value or []
        self._interested = {str(c).strip().lower() for c in interested if str(c).strip()}

        self.bridge = CvBridge()
        self._last_inference_time = 0.0
        self._last_status_log_time = 0.0
        self._status_interval_s = 5.0

        camera_topic = self.get_parameter("camera_rgb_topic").value
        out_topic = self.get_parameter("objects_segment_topic").value

        self.sub = self.create_subscription(Image, camera_topic, self._callback, 10)
        self.pub = self.create_publisher(ObjectsSegment, out_topic, 10)

        self.get_logger().info(
            f"UltralyticsNodeCPU: camera={camera_topic} -> {out_topic} "
            f"(imgsz={self._imgsz} conf={self._conf} interval={self._interval_s}s)"
        )

    def _callback(self, msg: Image) -> None:
        now = time.time()
        if now - self._last_inference_time < self._interval_s:
            return
        self._last_inference_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        height, width = cv_image.shape[:2]
        if now - self._last_status_log_time >= self._status_interval_s:
            self._last_status_log_time = now
            self.get_logger().info(f"CPU inference: image {width}x{height}")

        try:
            results = self.model.predict(
                cv_image,
                device="cpu",
                conf=self._conf,
                iou=self._iou,
                imgsz=self._imgsz,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return

        if not results or len(results) == 0:
            return

        r = results[0]
        names = r.names or {}
        num_classes = len(names)

        objects_msg = ObjectsSegment()
        objects_msg.header = msg.header
        objects_msg.header.stamp = self.get_clock().now().to_msg()

        if r.boxes is None:
            self.pub.publish(objects_msg)
            return

        has_masks = r.masks is not None
        for idx in range(len(r.boxes)):
            cls_id = int(r.boxes.cls[idx].cpu().numpy())
            if cls_id < 0 or cls_id >= num_classes:
                continue
            name = names.get(cls_id, f"class_{cls_id}")
            if self._interested and str(name).strip().lower() not in self._interested:
                continue
            if not _is_tire_class(str(name)):
                continue

            conf = float(r.boxes.conf[idx].item())
            xyxy = r.boxes.xyxy[idx].cpu().numpy()
            x1, y1, x2, y2 = xyxy

            if has_masks:
                mask = r.masks.data.cpu().numpy()[idx, :, :]
                mask_resized = cv2.resize(mask, (width, height))
                binary = (mask_resized > 0.5).astype(np.uint8)
                y_indices, x_indices = np.where(binary > 0)
            else:
                x1i = max(0, int(x1))
                y1i = max(0, int(y1))
                x2i = min(width - 1, int(x2))
                y2i = min(height - 1, int(y2))
                if x2i <= x1i or y2i <= y1i:
                    continue
                box_w, box_h = x2i - x1i + 1, y2i - y1i + 1
                stride = 1 if max(box_w, box_h) <= 120 else (2 if max(box_w, box_h) <= 240 else 3)
                xs = np.arange(x1i, x2i + 1, stride)
                ys = np.arange(y1i, y2i + 1, stride)
                gx, gy = np.meshgrid(xs, ys)
                x_indices = gx.ravel()
                y_indices = gy.ravel()

            if len(x_indices) == 0 or len(y_indices) == 0:
                continue

            obj = ObjectSegment()
            obj.header = objects_msg.header
            obj.class_name = str(name)
            obj.probability = conf
            obj.x_indices = x_indices.tolist()
            obj.y_indices = y_indices.tolist()
            objects_msg.objects.append(obj)

        if objects_msg.objects:
            self.pub.publish(objects_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltralyticsNodeCPU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
