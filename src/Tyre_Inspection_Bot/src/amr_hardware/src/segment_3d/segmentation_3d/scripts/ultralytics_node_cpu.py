#!/usr/bin/env python3
"""
CPU-based tire detection node using ONNX model.
Runs entirely on CPU to avoid CUDA OOM when GPU is saturated by Aurora/depth/costmaps.
Publishes to /ultralytics_tire/segmentation/objects_segment (same topic as GPU node).
Use when use_cpu_inference:=true in launch.
"""
import os
import time
from typing import Any, Dict, Tuple

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


def _parse_imgsz(raw: Any) -> int:
    """ROS may pass imgsz as str/float; ONNX expects a fixed square side."""
    if raw is None:
        return 480
    if isinstance(raw, bool):
        return 480
    if isinstance(raw, (int, float)):
        return max(32, int(round(float(raw))))
    s = str(raw).strip().lower()
    try:
        return max(32, int(round(float(s))))
    except ValueError:
        return 480


class UltralyticsNodeCPU(Node):
    """Tire detection using ONNX model on CPU. Zero GPU memory."""

    def __init__(self):
        super().__init__("ultralytics_tire_cpu")

        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_rgb_topic", "/slamware_ros_sdk_server_node/left_image_raw")
        self.declare_parameter("objects_segment_topic", "/ultralytics_tire/segmentation/objects_segment")
        self.declare_parameter("segmentation_image_topic", "/ultralytics_tire/segmentation/image")
        self.declare_parameter("imgsz", 480)
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("inference_interval_s", 0.2)  # 5 Hz
        self.declare_parameter("interested_class_names", ["wheel"])

        model_path = self.get_parameter("model_path").value
        if not model_path or not os.path.isfile(model_path):
            workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
            for cand in [
                os.path.join(workspace, "src", "Tyre_Inspection_Bot", "best_fallback.onnx"),
                os.path.join(workspace, "best_fallback.onnx"),
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
        self._imgsz = _parse_imgsz(self.get_parameter("imgsz").value)
        self.get_logger().info(
            f"ONNX CPU inference square size imgsz={self._imgsz} (must match exported ONNX, e.g. export_onnx.sh)"
        )
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
        seg_img_topic = (self.get_parameter("segmentation_image_topic").value or "").strip()

        self.sub = self.create_subscription(Image, camera_topic, self._callback, 10)
        self.pub = self.create_publisher(ObjectsSegment, out_topic, 10)
        self._img_pub = (
            self.create_publisher(Image, seg_img_topic, 10) if seg_img_topic else None
        )

        self.get_logger().info(
            f"UltralyticsNodeCPU: camera={camera_topic} -> {out_topic} "
            f"(imgsz={self._imgsz} conf={self._conf} interval={self._interval_s}s)"
        )

    @staticmethod
    def _letterbox_bgr(img: np.ndarray, imgsz: int) -> Tuple[np.ndarray, Dict[str, float]]:
        """YOLO-style letterbox: scale to fit inside imgsz×imgsz, pad with gray 114 (BGR).

        Returns (canvas_bgr, meta) for mapping model outputs back to the original resolution.
        """
        h, w = img.shape[:2]
        if h <= 0 or w <= 0:
            raise ValueError("empty image")
        r = min(imgsz / float(h), imgsz / float(w))
        new_w = max(1, int(round(w * r)))
        new_h = max(1, int(round(h * r)))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        pad_w = imgsz - new_w
        pad_h = imgsz - new_h
        pad_left = pad_w // 2
        pad_top = pad_h // 2
        canvas = np.full((imgsz, imgsz, 3), 114, dtype=np.uint8)
        canvas[pad_top : pad_top + new_h, pad_left : pad_left + new_w] = resized
        meta: Dict[str, float] = {
            "r": float(r),
            "pad_left": float(pad_left),
            "pad_top": float(pad_top),
            "new_w": float(new_w),
            "new_h": float(new_h),
            "orig_w": float(w),
            "orig_h": float(h),
            "imgsz": float(imgsz),
        }
        return canvas, meta

    @staticmethod
    def _xyxy_letterbox_to_orig(xyxy: np.ndarray, meta: Dict[str, float]) -> np.ndarray:
        """Map boxes from letterboxed canvas coordinates to original camera pixels."""
        r = meta["r"]
        pl = meta["pad_left"]
        pt = meta["pad_top"]
        ow = meta["orig_w"]
        oh = meta["orig_h"]
        out = xyxy.astype(np.float64).copy()
        out[0] = (out[0] - pl) / r
        out[2] = (out[2] - pl) / r
        out[1] = (out[1] - pt) / r
        out[3] = (out[3] - pt) / r
        out[0] = np.clip(out[0], 0.0, ow - 1e-6)
        out[2] = np.clip(out[2], 0.0, ow - 1e-6)
        out[1] = np.clip(out[1], 0.0, oh - 1e-6)
        out[3] = np.clip(out[3], 0.0, oh - 1e-6)
        return out.astype(np.float32)

    @staticmethod
    def _mask_letterbox_to_orig(
        mask: np.ndarray, meta: Dict[str, float], width: int, height: int
    ) -> np.ndarray:
        """Map a mask on the letterboxed square to original image size."""
        imgsz = int(meta["imgsz"])
        mh, mw = mask.shape[:2]
        if mh != imgsz or mw != imgsz:
            mask = cv2.resize(mask, (imgsz, imgsz), interpolation=cv2.INTER_LINEAR)
        pl, pt = int(meta["pad_left"]), int(meta["pad_top"])
        nw, nh = int(meta["new_w"]), int(meta["new_h"])
        crop = mask[pt : pt + nh, pl : pl + nw]
        if crop.size == 0:
            return np.zeros((height, width), dtype=np.float32)
        return cv2.resize(crop, (width, height), interpolation=cv2.INTER_LINEAR)

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
        try:
            lb, lb_meta = self._letterbox_bgr(cv_image, self._imgsz)
        except Exception as e:
            self.get_logger().error(f"letterbox failed: {e}")
            return

        if now - self._last_status_log_time >= self._status_interval_s:
            self._last_status_log_time = now
            self.get_logger().info(
                f"CPU inference: image {width}x{height} -> letterbox ONNX {self._imgsz}x{self._imgsz} "
                f"(scale={lb_meta['r']:.4f})"
            )

        try:
            # Input is already imgsz×imgsz; imgsz matches exported ONNX static shape.
            results = self.model.predict(
                lb,
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
        if self._img_pub is not None:
            try:
                plotted = r.plot()
                img_msg = self.bridge.cv2_to_imgmsg(plotted, "bgr8")
                img_msg.header = msg.header
                self._img_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().debug(f"segmentation image publish skipped: {e}")
        names = r.names or {}
        num_classes = len(names)

        objects_msg = ObjectsSegment()
        # Keep RGB/camera stamp (do not replace with now()) so tyre_3d_projection_node can
        # ApproximateTime-sync ObjectsSegment with depth_image_raw from the same capture clock.
        objects_msg.header = msg.header

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
            xyxy = self._xyxy_letterbox_to_orig(xyxy, lb_meta)
            x1, y1, x2, y2 = xyxy

            if has_masks:
                mask = r.masks.data.cpu().numpy()[idx, :, :]
                mask_resized = self._mask_letterbox_to_orig(mask, lb_meta, width, height)
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
