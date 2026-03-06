#!/usr/bin/env python3
# Shim torchvision.ops.nms before any ultralytics import (NVIDIA torch ABI lacks torchvision::nms)
import sys
import types
import torch as _torch

def _nms_pure(boxes, scores, iou_threshold):
    """Pure PyTorch NMS for compatibility with NVIDIA torch build (no torchvision ABI)."""
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort(descending=True)
    keep = []
    while order.numel() > 0:
        i = order[0].item()
        keep.append(i)
        if order.numel() == 1:
            break
        xx1 = _torch.maximum(boxes[order[0], 0].unsqueeze(0), boxes[order[1:], 0])
        yy1 = _torch.maximum(boxes[order[0], 1].unsqueeze(0), boxes[order[1:], 1])
        xx2 = _torch.minimum(boxes[order[0], 2].unsqueeze(0), boxes[order[1:], 2])
        yy2 = _torch.minimum(boxes[order[0], 3].unsqueeze(0), boxes[order[1:], 3])
        w = (xx2 - xx1).clamp(min=0)
        h = (yy2 - yy1).clamp(min=0)
        inter = w * h
        iou = inter / (areas[order[0]] + areas[order[1:]] - inter + 1e-8)
        inds = (iou <= iou_threshold).nonzero(as_tuple=True)[0]
        order = order[inds + 1]
    return _torch.tensor(keep, device=boxes.device, dtype=_torch.long)

_tv_ops = types.SimpleNamespace(nms=_nms_pure)

class _Compose:
    def __init__(self, transforms):
        self.transforms = transforms if isinstance(transforms, list) else list(transforms)
    def __call__(self, x):
        for t in self.transforms:
            x = t(x)
        return x

_tv_transforms = types.ModuleType("torchvision.transforms")
_tv_transforms.Compose = _Compose

class _ImageFolder(_torch.utils.data.Dataset):
    """Minimal stub for torchvision.datasets.ImageFolder (NVIDIA torch shim)."""
    def __init__(self, root, transform=None, *args, **kwargs):
        super().__init__()
        self.root = root
        self.transform = transform
    def __len__(self):
        return 0
    def __getitem__(self, index):
        raise IndexError(index)

_tv_datasets = types.ModuleType("torchvision.datasets")
_tv_datasets.ImageFolder = _ImageFolder

_tv = types.ModuleType("torchvision")
_tv.ops = _tv_ops
_tv.transforms = _tv_transforms
_tv.datasets = _tv_datasets

sys.modules["torchvision"] = _tv
sys.modules["torchvision.ops"] = _tv_ops
sys.modules["torchvision.transforms"] = _tv_transforms
sys.modules["torchvision.datasets"] = _tv_datasets

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import ros2_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from segmentation_msgs.msg import ObjectsSegment, ObjectSegment
from ultralytics import YOLO
import cv2
import time
import os
import torch

class UltralyticsSegmentationNode(Node):
    def __init__(self):
        super().__init__('ultralytics_segmentation')
        
        # Declare parameters for model paths
        self.declare_parameter("navigation_model", "yolov8m-seg.pt")  # Model 1 for navigation
        self.declare_parameter("inspection_model", "best_fallback.pt")  # Canonical wheel detection model
        self.declare_parameter("prefer_tensorrt", True)
        self.declare_parameter("prefer_tensorrt_inspection", "auto")  # "auto"|"true"|"false"; auto=use engine if exists
        # device: launch often passes 0 as int from YAML; declare int, convert to str for YOLO
        self.declare_parameter("device", 0)
        self.declare_parameter("mode_topic", "/segmentation_mode")
        self.declare_parameter("default_mode", "navigation")
        self.declare_parameter("camera_rgb_topic", "/slamware_ros_sdk_server_node/left_image_raw")  # Aurora left camera (default)
        self.declare_parameter("objects_segment_topic", "/ultralytics/segmentation/objects_segment")
        self.declare_parameter("segmentation_image_topic", "/ultralytics/segmentation/image")
        self.declare_parameter("subscribe_mode_topic", True)
        self.declare_parameter("fixed_mode", "")  # Optional hard override: "navigation" or "inspection"
        self.declare_parameter("log_raw_detections", False)  # When True, log class_id, class_name, confidence, bbox (for perception validation)
        self.declare_parameter("confidence", 0.25)  # YOLO detection confidence threshold (lower to 0.1 to see weak detections)
        self.declare_parameter("iou", 0.45)  # YOLO NMS IoU threshold
        self.declare_parameter("half", True)  # Use FP16 when GPU; launch may pass as bool or string
        self.declare_parameter("max_det", 50)  # Max detections per image; lower reduces NMS time (avoids "NMS time limit exceeded")
        self.declare_parameter("imgsz", 640)  # Inference input size; 480 or 416 for faster inference on Jetson
        self.declare_parameter("inference_interval_s", 0.1)  # Min seconds between inferences; 10 Hz for 16 GB Jetson
        # Use non-empty default so rclpy infers STRING_ARRAY (empty list would infer BYTE_ARRAY)
        self.declare_parameter(
            "interested_class_names",
            [""],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="If non-empty, only publish these class names (e.g. ['wheel']); default = publish all",
            ),
        )
        self.declare_parameter("use_vehicle_yolo", False)
        self.declare_parameter("model_load_delay_s", 1.0)  # Delay before loading model; 1.0 s for 16 GB Jetson; increase if OOM

        # Parse use_vehicle_yolo (launch may pass string "true"/"false")
        use_vehicle_yolo_param = self.get_parameter("use_vehicle_yolo").value
        if isinstance(use_vehicle_yolo_param, str):
            self.use_vehicle_yolo = use_vehicle_yolo_param.strip().lower() in ("true", "1", "yes")
        else:
            self.use_vehicle_yolo = bool(use_vehicle_yolo_param)

        nav_model_path = self.get_parameter("navigation_model").value
        insp_model_path = self.get_parameter("inspection_model").value
        prefer_trt = bool(self.get_parameter("prefer_tensorrt").value)
        dev_val = self.get_parameter("device").value
        requested = str(dev_val) if dev_val is not None else "0"
        if not torch.cuda.is_available():
            self._device = "cpu"
            self._half = False
            self.get_logger().warn(
                f"CUDA not available (torch.cuda.is_available()=False). Using device=cpu. "
                f"Requested device={requested} ignored."
            )
        else:
            self._device = f"cuda:{requested}" if requested.isdigit() else requested
            half_val = self.get_parameter("half").value
            self._half = half_val if isinstance(half_val, bool) else (str(half_val).lower() == "true")
        self.get_logger().info(f"Using device: {self._device} (half={self._half})")

        # Load navigation model only when use_vehicle_yolo is True (saves GPU memory on Jetson)
        if self.use_vehicle_yolo:
            nav_candidates = []
            if prefer_trt and nav_model_path.endswith(".pt"):
                nav_candidates.append(nav_model_path[:-2] + "engine")
            nav_candidates.append(nav_model_path)
            self.get_logger().info(f"Loading navigation model: {nav_candidates[0]} (fallbacks: {nav_candidates[1:]})")
            nav_loaded = None
            for cand in nav_candidates:
                if os.path.exists(cand):
                    nav_loaded = cand
                    break
            if nav_loaded is None:
                nav_loaded = nav_model_path
            self.navigation_model = YOLO(nav_loaded)
            self.get_logger().info(f"Navigation model loaded: {nav_loaded}")
        else:
            self.navigation_model = None
            self.get_logger().info("Vehicle YOLO disabled – not loading navigation model")

        # Always load inspection model (fallback to navigation only if inspection missing and nav loaded)
        self.get_logger().info(f"Loading inspection model: {insp_model_path}")
        resolved_insp_path = insp_model_path
        if not os.path.exists(insp_model_path):
            workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
            for candidate in [
                os.path.join(workspace, "best_fallback.pt"),
                os.path.join(workspace, "src", "Tyre_Inspection_Bot", "best_fallback.pt"),
                os.path.join(workspace, "best_fallback.engine"),
                os.path.join(workspace, "src", "Tyre_Inspection_Bot", "best_fallback.engine"),
            ]:
                if os.path.exists(candidate):
                    resolved_insp_path = candidate
                    self.get_logger().info(f"Using inspection model at: {resolved_insp_path}")
                    break
        v = self.get_parameter("prefer_tensorrt_inspection").value
        v_str = str(v).strip().lower() if v is not None else ""
        if v_str in ("auto", ""):
            # Auto: use engine if it exists alongside .pt
            prefer_trt_inspection = prefer_trt and resolved_insp_path.endswith(".pt") and os.path.exists(
                resolved_insp_path[:-2] + "engine"
            )
        else:
            prefer_trt_inspection = v if isinstance(v, bool) else (v_str in ("true", "1", "yes"))
        if prefer_trt and prefer_trt_inspection and resolved_insp_path.endswith(".pt"):
            engine_path = resolved_insp_path[:-2] + "engine"
            if os.path.exists(engine_path):
                self.get_logger().info(f"Using TensorRT engine for inspection: {engine_path}")
                resolved_insp_path = engine_path
        # Delay before loading to let other nodes (Aurora, depth) settle; reduces CUDA OOM on Jetson
        delay_s = float(self.get_parameter("model_load_delay_s").value)
        if delay_s > 0 and torch.cuda.is_available():
            self.get_logger().info(f"Delaying model load by {delay_s}s to allow GPU to settle...")
            try:
                torch.cuda.empty_cache()
            except Exception:
                pass
            time.sleep(delay_s)
        # Store .pt path for fallback when engine produces invalid class IDs
        self._inspection_using_engine = resolved_insp_path.endswith(".engine")
        self._inspection_pt_path = (resolved_insp_path[:-7] + ".pt") if self._inspection_using_engine else resolved_insp_path
        if resolved_insp_path.endswith(".pt"):
            self.get_logger().info(f"WHEEL DETECTION: using {os.path.basename(resolved_insp_path)} at {resolved_insp_path}")
        elif resolved_insp_path.endswith(".engine"):
            self.get_logger().info(f"Using TensorRT engine for inspection: {resolved_insp_path}")
        if os.path.exists(resolved_insp_path):
            # Force task='segment' for TensorRT engines to avoid detect misinterpretation (COCO indices)
            if resolved_insp_path.endswith(".engine"):
                self.inspection_model = YOLO(resolved_insp_path, task="segment")
            else:
                self.inspection_model = YOLO(resolved_insp_path)
            try:
                names = getattr(self.inspection_model, "names", None)
                if names is not None:
                    class_list = list(names.values()) if isinstance(names, dict) else list(names)
                    self.get_logger().info(
                        f"Inspection model class names (best_fallback.pt): {class_list}"
                    )
                    _wheel_names = ("wheel", "tire", "tyre", "car-tire", "car_tire", "car-tyre", "car_tyre")
                    wheel_ok = any(str(c).lower() in _wheel_names for c in class_list)
                    if not wheel_ok:
                        self.get_logger().warn(
                            "Inspection model has no wheel-like class (wheel/tire/tyre); "
                            "set wheel_labels and interested_classes to match model output."
                        )
                else:
                    self.get_logger().warn("Inspection model has no .names attribute")
            except Exception as e:
                self.get_logger().warn(f"Could not read inspection model names: {e}")
        else:
            if self.navigation_model is not None:
                self.get_logger().warn(
                    f"Inspection model not found at {insp_model_path} or workspace. "
                    "Falling back to navigation model. Wheel detection may fail (yolov8 has no 'wheel' class)."
                )
                self.inspection_model = self.navigation_model
            else:
                self.inspection_model = None
                self.get_logger().error(
                    "Inspection model not found and no navigation model loaded (use_vehicle_yolo=false). "
                    "Cannot run inference."
                )

        # Initial active model: inspection when default is inspection or when navigation model not loaded
        default_mode = self.get_parameter("default_mode").value
        fixed_mode = str(self.get_parameter("fixed_mode").value).strip().lower()
        if fixed_mode in ("navigation", "inspection"):
            default_mode = fixed_mode
        if default_mode == "inspection" or self.navigation_model is None:
            self.segmentation_model = self.inspection_model
            self.current_mode = "inspection"
        else:
            self.segmentation_model = self.navigation_model
            self.current_mode = "navigation"

        self.get_logger().info(f"Initial segmentation mode: {self.current_mode}")
        self._last_status_log_time = 0.0
        self._status_log_interval_s = 5.0
        self._inference_interval_s = float(self.get_parameter("inference_interval_s").value)
        self._last_inference_time = 0.0
        _val = self.get_parameter("log_raw_detections").value
        self._log_raw_detections = _val if isinstance(_val, bool) else (str(_val).lower() == "true")
        self._last_raw_log_time = 0.0
        self._raw_log_interval_s = 2.0  # Throttle raw detection logs
        self._warned_invalid_class_indices = set()  # Log each invalid index once per session
        self._engine_invalid_count = 0  # Count invalid class IDs when using TensorRT engine
        self._engine_fallback_threshold = 5  # Switch to .pt after this many invalid detections
        _conf = float(self.get_parameter("confidence").value)
        _iou = float(self.get_parameter("iou").value)
        if not (0 <= _conf <= 1):
            self.get_logger().warn(f"confidence {_conf} out of range [0,1]; clamping")
            _conf = max(0.0, min(1.0, _conf))
        if not (0 <= _iou <= 1):
            self.get_logger().warn(f"iou {_iou} out of range [0,1]; clamping")
            _iou = max(0.0, min(1.0, _iou))
        self._conf = _conf
        self._iou = _iou
        self._max_det = int(self.get_parameter("max_det").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        _interested = self.get_parameter("interested_class_names").value or []
        # Exclude empty strings so default [""] means "publish all"
        self._interested_class_names = {str(c).strip().lower() for c in _interested if str(c).strip()}
        if self._interested_class_names:
            self.get_logger().info(f"Publishing only classes: {sorted(self._interested_class_names)}")
        self.get_logger().info(f"Inference thresholds: conf={self._conf} iou={self._iou} max_det={self._max_det} imgsz={self._imgsz}")

        # Warm-up inference (trigger JIT/GPU init and catch load errors early)
        if self.inspection_model is not None:
            try:
                _dummy = np.zeros((480, 640, 3), dtype=np.uint8)
                _ = self.inspection_model(
                    _dummy, device=self._device, half=self._half,
                    conf=self._conf, iou=self._iou, max_det=self._max_det,
                    imgsz=self._imgsz, verbose=False
                )
                self.get_logger().info("Warm-up inference OK (inspection model)")
            except Exception as e:
                self.get_logger().warn(f"Warm-up inference failed: {e}")
        else:
            self.get_logger().warn("No inspection model loaded – skipping warm-up")

        # Publishers
        objects_segment_topic = self.get_parameter("objects_segment_topic").value
        seg_image_topic = self.get_parameter("segmentation_image_topic").value
        self.objects_segment_pub = self.create_publisher(
            ObjectsSegment, 
            objects_segment_topic,
            10
        )
        self.seg_image_pub = self.create_publisher(
            Image, 
            seg_image_topic,
            10
        )
        self.get_logger().info(f"Publishing objects topic: {objects_segment_topic}")
        self.get_logger().info(f"Publishing image topic: {seg_image_topic}")

        # Subscribers
        camera_topic = self.get_parameter("camera_rgb_topic").value
        self.rgb_sub = self.create_subscription(
            Image,
            camera_topic,
            self.callback,
            10
        )
        self.get_logger().info(f"Subscribing to camera RGB topic: {camera_topic}")
        
        # Subscribe to mode topic to switch between models
        self.mode_sub = None
        subscribe_mode_topic = bool(self.get_parameter("subscribe_mode_topic").value)
        if subscribe_mode_topic and fixed_mode not in ("navigation", "inspection"):
            mode_topic = self.get_parameter("mode_topic").value
            self.mode_sub = self.create_subscription(
                String,
                mode_topic,
                self.mode_callback,
                10
            )
            self.get_logger().info(f"Subscribed to mode topic: {mode_topic}")
        elif fixed_mode in ("navigation", "inspection"):
            self.get_logger().info(f"Mode topic disabled; fixed_mode={fixed_mode}")
        else:
            self.get_logger().info("Mode topic subscription disabled by parameter")

    def mode_callback(self, msg: String):
        """Callback to switch between navigation and inspection models."""
        mode = msg.data.lower().strip()

        if mode == "navigation":
            if self.navigation_model is None:
                self.get_logger().warn(
                    "Cannot switch to navigation mode – navigation model not loaded (use_vehicle_yolo=false)"
                )
                return
            if self.current_mode != "navigation":
                self.get_logger().info("Switching to navigation mode (Model 1)")
                self.segmentation_model = self.navigation_model
                self.current_mode = "navigation"
        elif mode == "inspection":
            if self.current_mode != "inspection":
                self.get_logger().info("Switching to inspection mode (Model 2)")
                self.segmentation_model = self.inspection_model
                self.current_mode = "inspection"
        else:
            self.get_logger().warn(f"Unknown mode received: '{mode}'. Expected 'navigation' or 'inspection'")

    def callback(self, rgb_data):
        # Throttle inference to reduce CPU load (let Nav2 controller keep up)
        now = time.time()
        if now - self._last_inference_time < self._inference_interval_s:
            return
        self._last_inference_time = now

        # Convert the RGB image to a NumPy array
        try:
            image = rnp.numpify(rgb_data)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if self.segmentation_model is None:
            if now - self._last_status_log_time >= self._status_log_interval_s:
                self._last_status_log_time = now
                self.get_logger().warn("No segmentation model loaded – skipping inference")
            return

        height, width, _ = image.shape
        if now - self._last_status_log_time >= self._status_log_interval_s:
            self._last_status_log_time = now
            self.get_logger().info(f"Received image with shape: {image.shape}")

        # Apply segmentation model to the RGB image (conf/iou control detection threshold and NMS)
        seg_result = self.segmentation_model(
            image, device=self._device, half=self._half, conf=self._conf, iou=self._iou,
            max_det=self._max_det, imgsz=self._imgsz, verbose=False
        )

        # Prepare the ObjectsSegment message
        objects_msg = ObjectsSegment()
        objects_msg.header = rgb_data.header  # Copy the header from the RGB image
        objects_msg.header.stamp = self.get_clock().now().to_msg()

        has_masks = seg_result[0].masks is not None
        if not has_masks:
            if now - self._last_status_log_time >= self._status_log_interval_s:
                self.get_logger().warn("Segmentation result has no masks; falling back to bbox masks")

        do_log_raw = (
            self._log_raw_detections
            and (now - self._last_raw_log_time) >= self._raw_log_interval_s
        )

        names_dict = seg_result[0].names
        num_classes = len(names_dict) if names_dict is not None else 0
        invalid_in_this_inference = False
        for index, cls in enumerate(seg_result[0].boxes.cls):
            class_index = int(cls.cpu().numpy())
            # TensorRT engines can return out-of-range class IDs (e.g. 37, 39 when model has 23 classes).
            # Filter strictly by index range; also check dict membership for dict-style names.
            if class_index < 0 or class_index >= num_classes:
                invalid_in_this_inference = True
                if class_index not in self._warned_invalid_class_indices:
                    self._warned_invalid_class_indices.add(class_index)
                    valid_max = max(0, num_classes - 1) if num_classes > 0 else 0
                    self.get_logger().warn(
                        f"Skipping detection with invalid class index {class_index} "
                        f"(valid range 0-{valid_max}); TensorRT may return out-of-range indices"
                    )
                continue
            if isinstance(names_dict, dict) and class_index not in names_dict:
                continue
            name = names_dict[class_index]
            conf = float(seg_result[0].boxes.conf[index].item())
            xyxy = seg_result[0].boxes.xyxy[index].cpu().numpy()

            if do_log_raw:
                self.get_logger().info(
                    f"[raw_det] class_id={class_index} class_name={name} confidence={conf:.3f} "
                    f"xyxy=[{xyxy[0]:.0f},{xyxy[1]:.0f},{xyxy[2]:.0f},{xyxy[3]:.0f}]"
                )

            # Optional filter: publish only interested classes (default: all)
            if self._interested_class_names and str(name).strip().lower() not in self._interested_class_names:
                continue

            if has_masks:
                mask = seg_result[0].masks.data.cpu().numpy()[index, :, :]
                mask_resized = cv2.resize(mask, (width, height))
                binary_mask = (mask_resized > 0.5).astype(np.uint8)
                y_indices, x_indices = np.where(binary_mask > 0)
            else:
                # Fallback: approximate mask from detection bounding box
                x1, y1, x2, y2 = xyxy
                x1 = max(0, int(x1))
                y1 = max(0, int(y1))
                x2 = min(width - 1, int(x2))
                y2 = min(height - 1, int(y2))
                if x2 <= x1 or y2 <= y1:
                    continue
                box_w = x2 - x1 + 1
                box_h = y2 - y1 + 1
                max_dim = max(box_w, box_h)
                stride = 1 if max_dim <= 120 else (2 if max_dim <= 240 else 3)
                xs = np.arange(x1, x2 + 1, stride)
                ys = np.arange(y1, y2 + 1, stride)
                grid_x, grid_y = np.meshgrid(xs, ys)
                x_indices = grid_x.ravel()
                y_indices = grid_y.ravel()

            if len(x_indices) == 0 or len(y_indices) == 0:
                self.get_logger().warn(f"No valid indices found for object: {name}")
                continue

            obj_msg = ObjectSegment()
            obj_msg.header = objects_msg.header
            obj_msg.class_name = name
            obj_msg.probability = conf
            obj_msg.x_indices = x_indices.tolist()
            obj_msg.y_indices = y_indices.tolist()
            objects_msg.objects.append(obj_msg)

        if do_log_raw:
            self._last_raw_log_time = now

        # Fallback: if using engine and this inference had invalid IDs, count and switch to .pt
        if invalid_in_this_inference and (
            getattr(self, "_inspection_using_engine", False)
            and os.path.exists(getattr(self, "_inspection_pt_path", ""))
            and self.current_mode == "inspection"
        ):
            self._engine_invalid_count += 1
            if self._engine_invalid_count >= self._engine_fallback_threshold:
                self.get_logger().warn(
                    f"TensorRT engine produced invalid class IDs in {self._engine_invalid_count} "
                    f"inferences. Falling back to PyTorch model: {self._inspection_pt_path}"
                )
                self.inspection_model = YOLO(self._inspection_pt_path)
                self.segmentation_model = self.inspection_model
                self._inspection_using_engine = False
                self._engine_invalid_count = 0
                self._warned_invalid_class_indices.clear()

        # Publish the ObjectsSegment message
        if objects_msg.objects:
            if now - self._last_status_log_time >= self._status_log_interval_s:
                self._last_status_log_time = now
                self.get_logger().info(f"Publishing {len(objects_msg.objects)} segmented objects")
            self.objects_segment_pub.publish(objects_msg)

        # Segmentation Visualization
        if self.seg_image_pub.get_subscription_count() > 0:
            try:
                # Generate and publish the annotated segmentation image
                seg_annotated = seg_result[0].plot(show=False)
                self.seg_image_pub.publish(rnp.msgify(Image, seg_annotated, encoding="rgb8"))
            except Exception as e:
                self.get_logger().error(f"Error while publishing segmentation image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltralyticsSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

