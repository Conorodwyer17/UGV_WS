#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from segmentation_msgs.msg import ObjectsSegment, ObjectSegment
from ultralytics import YOLO
import cv2
import time
import os

class UltralyticsSegmentationNode(Node):
    def __init__(self):
        super().__init__('ultralytics_segmentation')
        
        # Declare parameters for model paths
        self.declare_parameter("navigation_model", "yolov8m-seg.pt")  # Model 1 for navigation
        self.declare_parameter("inspection_model", "best.pt")  # Model 2 for inspection
        self.declare_parameter("prefer_tensorrt", True)
        self.declare_parameter("device", "0")
        self.declare_parameter("half", True)
        self.declare_parameter("mode_topic", "/segmentation_mode")
        self.declare_parameter("default_mode", "navigation")
        self.declare_parameter("camera_rgb_topic", "/slamware_ros_sdk_server_node/left_image_raw")  # Aurora left camera (default)
        
        # Load both YOLO segmentation models
        nav_model_path = self.get_parameter("navigation_model").value
        insp_model_path = self.get_parameter("inspection_model").value
        prefer_trt = bool(self.get_parameter("prefer_tensorrt").value)
        self._device = str(self.get_parameter("device").value)
        self._half = bool(self.get_parameter("half").value)
        
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
        self.navigation_model = YOLO(nav_loaded)  # Model 1 for navigation to trucks
        
        # Try to load inspection model, fallback to navigation model if not found
        # Resolve path: try as-is, then workspace root, then Tyre_Inspection_Bot directory
        self.get_logger().info(f"Loading inspection model: {insp_model_path}")
        resolved_insp_path = insp_model_path
        if not os.path.exists(insp_model_path):
            for candidate in [
                os.path.expanduser("~/ugv_ws/best.pt"),
                os.path.expanduser("~/ugv_ws/src/Tyre_Inspection_Bot/best.pt"),
                os.path.expanduser("~/ugv_ws/best.engine"),
                os.path.expanduser("~/ugv_ws/src/Tyre_Inspection_Bot/best.engine"),
            ]:
                if os.path.exists(candidate):
                    resolved_insp_path = candidate
                    self.get_logger().info(f"Using inspection model at: {resolved_insp_path}")
                    break
        if prefer_trt and resolved_insp_path.endswith(".pt"):
            engine_path = resolved_insp_path[:-2] + "engine"
            if os.path.exists(engine_path):
                self.get_logger().info(f"Using TensorRT engine for inspection: {engine_path}")
                resolved_insp_path = engine_path
        if os.path.exists(resolved_insp_path):
            self.inspection_model = YOLO(resolved_insp_path)  # Model 2 for inspection under truck
        else:
            self.get_logger().warn(
                f"Inspection model not found at {insp_model_path} or workspace. "
                "Using navigation model for inspection. Tire detection may fail (yolov8 has no 'tire' class)."
            )
            self.inspection_model = self.navigation_model  # Fallback to navigation model
        
        # Current active model (default to navigation)
        default_mode = self.get_parameter("default_mode").value
        if default_mode == "inspection":
            self.segmentation_model = self.inspection_model
            self.current_mode = "inspection"
        else:
            self.segmentation_model = self.navigation_model
            self.current_mode = "navigation"
        
        self.get_logger().info(f"Initial segmentation mode: {self.current_mode}")
        self._last_status_log_time = 0.0
        self._status_log_interval_s = 5.0

        # Publishers
        self.objects_segment_pub = self.create_publisher(
            ObjectsSegment, 
            "/ultralytics/segmentation/objects_segment", 
            10
        )
        self.seg_image_pub = self.create_publisher(
            Image, 
            "/ultralytics/segmentation/image", 
            10
        )

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
        mode_topic = self.get_parameter("mode_topic").value
        self.mode_sub = self.create_subscription(
            String,
            mode_topic,
            self.mode_callback,
            10
        )
        self.get_logger().info(f"Subscribed to mode topic: {mode_topic}")

    def mode_callback(self, msg: String):
        """Callback to switch between navigation and inspection models."""
        mode = msg.data.lower().strip()
        
        if mode == "navigation":
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
        # Convert the RGB image to a NumPy array
        try:
            image = rnp.numpify(rgb_data)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        height, width, _ = image.shape
        now = time.time()
        if now - self._last_status_log_time >= self._status_log_interval_s:
            self._last_status_log_time = now
            self.get_logger().info(f"Received image with shape: {image.shape}")

        # Apply segmentation model to the RGB image
        seg_result = self.segmentation_model(image, device=self._device, half=self._half)

        # Prepare the ObjectsSegment message
        objects_msg = ObjectsSegment()
        objects_msg.header = rgb_data.header  # Copy the header from the RGB image
        objects_msg.header.stamp = self.get_clock().now().to_msg()

        has_masks = seg_result[0].masks is not None
        if not has_masks:
            if now - self._last_status_log_time >= self._status_log_interval_s:
                self.get_logger().warn("Segmentation result has no masks; falling back to bbox masks")

        for index, cls in enumerate(seg_result[0].boxes.cls):
            class_index = int(cls.cpu().numpy())
            name = seg_result[0].names[class_index]

            if has_masks:
                mask = seg_result[0].masks.data.cpu().numpy()[index, :, :]
                mask_resized = cv2.resize(mask, (width, height))
                binary_mask = (mask_resized > 0.5).astype(np.uint8)
                y_indices, x_indices = np.where(binary_mask > 0)
            else:
                # Fallback: approximate mask from detection bounding box
                xyxy = seg_result[0].boxes.xyxy[index].cpu().numpy()
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
            obj_msg.probability = float(seg_result[0].boxes.conf[index].item())
            obj_msg.x_indices = x_indices.tolist()
            obj_msg.y_indices = y_indices.tolist()
            objects_msg.objects.append(obj_msg)

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

