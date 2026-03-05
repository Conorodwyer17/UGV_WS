#!/usr/bin/env python3
"""
Centroid Servo Node — Image-based fine positioning for tire inspection.

Subscribes to ObjectsSegment (tire/wheel detections), computes image centroid error,
and publishes cmd_vel to center the tire in the camera view. Enabled only when
distance_remaining < proximity_gate_distance_m (from runtime_diagnostics).

Usage:
  ros2 run segmentation_3d centroid_servo_node
"""
from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from segmentation_msgs.msg import ObjectsSegment
from std_msgs.msg import Bool, String


# Tire/wheel class names from YOLO (best_fallback.pt outputs "wheel")
TIRE_CLASSES = frozenset({
    "tire", "tyre", "wheel", "car_tire", "car_tyre", "car-tire", "car-tyre",
})


def _is_tire_class(name: str) -> bool:
    return (name or "").strip().lower() in TIRE_CLASSES


def _centroid_from_indices(x_indices: List[int], y_indices: List[int]) -> Optional[Tuple[float, float]]:
    if not x_indices or not y_indices or len(x_indices) != len(y_indices):
        return None
    n = len(x_indices)
    cx = sum(x_indices) / n
    cy = sum(y_indices) / n
    return (cx, cy)


class CentroidServoNode(Node):
    """Publishes cmd_vel to center tire in image when enabled."""

    def __init__(self):
        super().__init__("centroid_servo_node")

        self.declare_parameter("objects_segment_topic", "/ultralytics_tire/segmentation/objects_segment")
        self.declare_parameter("runtime_diagnostics_topic", "/inspection_manager/runtime_diagnostics")
        self.declare_parameter("cmd_vel_topic", "/inspection/centroid_cmd_vel")
        self.declare_parameter("centroid_centered_topic", "/inspection_manager/centroid_centered")
        self.declare_parameter("proximity_gate_distance_m", 0.5)
        self.declare_parameter("image_width", 416)
        self.declare_parameter("image_height", 224)
        self.declare_parameter("Kp_linear", 0.15)
        self.declare_parameter("Kp_angular", 0.4)
        self.declare_parameter("max_linear", 0.08)
        self.declare_parameter("max_angular", 0.3)
        self.declare_parameter("centroid_error_threshold", 0.03)  # normalized, 3% of image
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("distance_gain_scaling", True)  # scale gains by distance to reduce overshoot when close
        self.declare_parameter("min_distance_for_full_gain_m", 0.3)  # below this, scale down gains

        self._objects_topic = self.get_parameter("objects_segment_topic").value
        self._diag_topic = self.get_parameter("runtime_diagnostics_topic").value
        self._cmd_topic = self.get_parameter("cmd_vel_topic").value
        self._centered_topic = self.get_parameter("centroid_centered_topic").value
        self._prox_gate = float(self.get_parameter("proximity_gate_distance_m").value)
        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)
        self._Kp_lin = float(self.get_parameter("Kp_linear").value)
        self._Kp_ang = float(self.get_parameter("Kp_angular").value)
        self._max_lin = float(self.get_parameter("max_linear").value)
        self._max_ang = float(self.get_parameter("max_angular").value)
        self._err_thresh = float(self.get_parameter("centroid_error_threshold").value)
        self._rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._distance_gain_scaling = bool(self.get_parameter("distance_gain_scaling").value)
        self._min_dist_full_gain = float(self.get_parameter("min_distance_for_full_gain_m").value)

        self._enabled = False
        self._distance_remaining: Optional[float] = None
        self._last_centroid: Optional[Tuple[float, float]] = None
        self._last_segment_time: Optional[float] = None
        self._centered_published = False

        self.create_subscription(
            ObjectsSegment,
            self._objects_topic,
            self._on_objects,
            10,
        )
        self.create_subscription(
            String,
            self._diag_topic,
            self._on_diagnostics,
            10,
        )

        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._centered_pub = self.create_publisher(Bool, self._centered_topic, 10)

        dt = 1.0 / self._rate_hz
        self._timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            "Centroid servo: objects=%s diag=%s cmd=%s proximity_gate=%.2fm"
            % (self._objects_topic, self._diag_topic, self._cmd_topic, self._prox_gate)
        )

    def _on_objects(self, msg: ObjectsSegment) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        best_centroid = None
        best_conf = 0.0
        for obj in msg.objects:
            if not _is_tire_class(obj.class_name):
                continue
            c = _centroid_from_indices(list(obj.x_indices), list(obj.y_indices))
            if c is not None and obj.probability > best_conf:
                best_conf = obj.probability
                best_centroid = c
        if best_centroid is not None:
            self._last_centroid = best_centroid
            self._last_segment_time = now

    def _on_diagnostics(self, msg: String) -> None:
        try:
            d = json.loads(msg.data) if msg.data else {}
            nf = d.get("nav_feedback")
            if nf and isinstance(nf, dict):
                dist_rem = nf.get("distance_remaining")
                self._distance_remaining = float(dist_rem) if dist_rem is not None else None
                if dist_rem is not None and dist_rem < self._prox_gate:
                    self._enabled = True
                    return
            self._enabled = False
            self._distance_remaining = None
        except (json.JSONDecodeError, TypeError):
            self._enabled = False
            self._distance_remaining = None

    def _control_loop(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        if not self._enabled:
            self._cmd_pub.publish(twist)
            self._centered_published = False
            return

        if self._last_centroid is None:
            self._cmd_pub.publish(twist)
            return

        cx, cy = self._last_centroid
        img_cx = self._img_w / 2.0
        img_cy = self._img_h / 2.0

        err_x = (cx - img_cx) / self._img_w if self._img_w > 0 else 0.0
        err_y = (cy - img_cy) / self._img_h if self._img_h > 0 else 0.0

        err_norm = math.sqrt(err_x * err_x + err_y * err_y)
        if err_norm < self._err_thresh:
            self._cmd_pub.publish(twist)
            if not self._centered_published:
                centered_msg = Bool()
                centered_msg.data = True
                self._centered_pub.publish(centered_msg)
                self._centered_published = True
                self.get_logger().info("Centroid centered (error=%.3f < %.3f)" % (err_norm, self._err_thresh))
            return

        self._centered_published = False

        # err_y > 0: centroid below center -> drive forward (positive linear.x)
        # err_x > 0: centroid right of center -> turn right (negative angular.z for typical camera)
        lin = self._Kp_lin * err_y
        ang = -self._Kp_ang * err_x

        # Distance-based gain scaling: when very close (< min_dist_full_gain), reduce gains to avoid overshoot
        if self._distance_gain_scaling and self._distance_remaining is not None and self._min_dist_full_gain > 0:
            scale = min(1.0, self._distance_remaining / self._min_dist_full_gain)
            scale = max(0.3, scale)  # never reduce below 30% to avoid dead zone
            lin *= scale
            ang *= scale

        lin = max(-self._max_lin, min(self._max_lin, lin))
        ang = max(-self._max_ang, min(self._max_ang, ang))

        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CentroidServoNode()
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
