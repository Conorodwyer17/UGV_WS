#!/usr/bin/env python3
"""
Simulated detection node (Phase C): publishes ObjectsSegment from ground-truth vehicle boxes.

Subscribes to vehicle_bounding_boxes, computes tire positions from vehicle geometry,
projects to camera image, and publishes ObjectsSegment with tire masks. Enables
testing of centroid_servo and segmentation_processor in simulation without YOLO.

Use with use_mock:=true and use_simulated_detection:=true. Requires valid depth at
tire locations (aurora_mock with synthetic_vehicle_depth or similar).
"""
import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from segmentation_msgs.msg import ObjectsSegment, ObjectSegment
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def _transform_point(transform: TransformStamped, px: float, py: float, pz: float) -> tuple:
    """Transform point from source frame to target frame. Returns (x, y, z)."""
    t = transform.transform.translation
    r = transform.transform.rotation
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    # Rotation matrix (from quaternion)
    R = [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]
    # p_target = R @ p_source + t
    rx = R[0][0] * px + R[0][1] * py + R[0][2] * pz + t.x
    ry = R[1][0] * px + R[1][1] * py + R[1][2] * pz + t.y
    rz = R[2][0] * px + R[2][1] * py + R[2][2] * pz + t.z
    return (rx, ry, rz)


def _estimate_tire_positions_from_box(
    box: BoundingBox3d,
    robot_pos: tuple,
    wheelbase_m: float = 2.7,
    track_m: float = 1.6,
) -> list:
    """Estimate 4 tire positions (FL, FR, RL, RR) from vehicle box. Same logic as vehicle_modeler."""
    cx = (box.xmin + box.xmax) / 2.0
    cy = (box.ymin + box.ymax) / 2.0
    z_ground = box.zmin
    ex = abs(box.xmax - box.xmin)
    ey = abs(box.ymax - box.ymin)
    if not (math.isfinite(cx) and math.isfinite(cy)) or (ex < 0.1 and ey < 0.1):
        return []
    rx, ry, _ = robot_pos
    # Front = end closer to robot
    if abs(ex - ey) < 0.5:
        dx, dy = rx - cx, ry - cy
        dist = math.hypot(dx, dy)
        fwd_x, fwd_y = (dx / dist, dy / dist) if dist >= 0.01 else (1.0, 0.0)
    elif ex >= ey:
        d_plus = math.hypot(cx + ex / 2 - rx, cy - ry)
        d_minus = math.hypot(cx - ex / 2 - rx, cy - ry)
        fwd_x, fwd_y = (1.0, 0.0) if d_plus < d_minus else (-1.0, 0.0)
    else:
        d_plus = math.hypot(cx - rx, cy + ey / 2 - ry)
        d_minus = math.hypot(cx - rx, cy - ey / 2 - ry)
        fwd_x, fwd_y = (0.0, 1.0) if d_plus < d_minus else (0.0, -1.0)
    right_x, right_y = -fwd_y, fwd_x
    hw, ht = wheelbase_m * 0.5, track_m * 0.5
    return [
        (cx + fwd_x * hw - right_x * ht, cy + fwd_y * hw - right_y * ht, z_ground),
        (cx + fwd_x * hw + right_x * ht, cy + fwd_y * hw + right_y * ht, z_ground),
        (cx - fwd_x * hw - right_x * ht, cy - fwd_y * hw - right_y * ht, z_ground),
        (cx - fwd_x * hw + right_x * ht, cy - fwd_y * hw + right_y * ht, z_ground),
    ]


def _project_to_image(x: float, y: float, z: float, fx: float, fy: float, cx: float, cy: float):
    """Project 3D point in camera optical frame to pixel (u, v). Returns (u, v) or None if behind camera."""
    if z <= 0.01:
        return None
    u = fx * x / z + cx
    v = fy * y / z + cy
    return (int(round(u)), int(round(v)))


def _mask_from_center(center_u: int, center_v: int, radius: int, width: int, height: int) -> tuple:
    """Generate x_indices, y_indices for a circular mask around center."""
    x_indices, y_indices = [], []
    for du in range(-radius, radius + 1):
        for dv in range(-radius, radius + 1):
            if du * du + dv * dv <= radius * radius:
                u = center_u + du
                v = center_v + dv
                if 0 <= u < width and 0 <= v < height:
                    x_indices.append(u)
                    y_indices.append(v)
    return (x_indices, y_indices)


class SimulatedDetectionNode(Node):
    def __init__(self):
        super().__init__("simulated_detection_node")
        self.declare_parameter("vehicle_boxes_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("output_topic", "/ultralytics_tire/segmentation/objects_segment")
        self.declare_parameter("map_frame", "slamware_map")
        self.declare_parameter("camera_frame", "camera_left")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("fx", 500.0)
        self.declare_parameter("fy", 500.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("tire_mask_radius", 15)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("confidence_min", 0.82)
        self.declare_parameter("confidence_max", 0.95)
        self.declare_parameter("miss_probability", 0.0)  # Occasional miss for realism
        self.declare_parameter("wheelbase_m", 2.7)
        self.declare_parameter("track_m", 1.6)

        self._vehicle_topic = self.get_parameter("vehicle_boxes_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._map_frame = self.get_parameter("map_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._img_w = self.get_parameter("image_width").value
        self._img_h = self.get_parameter("image_height").value
        self._fx = self.get_parameter("fx").value
        self._fy = self.get_parameter("fy").value
        self._cx = self.get_parameter("cx").value
        self._cy = self.get_parameter("cy").value
        self._mask_radius = self.get_parameter("tire_mask_radius").value
        self._conf_min = self.get_parameter("confidence_min").value
        self._conf_max = self.get_parameter("confidence_max").value
        self._miss_prob = self.get_parameter("miss_probability").value
        self._wheelbase = self.get_parameter("wheelbase_m").value
        self._track = self.get_parameter("track_m").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_boxes: BoundingBoxes3d | None = None

        self._pub = self.create_publisher(ObjectsSegment, self._output_topic, 10)
        self._sub = self.create_subscription(
            BoundingBoxes3d, self._vehicle_topic, self._vehicle_cb, 10
        )
        period = 1.0 / self.get_parameter("publish_rate_hz").value
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f"Simulated detection: {self._vehicle_topic} -> {self._output_topic}"
        )

    def _vehicle_cb(self, msg: BoundingBoxes3d):
        self._last_boxes = msg

    def _get_robot_pos(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5)
            )
            return (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        except TransformException:
            return None

    def _publish(self):
        if self._last_boxes is None or not self._last_boxes.bounding_boxes:
            return
        robot_pos = self._get_robot_pos()
        if robot_pos is None:
            return

        msg = ObjectsSegment()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_frame

        for box in self._last_boxes.bounding_boxes:
            tires = _estimate_tire_positions_from_box(
                box, robot_pos, self._wheelbase, self._track
            )
            for tire_xyz in tires:
                if random.random() < self._miss_prob:
                    continue
                try:
                    t = self._tf_buffer.lookup_transform(
                        self._camera_frame, self._map_frame,
                        rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2)
                    )
                    x, y, z = _transform_point(t, tire_xyz[0], tire_xyz[1], tire_xyz[2])
                except (TransformException, Exception):
                    continue
                uv = _project_to_image(x, y, z, self._fx, self._fy, self._cx, self._cy)
                if uv is None:
                    continue
                u, v = uv
                if u < 0 or u >= self._img_w or v < 0 or v >= self._img_h:
                    continue
                xi, yi = _mask_from_center(u, v, self._mask_radius, self._img_w, self._img_h)
                if not xi:
                    continue
                obj = ObjectSegment()
                obj.header = msg.header
                obj.class_name = "wheel"
                obj.probability = random.uniform(self._conf_min, self._conf_max)
                obj.x_indices = xi
                obj.y_indices = yi
                msg.objects.append(obj)

        if msg.objects:
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
