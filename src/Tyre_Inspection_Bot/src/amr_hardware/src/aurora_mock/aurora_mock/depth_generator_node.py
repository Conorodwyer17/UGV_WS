#!/usr/bin/env python3
"""
Depth generator node — offloads heavy depth/point_cloud work from aurora_mock_node.

Subscribes to /aurora_semantic/vehicle_bounding_boxes and odom (for robot pose).
Publishes /slamware_ros_sdk_server_node/depth_image_raw and point_cloud at 10 Hz.
Uses 5 Hz rate limit for heavy path (TF lookup, point cloud generation, projection);
fallback (center region) on other cycles so depth is always published at 10 Hz.

Runs only when synthetic_vehicle_depth is true; keeps aurora_mock_node lightweight
so odometry and TF are never blocked by perception work.
"""

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from .vehicle_point_cloud_generator import (
    generate_vehicle_points_from_box,
    transform_points_to_frame,
    project_points_to_depth_image,
)


class DepthGeneratorNode(Node):
    def __init__(self):
        super().__init__("depth_generator_node")

        self.declare_parameter("topic_prefix", "/slamware_ros_sdk_server_node")
        self.declare_parameter("vehicle_boxes_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("odom_topic", "/slamware_ros_sdk_server_node/odom")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("heavy_interval_s", 0.2)  # 5 Hz for TF + point cloud + projection
        self.declare_parameter("synthetic_depth_m", 2.0)
        self.declare_parameter("synthetic_depth_region", 80)
        self.declare_parameter("depth_fx", 180.0)
        self.declare_parameter("depth_fy", 180.0)
        self.declare_parameter("depth_cx", 208.0)
        self.declare_parameter("depth_cy", 112.0)

        self._prefix = self.get_parameter("topic_prefix").value
        self._vehicle_boxes_topic = self.get_parameter("vehicle_boxes_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._publish_period = 1.0 / self.get_parameter("publish_rate_hz").value
        self._heavy_interval_ns = int(self.get_parameter("heavy_interval_s").value * 1e9)
        self._synthetic_depth_m = self.get_parameter("synthetic_depth_m").value
        self._synthetic_depth_r = self.get_parameter("synthetic_depth_region").value
        self._depth_fx = self.get_parameter("depth_fx").value
        self._depth_fy = self.get_parameter("depth_fy").value
        self._depth_cx = self.get_parameter("depth_cx").value
        self._depth_cy = self.get_parameter("depth_cy").value

        self._last_vehicle_boxes: BoundingBoxes3d | None = None
        self._last_heavy_time_ns: int | None = None
        self._robot_x = 0.0
        self._robot_y = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            BoundingBoxes3d, self._vehicle_boxes_topic, self._vehicle_boxes_cb, 10
        )
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)

        self._depth_pub = self.create_publisher(
            Image, f"{self._prefix}/depth_image_raw", 10
        )
        self._pc_pub = self.create_publisher(
            PointCloud2, f"{self._prefix}/point_cloud", 10
        )

        self._timer = self.create_timer(self._publish_period, self._publish_depth_and_cloud)
        self.get_logger().info(
            f"depth_generator_node: publishing depth + point_cloud to {self._prefix}/*"
        )

    def _vehicle_boxes_cb(self, msg: BoundingBoxes3d):
        self._last_vehicle_boxes = msg

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _publish_depth_and_cloud(self):
        now_rcl = self.get_clock().now()
        now = now_rcl.to_msg()

        do_heavy = True
        if self._last_heavy_time_ns is not None:
            elapsed_ns = now_rcl.nanoseconds - self._last_heavy_time_ns
            if elapsed_ns < self._heavy_interval_ns:
                do_heavy = False

        depth_vals = [0.0] * (224 * 416)
        points_slamware = None

        if do_heavy and self._last_vehicle_boxes and self._last_vehicle_boxes.bounding_boxes:
            try:
                t = self._tf_buffer.lookup_transform(
                    "camera_depth_optical_frame", "slamware_map",
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2)
                )
                tr = t.transform.translation
                rot = t.transform.rotation
                robot_pos = (self._robot_x, self._robot_y, 0.0)
                all_points = None
                for box in self._last_vehicle_boxes.bounding_boxes:
                    pts = generate_vehicle_points_from_box(box, robot_pos)
                    if pts.size == 0:
                        continue
                    pts_cam = transform_points_to_frame(
                        pts,
                        (tr.x, tr.y, tr.z),
                        (rot.x, rot.y, rot.z, rot.w),
                    )
                    if all_points is None:
                        all_points = pts_cam
                        points_slamware = pts
                    else:
                        all_points = np.vstack([all_points, pts_cam])
                        points_slamware = np.vstack([points_slamware, pts])
                if all_points is not None and all_points.size > 0:
                    depth_img = project_points_to_depth_image(
                        all_points, 416, 224,
                        self._depth_fx, self._depth_fy, self._depth_cx, self._depth_cy,
                    )
                    for v in range(224):
                        for u in range(416):
                            z = depth_img[v, u]
                            if z > 0.01:
                                depth_vals[v * 416 + u] = z
                    self._last_heavy_time_ns = now_rcl.nanoseconds
                else:
                    self._last_heavy_time_ns = now_rcl.nanoseconds  # avoid hammering TF
            except (TransformException, Exception):
                self._last_heavy_time_ns = now_rcl.nanoseconds
                pass

        if points_slamware is None or points_slamware.size == 0:
            # Fallback: center region so depth is always valid at 10 Hz
            cw, ch = 416 // 2, 224 // 2
            r = self._synthetic_depth_r
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    u, v = cw + dx, ch + dy
                    if 0 <= u < 416 and 0 <= v < 224:
                        depth_vals[v * 416 + u] = self._synthetic_depth_m

        depth = Image()
        depth.header.stamp = now
        depth.header.frame_id = "camera_depth_optical_frame"
        depth.height = 224
        depth.width = 416
        depth.encoding = "32FC1"
        depth.is_bigendian = 0
        depth.step = 416 * 4
        depth.data = struct.pack(f"{224 * 416}f", *depth_vals)
        self._depth_pub.publish(depth)

        # Point cloud: publish vehicle points in slamware_map (or empty)
        pc_msg = PointCloud2()
        pc_msg.header.stamp = now
        pc_msg.header.frame_id = "slamware_map"
        pc_msg.is_dense = True
        pc_msg.point_step = 12
        pc_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        if points_slamware is not None and points_slamware.size > 0:
            n = points_slamware.shape[0]
            pc_msg.height = 1
            pc_msg.width = n
            pc_msg.row_step = 12 * n
            pc_msg.data = points_slamware.astype(np.float32).tobytes()
        else:
            pc_msg.height = 1
            pc_msg.width = 0
            pc_msg.row_step = 0
            pc_msg.data = b""
        self._pc_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthGeneratorNode()
    try:
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
