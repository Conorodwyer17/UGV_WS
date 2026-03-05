#!/usr/bin/env python3
"""
Aurora Mock Node — synthetic Aurora topics for simulation without hardware.

Subscribes to /cmd_vel, integrates odometry via differential drive model, and publishes:
- /slamware_ros_sdk_server_node/odom (nav_msgs/Odometry)
- /slamware_ros_sdk_server_node/scan (sensor_msgs/LaserScan) — minimal scan for costmap
- /slamware_ros_sdk_server_node/map (nav_msgs/OccupancyGrid) — empty map
- /slamware_ros_sdk_server_node/left_image_raw (sensor_msgs/Image) — 640x480
- /slamware_ros_sdk_server_node/semantic_labels (sensor_msgs/Image) — mono8
- When synthetic_vehicle_depth=false: depth_image_raw (empty) and point_cloud (empty).
- When synthetic_vehicle_depth=true: depth and point_cloud are published by depth_generator_node (separate process).
- TF: odom -> base_link

Optional sensor realism (Phase B): configurable noise, dropout for stress-testing.
Set enable_*_noise=true and tune *_noise_std, *_dropout_prob. Disable for deterministic runs.
"""

import math
import struct
import sys
import time
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

from .sensor_realism import (
    apply_odom_noise,
    apply_scan_noise,
    apply_depth_noise,
    should_dropout,
    next_period_with_jitter,
    LatencyQueue,
)

# Nav2 map layer expects TRANSIENT_LOCAL + RELIABLE (matches map_server)
MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


# UGV Rover (from ugv_base_ros / motor_driver)
DEFAULT_TRACK_WIDTH = 0.172  # m


class AuroraMockNode(Node):
    def __init__(self, **kwargs):
        super().__init__("aurora_mock_node", **kwargs)

        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("odom_rate_hz", 100.0)  # High rate for TF; avoids extrapolation into future
        self.declare_parameter("odom_stamp_lookahead_s", 0.02)  # Stamp odom slightly ahead so controller finds it
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("map_rate_hz", 2.0)
        self.declare_parameter("image_rate_hz", 10.0)
        self.declare_parameter("topic_prefix", "/slamware_ros_sdk_server_node")
        # Sensor realism (Phase B) — all disabled by default for deterministic testing
        self.declare_parameter("enable_odom_noise", False)
        self.declare_parameter("odom_position_noise_std", 0.01)
        self.declare_parameter("odom_velocity_noise_std", 0.01)
        self.declare_parameter("odom_drift_rate", 0.0)
        self.declare_parameter("enable_scan_noise", False)
        self.declare_parameter("scan_range_noise_std", 0.01)
        self.declare_parameter("scan_dropout_prob", 0.0)
        self.declare_parameter("enable_depth_noise", False)
        self.declare_parameter("depth_noise_std", 0.01)
        self.declare_parameter("message_dropout_prob", 0.0)
        self.declare_parameter("synthetic_vehicle_depth", False)  # Fill depth at vehicle region for segment_3d
        self.declare_parameter("synthetic_depth_m", 2.0)
        self.declare_parameter("synthetic_depth_region", 80)  # Half-width of square region in pixels (depth 416x224)
        self.declare_parameter("depth_fx", 180.0)
        self.declare_parameter("depth_fy", 180.0)
        self.declare_parameter("depth_cx", 208.0)
        self.declare_parameter("depth_cy", 112.0)
        # Timing realism (Phase 1): rate jitter from bag analysis
        self.declare_parameter("enable_timing_realism", False)
        self.declare_parameter("odom_rate_jitter_std", 0.03)
        self.declare_parameter("scan_rate_jitter_std", 0.05)
        self.declare_parameter("image_rate_jitter_std", 0.05)
        self.declare_parameter("map_rate_jitter_std", 0.05)
        # Latency (Phase 2): per-topic delay in ms
        self.declare_parameter("odom_latency_ms", 0.0)
        self.declare_parameter("scan_latency_ms", 0.0)
        self.declare_parameter("depth_latency_ms", 0.0)
        self.declare_parameter("image_latency_ms", 0.0)
        self.declare_parameter("log_callback_timing", False)

        self._odom_frame = self.get_parameter("odom_frame_id").value
        self._base_frame = self.get_parameter("base_frame_id").value
        self._prefix = self.get_parameter("topic_prefix").value
        self._enable_odom_noise = self.get_parameter("enable_odom_noise").value
        self._odom_pos_std = self.get_parameter("odom_position_noise_std").value
        self._odom_vel_std = self.get_parameter("odom_velocity_noise_std").value
        self._odom_drift = self.get_parameter("odom_drift_rate").value
        self._enable_scan_noise = self.get_parameter("enable_scan_noise").value
        self._scan_noise_std = self.get_parameter("scan_range_noise_std").value
        self._scan_dropout = self.get_parameter("scan_dropout_prob").value
        self._enable_depth_noise = self.get_parameter("enable_depth_noise").value
        self._depth_noise_std = self.get_parameter("depth_noise_std").value
        self._msg_dropout = self.get_parameter("message_dropout_prob").value
        self._synthetic_depth = self.get_parameter("synthetic_vehicle_depth").value
        self._synthetic_depth_m = self.get_parameter("synthetic_depth_m").value
        self._synthetic_depth_r = self.get_parameter("synthetic_depth_region").value
        self._depth_fx = self.get_parameter("depth_fx").value
        self._depth_fy = self.get_parameter("depth_fy").value
        self._depth_cx = self.get_parameter("depth_cx").value
        self._depth_cy = self.get_parameter("depth_cy").value
        self._enable_timing_realism = self.get_parameter("enable_timing_realism").value
        self._odom_jitter = self.get_parameter("odom_rate_jitter_std").value
        self._scan_jitter = self.get_parameter("scan_rate_jitter_std").value
        self._image_jitter = self.get_parameter("image_rate_jitter_std").value
        self._map_jitter = self.get_parameter("map_rate_jitter_std").value
        self._odom_latency_ms = self.get_parameter("odom_latency_ms").value
        self._scan_latency_ms = self.get_parameter("scan_latency_ms").value
        self._depth_latency_ms = self.get_parameter("depth_latency_ms").value
        self._image_latency_ms = self.get_parameter("image_latency_ms").value
        self._log_callback_timing = self.get_parameter("log_callback_timing").value
        self._latency_queue = LatencyQueue()

        # Differential drive state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0
        self._vtheta = 0.0
        self._last_cmd_time = None

        self._tf_broadcaster = TransformBroadcaster(self)

        # Callback group for odom + heartbeat only (kept for consistency; image path is now lightweight)
        self._odom_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_cb, 10)

        # Publishers
        self._odom_pub = self.create_publisher(
            Odometry, f"{self._prefix}/odom", 10
        )
        self._scan_pub = self.create_publisher(
            LaserScan, f"{self._prefix}/scan", 10
        )
        self._map_pub = self.create_publisher(
            OccupancyGrid, f"{self._prefix}/map", MAP_QOS
        )
        if not self._synthetic_depth:
            self._depth_pub = self.create_publisher(
                Image, f"{self._prefix}/depth_image_raw", 10
            )
            self._pc_pub = self.create_publisher(
                PointCloud2, f"{self._prefix}/point_cloud", 10
            )
        else:
            self._depth_pub = None
            self._pc_pub = None
        self._left_pub = self.create_publisher(
            Image, f"{self._prefix}/left_image_raw", 10
        )
        self._semantic_pub = self.create_publisher(
            Image, f"{self._prefix}/semantic_labels", 10
        )

        # Timers (variable period when enable_timing_realism)
        self._odom_period = 1.0 / self.get_parameter("odom_rate_hz").value
        self._odom_stamp_lookahead = self.get_parameter("odom_stamp_lookahead_s").value
        self._scan_period = 1.0 / self.get_parameter("scan_rate_hz").value
        self._map_period = 1.0 / self.get_parameter("map_rate_hz").value
        self._img_period = 1.0 / self.get_parameter("image_rate_hz").value
        self._odom_timer = self.create_timer(
            self._odom_period, self._publish_odom, callback_group=self._odom_cb_group
        )
        self._last_odom_stamp_ns: int | None = None
        self._odom_publish_count = 0
        self._odom_heartbeat_timer = self.create_timer(
            10.0, self._odom_heartbeat, callback_group=self._odom_cb_group
        )
        self._scan_timer = self.create_timer(self._scan_period, self._publish_scan)
        self._map_timer = self.create_timer(self._map_period, self._publish_map)
        self._img_timer = self.create_timer(self._img_period, self._publish_images)
        if not self._synthetic_depth:
            self._pc_timer = self.create_timer(self._map_period, self._publish_pointcloud)
        else:
            self._pc_timer = None
        self.create_timer(0.01, self._process_latency_queues)

        self.get_logger().info(
            f"Aurora mock: publishing to {self._prefix}/* (odom, scan, map, images)"
        )

    def _odom_heartbeat(self):
        """Periodic debug: confirm odom is publishing and stamp is fresh."""
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        count = self._odom_publish_count
        self._odom_publish_count = 0
        if self._last_odom_stamp_ns is not None:
            gap_s = (now_ns - self._last_odom_stamp_ns) / 1e9
            if gap_s > 0.5:
                self.get_logger().warn(
                    f"Odom stale: {gap_s:.2f}s since last publish (may cause TF extrapolation errors)"
                )
            else:
                self.get_logger().debug(
                    f"Odom heartbeat: gap={gap_s:.3f}s, rate={count/10.0:.1f} Hz"
                )
        else:
            self.get_logger().debug("Odom heartbeat: no odom published yet")

    def _process_latency_queues(self):
        """Process due messages in latency queue (10 Hz)."""
        now_ns = self.get_clock().now().nanoseconds
        self._latency_queue.process_due(now_ns)

    def _cmd_vel_cb(self, msg: Twist):
        self._vx = float(msg.linear.x)
        self._vtheta = float(msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _integrate_odom(self, dt: float):
        """Integrate velocity into pose (differential drive)."""
        if dt <= 0:
            return
        # x += cos(theta) * vx * dt
        # y += sin(theta) * vx * dt
        # theta += vtheta * dt
        self._x += math.cos(self._theta) * self._vx * dt
        self._y += math.sin(self._theta) * self._vx * dt
        self._theta += self._vtheta * dt
        # Normalize theta to [-pi, pi]
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

    def _yaw_to_quaternion(self, yaw):
        return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

    def _publish_odom(self):
        t0 = time.perf_counter() if self._log_callback_timing else None
        now = self.get_clock().now()
        # First publish: use time 0 so TF buffer "latest" (Time()) lookup finds slamware_map->base_link
        if self._last_odom_stamp_ns is None:
            stamp = rclpy.time.Time()
        else:
            stamp = now + rclpy.duration.Duration(nanoseconds=int(self._odom_stamp_lookahead * 1e9))
        if self._last_cmd_time is not None:
            dt = (now - self._last_cmd_time).nanoseconds / 1e9
            self._integrate_odom(dt)
            self._last_cmd_time = now

        x, y, theta, vx, vtheta = self._x, self._y, self._theta, self._vx, self._vtheta
        if self._enable_odom_noise:
            x, y, theta, vx, vtheta = apply_odom_noise(
                x, y, theta, vx, vtheta,
                self._odom_pos_std, self._odom_vel_std, self._odom_drift,
            )

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        q = self._yaw_to_quaternion(theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = vtheta
        for i in [0, 7, 35]:
            msg.pose.covariance[i] = 0.01
            msg.twist.covariance[i] = 0.01

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        if self._odom_latency_ms > 0:
            publish_at_ns = now.nanoseconds + int(self._odom_latency_ms * 1e6)
            def _publish_odom_delayed(m=msg, tf=t, stamp_ns=stamp.nanoseconds):
                self._odom_pub.publish(m)
                self._tf_broadcaster.sendTransform(tf)
                self._last_odom_stamp_ns = stamp_ns
            self._latency_queue.add(_publish_odom_delayed, publish_at_ns)
        else:
            self._odom_pub.publish(msg)
            self._tf_broadcaster.sendTransform(t)
            self._last_odom_stamp_ns = stamp.nanoseconds
        self._odom_publish_count += 1
        if self._log_callback_timing and t0 is not None:
            dt_ms = (time.perf_counter() - t0) * 1000
            if dt_ms > 5.0:
                self.get_logger().warn(f"Odom callback took {dt_ms:.1f} ms")

        if self._enable_timing_realism and self._odom_timer:
            self._odom_timer.cancel()
            self._odom_timer = self.create_timer(
                next_period_with_jitter(self._odom_period, self._odom_jitter),
                self._publish_odom,
            )

    def _publish_scan(self):
        """Minimal LaserScan so costmap has an observation source."""
        if should_dropout(self._msg_dropout):
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"  # Costmap expects laser frame; TF base_link->laser in launch
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0  # 1 deg
        msg.range_min = 0.1
        msg.range_max = 5.0
        # Empty scan (all inf) — costmap will have no obstacles from scan
        ranges = [float("inf")] * 360
        if self._enable_scan_noise:
            ranges = apply_scan_noise(ranges, self._scan_noise_std, self._scan_dropout, 5.0)
        msg.ranges = ranges
        if self._scan_latency_ms > 0:
            publish_at_ns = self.get_clock().now().nanoseconds + int(self._scan_latency_ms * 1e6)
            self._latency_queue.add(lambda m=msg: self._scan_pub.publish(m), publish_at_ns)
        else:
            self._scan_pub.publish(msg)
        if self._enable_timing_realism and self._scan_timer:
            self._scan_timer.cancel()
            self._scan_timer = self.create_timer(
                next_period_with_jitter(self._scan_period, self._scan_jitter),
                self._publish_scan,
            )

    def _publish_map(self):
        """Empty occupancy grid so Nav2 has a map topic. 20x20 m to cover far-side tire goals."""
        msg = OccupancyGrid()
        now = self.get_clock().now()
        # Avoid stamp 0 so costmap/controller do not reject (some layers require positive time)
        if now.nanoseconds == 0:
            now = rclpy.time.Time(nanoseconds=50_000_000)  # 0.05 s
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "slamware_map"
        msg.info.resolution = 0.05
        # 400x400 cells = 20x20 m; origin (-10,-10) covers [-10,10] for goals at vehicle front
        msg.info.width = 400
        msg.info.height = 400
        msg.info.origin.position.x = -10.0
        msg.info.origin.position.y = -10.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [-1] * (400 * 400)  # -1 = unknown
        self._map_pub.publish(msg)
        if not getattr(self, "_map_first_logged", False):
            self.get_logger().info("Map first published (frame=slamware_map, 20x20 m)")
            self._map_first_logged = True
        if self._enable_timing_realism and self._map_timer:
            self._map_timer.cancel()
            self._map_timer = self.create_timer(
                next_period_with_jitter(self._map_period, self._map_jitter),
                self._publish_map,
            )

    def _publish_images(self):
        """Publish left RGB and semantic images. When synthetic_vehicle_depth=false, also publish empty depth."""
        if should_dropout(self._msg_dropout):
            return
        now = self.get_clock().now().to_msg()

        # Depth only when not synthetic (depth_generator_node publishes when synthetic)
        if self._depth_pub is not None:
            depth_vals = [0.0] * (224 * 416)
            if self._enable_depth_noise:
                depth_vals = apply_depth_noise(depth_vals, self._depth_noise_std)
            depth = Image()
            depth.header.stamp = now
            depth.header.frame_id = "camera_depth_optical_frame"
            depth.height = 224
            depth.width = 416
            depth.encoding = "32FC1"
            depth.is_bigendian = 0
            depth.step = 416 * 4
            depth.data = struct.pack(f"{224 * 416}f", *depth_vals)
            img_latency = max(self._depth_latency_ms, self._image_latency_ms)
            if img_latency > 0:
                publish_at_ns = self.get_clock().now().nanoseconds + int(img_latency * 1e6)
                self._latency_queue.add(lambda d=depth: self._depth_pub.publish(d), publish_at_ns)
            else:
                self._depth_pub.publish(depth)

        # Left RGB and semantic (always)
        rgb = Image()
        rgb.header.stamp = now
        rgb.header.frame_id = "camera_left"
        rgb.height = 480
        rgb.width = 640
        rgb.encoding = "rgb8"
        rgb.is_bigendian = 0
        rgb.step = 640 * 3
        rgb.data = bytes(640 * 480 * 3)
        self._left_pub.publish(rgb)
        sem = Image()
        sem.header.stamp = now
        sem.header.frame_id = "camera_left"
        sem.height = 480
        sem.width = 640
        sem.encoding = "mono8"
        sem.is_bigendian = 0
        sem.step = 640
        sem.data = bytes(640 * 480)
        self._semantic_pub.publish(sem)

        if self._enable_timing_realism and self._img_timer:
            self._img_timer.cancel()
            self._img_timer = self.create_timer(
                next_period_with_jitter(self._img_period, self._image_jitter),
                self._publish_images,
            )

    def _publish_pointcloud(self):
        """Publish empty PointCloud2 (only when synthetic_vehicle_depth=false)."""
        if self._pc_pub is None:
            return
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "slamware_map"
        msg.height = 1
        msg.width = 0
        msg.is_dense = True
        msg.point_step = 12  # x,y,z float32
        msg.row_step = 0
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = b""
        self._pc_pub.publish(msg)
        if self._enable_timing_realism and self._pc_timer:
            self._pc_timer.cancel()
            self._pc_timer = self.create_timer(
                next_period_with_jitter(self._map_period, self._map_jitter),
                self._publish_pointcloud,
            )


def main(args=None):
    rclpy.init(args=args)
    # Ensure use_sim_time is applied when launch passes -p use_sim_time:=true (so clock matches /clock)
    argv = args if args is not None else sys.argv
    use_sim = any("use_sim_time:=true" in str(a).lower() for a in argv)
    node = AuroraMockNode(
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim)]
    )
    try:
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
