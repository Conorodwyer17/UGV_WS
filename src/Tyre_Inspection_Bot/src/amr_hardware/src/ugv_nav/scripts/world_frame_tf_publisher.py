#!/usr/bin/env python3
"""
Publish bridging and sensor TF transforms with CURRENT timestamps.

static_transform_publisher uses stamp 0.0, causing tf2_monitor to report ~70s delay
and costmap obstacle layers to reject scans (transform "too old").
This node publishes to /tf at 10 Hz with now() so transforms are always fresh.

Published transforms:
  map → slamware_map         (identity; Nav2 uses 'map', Aurora publishes to 'slamware_map')
  slamware_map → odom        (identity; Aurora chain requires this link)
  base_link → base_footprint (z=−0.08 m; RSP stamp=0 causes ~20s delay)
  base_link → camera_depth_optical_frame  (depth costmap; RSP stamp=0 causes ~67s delay)

NOT published:
  base_link → laser  (Aurora SDK publishes slamware_map→laser directly; dual-publishing causes TF conflict)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WorldFrameTfPublisher(Node):
    def __init__(self):
        super().__init__("world_frame_tf_publisher")
        self.declare_parameter("rate_hz", 10.0)
        rate = self.get_parameter("rate_hz").value
        self._broadcaster = TransformBroadcaster(self)
        period = 1.0 / rate if rate > 0 else 0.1
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"Publishing map->slamware_map, slamware_map->odom, base_link->base_footprint, "
            f"base_link->camera_depth_optical_frame at {rate} Hz (current time). "
            "Note: base_link->laser NOT published (Aurora provides slamware_map->laser directly)."
        )

    def _publish(self):
        now = self.get_clock().now().to_msg()

        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = "map"
        t1.child_frame_id = "slamware_map"
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "slamware_map"
        t2.child_frame_id = "odom"
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # base_link->base_footprint with current time (RSP uses stamp 0, causes ~20s delay)
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = "base_link"
        t3.child_frame_id = "base_footprint"
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = -0.08  # base_footprint 8cm below base_link
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0

        # NOTE: base_link→laser is NOT published here.
        # Aurora SDK publishes slamware_map→laser directly at scan rate. Publishing base_link→laser
        # here would create a TF conflict (dual parents for 'laser'), causing Nav2 costmap to see
        # inconsistent laser positions. nav_aurora.yaml transform_tolerance=2.0 covers the startup gap.

        # base_link->camera_depth_optical_frame: Point cloud uses this frame. RSP has 65s delay (stamp 0).
        # Costmap rejects observations when transform is "too old". Aurora SDK: 4.18cm fwd, 3cm left, optical quat.
        t5 = TransformStamped()
        t5.header.stamp = now
        t5.header.frame_id = "base_link"
        t5.child_frame_id = "camera_depth_optical_frame"
        t5.transform.translation.x = 0.0418
        t5.transform.translation.y = 0.03
        t5.transform.translation.z = 0.0
        t5.transform.rotation.x = -0.5
        t5.transform.rotation.y = 0.5
        t5.transform.rotation.z = -0.5
        t5.transform.rotation.w = 0.5

        self._broadcaster.sendTransform([t1, t2, t3, t5])


def main(args=None):
    rclpy.init(args=args)
    node = WorldFrameTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
