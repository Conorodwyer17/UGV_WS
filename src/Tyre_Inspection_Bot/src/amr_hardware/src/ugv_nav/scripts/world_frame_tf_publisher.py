#!/usr/bin/env python3
"""
Publish map->slamware_map and slamware_map->odom with CURRENT timestamps.

static_transform_publisher uses stamp 0.0, causing tf2_monitor to report ~70s delay
and costmap obstacle layers to reject scans (transform "too old").
This node publishes to /tf at 10 Hz with now() so transforms are always fresh.
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
            f"Publishing map->slamware_map, slamware_map->odom, base_link->base_footprint, base_link->laser, base_link->camera_depth_optical_frame at {rate} Hz (current time)"
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

        # base_link->laser: Aurora only publishes slamware_map->laser when it has scan data.
        # Provide fallback so costmap obstacle layer can transform scans (Aurora SDK: 3.15cm above base).
        t4 = TransformStamped()
        t4.header.stamp = now
        t4.header.frame_id = "base_link"
        t4.child_frame_id = "laser"
        t4.transform.translation.x = 0.0
        t4.transform.translation.y = 0.0
        t4.transform.translation.z = 0.0315
        t4.transform.rotation.x = 0.0
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = 1.0

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

        self._broadcaster.sendTransform([t1, t2, t3, t4, t5])


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
