#!/usr/bin/env python3
"""
Optional thesis / bench demo: publish synthetic geometry_msgs/PoseArray on /tyre_3d_positions
so RViz and inspection_manager can be exercised without a live tyre_3d_projection_node.

Does not replace real projections — use for visualization-only or when projection is disabled.

Example:
  ros2 run inspection_manager demo_cycle_tyre_poses --ros-args \\
    -p frame_id:=slamware_map -p period_s:=8.0

With simulated Nav2 completion (pair with inspection_manager demo_mode and
demo_simulate_nav_success_topic matching navigation_success_topic):
  ros2 run inspection_manager demo_cycle_tyre_poses --ros-args \\
    -p simulate_navigation:=true -p navigation_success_topic:=/navigation_success
"""
from __future__ import annotations

import math
import sys
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Empty


class DemoCycleTyrePoses(Node):
    def __init__(self) -> None:
        super().__init__("demo_cycle_tyre_poses")
        self.declare_parameter("frame_id", "slamware_map")
        self.declare_parameter("period_s", 8.0)
        self.declare_parameter("output_topic", "/tyre_3d_positions")
        # Default square around origin (m); edit or override via params x0..y3 if needed
        self.declare_parameter(
            "poses_flat",
            [
                1.5,
                0.8,
                0.0,
                1.5,
                -0.8,
                0.0,
                -1.5,
                0.8,
                0.0,
                -1.5,
                -0.8,
                0.0,
            ],
        )
        self.declare_parameter("simulate_navigation", False)
        self.declare_parameter("navigation_success_delay_s", 2.0)
        self.declare_parameter("navigation_success_topic", "/navigation_success")

        topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(PoseArray, topic, 10)
        period = float(self.get_parameter("period_s").value)
        period = max(1.0, period)
        self._frame_id = str(self.get_parameter("frame_id").value)
        flat = list(self.get_parameter("poses_flat").value)
        self._poses_xyz: list[tuple[float, float, float]] = []
        for i in range(0, len(flat) - 2, 3):
            self._poses_xyz.append((float(flat[i]), float(flat[i + 1]), float(flat[i + 2])))
        if not self._poses_xyz:
            self._poses_xyz = [(1.0, 0.0, 0.0)]
        self._idx = 0
        self._simulate_nav = bool(self.get_parameter("simulate_navigation").value)
        self._nav_delay = max(0.1, float(self.get_parameter("navigation_success_delay_s").value))
        nav_topic = str(self.get_parameter("navigation_success_topic").value)
        self._nav_success_pub: Optional[Any] = None
        self._pending_nav_timer: Optional[Any] = None
        if self._simulate_nav:
            self._nav_success_pub = self.create_publisher(Empty, nav_topic, 10)
            self.get_logger().info(
                f"simulate_navigation: will publish std_msgs/Empty on {nav_topic} "
                f"after {self._nav_delay:.1f}s each cycle (use with inspection_manager demo_mode + "
                f"demo_simulate_nav_success_topic)"
            )
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"demo_cycle_tyre_poses: publishing one pose every {period:.1f}s on {topic} frame={self._frame_id} "
            f"({len(self._poses_xyz)} waypoints, cycles)"
        )

    def _cancel_pending_nav_timer(self) -> None:
        if self._pending_nav_timer is not None:
            try:
                self._pending_nav_timer.cancel()
            except Exception:
                pass
            self._pending_nav_timer = None

    def _publish_nav_success(self) -> None:
        """One-shot: cancel repeating timer then publish Empty (rclpy timers repeat until cancelled)."""
        if self._pending_nav_timer is not None:
            try:
                self._pending_nav_timer.cancel()
            except Exception:
                pass
            self._pending_nav_timer = None
        if self._nav_success_pub is None:
            return
        self._nav_success_pub.publish(Empty())
        self.get_logger().info("simulate_navigation: published Empty (synthetic Nav2 arrival)")

    def _tick(self) -> None:
        x, y, z = self._poses_xyz[self._idx % len(self._poses_xyz)]
        self._idx += 1

        msg = PoseArray()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.z = math.sin(math.pi / 4.0)
        p.orientation.w = math.cos(math.pi / 4.0)
        msg.poses = [p]
        self._pub.publish(msg)
        self.get_logger().info(f"Published PoseArray pose ({x:.2f}, {y:.2f}, {z:.2f})")

        if self._simulate_nav and self._nav_success_pub is not None:
            self._cancel_pending_nav_timer()
            self._pending_nav_timer = self.create_timer(self._nav_delay, self._publish_nav_success)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DemoCycleTyrePoses()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
