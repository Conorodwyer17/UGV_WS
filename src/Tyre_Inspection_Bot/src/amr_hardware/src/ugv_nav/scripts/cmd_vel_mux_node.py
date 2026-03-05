#!/usr/bin/env python3
"""
cmd_vel Mux Node — Selects between Nav2 and centroid servo cmd_vel.

When centroid_servo_enable is True, forwards centroid_cmd_vel.
Otherwise forwards cmd_vel_nav (from Nav2).

Requires launch remap: Nav2 cmd_vel -> cmd_vel_nav_source.
Mux subscribes to cmd_vel_nav_source and centroid_cmd_vel, publishes to cmd_vel_nav.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelMuxNode(Node):
    """Mux between Nav2 and centroid servo cmd_vel."""

    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.declare_parameter("cmd_vel_nav_topic", "cmd_vel_nav_source")
        self.declare_parameter("cmd_vel_centroid_topic", "/inspection/centroid_cmd_vel")
        self.declare_parameter("centroid_servo_enable_topic", "/inspection_manager/centroid_servo_enable")
        self.declare_parameter("cmd_vel_out_topic", "cmd_vel_nav")

        nav_topic = self.get_parameter("cmd_vel_nav_topic").value
        centroid_topic = self.get_parameter("cmd_vel_centroid_topic").value
        enable_topic = self.get_parameter("centroid_servo_enable_topic").value
        out_topic = self.get_parameter("cmd_vel_out_topic").value

        self._centroid_enabled = False
        self._last_nav: Twist | None = None
        self._last_centroid: Twist | None = None

        self.create_subscription(Twist, nav_topic, self._on_nav, 10)
        self.create_subscription(Twist, centroid_topic, self._on_centroid, 10)
        self.create_subscription(Bool, enable_topic, self._on_enable, 10)

        self._pub = self.create_publisher(Twist, out_topic, 10)

        self.get_logger().info(
            "cmd_vel mux: nav=%s centroid=%s enable=%s out=%s"
            % (nav_topic, centroid_topic, enable_topic, out_topic)
        )

    def _on_nav(self, msg: Twist) -> None:
        self._last_nav = msg
        if not self._centroid_enabled:
            self._pub.publish(msg)

    def _on_centroid(self, msg: Twist) -> None:
        self._last_centroid = msg
        if self._centroid_enabled:
            self._pub.publish(msg)

    def _on_enable(self, msg: Bool) -> None:
        self._centroid_enabled = msg.data
        if self._centroid_enabled and self._last_centroid is not None:
            self._pub.publish(self._last_centroid)
        elif not self._centroid_enabled and self._last_nav is not None:
            self._pub.publish(self._last_nav)
        elif not self._centroid_enabled:
            zero = Twist()
            zero.linear.x = 0.0
            zero.angular.z = 0.0
            self._pub.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
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
