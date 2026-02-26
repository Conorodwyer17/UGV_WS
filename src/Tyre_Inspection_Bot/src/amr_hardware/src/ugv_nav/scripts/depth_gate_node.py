#!/usr/bin/env python3
"""
Depth Gate Node — Gates cmd_vel based on depth validity.

Subscribes to:
  - cmd_vel_nav: velocity commands from Nav2 (remapped from cmd_vel)
  - /stereo/navigation_permitted: Bool — from aurora_sdk_bridge (True when depth valid)

Publishes to:
  - cmd_vel: forwarded to motor_driver (zero when navigation_permitted is False)

aurora_sdk_bridge publishes /stereo/navigation_permitted based on depth validity
(valid pixel ratio). When False, halt motion; when True, forward cmd_vel.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class DepthGateNode(Node):
    def __init__(self):
        super().__init__("depth_gate")

        self.declare_parameter("cmd_vel_nav_topic", "cmd_vel_nav")
        self.declare_parameter("cmd_vel_out_topic", "cmd_vel")
        self.declare_parameter("navigation_permitted_topic", "/stereo/navigation_permitted")
        self.declare_parameter(
            "nav_permitted_default",
            False,
        )  # Phase G: fail-safe when no message
        self.declare_parameter("stale_timeout_s", 0.3)  # Halt if no nav_permitted for 300ms

        self._cmd_nav_topic = self.get_parameter("cmd_vel_nav_topic").value
        self._cmd_out_topic = self.get_parameter("cmd_vel_out_topic").value
        self._nav_topic = self.get_parameter("navigation_permitted_topic").value
        self._nav_default = self.get_parameter("nav_permitted_default").value
        self._stale_timeout = self.get_parameter("stale_timeout_s").value

        self._nav_permitted = self._nav_default
        self._nav_permitted_seen = False
        self._last_nav_time = 0.0
        self._last_cmd = Twist()

        self._cmd_sub = self.create_subscription(
            Twist,
            self._cmd_nav_topic,
            self._cmd_cb,
            10,
        )
        self._nav_sub = self.create_subscription(
            Bool,
            self._nav_topic,
            self._nav_cb,
            10,
        )
        self._cmd_pub = self.create_publisher(Twist, self._cmd_out_topic, 10)

        # Publish at 20 Hz so robot stops promptly when nav_permitted goes false
        self._timer = self.create_timer(0.05, self._publish)
        self.get_logger().info(
            "depth_gate: "
            f"{self._cmd_nav_topic} -> {self._cmd_out_topic} "
            f"gated by {self._nav_topic}"
        )

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg

    def _nav_cb(self, msg: Bool):
        self._nav_permitted_seen = True
        self._nav_permitted = msg.data
        self._last_nav_time = self.get_clock().now().nanoseconds / 1e9

    def _publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._nav_permitted_seen and (now - self._last_nav_time) > self._stale_timeout:
            self._nav_permitted = False
        if self._nav_permitted:
            self._cmd_pub.publish(self._last_cmd)
        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
