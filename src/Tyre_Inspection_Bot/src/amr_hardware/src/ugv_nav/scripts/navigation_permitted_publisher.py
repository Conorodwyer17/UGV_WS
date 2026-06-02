#!/usr/bin/env python3
"""
Publishes /stereo/navigation_permitted = True when Aurora native mode is used.

aurora_sdk_bridge normally publishes this based on depth validity. When the
bridge is not used, this node provides it so depth_gate forwards cmd_vel and
the robot can move.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class NavigationPermittedPublisher(Node):
    def __init__(self):
        super().__init__("navigation_permitted_publisher")
        self.declare_parameter("topic", "/stereo/navigation_permitted")
        self.declare_parameter("value", True)
        self.declare_parameter("rate_hz", 10.0)

        topic = self.get_parameter("topic").value
        value = self.get_parameter("value").value
        rate = self.get_parameter("rate_hz").value

        self._pub = self.create_publisher(Bool, topic, 10)
        period = 1.0 / rate if rate > 0 else 0.1
        self._timer = self.create_timer(period, self._timer_cb)
        self._msg = Bool()
        self._msg.data = bool(value)
        self.get_logger().info(
            f"Publishing {topic} = {value} at {rate} Hz (Aurora native mode: allow navigation)"
        )

    def _timer_cb(self):
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationPermittedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
