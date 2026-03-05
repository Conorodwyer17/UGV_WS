#!/usr/bin/env python3
"""
Stub motor node for simulation no-move mode.

Subscribes to /cmd_vel and discards all commands. Used when sim_no_move:=true
to test the full stack (Nav2, inspection manager, centroid servo) without
physically moving the robot.

Does not publish wheel odometry; Aurora provides odometry for SLAM.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StubMotorNode(Node):
    """Subscribe to /cmd_vel and do nothing."""

    def __init__(self):
        super().__init__("stub_motor_node")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        topic = self.get_parameter("cmd_vel_topic").value
        self._sub = self.create_subscription(Twist, topic, self._on_cmd_vel, 10)
        self._count = 0
        self.get_logger().info(f"Stub motor: subscribing to {topic} (no motion)")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._count += 1
        # Discard; no motion. Log first receipt only.
        if self._count == 1:
            self.get_logger().info("Stub motor: receiving cmd_vel (discarding)")


def main(args=None):
    rclpy.init(args=args)
    node = StubMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
