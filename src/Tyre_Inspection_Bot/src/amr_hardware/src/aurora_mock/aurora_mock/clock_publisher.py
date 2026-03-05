#!/usr/bin/env python3
"""
Publish /clock for use_sim_time simulation.
When use_mock:=true, this node publishes monotonic time so all nodes share synchronized timestamps.
Uses time.monotonic() to guarantee timestamps never decrease, avoiding "Detected jump back in time"
TF buffer warnings (wall clock can step backward due to NTP, suspend/resume, etc.).
Must run with use_sim_time=false (it is the source of /clock).
"""
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockPublisher(Node):
    """Publish /clock at fixed rate for use_sim_time. Uses monotonic time (never decreases)."""

    def __init__(self):
        super().__init__("clock_publisher")
        self._start_time = time.monotonic()
        self.declare_parameter("publish_rate_hz", 100.0)
        self._pub = self.create_publisher(Clock, "/clock", 10)
        period = 1.0 / self.get_parameter("publish_rate_hz").value
        self._publish()  # Publish first message immediately so nodes get clock before first timer tick
        self.create_timer(period, self._publish)
        self.get_logger().info(f"Publishing /clock at {self.get_parameter('publish_rate_hz').value} Hz")

    def _publish(self):
        elapsed = time.monotonic() - self._start_time
        msg = Clock()
        msg.clock.sec = int(elapsed)
        msg.clock.nanosec = int((elapsed - msg.clock.sec) * 1e9)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
