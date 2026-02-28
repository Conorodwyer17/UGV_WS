#!/usr/bin/env python3
"""
Minimal ROS2 <-> ESP32 serial bridge.

Reads Waveshare JSON telemetry lines (T=1001) and publishes:
- /esp32/encoders_raw (std_msgs/Int32MultiArray): [odl, odr]
- /esp32/telemetry_json (std_msgs/String): raw serial JSON line
- /odom (nav_msgs/Odometry): differential-drive odometry estimate
"""

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

try:
    import serial
except Exception:
    serial = None


class Esp32SerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("esp32_serial_bridge")

        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("wheel_radius_m", 0.065)
        self.declare_parameter("wheel_base_m", 0.29)
        self.declare_parameter("ticks_per_rev", 2048.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_base = float(self.get_parameter("wheel_base_m").value)
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.telemetry_pub = self.create_publisher(String, "/esp32/telemetry_json", 20)
        self.encoder_pub = self.create_publisher(Int32MultiArray, "/esp32/encoders_raw", 20)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)

        self._ser = None
        self._last_left_ticks = None
        self._last_right_ticks = None
        self._last_time = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        if serial is None:
            self.get_logger().error("pyserial unavailable; install with `pip3 install pyserial`.")
            return
        try:
            self._ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
            self.get_logger().info(
                f"Connected to ESP32 serial {self.serial_port} @ {self.baud_rate}."
            )
        except Exception as exc:
            self.get_logger().error(f"Unable to open serial port: {exc}")
            return

        self.create_timer(0.01, self._read_once)

    def _read_once(self) -> None:
        if self._ser is None:
            return
        raw = self._ser.readline()
        if not raw:
            return

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return

        self.telemetry_pub.publish(String(data=line))
        try:
            payload = json.loads(line)
        except Exception:
            return
        if int(payload.get("T", -1)) != 1001:
            return

        left_ticks = int(payload.get("odl", 0))
        right_ticks = int(payload.get("odr", 0))
        self.encoder_pub.publish(Int32MultiArray(data=[left_ticks, right_ticks]))
        self._publish_odom(left_ticks, right_ticks)

    def _publish_odom(self, left_ticks: int, right_ticks: int) -> None:
        now = self.get_clock().now()
        now_s = time.time()
        if self._last_left_ticks is None:
            self._last_left_ticks = left_ticks
            self._last_right_ticks = right_ticks
            self._last_time = now_s
            return

        dt = now_s - self._last_time if self._last_time is not None else 0.0
        if dt <= 1e-6:
            return

        d_left_ticks = left_ticks - self._last_left_ticks
        d_right_ticks = right_ticks - self._last_right_ticks
        self._last_left_ticks = left_ticks
        self._last_right_ticks = right_ticks
        self._last_time = now_s

        meter_per_tick = (2.0 * math.pi * self.wheel_radius) / max(self.ticks_per_rev, 1.0)
        dl = d_left_ticks * meter_per_tick
        dr = d_right_ticks * meter_per_tick
        ds = 0.5 * (dl + dr)
        dtheta = (dr - dl) / max(self.wheel_base, 1e-6)

        self._yaw += dtheta
        self._x += ds * math.cos(self._yaw)
        self._y += ds * math.sin(self._yaw)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self._yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self._yaw * 0.5)
        msg.twist.twist.linear.x = ds / dt
        msg.twist.twist.angular.z = dtheta / dt
        self.odom_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "_ser", None) is not None and node._ser.is_open:
            node._ser.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
