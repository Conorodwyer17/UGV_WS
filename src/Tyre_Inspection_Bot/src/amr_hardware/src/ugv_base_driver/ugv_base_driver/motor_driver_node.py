#!/usr/bin/env python3
"""
ROS 2 node: subscribe to /cmd_vel (Twist), send Waveshare JSON over UART to ESP32.
Read feedback T:1001 (odl, odr in cm) and publish wheel odometry on /wheel/odometry.
Protocol: CMD_ROS_CTRL {"T":13,"X":linear_m/s,"Z":angular_rad_s}
Feedback: {"T":1001,"L":...,"R":...,"odl":cm,"odr":cm,...}
"""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import serial
except ImportError:
    serial = None

# UGV Rover (mainType 2) from ugv_base_ros ugv_config.h
DEFAULT_TRACK_WIDTH = 0.172  # m
DEFAULT_WHEEL_DIAMETER = 0.08  # m


def _send_json(ser, obj):
    """Send one JSON line to ESP32 (compact, newline)."""
    if ser is None or not ser.is_open:
        return
    line = json.dumps(obj, separators=(',', ':')) + '\n'
    ser.write(line.encode('utf-8'))


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        default_port = os.environ.get('UGV_UART_PORT', '/dev/ttyTHS1')
        self.declare_parameter('uart_port', default_port)
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.05)  # Short timeout for non-blocking read
        self.declare_parameter('publish_wheel_odom', True)
        self.declare_parameter('track_width', DEFAULT_TRACK_WIDTH)
        self.declare_parameter('wheel_diameter', DEFAULT_WHEEL_DIAMETER)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('wheel_odom_rate_hz', 20.0)

        port = self.get_parameter('uart_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value

        self.serial = None
        self._wheel_odom_enabled = self.get_parameter('publish_wheel_odom').value
        self._track_width = self.get_parameter('track_width').value
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._base_frame = self.get_parameter('base_frame_id').value
        self._odom_rate = self.get_parameter('wheel_odom_rate_hz').value

        # Wheel odom state: odl, odr in cm (from ESP32)
        self._last_odl_cm = None
        self._last_odr_cm = None
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time = None
        self._odom_lock = threading.Lock()

        if serial is None:
            self.get_logger().error('pyserial not installed: pip install pyserial')
            return
        try:
            self.serial = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
            _send_json(self.serial, {'T': 900, 'main': 2, 'module': 0})
            time.sleep(0.3)
            self.get_logger().info('Sent chassis config (UGV Rover, no module)')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32 on {port}: {e}')
            self.get_logger().warn('Motor driver will keep running but cmd_vel will be ignored')

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Subscribed to /cmd_vel')

        if self._wheel_odom_enabled and self.serial and self.serial.is_open:
            self._wheel_odom_pub = self.create_publisher(
                Odometry, 'wheel/odometry', 10
            )
            self._reader_thread = threading.Thread(target=self._feedback_reader, daemon=True)
            self._reader_thread.start()
            self._odom_timer = self.create_timer(
                1.0 / self._odom_rate, self._publish_wheel_odom
            )
            self.get_logger().info(
                f'Publishing wheel odometry on /wheel/odometry at {self._odom_rate} Hz'
            )
        else:
            self._wheel_odom_pub = None
            self._odom_timer = None

    def _feedback_reader(self):
        """Background thread: read JSON lines, parse T:1001, update odom state."""
        while rclpy.ok() and self.serial and self.serial.is_open:
            try:
                line = self.serial.readline()
                if not line:
                    continue
                line = line.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                obj = json.loads(line)
                if obj.get('T') != 1001:
                    continue
                odl = obj.get('odl')
                odr = obj.get('odr')
                if odl is None or odr is None:
                    continue
                odl_cm = float(odl)
                odr_cm = float(odr)
                now = time.time()
                with self._odom_lock:
                    if self._last_odl_cm is not None:
                        d_left_m = (odl_cm - self._last_odl_cm) / 100.0
                        d_right_m = (odr_cm - self._last_odr_cm) / 100.0
                        d_center = (d_left_m + d_right_m) / 2.0
                        d_theta = (d_right_m - d_left_m) / self._track_width
                        self._x += d_center * math.cos(self._theta)
                        self._y += d_center * math.sin(self._theta)
                        self._theta += d_theta
                    self._last_odl_cm = odl_cm
                    self._last_odr_cm = odr_cm
                    self._last_odom_time = now
            except json.JSONDecodeError:
                continue
            except Exception as e:
                self.get_logger().warn(f'Feedback parse error: {e}')
                continue

    def _publish_wheel_odom(self):
        """Publish wheel odometry from accumulated pose."""
        if self._wheel_odom_pub is None:
            return
        with self._odom_lock:
            if self._last_odom_time is None:
                return
            x, y, theta = self._x, self._y, self._theta
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        msg.twist.twist.linear.x = 0.0  # Velocity from delta would need time
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = 0.0
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[7] = 0.01
        msg.twist.covariance[35] = 0.01
        self._wheel_odom_pub.publish(msg)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

    def cmd_vel_callback(self, msg):
        """Twist -> Waveshare CMD_ROS_CTRL: {"T":13,"X":linear_x_m/s,"Z":angular_z_rad_s}"""
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn_once('ESP32 not connected, ignoring cmd_vel')
            return
        cmd = {
            'T': 13,
            'X': float(msg.linear.x),
            'Z': float(msg.angular.z)
        }
        try:
            _send_json(self.serial, cmd)
        except Exception as e:
            self.get_logger().error(f'UART write failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial and node.serial.is_open:
            _send_json(node.serial, {'T': 1, 'L': 0.0, 'R': 0.0})
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
