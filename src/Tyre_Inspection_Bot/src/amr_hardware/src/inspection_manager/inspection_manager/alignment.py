#!/usr/bin/env python3
"""Visual-servo alignment action server and convergence helpers."""

import math
import time
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from inspection_manager_interfaces.action import AlignTire


class AlignmentController:
    @staticmethod
    def is_converged(position_rms_m: float, angular_deg: float, control_quality: float) -> bool:
        return position_rms_m <= 0.05 and angular_deg <= 3.0 and control_quality >= 0.9


class VisualServoAlignServer(Node):
    def __init__(self) -> None:
        super().__init__("visual_servo_align_server")
        self.declare_parameter("feedback_hz", 10.0)
        self.declare_parameter("max_runtime_s", 8.0)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("quality_decay_s", 2.0)
        self.feedback_hz = float(self.get_parameter("feedback_hz").value)
        self.max_runtime_s = float(self.get_parameter("max_runtime_s").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.quality_decay_s = float(self.get_parameter("quality_decay_s").value)
        self._last_detection_ts: Optional[float] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.server = ActionServer(
            self,
            AlignTire,
            "/visual_servo/align",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )
        self.get_logger().info("visual_servo align action ready: /visual_servo/align")

    def goal_cb(self, _goal: AlignTire.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _pose_error(self, target_x: float, target_y: float, target_yaw: float) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        pos_err = math.hypot(target_x - x, target_y - y)
        yaw_err = abs(target_yaw - yaw)
        yaw_err = min(yaw_err, 2.0 * math.pi - yaw_err)
        return (pos_err, math.degrees(yaw_err))

    def _control_quality(self) -> float:
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_detection_ts is None:
            return 0.90
        stale = max(0.0, now - self._last_detection_ts)
        q = max(0.0, 1.0 - stale / max(0.1, self.quality_decay_s))
        return q

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        period_s = 1.0 / max(1.0, self.feedback_hz)
        start_s = self.get_clock().now().nanoseconds / 1e9
        stable_hits = 0
        last = (999.0, 999.0, 0.0)
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = AlignTire.Result()
                result.success = False
                result.status = "cancelled"
                result.position_rms = float(last[0])
                result.angular_deg = float(last[1])
                result.control_quality = float(last[2])
                return result

            err = self._pose_error(goal.target_x, goal.target_y, goal.target_yaw_rad)
            if err is not None:
                quality = self._control_quality()
                last = (err[0], err[1], quality)
            fb = AlignTire.Feedback()
            fb.position_rms = float(last[0])
            fb.angular_deg = float(last[1])
            fb.control_quality = float(last[2])
            goal_handle.publish_feedback(fb)

            if AlignmentController.is_converged(fb.position_rms, fb.angular_deg, fb.control_quality):
                stable_hits += 1
            else:
                stable_hits = 0
            if stable_hits >= 3:
                goal_handle.succeed()
                result = AlignTire.Result()
                result.success = True
                result.status = "aligned"
                result.position_rms = float(last[0])
                result.angular_deg = float(last[1])
                result.control_quality = float(last[2])
                return result

            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - start_s > self.max_runtime_s:
                goal_handle.abort()
                result = AlignTire.Result()
                result.success = False
                result.status = "timeout"
                result.position_rms = float(last[0])
                result.angular_deg = float(last[1])
                result.control_quality = float(last[2])
                return result
            time.sleep(period_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualServoAlignServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
