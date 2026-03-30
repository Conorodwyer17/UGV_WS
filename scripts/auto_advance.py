#!/usr/bin/env python3
"""
Auto-advance demo helper: after each new NavigateToPose goal published by inspection_manager,
wait a fixed delay and publish std_msgs/Empty on /navigation_success so demo_mode treats
the goal as reached (stub motor never completes Nav2).

Subscribes: geometry_msgs/PoseStamped on /inspection_manager/current_goal
Publishes: std_msgs/Empty on /navigation_success (configurable)

Usage (with workspace sourced):
  python3 ~/ugv_ws/scripts/auto_advance.py
  python3 ~/ugv_ws/scripts/auto_advance.py --ros-args -p delay_s:=4.0 -p success_topic:=/navigation_success

Stop with Ctrl+C.
"""
from __future__ import annotations

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty


def _pose_key(msg: PoseStamped) -> Tuple[float, float, float]:
    """Coarse key to detect 'new' goal vs duplicate publishes."""
    p = msg.pose.position
    return (round(p.x, 3), round(p.y, 3), round(p.z, 3))


class AutoAdvance(Node):
    def __init__(self) -> None:
        super().__init__("auto_advance")

        self.declare_parameter("delay_s", 3.0)
        self.declare_parameter("success_topic", "/navigation_success")
        self.declare_parameter("current_goal_topic", "/inspection_manager/current_goal")

        delay = float(self.get_parameter("delay_s").value)
        success_topic = str(self.get_parameter("success_topic").value).strip()
        goal_topic = str(self.get_parameter("current_goal_topic").value).strip()

        self._delay = max(0.5, delay)
        self._pub = self.create_publisher(Empty, success_topic, 10)
        self._timer = None
        self._last_key: Optional[Tuple[float, float, float]] = None

        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)

        self.get_logger().info(
            f"auto_advance: will publish Empty on '{success_topic}' "
            f"{self._delay:.1f}s after each new goal on '{goal_topic}'"
        )

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

    def _on_goal(self, msg: PoseStamped) -> None:
        key = _pose_key(msg)
        if self._last_key == key:
            return
        self._last_key = key
        self._cancel_timer()
        self._timer = self.create_timer(self._delay, self._emit_once)

    def _emit_once(self) -> None:
        """Timer callback: publish once and cancel periodic timer."""
        self._pub.publish(Empty())
        self.get_logger().info("Published Empty (simulated Nav2 success)")
        self._cancel_timer()


def main() -> None:
    rclpy.init()
    node = AutoAdvance()
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
