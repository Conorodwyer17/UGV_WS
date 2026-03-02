#!/usr/bin/env python3
"""
Stress test harness for autonomous tyre inspection.
Runs repeated missions in loop, logs success rate.
Usage: python3 sim/stress_test_harness.py [--iterations N]
Prerequisites: Full system running (Aurora, Nav2, perception, inspection_manager).
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped


def parse_args():
    p = argparse.ArgumentParser(description="Stress test: run N missions, report success rate")
    p.add_argument("--iterations", type=int, default=5, help="Number of mission iterations")
    p.add_argument("--timeout", type=float, default=300.0, help="Time limit per mission (s)")
    return p.parse_args()


class StressTestNode(Node):
    def __init__(self):
        super().__init__("stress_test_harness")
        self._client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._photos_dir = os.path.expanduser("~/ugv_ws/tire_inspection_photos")
        if not os.path.isdir(self._photos_dir):
            self._photos_dir = "/tmp/tire_inspection_photos"

    def _count_photos(self) -> int:
        if not os.path.isdir(self._photos_dir):
            return 0
        return len([f for f in os.listdir(self._photos_dir) if f.endswith(".jpg")])

    def run_one_mission(self, iteration: int) -> bool:
        """Run one FollowWaypoints mission. Returns True if success."""
        n_before = self._count_photos()
        goal = FollowWaypoints.Goal()
        for i in range(4):
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = 1.0 + i * 0.5
            p.pose.position.y = 0.0
            p.pose.orientation.w = 1.0
            goal.poses.append(p)
        future = self._client.send_goal_async(goal)
        if not rclpy.spin_until_future_complete(self, future, timeout_sec=10.0):
            self.get_logger().error(f"Iteration {iteration}: goal send timeout")
            return False
        if not future.result().accepted:
            self.get_logger().error(f"Iteration {iteration}: goal rejected")
            return False
        result_future = future.result().get_result_async()
        if not rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0):
            self.get_logger().error(f"Iteration {iteration}: result timeout")
            return False
        result = result_future.result().result
        missed = list(result.missed_waypoints) if result else []
        n_after = self._count_photos()
        photos_added = n_after - n_before
        ok = len(missed) == 0 and photos_added >= 4
        self.get_logger().info(f"Iteration {iteration}: missed={missed}, photos_added={photos_added}, ok={ok}")
        return ok


def main():
    args = parse_args()
    rclpy.init()
    node = StressTestNode()
    node.get_logger().info(f"Stress test: {args.iterations} iterations")
    if not node._client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("follow_waypoints server not available")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    successes = 0
    for i in range(1, args.iterations + 1):
        ok = node.run_one_mission(i)
        if ok:
            successes += 1
        time.sleep(2.0)  # Brief pause between missions
    rate = successes / args.iterations if args.iterations > 0 else 0.0
    node.get_logger().info(f"Stress test complete: {successes}/{args.iterations} success ({100*rate:.1f}%)")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if rate >= 1.0 else 1)


if __name__ == "__main__":
    main()
