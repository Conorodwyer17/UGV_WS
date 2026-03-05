#!/usr/bin/env python3
"""
ROS2 test node: verify 4-tyre mission flow.
Prerequisites: Aurora + Nav2 + perception running.
Sends FollowWaypoints with 4 poses, checks result and photo count.
Usage: python3 sim/test_mission.py
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import os


class TestMissionNode(Node):
    def __init__(self):
        super().__init__('test_mission_node')
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._photos_dir = os.path.expanduser('~/ugv_ws/tire_inspection_photos')
        if not os.path.isdir(self._photos_dir):
            self._photos_dir = '/tmp/tire_inspection_photos'

    def run_test(self):
        self.get_logger().info('Waiting for follow_waypoints action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('follow_waypoints server not available')
            return False
        goal = FollowWaypoints.Goal()
        # Placeholder: 4 poses (replace with real tyre goals from detection)
        for i in range(4):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = 1.0 + i * 0.5
            p.pose.position.y = 0.0
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            goal.poses.append(p)
        self.get_logger().info('Sending FollowWaypoints goal...')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        if not future.result().accepted:
            self.get_logger().error('Goal rejected')
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)
        result = result_future.result().result
        self.get_logger().info(f'Result: missed_waypoints={result.missed_waypoints}')
        n_photos = len([f for f in os.listdir(self._photos_dir) if f.endswith('.jpg')]) if os.path.isdir(self._photos_dir) else 0
        self.get_logger().info(f'Photos in {self._photos_dir}: {n_photos}')
        return len(result.missed_waypoints) == 0 and n_photos >= 4


def main():
    rclpy.init()
    node = TestMissionNode()
    ok = node.run_test()
    node.destroy_node()
    rclpy.shutdown()
    exit(0 if ok else 1)


if __name__ == '__main__':
    main()
