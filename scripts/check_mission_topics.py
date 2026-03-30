#!/usr/bin/env python3
"""Wait until critical mission/visualisation topics publish (Jetson preflight).

Usage (after `source install/setup.bash`):
  python3 ~/ugv_ws/scripts/check_mission_topics.py
  python3 ~/ugv_ws/scripts/check_mission_topics.py --timeout 30
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--timeout", type=float, default=45.0, help="Seconds to wait (default: 45)")
    args = p.parse_args()

    topics = [
        ("/ultralytics_tire/segmentation/image", Image),
        ("/aurora_semantic/vehicle_markers", MarkerArray),
        ("/tyre_markers", MarkerArray),
        ("/plan", Path),
        ("/local_plan", Path),
    ]
    rclpy.init()
    node = Node("check_mission_topics")
    received = set()

    def make_cb(name):
        def _cb(_msg):
            received.add(name)

        return _cb

    for name, cls in topics:
        node.create_subscription(cls, name, make_cb(name), 10)

    deadline = time.monotonic() + args.timeout
    last_status = 0.0
    print("Waiting for:", [n for n, _ in topics])
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        missing = [n for n, _ in topics if n not in received]
        if not missing:
            print("OK: all critical topics have published at least once.")
            node.destroy_node()
            rclpy.shutdown()
            return 0
        now = time.monotonic()
        if now - last_status >= 5.0:
            last_status = now
            print(f"  … {now - (deadline - args.timeout):.0f}s elapsed, missing: {missing}")

    missing = [n for n, _ in topics if n not in received]
    print(f"TIMEOUT ({args.timeout}s): missing: {missing}", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
