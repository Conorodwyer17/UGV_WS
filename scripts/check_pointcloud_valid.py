#!/usr/bin/env python3
"""Count valid (non-NaN, in-range) points in one /segmentation_processor/registered_pointcloud message."""
import sys
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = Node("check_pointcloud_valid")
    msg = [None]

    def cb(m):
        msg[0] = m

    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
    sub = node.create_subscription(PointCloud2, "/segmentation_processor/registered_pointcloud", cb, qos)

    timeout = 5.0
    start = node.get_clock().now()
    while rclpy.ok() and (node.get_clock().now() - start).nanoseconds / 1e9 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if msg[0] is not None:
            break

    node.destroy_node()
    rclpy.shutdown()

    if msg[0] is None:
        print("No message received", file=sys.stderr)
        sys.exit(1)

    m = msg[0]
    # Find x,y,z offsets
    offsets = {}
    for f in m.fields:
        offsets[f.name] = f.offset
    if "x" not in offsets or "y" not in offsets or "z" not in offsets:
        print("Missing x/y/z fields", file=sys.stderr)
        sys.exit(1)

    point_step = m.point_step
    valid = 0
    in_range = 0  # 0.1 < z < 3.0 (obstacle_max_range)
    for i in range(0, len(m.data), point_step):
        x = struct.unpack_from("<f", m.data, i + offsets["x"])[0]
        y = struct.unpack_from("<f", m.data, i + offsets["y"])[0]
        z = struct.unpack_from("<f", m.data, i + offsets["z"])[0]
        if not (x != x or y != y or z != z):  # not NaN
            valid += 1
            if 0.1 < z < 3.0:
                in_range += 1

    print(f"valid_points={valid} in_range_0.1_3m={in_range} total={m.width * m.height}")


if __name__ == "__main__":
    main()
