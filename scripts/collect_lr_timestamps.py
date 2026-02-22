#!/usr/bin/env python3
"""
Phase E: Collect 200 left/right timestamp pairs, output sync_stats.json.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
import json
import sys

def main():
    rclpy.init()
    node = Node("collect_lr_timestamps")
    left_ts = []
    right_ts = []
    deltas_ms = []

    def cb(lmsg, rmsg):
        lt = lmsg.header.stamp.sec + lmsg.header.stamp.nanosec * 1e-9
        rt = rmsg.header.stamp.sec + rmsg.header.stamp.nanosec * 1e-9
        left_ts.append(lt)
        right_ts.append(rt)
        deltas_ms.append(abs(lt - rt) * 1000)

    qos = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    sub_l = Subscriber(node, Image, "/slamware_ros_sdk_server_node/left_image_raw", qos_profile=qos)
    sub_r = Subscriber(node, Image, "/slamware_ros_sdk_server_node/right_image_raw", qos_profile=qos)
    sync = ApproximateTimeSynchronizer([sub_l, sub_r], queue_size=10, slop=0.2)
    sync.registerCallback(cb)

    target = 200
    node.get_logger().info(f"Collecting {target} pairs...")
    while rclpy.ok() and len(deltas_ms) < target:
        rclpy.spin_once(node, timeout_sec=0.5)
        if len(deltas_ms) % 50 == 0 and len(deltas_ms) > 0:
            node.get_logger().info(f"  {len(deltas_ms)}/{target}")

    if not deltas_ms:
        node.get_logger().error("No pairs collected")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    import numpy as np
    arr = np.array(deltas_ms)
    stats = {
        "n_pairs": len(deltas_ms),
        "sync_mean_ms": float(np.mean(arr)),
        "sync_max_ms": float(np.max(arr)),
        "sync_min_ms": float(np.min(arr)),
        "sync_std_ms": float(np.std(arr)),
        "sync_p95_ms": float(np.percentile(arr, 95)),
        "sync_p99_ms": float(np.percentile(arr, 99)),
    }
    out = sys.argv[1] if len(sys.argv) > 1 else "logs/cursor_runs/20260222T180830Z/phaseE/sync_stats.json"
    import os
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w") as f:
        json.dump(stats, f, indent=2)
    node.get_logger().info(f"Wrote {out}")
    print(json.dumps(stats, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
