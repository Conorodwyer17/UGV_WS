#!/usr/bin/env python3
"""
Phase D: TF latency histogram. Runs for 60s, records transform age to CSV.
"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import csv
import sys
import time

def main():
    rclpy.init()
    node = Node("tf_latency_trace")
    buffer = Buffer()
    TransformListener(buffer, node)

    out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/tf_trace.csv"
    duration_s = int(sys.argv[2]) if len(sys.argv) > 2 else 60

    chain = [("camera_left", "base_link"), ("base_link", "odom"), ("odom", "slamware_map")]
    rows = []
    start = time.monotonic()

    while time.monotonic() - start < duration_s:
        now = node.get_clock().now()
        for parent, child in chain:
            try:
                t = buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.5))
                age_ms = (now - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e6
                rows.append({"parent": parent, "child": child, "age_ms": age_ms, "t": time.monotonic() - start})
            except Exception:
                rows.append({"parent": parent, "child": child, "age_ms": -1, "t": time.monotonic() - start})
        rclpy.spin_once(node, timeout_sec=0.1)

    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "parent", "child", "age_ms"])
        w.writeheader()
        w.writerows(rows)

    ages = [r["age_ms"] for r in rows if r["age_ms"] >= 0]
    if ages:
        print(f"Wrote {len(rows)} samples to {out_path}")
        print(f"Age: min={min(ages):.1f} max={max(ages):.1f} mean={sum(ages)/len(ages):.1f} ms")
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
