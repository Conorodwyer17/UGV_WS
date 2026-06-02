#!/usr/bin/env python3
"""Query Nav2 GetCosts at a map-frame pose (debug: is a tyre goal in free space?)."""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.srv import GetCosts

# nav2_costmap_2d: >= 253 = inscribed/inflated obstacle
INSCRIBED = 253


def main() -> int:
    ap = argparse.ArgumentParser(description="Print global/local costmap cost at (x,y) in map frame.")
    ap.add_argument("x", type=float, help="Goal x in map frame")
    ap.add_argument("y", type=float, help="Goal y in map frame")
    ap.add_argument(
        "--service",
        default="/global_costmap/get_cost_global_costmap",
        help="GetCosts service (try /local_costmap/get_cost_local_costmap for rolling window)",
    )
    ap.add_argument("--frame", default="map", help="frame_id for PoseStamped")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("costmap_value_at_xy")
    cli = node.create_client(GetCosts, args.service)
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Service not available: {args.service}")
        rclpy.shutdown()
        return 1

    ps = PoseStamped()
    ps.header.frame_id = args.frame
    ps.pose.position.x = args.x
    ps.pose.position.y = args.y
    ps.pose.position.z = 0.0
    ps.pose.orientation.w = 1.0

    req = GetCosts.Request()
    req.use_footprint = False
    req.poses = [ps]

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.done():
        node.get_logger().error("Service call timed out")
        rclpy.shutdown()
        return 1
    resp = fut.result()
    if not resp.success or not resp.costs:
        node.get_logger().error(f"GetCosts failed: success={resp.success}")
        rclpy.shutdown()
        return 1

    c = float(resp.costs[0])
    status = "free" if c < INSCRIBED else "inscribed_or_obstacle"
    print(f"cost={c:.1f} ({status}) at ({args.x:.3f},{args.y:.3f}) frame={args.frame} service={args.service}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
