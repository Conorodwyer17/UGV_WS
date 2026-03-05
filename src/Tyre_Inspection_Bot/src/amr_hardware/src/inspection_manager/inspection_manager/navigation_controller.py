import math
import time
from typing import Callable, List

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose


def send_nav_goal(node, pose, done_cb, feedback_cb=None) -> bool:
    """Send a NavigateToPose goal. Returns True if goal was sent, False otherwise.
    Refuses to send if a goal is already in flight (one goal at a time; Nav2 replans path dynamically).
    feedback_cb: optional callback(msg) for Nav2 feedback (distance_remaining, number_of_recoveries, etc.).
    """
    node._last_nav2_server_check_time = getattr(node, "_last_nav2_server_check_time", 0.0)
    node._last_nav2_server_available = getattr(node, "_last_nav2_server_available", False)

    # Goal-in-flight guard: do not send a new goal while one is active
    active = getattr(node, "_active_nav_goal_handle", None)
    if active is not None:
        node.get_logger().warn(
            "Nav goal refused: goal already in flight. Wait for success/failure before sending next goal."
        )
        return False
    pending = getattr(node, "pending_goal_handle", None)
    if pending is not None and not pending.done():
        node.get_logger().warn(
            "Nav goal refused: previous goal not yet accepted. Wait before sending next goal."
        )
        return False

    # Minimum dispatch interval: avoid rapid-fire goals so robot can respond
    try:
        min_interval = float(node.get_parameter("nav_goal_min_interval_s").value or 0)
    except Exception:
        min_interval = 1.0
    last_dispatch = getattr(node, "_last_goal_dispatch_time", None)
    if last_dispatch is None:
        last_dispatch = 0.0
    if min_interval > 0 and last_dispatch > 0:
        elapsed = time.time() - last_dispatch
        if elapsed < min_interval:
            node.get_logger().warn(
                f"Nav goal refused: last dispatch {elapsed:.2f}s ago (< {min_interval:.2f}s min interval). Wait before next goal."
            )
            return False

    if not node.nav_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().error("Nav2 action server not available.")
        node._last_nav2_server_available = False
        return False
    node._last_nav2_server_available = True

    x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        node.get_logger().error(
            f"Nav goal rejected: pose position has non-finite value (x={x}, y={y}, z={z})"
        )
        return False

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = pose
    node.pending_goal_handle = node.nav_client.send_goal_async(
        goal_msg, feedback_callback=feedback_cb
    )
    node.pending_goal_handle.add_done_callback(done_cb)
    node._last_goal_dispatch_time = time.time()  # for nav_goal_min_interval_s
    return True


def send_follow_waypoints(
    node, poses: List[PoseStamped], done_cb: Callable
) -> bool:
    """Send a FollowWaypoints goal with 4 tyre poses. Returns True if goal was sent.
    Requires waypoint_follower with stop_on_failure: false (skip unreachable).
    Use PhotoAtWaypoint plugin for capture in batch mode (no wheel-in-view check).
    """
    if not getattr(node, "follow_waypoints_client", None):
        node.get_logger().error("FollowWaypoints client not initialized (use_follow_waypoints?).")
        return False
    active = getattr(node, "_active_follow_waypoints_handle", None)
    if active is not None:
        node.get_logger().warn("FollowWaypoints refused: goal already in flight.")
        return False
    if not poses:
        node.get_logger().error("FollowWaypoints refused: empty poses list.")
        return False
    if not node.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("FollowWaypoints action server not available.")
        return False
    for i, p in enumerate(poses):
        x, y, z = p.pose.position.x, p.pose.position.y, p.pose.position.z
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            node.get_logger().error(
                f"FollowWaypoints rejected: pose[{i}] has non-finite value (x={x}, y={y}, z={z})"
            )
            return False
    goal = FollowWaypoints.Goal()
    goal.poses = poses
    node._active_follow_waypoints_handle = node.follow_waypoints_client.send_goal_async(goal)
    node._active_follow_waypoints_handle.add_done_callback(done_cb)
    node._last_goal_dispatch_time = time.time()
    return True
