#!/usr/bin/env python3
"""Bring up Nav2 lifecycle nodes safely.

This configures and activates each Nav2 lifecycle node directly.
It is idempotent: if a node is already inactive or active, it will
skip invalid transitions instead of failing the whole bringup.
"""

import sys
import time
from typing import Dict, Tuple

import rclpy
import tf2_ros
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

NAV2_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]
MAX_BRINGUP_TIME_S = 90.0
RETRY_DELAY_S = 2.0
TF_TARGET_FRAME = "odom"
TF_SOURCE_FRAME = "base_link"
TF_WAIT_TIMEOUT_S = 60.0
# Time to wait for each Nav2 lifecycle service (get_state, change_state) to appear and respond
SERVICE_WAIT_TIMEOUT_S = 60.0
SERVICE_WAIT_LOG_INTERVAL_S = 5.0


def _wait_for_service(node: Node, client, name: str, timeout_s: float) -> bool:
    node.get_logger().info(f"Waiting for service {name} (timeout {timeout_s:.0f}s)...")
    elapsed = 0.0
    while elapsed < timeout_s:
        if client.wait_for_service(timeout_sec=SERVICE_WAIT_LOG_INTERVAL_S):
            node.get_logger().info(f"Service {name} available after {elapsed:.0f}s")
            return True
        elapsed += SERVICE_WAIT_LOG_INTERVAL_S
        if elapsed < timeout_s:
            node.get_logger().info(
                f"Still waiting for {name}... ({elapsed:.0f}s / {timeout_s:.0f}s)"
            )
    node.get_logger().error(f"Service not available: {name} after {timeout_s:.0f}s")
    return False


def _call_get_state(node: Node, client) -> int:
    req = GetState.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        node.get_logger().error("GetState timed out")
        return State.PRIMARY_STATE_UNKNOWN
    resp = future.result()
    return resp.current_state.id if resp else State.PRIMARY_STATE_UNKNOWN


def _call_change_state(node: Node, client, transition_id: int) -> bool:
    req = ChangeState.Request()
    req.transition.id = transition_id
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        node.get_logger().error(f"ChangeState timed out (transition {transition_id})")
        return False
    resp = future.result()
    return bool(resp and resp.success)


def _state_name(state_id: int) -> str:
    names = {
        State.PRIMARY_STATE_UNKNOWN: "unknown",
        State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
        State.PRIMARY_STATE_INACTIVE: "inactive",
        State.PRIMARY_STATE_ACTIVE: "active",
        State.PRIMARY_STATE_FINALIZED: "finalized",
    }
    return names.get(state_id, f"state_{state_id}")


def _wait_for_tf(node: Node) -> bool:
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    start = time.time()
    last_log = 0.0
    while (time.time() - start) < TF_WAIT_TIMEOUT_S:
        try:
            buffer.lookup_transform(
                TF_TARGET_FRAME, TF_SOURCE_FRAME, rclpy.time.Time()
            )
            node.get_logger().info(
                f"TF ready: {TF_TARGET_FRAME} -> {TF_SOURCE_FRAME}"
            )
            return True
        except Exception:
            if time.time() - last_log > 5.0:
                node.get_logger().warn(
                    f"Waiting for TF {TF_TARGET_FRAME}->{TF_SOURCE_FRAME}..."
                )
                last_log = time.time()
            rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().error(
        f"TF not available: {TF_TARGET_FRAME}->{TF_SOURCE_FRAME} after "
        f"{TF_WAIT_TIMEOUT_S:.0f}s"
    )
    return False


def main() -> int:
    rclpy.init()
    node = Node("nav_lifecycle_startup")
    node.get_logger().info("Starting Nav2 lifecycle bringup (per-node)")

    clients: Dict[str, Tuple] = {}
    for name in NAV2_NODES:
        get_state = node.create_client(GetState, f"/{name}/get_state")
        change_state = node.create_client(ChangeState, f"/{name}/change_state")
        clients[name] = (get_state, change_state)

    for name, (get_state, change_state) in clients.items():
        if not _wait_for_service(node, get_state, f"/{name}/get_state", SERVICE_WAIT_TIMEOUT_S):
            rclpy.shutdown()
            return 1
        if not _wait_for_service(node, change_state, f"/{name}/change_state", SERVICE_WAIT_TIMEOUT_S):
            rclpy.shutdown()
            return 1

    if not _wait_for_tf(node):
        rclpy.shutdown()
        return 1

    start_time = time.time()

    # Phase 1: configure all nodes
    pending_config = set(NAV2_NODES)
    while pending_config and (time.time() - start_time) < MAX_BRINGUP_TIME_S:
        for name in list(pending_config):
            get_state, change_state = clients[name]
            state_id = _call_get_state(node, get_state)
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                node.get_logger().info(f"Configuring {name}")
                if not _call_change_state(
                    node, change_state, Transition.TRANSITION_CONFIGURE
                ):
                    node.get_logger().warn(
                        f"Configure failed for {name}; will retry"
                    )
                continue
            if state_id in (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE):
                pending_config.remove(name)
        if pending_config:
            time.sleep(RETRY_DELAY_S)

    if pending_config:
        node.get_logger().error(
            f"Configure timed out for: {sorted(pending_config)}"
        )
        rclpy.shutdown()
        return 1

    # Phase 2: activate all nodes
    pending_activate = set(NAV2_NODES)
    while pending_activate and (time.time() - start_time) < MAX_BRINGUP_TIME_S:
        for name in list(pending_activate):
            get_state, change_state = clients[name]
            state_id = _call_get_state(node, get_state)
            if state_id == State.PRIMARY_STATE_INACTIVE:
                node.get_logger().info(f"Activating {name}")
                if not _call_change_state(
                    node, change_state, Transition.TRANSITION_ACTIVATE
                ):
                    node.get_logger().warn(
                        f"Activate failed for {name}; will retry"
                    )
                continue
            if state_id == State.PRIMARY_STATE_ACTIVE:
                pending_activate.remove(name)
            elif state_id == State.PRIMARY_STATE_UNCONFIGURED:
                node.get_logger().warn(
                    f"{name} reverted to unconfigured; reconfiguring"
                )
                _call_change_state(
                    node, change_state, Transition.TRANSITION_CONFIGURE
                )
        if pending_activate:
            time.sleep(RETRY_DELAY_S)

    if pending_activate:
        node.get_logger().error(
            f"Activate timed out for: {sorted(pending_activate)}"
        )
        rclpy.shutdown()
        return 1

    node.get_logger().info("Nav2 lifecycle bringup complete")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
