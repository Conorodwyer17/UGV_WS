import math
import time
from dataclasses import dataclass
from typing import Optional
from std_msgs.msg import Bool, String


@dataclass
class WaitStateContext:
    """
    Clear state context for WAIT_* states. Encapsulates dispatch gating and recovery tracking.
    WAIT states must always produce: dispatch, recover, or abort.
    """
    dispatched_approach_this_wait: bool = False
    wait_start_time: Optional[float] = None
    rotation_attempts: int = 0
    dispatch_retry_count: int = 0
    max_dispatch_retries_before_recover: int = 5  # After this many failed dispatches, trigger recover (rotate)


class MissionState:
    IDLE = "IDLE"
    INIT = "INIT"  # Verify Aurora/sensor health, load map
    SEARCH_VEHICLE = "SEARCH_VEHICLE"  # Search for vehicles (car or truck)
    PATROL_SEARCH = "PATROL_SEARCH"  # Slow patrol when no vehicles are detected
    TURN_IN_PLACE_SEARCH = "TURN_IN_PLACE_SEARCH"  # Rotate to look for vehicles when none detected
    WAIT_VEHICLE_BOX = "WAIT_VEHICLE_BOX"  # Wait for vehicle detection
    TURN_IN_PLACE_VEHICLE = "TURN_IN_PLACE_VEHICLE"  # Rotate to find vehicle
    APPROACH_VEHICLE = "APPROACH_VEHICLE"  # Navigate to detected vehicle (ANCHOR published when vehicle first detected)
    WAIT_TIRE_BOX = "WAIT_TIRE_BOX"  # Wait for tire detection (PLAN_NEXT_TIRE)
    TURN_IN_PLACE_TIRE = "TURN_IN_PLACE_TIRE"  # Rotate to find tires
    INSPECT_TIRE = "INSPECT_TIRE"  # Navigate to tire (NAV_TO_TIRE_APPROACH) then PHOTO_CAPTURE
    FACE_TIRE = "FACE_TIRE"  # Final rotate to face tire before capture
    WAIT_WHEEL_FOR_CAPTURE = "WAIT_WHEEL_FOR_CAPTURE"  # At goal: switch to wheel model, require wheel in view before photo
    VERIFY_CAPTURE = "VERIFY_CAPTURE"  # Wait for capture_result; retry or proceed to next tire
    FOLLOW_WAYPOINTS_BATCH = "FOLLOW_WAYPOINTS_BATCH"  # FollowWaypoints in flight (4-tire batch); waiting for result
    NEXT_VEHICLE = "NEXT_VEHICLE"  # Move to next vehicle
    DONE = "DONE"
    ERROR = "ERROR"  # Failure recovery; prevents infinite loops


def set_state(node, new_state: str, cause: str = "unknown") -> None:
    """Update internal state, log transition, and publish for debugging.
    cause: machine-readable string (use TransitionReason.X.value for taxonomy)."""
    cause_str = cause.value if hasattr(cause, "value") else str(cause)
    node._last_transition_cause = cause_str
    if new_state == node.current_state:
        return
    prev_state = node.current_state

    # Spin protection: detect search/tire wait cycles without progress
    max_repeats = node.get_parameter("max_state_repeats").value
    # Count as "cycle return" only states that mean we're retrying without progress (e.g. back to wait after turn).
    # Do NOT count WAIT_WHEEL_FOR_CAPTURE -> VERIFY_CAPTURE: that is forward progress (triggering photo).
    is_cycle_return = (
        (new_state == MissionState.SEARCH_VEHICLE and prev_state == MissionState.TURN_IN_PLACE_SEARCH)
        or (new_state == MissionState.WAIT_VEHICLE_BOX and prev_state == MissionState.TURN_IN_PLACE_VEHICLE)
        or (new_state == MissionState.WAIT_TIRE_BOX and prev_state == MissionState.TURN_IN_PLACE_TIRE)
        or (new_state == MissionState.FACE_TIRE and prev_state == MissionState.INSPECT_TIRE)
        or (new_state == MissionState.WAIT_WHEEL_FOR_CAPTURE and prev_state == MissionState.FACE_TIRE)
        or (new_state == MissionState.VERIFY_CAPTURE and prev_state == MissionState.FACE_TIRE)
        or (new_state == MissionState.WAIT_TIRE_BOX and prev_state == MissionState.VERIFY_CAPTURE)
        or (new_state == MissionState.WAIT_TIRE_BOX and prev_state == MissionState.WAIT_WHEEL_FOR_CAPTURE)
        or (new_state == MissionState.NEXT_VEHICLE and prev_state == MissionState.VERIFY_CAPTURE)
    )
    # Return-later requeue is forward progress (retrying deferred tires), not a cycle
    if cause_str == "return_later_requeue" and new_state == MissionState.WAIT_TIRE_BOX:
        is_cycle_return = False
    if is_cycle_return:
        node._state_repeat_count += 1
        if node._state_repeat_count >= max_repeats:
            node._last_transition_cause = "spin_protection"
            node.get_logger().error(
                f"Spin protection: {new_state} cycle repeated {node._state_repeat_count} times. Transitioning to ERROR."
            )
            expected = node._mission_report.get("total_tires_expected", 4)
            captured = getattr(node, "_total_tires_captured", 0)
            if expected > 0 and captured < expected:
                node.get_logger().error(
                    f"Mission aborted before completing all tires: {captured}/{expected} captured. "
                    "Far-side tires were not visited. Restart mission to complete inspection."
                )
            node.current_state = MissionState.ERROR
            node._mission_report["error_states_encountered"] += 1
            msg = String()
            msg.data = MissionState.ERROR
            node.state_pub.publish(msg)
            node._mission_log_append(
                "mission_end",
                {
                    **dict(node._mission_report),
                    "final_state": MissionState.ERROR,
                    "final_cause": node._last_transition_cause,
                },
            )
            node._publish_mission_report()
            return
    else:
        node._state_repeat_count = 0

    # Disable centroid servo when leaving INSPECT_TIRE
    if prev_state == MissionState.INSPECT_TIRE and new_state != MissionState.INSPECT_TIRE:
        if hasattr(node, "centroid_servo_enable_pub"):
            disable_msg = Bool()
            disable_msg.data = False
            node.centroid_servo_enable_pub.publish(disable_msg)
        node._centroid_handoff_initiated = False

    node.get_logger().info(
        f"State transition: {node.current_state} -> {new_state} (cause: {node._last_transition_cause})"
    )
    node.get_logger().info(
        f"MISSION_STATE: {new_state} (from {prev_state}, cause={node._last_transition_cause})"
    )
    node._mission_log_append("state_transition", {
        "from": prev_state, "to": new_state, "cause": cause_str,
    }, sync=True)
    node.current_state = new_state
    msg = String()
    msg.data = node.current_state
    node.state_pub.publish(msg)

    # Reset return-later state when moving to next vehicle (or done)
    if new_state == MissionState.NEXT_VEHICLE or new_state == MissionState.DONE:
        if hasattr(node, "_return_later_pass_count"):
            node._return_later_pass_count = 0
        if hasattr(node, "_tires_deferred"):
            node._tires_deferred = []
        if hasattr(node, "_refinement_ema_cache"):
            node._refinement_ema_cache.clear()

    if new_state in (MissionState.DONE, MissionState.ERROR):
        report = dict(node._mission_report) if node._mission_report else {}
        report["final_state"] = new_state
        report["final_cause"] = node._last_transition_cause
        node._mission_log_append("mission_end", report, sync=True)

    # Publish segmentation mode based on state
    # Use navigation model (Model 1) while searching or waiting for vehicle detection
    if new_state in (MissionState.SEARCH_VEHICLE, MissionState.WAIT_VEHICLE_BOX, MissionState.PATROL_SEARCH):
        mode_msg = String()
        mode_msg.data = "navigation"
        node.segmentation_mode_pub.publish(mode_msg)
        node.get_logger().info("Published segmentation mode: navigation (Model 1) - detecting vehicles")
    # Use inspection model (Model 2) when waiting for tire detection or driving to tire
    elif new_state in (MissionState.WAIT_TIRE_BOX, MissionState.INSPECT_TIRE, MissionState.FOLLOW_WAYPOINTS_BATCH):
        mode_msg = String()
        mode_msg.data = "inspection"
        node.segmentation_mode_pub.publish(mode_msg)
        node.get_logger().info("Published segmentation mode: inspection (Model 2) - detecting tires/wheels")

    # Every time we enter WAIT_VEHICLE_BOX or WAIT_TIRE_BOX, create/reset state context
    if new_state in (MissionState.WAIT_VEHICLE_BOX, MissionState.WAIT_TIRE_BOX):
        if not hasattr(node, "_wait_context") or node._wait_context is None:
            node._wait_context = WaitStateContext()
        if new_state == MissionState.WAIT_VEHICLE_BOX:
            node._dispatched_approach_this_wait = False
            node._wait_context.dispatched_approach_this_wait = False
        node._wait_context.dispatch_retry_count = 0
    # Reset wait timer and rotation tracking when entering wait states
    if new_state in [MissionState.WAIT_VEHICLE_BOX, MissionState.WAIT_TIRE_BOX]:
        node.wait_start_time = time.time()
        if new_state == MissionState.WAIT_VEHICLE_BOX:
            if prev_state != MissionState.TURN_IN_PLACE_VEHICLE:
                node.rotation_attempts = 0
            node._vehicle_confirm_count = 0
            node._vehicle_confirm_center = None
        if new_state == MissionState.WAIT_TIRE_BOX:
            if prev_state != MissionState.TURN_IN_PLACE_TIRE:
                node.rotation_attempts = 0
            node._tire_confirm_count = 0
            node._tire_confirm_center = None
            if hasattr(node, "_sync_planned_tires_for_current_vehicle"):
                node._sync_planned_tires_for_current_vehicle()
    elif new_state == MissionState.SEARCH_VEHICLE:
        node.wait_start_time = time.time()
        node._vehicle_search_start_time = time.time()
        node._vehicle_confirm_count = 0
        node._vehicle_confirm_center = None
        # Reset rotation attempts only when starting fresh from IDLE
        if prev_state == MissionState.IDLE:
            node.rotation_attempts = 0
            if hasattr(node, "_patrol_attempts"):
                node._patrol_attempts = 0
            if hasattr(node, "_last_patrol_time"):
                node._last_patrol_time = None
            if hasattr(node, "_patrol_direction"):
                node._patrol_direction = 1
    if new_state == MissionState.WAIT_TIRE_BOX:
        node._tire_search_start_time = time.time()
        current_yaw = node._get_current_yaw()
        node.initial_wait_yaw = current_yaw
        if current_yaw is not None:
            node.get_logger().info(f"Starting wait state with initial yaw: {math.degrees(current_yaw):.2f}°")
    elif new_state not in [MissionState.TURN_IN_PLACE_VEHICLE, MissionState.TURN_IN_PLACE_TIRE]:
        node.wait_start_time = None
        # Don't reset rotation_attempts here - keep it for the next wait cycle
    if new_state in (MissionState.APPROACH_VEHICLE, MissionState.INSPECT_TIRE, MissionState.FACE_TIRE):
        node._approach_entered_time = time.time()
        node._progress_window_start = None
        node._progress_start_pose = None
        if new_state in (MissionState.APPROACH_VEHICLE, MissionState.INSPECT_TIRE):
            node._wheel_switch_during_approach_done = False  # switch to wheel when within wheel_detection_switch_distance_m
            node._proximity_gate_passed = False  # reset for new nav goal; set True when distance_remaining < threshold
            node._recovery_skip_initiated = False  # reset for new nav goal; set True when we cancel for recovery skip
            node._centroid_handoff_initiated = False  # reset for new nav goal; set True when we cancel to hand off to centroid servo
    else:
        node._approach_entered_time = None
    if new_state == MissionState.WAIT_WHEEL_FOR_CAPTURE:
        node._wheel_wait_start_time = time.time()
        node._wheel_confirm_count = 0
        node._wheel_capture_inspection_published = False
