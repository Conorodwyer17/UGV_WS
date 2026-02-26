import math
import time
from std_msgs.msg import String


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
    VERIFY_CAPTURE = "VERIFY_CAPTURE"  # Wait for capture_result; retry or proceed to next tire
    NEXT_VEHICLE = "NEXT_VEHICLE"  # Move to next vehicle
    DONE = "DONE"
    ERROR = "ERROR"  # Failure recovery; prevents infinite loops


def set_state(node, new_state: str, cause: str = "unknown") -> None:
    """Update internal state, log transition, and publish for debugging."""
    node._last_transition_cause = cause
    if new_state == node.current_state:
        return
    prev_state = node.current_state

    # Spin protection: detect search/tire wait cycles without progress
    max_repeats = node.get_parameter("max_state_repeats").value
    is_cycle_return = (
        (new_state == MissionState.SEARCH_VEHICLE and prev_state == MissionState.TURN_IN_PLACE_SEARCH)
        or (new_state == MissionState.WAIT_VEHICLE_BOX and prev_state == MissionState.TURN_IN_PLACE_VEHICLE)
        or (new_state == MissionState.WAIT_TIRE_BOX and prev_state == MissionState.TURN_IN_PLACE_TIRE)
        or (new_state == MissionState.FACE_TIRE and prev_state == MissionState.INSPECT_TIRE)
        or (new_state == MissionState.VERIFY_CAPTURE and prev_state == MissionState.FACE_TIRE)
        or (new_state == MissionState.WAIT_TIRE_BOX and prev_state == MissionState.VERIFY_CAPTURE)
        or (new_state == MissionState.NEXT_VEHICLE and prev_state == MissionState.VERIFY_CAPTURE)
    )
    if is_cycle_return:
        node._state_repeat_count += 1
        if node._state_repeat_count >= max_repeats:
            node._last_transition_cause = "spin_protection"
            node.get_logger().error(
                f"Spin protection: {new_state} cycle repeated {node._state_repeat_count} times. Transitioning to ERROR."
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

    node.get_logger().info(
        f"State transition: {node.current_state} -> {new_state} (cause: {node._last_transition_cause})"
    )
    node.get_logger().info(
        f"MISSION_STATE: {new_state} (from {prev_state}, cause={node._last_transition_cause})"
    )
    node._mission_log_append("state_transition", {
        "from": prev_state, "to": new_state, "cause": cause,
    }, sync=True)
    node.current_state = new_state
    msg = String()
    msg.data = node.current_state
    node.state_pub.publish(msg)

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
    # Use inspection model (Model 2) when waiting for tire detection
    elif new_state == MissionState.WAIT_TIRE_BOX:
        mode_msg = String()
        mode_msg.data = "inspection"
        node.segmentation_mode_pub.publish(mode_msg)
        node.get_logger().info("Published segmentation mode: inspection (Model 2) - detecting tires")

    # Every time we enter WAIT_VEHICLE_BOX we may dispatch approach on first tick if we have current_vehicle_box
    if new_state == MissionState.WAIT_VEHICLE_BOX:
        node._dispatched_approach_this_wait = False
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
    else:
        node._approach_entered_time = None
