"""
Mission policy module: reason-code taxonomy, mission contract, and central policy defaults.
Single source of truth for thresholds and fallback decisions.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Any, List, Optional


# ---- Reason-Code Taxonomy (machine-readable transition causes) ----
class TransitionReason(str, Enum):
    """Machine-readable reason for state transitions. Every transition carries one."""
    # Startup
    INIT = "init"
    TF_READY = "tf_ready"
    NAV2_READY = "nav2_ready"
    SENSOR_HEALTHY = "sensor_healthy"
    MISSION_START_REQUESTED = "mission_start_requested"

    # Success paths
    VEHICLE_CONFIRMED = "vehicle_confirmed"
    TIRE_CONFIRMED = "tire_confirmed"
    APPROACH_DISPATCHED = "approach_dispatched"
    TIRE_APPROACH_DISPATCHED = "tire_approach_dispatched"
    NAV_SUCCESS = "nav_success"
    VERIFY_SUCCESS = "verify_success"
    CAPTURE_SUCCESS = "capture_success"
    NEXT_TIRE = "next_tire"
    NEXT_VEHICLE = "next_vehicle"
    VERIFY_FAILURE = "verify_failure"

    # Recovery paths
    DETECTION_TIMEOUT = "detection_timeout"
    VEHICLE_SEARCH_TIMEOUT = "vehicle_search_timeout"
    TIRE_SEARCH_TIMEOUT = "tire_search_timeout"
    ROTATE_SEARCH = "rotate_search"
    TURN_IN_PLACE_VEHICLE = "turn_in_place_vehicle"
    TURN_IN_PLACE_TIRE = "turn_in_place_tire"
    PATROL_STEP = "patrol_step"
    PROGRESS_STALL = "progress_stall"
    APPROACH_TIMEOUT = "approach_timeout"
    PLANNED_TIRE_FALLBACK = "planned_tire_fallback"
    NAV_RETRY = "nav_retry"

    # Abort / fail-safe
    TF_UNAVAILABLE = "tf_unavailable"
    TF_UNAVAILABLE_AT_START = "tf_unavailable_at_start"
    MAP_TF_UNAVAILABLE_AT_START = "map_tf_unavailable_at_start"
    NAV2_UNAVAILABLE = "nav2_unavailable"
    DISPATCH_FAILURES = "dispatch_failures"
    NAV_FAILED = "nav_failed"
    SPIN_PROTECTION = "spin_protection"
    HARD_MISSION_TIMEOUT = "hard_mission_timeout"
    CONFIG_VALIDATION_FAILED = "config_validation_failed"
    TOPIC_NOT_ALIVE = "topic_not_alive"


# ---- Target Lifecycle (perception -> goal -> reach) ----
class TargetLifecycle(str, Enum):
    """Target lifecycle: DETECTED -> CONFIRMED -> DISPATCHED -> REACHED -> VERIFIED."""
    DETECTED = "detected"
    CONFIRMED = "confirmed"
    DISPATCHED = "dispatched"
    REACHED = "reached"
    VERIFIED = "verified"


# ---- Dispatch Failure Codes (goal generation / Nav2) ----
class DispatchFailureCode(str, Enum):
    """Explicit failure code when goal dispatch fails."""
    TF_UNAVAILABLE = "tf_unavailable"
    TARGET_STALE = "target_stale"
    TARGET_INVALID = "target_invalid"
    UNREACHABLE_GOAL = "unreachable_goal"
    MAP_TRANSFORM_FAILED = "map_transform_failed"
    NAV2_SERVER_UNAVAILABLE = "nav2_server_unavailable"
    NAV_PERMITTED_BLOCKED = "nav_permitted_blocked"
    GOAL_REJECTED = "goal_rejected"
    ROBOT_POSE_UNAVAILABLE = "robot_pose_unavailable"
    GOAL_OUT_OF_BOUNDS = "goal_out_of_bounds"


# ---- Mission Contract Defaults (immutable policy) ----
@dataclass(frozen=False)
class MissionPolicy:
    """
    Centralized mission policy. Load from node params but provide defaults.
    Single source for thresholds and fallback decisions.
    """
    # Frames
    world_frame: str = "slamware_map"
    base_frame: str = "base_link"
    map_frame: str = "map"
    require_goal_transform: bool = True

    # Detection
    detection_stale_s: float = 2.0
    vehicle_boxes_stale_s: float = 2.0
    vehicle_confirmations_required: int = 3
    tire_confirmations_required: int = 2
    vehicle_confirm_tolerance_m: float = 1.0
    tire_confirm_tolerance_m: float = 0.5
    vehicle_min_distance_m: float = 0.5
    vehicle_min_distance_confirmed_m: float = 0.2
    vehicle_anchor_reach_distance_m: float = 1.4
    vehicle_max_distance_m: float = 20.0
    max_tire_distance_from_vehicle: float = 5.0

    # Goals
    goal_max_age_s: float = 5.0  # Reject goals computed from detections older than this
    approach_offset: float = 0.5
    tire_offset: float = 0.3
    approach_goal_tolerance_m: float = 0.35
    tire_goal_tolerance_m: float = 0.25
    reuse_approach_goal_on_retry: bool = True

    # Timeouts
    tf_wait_timeout: float = 60.0
    tf_stable_s: float = 5.0
    nav2_wait_timeout: float = 90.0
    vehicle_search_timeout_s: float = 180.0
    tire_search_timeout_s: float = 90.0
    approach_timeout_s: float = 120.0

    # Recovery
    nav_retry_budget: int = 3
    max_rotation_attempts: int = 8
    max_state_repeats: int = 3
    dispatch_fail_abort_count: int = 3
    planned_tire_fallback_enabled: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "world_frame": self.world_frame,
            "base_frame": self.base_frame,
            "map_frame": self.map_frame,
            "require_goal_transform": self.require_goal_transform,
            "detection_stale_s": self.detection_stale_s,
            "vehicle_boxes_stale_s": self.vehicle_boxes_stale_s,
            "goal_max_age_s": self.goal_max_age_s,
        }


# ---- Startup Invariants ----
@dataclass
class StartupInvariants:
    """
    Mission startup invariants. All must pass before leaving IDLE.
    """
    tf_chain_healthy: bool = False
    nav2_available: bool = False
    detection_topic_alive: bool = False
    vehicle_boxes_topic_alive: Optional[bool] = None  # None = not required
    nav_permitted_topic_alive: Optional[bool] = None  # None = not required
    config_valid: bool = True

    def is_ready(self) -> bool:
        if not self.tf_chain_healthy or not self.nav2_available or not self.detection_topic_alive:
            return False
        if self.vehicle_boxes_topic_alive is False:
            return False
        if self.nav_permitted_topic_alive is False:
            return False
        if not self.config_valid:
            return False
        return True

    def failures(self) -> List[str]:
        out = []
        if not self.tf_chain_healthy:
            out.append("tf_chain_unhealthy")
        if not self.nav2_available:
            out.append("nav2_unavailable")
        if not self.detection_topic_alive:
            out.append("detection_topic_not_alive")
        if self.vehicle_boxes_topic_alive is False:
            out.append("vehicle_boxes_topic_not_alive")
        if self.nav_permitted_topic_alive is False:
            out.append("nav_permitted_topic_not_alive")
        if not self.config_valid:
            out.append("config_validation_failed")
        return out


def load_policy_from_node(node) -> MissionPolicy:
    """Build MissionPolicy from node parameters."""
    def p(name: str, default: Any = None):
        try:
            return node.get_parameter(name).value
        except Exception:
            return default

    return MissionPolicy(
        world_frame=p("world_frame", "slamware_map"),
        base_frame=p("base_frame", "base_link"),
        map_frame=p("map_frame", "map"),
        require_goal_transform=bool(p("require_goal_transform", True)),
        detection_stale_s=float(p("detection_stale_s", 2.0)),
        vehicle_boxes_stale_s=float(p("vehicle_boxes_stale_s", 2.0)),
        vehicle_confirmations_required=int(p("vehicle_confirmations_required", 3)),
        tire_confirmations_required=int(p("tire_confirmations_required", 2)),
        vehicle_confirm_tolerance_m=float(p("vehicle_confirm_tolerance_m", 1.0)),
        tire_confirm_tolerance_m=float(p("tire_confirm_tolerance_m", 0.5)),
        vehicle_min_distance_m=float(p("vehicle_min_distance_m", 0.5)),
        vehicle_min_distance_confirmed_m=float(p("vehicle_min_distance_confirmed_m", 0.2)),
        vehicle_anchor_reach_distance_m=float(p("vehicle_anchor_reach_distance_m", 1.4)),
        vehicle_max_distance_m=float(p("vehicle_max_distance_m", 20.0)),
        max_tire_distance_from_vehicle=float(p("max_tire_distance_from_vehicle", 5.0)),
        goal_max_age_s=float(p("goal_max_age_s", 5.0)),
        approach_offset=float(p("approach_offset", 0.5)),
        tire_offset=float(p("tire_offset", 0.4)),
        approach_goal_tolerance_m=float(p("approach_goal_tolerance_m", 0.35)),
        tire_goal_tolerance_m=float(p("tire_goal_tolerance_m", 0.25)),
        reuse_approach_goal_on_retry=bool(p("reuse_approach_goal_on_retry", True)),
        tf_wait_timeout=float(p("tf_wait_timeout", 60.0)),
        tf_stable_s=float(p("tf_stable_s", 5.0)),
        nav2_wait_timeout=float(p("nav2_wait_timeout", 90.0)),
        vehicle_search_timeout_s=float(p("vehicle_search_timeout_s", 180.0)),
        tire_search_timeout_s=float(p("tire_search_timeout_s", 90.0)),
        approach_timeout_s=float(p("approach_timeout_s", 120.0)),
        nav_retry_budget=int(p("nav_retry_budget", 3)),
        max_rotation_attempts=int(p("max_rotation_attempts", 8)),
        max_state_repeats=int(p("max_state_repeats", 3)),
        dispatch_fail_abort_count=int(p("dispatch_fail_abort_count", 3)),
        planned_tire_fallback_enabled=bool(p("planned_tire_fallback_enabled", True)),
    )
