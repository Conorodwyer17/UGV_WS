import json
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from typing import List, Optional, Dict, Any, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Bool, Empty, Header, String  # String for segmentation_mode
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
import yaml
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs

from .utils import should_trigger_photo, distance_point_to_aabb_2d
from .mission_state_machine import MissionState, set_state, WaitStateContext
from .mission_policy import (
    TransitionReason,
    TargetLifecycle,
    DispatchFailureCode,
    MissionPolicy,
    StartupInvariants,
    load_policy_from_node,
)
from .geometry_utils import (
    pose_stamped_from_standoff_xy,
    quaternion_from_yaw,
    standoff_goal_robot_tyre_xy,
    standoff_goal_vehicle_center_tire_xy,
    yaw_from_quaternion,
)
from .perception_handler import (
    parse_vehicle_labels,
    log_bounding_box,
    find_vehicle_box,
    tire_position_label,
    find_tire_for_inspection,
)
from .goal_generator import COSTMAP_INSCRIBED_INFLATED, compute_box_goal
from .navigation_controller import send_follow_waypoints, send_nav_goal
from .vehicle_modeler import (
    box_front_rear_points,
    estimate_tire_positions,
    estimate_tire_positions_from_box,
    get_vehicle_footprint,
)
from .tyre_order import classify_robot_side, inspection_order_indices, robot_lon_lat
from .vehicle_waypoints import build_tyre_approach_waypoints
from .tyre_geometry import TyreBasedVehicleGeometry, tyre_geometry_from_poses
from .transformer import (
    get_current_pose as tf_get_current_pose,
    transform_pose as tf_transform_pose,
    transform_vector_xy as tf_transform_vector_xy,
)

class VehicleInspectionManager(Node):
    """Mission manager for autonomous vehicle tire inspection using Aurora SLAM and detection."""

    def __init__(self):
        super().__init__("inspection_manager")

        # Vehicle detection parameters - support both car and truck
        self.declare_parameter("vehicle_labels", "car,truck")  # Comma-separated string of vehicle class names (e.g., "car,truck")
        self.declare_parameter("tire_label", "tire")  # Colab best.pt uses "tire"; use "car-tire" for legacy; must match segment_3d
        self.declare_parameter("standoff_distance", 2.0)  # Distance to stop before vehicle
        self.declare_parameter("approach_offset", 0.5)  # Offset when approaching vehicle
        self.declare_parameter("tire_offset", 1.0)  # Standoff when approaching tire (m); tyre_3d uses robot–tyre geometry
        self.declare_parameter("tire_staging_min_m", 0.7)  # Min distance for tire approach (staging); then IBVS/ICP refinement for final approach
        self.declare_parameter("detection_topic", "/darknet_ros_3d/tire_bounding_boxes")  # Tire detection topic (dual perception default)
        self.declare_parameter("vehicle_detection_topic", "/darknet_ros_3d/vehicle_bounding_boxes")  # Vehicle boxes from YOLO 3D stream
        self.declare_parameter("vehicle_boxes_topic", "")  # Optional semantic vehicle topic override (e.g. /aurora_semantic/vehicle_bounding_boxes)
        self.declare_parameter("world_frame", "slamware_map")  # Must match bounding box frame from segmentation_processor
        self.declare_parameter("base_frame", "base_link")  # Robot base frame
        self.declare_parameter("map_frame", "map")  # Nav2 map frame
        self.declare_parameter("require_goal_transform", True)  # Require world->map transform for goals
        self.declare_parameter("detection_timeout", 5.0)  # seconds to wait before recovery (legacy)
        self.declare_parameter("detection_stale_s", 2.0)  # seconds without detection_topic messages -> stale
        self.declare_parameter("require_detection_topic_at_startup", True)  # if False, only vehicle_boxes + nav_permitted required (e.g. Aurora semantic only)
        self.declare_parameter("vehicle_boxes_stale_s", 2.0)  # seconds without vehicle_boxes_topic -> stale
        self.declare_parameter("vehicle_search_timeout_s", 180.0)  # total seconds to search before ending mission
        self.declare_parameter("tire_search_timeout_s", 90.0)  # seconds to wait for tire detection before fallback/next vehicle
        self.declare_parameter("vehicle_patrol_interval_s", 10.0)  # seconds between patrol steps when no vehicles
        self.declare_parameter("patrol_step_m", 0.8)  # meters per patrol step
        self.declare_parameter("patrol_turn_deg", 20.0)  # degrees to bias patrol heading
        self.declare_parameter("patrol_pause_s", 1.0)  # pause after patrol goal completes
        self.declare_parameter("patrol_max_attempts", 30)  # max patrol steps per mission
        self.declare_parameter("vehicle_confirmations_required", 3)  # consecutive frames before accepting vehicle
        self.declare_parameter("tire_confirmations_required", 2)  # consecutive frames before accepting tire
        self.declare_parameter("vehicle_confirm_tolerance_m", 1.0)  # meters to consider same vehicle across frames
        self.declare_parameter("tire_confirm_tolerance_m", 0.5)  # meters to consider same tire across frames
        self.declare_parameter("vehicle_min_distance_m", 0.5)  # meters from robot to accept vehicle
        self.declare_parameter("vehicle_max_distance_m", 20.0)  # meters from robot to accept vehicle
        self.declare_parameter("vehicle_max_distance_from_start_m", 20.0)  # max distance from mission-start robot position to accept vehicle (reduces false vehicles from drift/far objects)
        self.declare_parameter("vehicle_min_distance_confirmed_m", 0.2)  # allow closer vehicles once confirmed
        self.declare_parameter("vehicle_anchor_reach_distance_m", 1.4)  # treat approach as reached near anchor
        self.declare_parameter("approach_goal_tolerance_m", 0.35)  # treat approach as success if within this distance
        self.declare_parameter("tire_goal_tolerance_m", 0.25)  # treat tire approach as success if within this distance
        self.declare_parameter("planned_tire_fallback_enabled", True)  # allow planned tire goals if no tire boxes
        self.declare_parameter("strict_planned_tire_order", True)  # true = always use pop(0) for tires 2-4 (2nd→3rd→4th nearest); no detection reordering
        self.declare_parameter("planned_tire_box_extent_m", 0.25)  # half-extent for planned tire box
        self.declare_parameter("refinement_max_distance_m", 1.2)  # when a wheel detection is within this distance of planned/tyre3d tire, use detection center as goal
        self.declare_parameter("refinement_ema_alpha", 0.3)  # EMA smoothing for refinement (0=no smoothing, 1=full); reduces goal flicker from detection jitter
        self.declare_parameter("max_vehicle_reacquire_attempts", 2)  # bounded reacquire loops before moving to next vehicle
        self.declare_parameter("rotation_angle", 0.785)  # 45 degrees in radians
        self.declare_parameter("max_rotation_attempts", 8)  # 8 * 45 = 360 degrees
        self.declare_parameter("rotation_position_offset", 0.1)  # Small forward offset to force Nav2 execution
        # Motion watchdogs and cmd_vel validation
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_timeout_s", 2.0)  # seconds after nav goal with no cmd_vel
        self.declare_parameter("dispatch_fail_abort_count", 3)  # consecutive dispatch failures before ERROR
        self.declare_parameter("progress_stall_timeout_s", 5.0)  # seconds with no forward progress
        self.declare_parameter("progress_min_delta_m", 0.05)  # meters of progress required per window
        self.declare_parameter("spin_detect_yaw_deg", 90.0)  # degrees of rotation without progress
        self.declare_parameter("spin_detect_distance_m", 0.1)  # max translation during spin
        self.declare_parameter("spin_detect_min_time_s", 3.0)  # minimum window before spin detection
        self.declare_parameter("segmentation_mode_topic", "/segmentation_mode")  # Topic to control segmentation model
        self.declare_parameter("tire_position_tolerance", 0.5)  # meters - tires closer than this are considered the same
        self.declare_parameter("max_tire_distance_from_vehicle", 5.0)  # meters - tires beyond this are from other vehicles
        self.declare_parameter("vehicle_duplicate_tolerance", 2.0)  # meters - vehicles closer than this are same (PRODUCTION_CONFIG)
        self.declare_parameter("use_dynamic_detection", True)  # Use detection instead of YAML file
        self.declare_parameter("trucks_file", "")  # Optional YAML file (for backward compatibility, but not used if use_dynamic_detection=True)
        self.declare_parameter("min_vehicle_probability", 0.5)  # Minimum probability for vehicle detection
        self.declare_parameter("min_tire_probability", 0.35)  # Minimum probability for tire detection
        self.declare_parameter("tire_cache_stale_s", 5.0)  # Keep last seen tire candidates for this many seconds
        self.declare_parameter("tire_cache_match_tolerance_m", 0.6)  # Merge nearby tire detections into one cache entry
        self.declare_parameter("vehicle_cache_stale_s", 5.0)  # Keep last seen vehicle candidate for this many seconds
        self.declare_parameter("prefer_cached_tires", True)  # Use cache-first tire selection in WAIT_TIRE_BOX
        self.declare_parameter("photo_capture_topic", "/inspection_manager/capture_photo")  # Topic to trigger photo capture
        self.declare_parameter("capture_metadata_topic", "/inspection_manager/capture_metadata")  # JSON metadata for each capture (pose, tire_class, etc.)
        self.declare_parameter("capture_result_topic", "/inspection_manager/capture_result")  # SUCCESS|FAILURE,filename,bytes from photo_capture_service
        self.declare_parameter("photo_trigger_distance", 0.5)  # only trigger when within this m of goal; 0 disables
        self.declare_parameter("proximity_gate_distance_m", 0.3)  # log "approaching" when Nav2 feedback distance_remaining < this; 0 disables
        self.declare_parameter("max_recoveries_before_skip", 2)  # cancel and skip tire when number_of_recoveries >= this; 0 disables
        self.declare_parameter("recovery_skip_min_nav_time_s", 20.0)  # only skip if navigating at least this long (avoid false skip)
        self.declare_parameter("recovery_skip_distance_threshold_m", 0.5)  # only skip if distance_remaining > this (still far from goal)
        self.declare_parameter("capture_max_distance_to_tire_m", 1.2)  # max m from tire center to allow capture (goal is offset from box face, not center); 0 disables
        self.declare_parameter("capture_max_distance_to_tire_face_m", 0.35)  # max m from nearest tire box face to allow capture; adds robustness if TF drifts; 0 disables
        self.declare_parameter("face_tire_final_yaw", True)  # rotate in place to face tire before capture
        self.declare_parameter("face_tire_timeout_s", 20.0)  # max seconds to wait for final rotate
        self.declare_parameter("face_tire_yaw_tolerance", 0.2)  # radians; target yaw tolerance
        self.declare_parameter("face_tire_yaw_speed", 0.4)  # rad/s for final rotate
        self.declare_parameter("rotation_search_enabled", True)  # allow rotations when no detections
        self.declare_parameter("require_nav_permitted", False)  # gate nav when depth gate disallows motion
        self.declare_parameter("nav_permitted_topic", "/stereo/navigation_permitted")
        self.declare_parameter("turn_in_place_enabled", True)  # allow recovery rotations during wait states
        self.declare_parameter("vehicle_detected_topic", "/inspection_manager/vehicle_detected")  # JSON: vehicle_id, frame_id, x,y,z,yaw, confidence, timestamp (anchor)
        self.declare_parameter("max_state_repeats", 3)  # Spin protection: transition to ERROR if same state entered this many times consecutively
        self.declare_parameter("expected_tires_per_vehicle", 4)  # For mission completion validation
        self.declare_parameter("vehicle_wheelbase_m", 2.7)  # meters (car default)
        self.declare_parameter("vehicle_track_m", 1.6)  # meters (car default)
        self.declare_parameter("vehicle_wheelbase_truck_m", 3.5)  # 0 = use car default
        self.declare_parameter("vehicle_track_truck_m", 1.8)  # 0 = use car default
        self.declare_parameter("vehicle_wheelbase_bus_m", 4.0)  # 0 = use car default
        self.declare_parameter("vehicle_track_bus_m", 2.0)  # 0 = use car default
        self.declare_parameter("commit_min_extent_m", 0.5)  # reject box if either extent < this
        self.declare_parameter("commit_max_extent_m", 15.0)  # reject box if either extent > this
        self.declare_parameter("commit_max_aspect_ratio", 10.0)  # reject if max/min extent > this
        self.declare_parameter("tf_watchdog_timeout", 0.2)  # seconds - pause mission if TF fails > this
        self.declare_parameter("tf_unavailable_abort_s", 60.0)  # after this many seconds of continuous TF failure, transition to ERROR
        self.declare_parameter("tf_wait_timeout", 60.0)  # max seconds to wait for TF (slamware_map->base_link) in IDLE before starting mission
        self.declare_parameter("tf_stable_s", 3.0)  # require TF valid for this many seconds in IDLE before starting (avoids start-then-immediate-pause)
        self.declare_parameter("hard_mission_timeout", 1800.0)  # seconds - max mission duration
        self.declare_parameter("nav_retry_budget", 3)  # max nav failures before abort
        self.declare_parameter("approach_nav_retry_delay_s", 3.0)  # delay before retrying approach goal (avoids preemption storm; controller needs time)
        self.declare_parameter("nav_goal_min_interval_s", 1.0)  # min seconds between goal dispatches; 0=disabled; avoids flooding Nav2
        self.declare_parameter("wheel_detection_switch_distance_m", 1.0)  # switch to best_fallback.pt when within this distance of vehicle edge (bounding box = costmap)
        self.declare_parameter("approach_nearest_corner", True)  # true = go to nearest corner (tire); false = approach nearest face (can end at bumper)
        self.declare_parameter("try_next_corner_on_nav_fail", True)  # when corner unreachable, try next nearest instead of retrying same
        self.declare_parameter("skip_tire_on_nav_failure", True)  # when True, skip unreachable tire on first Nav2 FAILED/ABORTED (research: stop_on_failure: false)
        self.declare_parameter("use_follow_waypoints", False)  # when True, send 4 tire poses as FollowWaypoints batch (requires waypoint task plugin for capture)
        self.declare_parameter(
            "use_batch_waypoints",
            False,
        )  # when True, build full approach list (perimeter+standoff) in map frame and FollowWaypoints once
        self.declare_parameter("reuse_approach_goal_on_retry", True)  # keep approach goal stable across retries
        self.declare_parameter("max_capture_retries", 2)  # retry photo capture this many times on failure
        self.declare_parameter("capture_verify_timeout_s", 10.0)  # timeout waiting for capture_result in VERIFY_CAPTURE
        self.declare_parameter("save_directory", "~/ugv_ws/tire_inspection_photos")  # photo save dir (for mission report image_path)
        self.declare_parameter("capture_require_wheel_detection", True)  # at goal: switch to wheel model (best_fallback.pt), only capture if wheel in view
        self.declare_parameter("capture_wheel_wait_timeout_s", 8.0)  # max seconds to wait for wheel detection (YOLO ~2s/frame; allow time for 1–2 frames)
        self.declare_parameter("capture_wheel_required_frames", 1)  # number of detection frames with wheel in view before triggering photo (1 = one sighting enough)
        self.declare_parameter("capture_on_wheel_timeout", True)  # if True: on wheel-wait timeout still take photo and proceed; if False: skip this tire
        self.declare_parameter("dry_run", False)  # validate goals without sending to Nav2
        # No-motion / bench: skip photo_trigger and capture distance gates once capture is requested (stub motor).
        self.declare_parameter("demo_mode", False)
        # Empty msg on this topic acts like Nav2 success in INSPECT_TIRE when demo_mode.
        self.declare_parameter("demo_simulate_nav_success_topic", "")
        self.declare_parameter("enable_runtime_diagnostics", True)  # publish /inspection_manager/runtime_diagnostics at 5Hz
        self.declare_parameter("require_sensor_health", False)  # wait for aurora_interface/healthy before SEARCH_VEHICLE
        self.declare_parameter("sensor_health_timeout", 30.0)  # seconds to wait for sensor health in INIT
        self.declare_parameter("start_mission_on_ready", True)  # if False, stay in IDLE until /inspection_manager/start_mission is True
        self.declare_parameter("start_mission_topic", "/inspection_manager/start_mission")
        self.declare_parameter("nav2_wait_timeout", 90.0)  # max seconds to wait for Nav2 navigate_to_pose before starting mission (full-system-on-launch)
        self.declare_parameter("approach_timeout_s", 120.0)  # max seconds in APPROACH_VEHICLE before cancelling and retrying (avoids stuck nav)
        self.declare_parameter("goal_max_age_s", 5.0)  # reject goals computed from detections older than this
        self.declare_parameter("detection_stamp_max_age_s", 0.5)  # strict: reject detections older than this before creating goals (TF/time sync)
        self.declare_parameter("goal_costmap_precheck", True)  # reject goal if in occupied costmap cell (system_reaudit_report; reduces Nav2 failures)
        self.declare_parameter("goal_costmap_get_cost_service", "local_costmap/get_cost_local_costmap")  # Nav2 GetCosts service for precheck
        self.declare_parameter(
            "robot_pose_costmap_get_cost_service",
            "global_costmap/get_cost_global_costmap",
        )  # GetCosts on global costmap at robot pose (lethal start check)
        self.declare_parameter("pre_nav_lethal_escape_enabled", True)
        self.declare_parameter("pre_nav_lethal_max_backup_attempts", 2)
        self.declare_parameter("pre_nav_lethal_backup_distance_m", 0.35)
        self.declare_parameter("pre_nav_lethal_backup_speed_m", 0.12)
        self.declare_parameter("post_capture_backup_enable", True)
        self.declare_parameter("post_capture_backup_distance_m", 0.45)
        self.declare_parameter("post_capture_backup_speed_m", 0.12)
        self.declare_parameter("post_capture_costmap_settle_s", 0.35)
        self.declare_parameter("stop_cmd_vel_repeat", 3)
        self.declare_parameter("startup_detection_wait_timeout_s", 30.0)  # wait for first detection message before mission start; abort if exceeded
        self.declare_parameter("mission_log_path", "")  # JSONL event log (mission_start, state_transition, nav_result, etc.)
        self.declare_parameter("mission_report_path", "")  # full report JSON at mission end
        self.declare_parameter("mission_log_fsync", True)  # fsync each log entry for power-loss safety
        self.declare_parameter("reject_goal_offset_error_m", 0.15)  # reject goal if |goal_dist - offset| > this (0 = disabled)
        self.declare_parameter("odometry_filtered_topic", "")  # optional: when set, gate goals on pose covariance
        self.declare_parameter("max_pose_covariance_xy", 0.0)  # max allowed pose variance (x,y); 0 = disabled
        self.declare_parameter("centroid_servo_enabled", True)  # when True, hand off to centroid servo when distance_remaining < centroid_servo_proximity_m
        self.declare_parameter("centroid_servo_proximity_m", 0.5)  # hand off to centroid when distance_remaining < this
        self.declare_parameter("centroid_servo_require_detection_near_expected", True)  # only hand off when a tire detection is within capture_max_distance_detection_to_expected_tire_m of expected tire
        self.declare_parameter("capture_max_distance_detection_to_expected_tire_m", 1.0)  # max xy distance detection→expected for centroid/capture (wider when vehicle box is poor)
        self.declare_parameter("centroid_servo_wait_for_detection_timeout_s", 10.0)  # after this many seconds at tire, enable centroid even if no detection near expected (avoids blocking forever)
        self.declare_parameter("return_later_enabled", True)  # when True, retry skipped tires after completing remaining tires on same vehicle
        self.declare_parameter("max_return_later_passes", 1)  # how many retry passes for deferred tires before giving up
        self.declare_parameter("publish_mission_snapshot", False)  # when True, publish /inspection_debug/mission_snapshot at 1 Hz (observability_upgrade)
        # Direct tyre localisation (tyre_3d_projection_node): navigate from depth+YOLO without relying on vehicle boxes
        self.declare_parameter("use_tyre_3d_positions", False)
        self.declare_parameter("tyre_3d_positions_topic", "/tyre_3d_positions")
        self.declare_parameter("tyre_3d_stale_s", 2.0)  # max age of /tyre_3d_positions header to consider "fresh"
        self.declare_parameter("tyre_3d_min_range_m", 0.4)
        self.declare_parameter("tyre_3d_max_range_m", 5.0)
        self.declare_parameter(
            "prefer_tyre_3d_in_wait_tire_box",
            True,
        )  # WAIT_TIRE_BOX: dispatch nav to /tyre_3d_positions before planned box corners
        self.declare_parameter(
            "tyre_3d_prune_planned_radius_m",
            1.2,
        )  # remove planned corner near this (x,y) after a successful tyre_3d dispatch (avoids duplicate visits)
        # Real-motion: visit order around vehicle (FL,FR,RL,RR slots) + optional bridge poses for far-side tyres
        self.declare_parameter(
            "tyre_inspection_order_mode",
            "nearest",
        )  # nearest | vehicle_side — vehicle_side reorders FL/FR/RL/RR using robot side at commit
        self.declare_parameter("tyre_perimeter_bridge_enabled", False)  # tyre_3d_direct: insert front/rear bridge if opposite lateral side
        self.declare_parameter(
            "vehicle_perimeter_clearance_m",
            1.0,
        )  # m expanded half-axes for perimeter polyline (tune 1.0–1.2 for wide clearance)
        self.declare_parameter("tyre_perimeter_lateral_eps_m", 0.2)  # m: same-side if lateral within this of centerline
        self.declare_parameter(
            "costmap_clear_settle_s",
            1.0,
        )  # sleep after global/local costmap clear so freespace updates before next goal
        self.declare_parameter(
            "use_tyre_geometry",
            True,
        )  # infer vehicle axes / visit order from /tyre_3d_positions (stable) vs jittery vehicle box

        # Dynamic vehicle detection - vehicles found during mission
        self.detected_vehicles: List[dict] = []  # List of {"box": BoundingBox3d, "position": (x,y,z), "inspected": bool}
        self.current_vehicle_idx = 0
        self.current_state = MissionState.IDLE
        self.pending_goal_handle = None
        self._active_nav_goal_handle = None  # accepted Nav2 goal handle; cancelled when TF watchdog fires
        self.current_vehicle_box: Optional[BoundingBox3d] = None
        self.current_tire_idx = 0
        
        # Tire tracking (Phase F): unique ID, map coord, visited, image_captured
        self.inspected_tire_positions: List[tuple] = []  # legacy; kept for compatibility
        self._tire_registry: List[Dict[str, Any]] = []  # {id, vehicle_id, position, visited, image_captured}
        self._tire_id_counter = 0
        self._planned_tire_positions: List[tuple] = []  # planned tire centers for current vehicle
        self._pending_perimeter_nav_queue: List[PoseStamped] = []  # tyre_3d multi-segment approach (bridge → standoff)
        self._last_follow_waypoints_pose_count: int = 0
        self._last_batch_waypoints_perimeter: bool = False
        self._ordered_tyre_indices: List[int] = []
        self._tyre_geometry: Optional[TyreBasedVehicleGeometry] = None  # from tyre-only PCA / axle fit
        self._last_tyre_geometry_pose_count: int = 0

        # Recovery state tracking
        self.wait_start_time: Optional[float] = None
        self.rotation_attempts = 0
        self.initial_wait_yaw: Optional[float] = None  # Store yaw when starting to wait
        # Search/patrol tracking
        self._vehicle_search_start_time: Optional[float] = None
        self._tire_search_start_time: Optional[float] = None
        self._last_patrol_time: Optional[float] = None
        self._patrol_attempts: int = 0
        self._patrol_direction: int = 1
        self._vehicle_reacquire_attempts: int = 0

        # Spin protection: prevent infinite loops
        self._last_state: Optional[str] = None
        self._state_repeat_count = 0

        # Phase C: TF watchdog
        self._tf_last_valid_time: Optional[float] = None
        self._tf_watchdog_paused = False
        self._tf_watchdog_paused_since: Optional[float] = None  # when we first paused due to TF

        # Phase J: Safety locks
        self._mission_start_time: Optional[float] = None
        self._mission_start_robot_position: Optional[tuple] = None  # (x,y,z) in map at mission start; for vehicle proximity filter
        self._nav_retry_count = 0
        # Store last dispatched box/offset for nav retry on failure
        self._last_approach_box: Optional[BoundingBox3d] = None
        self._last_approach_offset: Optional[float] = None
        self._last_tire_box: Optional[BoundingBox3d] = None
        self._last_tire_offset: Optional[float] = None
        # Committed plan: once we lock a vehicle, all goals use this immutable state (no live jitter, no guesswork)
        self._committed_vehicle_box: Optional[BoundingBox3d] = None
        self._committed_vehicle_center: Optional[tuple] = None
        self._committed_vehicle_planned_tires: Optional[List[tuple]] = None  # 4 (x,y,z) in order FL,FR,RL,RR
        self._committed_vehicle_anchor_pose: Optional[dict] = None
        self._committed_vehicle_id: Optional[int] = None  # vehicle_id at commit time for validation
        # Detection confirmation
        self._vehicle_confirm_count = 0
        self._vehicle_confirm_center: Optional[tuple] = None
        self._tire_confirm_count = 0
        self._tire_confirm_center: Optional[tuple] = None
        # Progress stall tracking
        self._progress_stall_count = 0
        # Delayed approach retry: avoid sending a new goal immediately on abort (stops preemption storm)
        self._approach_retry_pending = False
        self._approach_retry_at: Optional[float] = None

        # In WAIT_VEHICLE_BOX: dispatch approach once per entry when we have current_vehicle_box (stops spin loop after rotation)
        self._dispatched_approach_this_wait = False
        # TF wait in IDLE: do not start mission until TF valid (Aurora ready) and stable
        self._tf_wait_start_time: Optional[float] = None
        self._tf_wait_last_log_elapsed = 0.0
        self._tf_stable_since: Optional[float] = None  # when we first saw TF valid; need stable for tf_stable_s
        # Mission start: when start_mission_on_ready is False, wait for this before leaving IDLE
        self._mission_start_requested = False
        # Nav2 wait (full-system-on-launch): wait for navigate_to_pose before leaving IDLE
        self._nav2_wait_start_time: Optional[float] = None
        self._nav2_wait_last_log_elapsed = 0.0
        # Mission log file paths (empty = disabled)
        self._mission_log_path = (self.get_parameter("mission_log_path").value or "").strip()
        self._mission_report_path = (self.get_parameter("mission_report_path").value or "").strip()
        self._mission_log_fsync = bool(self.get_parameter("mission_log_fsync").value)
        # Comprehensive logging: throttle tick_skipped and heartbeat
        self._tf_skip_log_last_time: Optional[float] = None
        self._mission_heartbeat_last_time: Optional[float] = None
        # Nav2 action feedback (distance_remaining, number_of_recoveries) for proximity gating and recovery-aware skip
        self._last_nav_feedback: Optional[Any] = None  # NavigateToPose.FeedbackMessage.feedback
        self._proximity_gate_passed: bool = False  # True when distance_remaining < proximity_gate_distance_m
        self._centroid_handoff_initiated: bool = False  # True when we cancelled Nav2 to hand off to centroid servo
        self._last_proximity_log_time: Optional[float] = None  # throttle "approaching" log
        self._recovery_skip_initiated: bool = False  # True when we cancelled goal for recovery-aware skip

        # Phase A: Last detection confidence for diagnostics
        self._last_detection_confidence: Optional[float] = None
        # Phase A: Detection stream health
        self._last_detection_msg_time: Optional[float] = None
        self._last_detection_stamp = None  # rclpy.time.Time from detection msg header; for TF/stamp checks
        self._last_tire_boxes: Optional[List[BoundingBox3d]] = None  # latest tire detections for spatial filter (centroid handoff)
        self._last_vehicle_boxes_stamp = None  # rclpy.time.Time from vehicle_boxes msg header
        self._pose_covariance_ok = True  # gated by odometry_filtered_topic when max_pose_covariance_xy > 0
        self._last_vehicle_boxes_time: Optional[float] = None
        self._detection_interval_ema: Optional[float] = None
        self._vehicle_boxes_interval_ema: Optional[float] = None
        self._last_detection_stale_log_time: Optional[float] = None
        self._last_vehicle_stale_log_time: Optional[float] = None
        self._last_tire_cache_log_time: Optional[float] = None
        # Perception fusion cache: keep vehicle/tire context alive across transitions
        self._vehicle_cache: Dict[str, Any] = {}  # {'box': BoundingBox3d, 'center': (x,y,z), 'last_seen': t, 'confidence_ema': p}
        self._tire_cache: List[Dict[str, Any]] = []  # [{'box', 'center', 'last_seen', 'confidence_ema'}]
        # Refinement EMA: smooth detection-based goal to reduce flicker (key = rounded target, value = smoothed center)
        self._refinement_ema_cache: Dict[tuple, tuple] = {}

        # Phase A/D: Current goal for diagnostics and dry-run
        self._current_goal_pose: Optional[PoseStamped] = None

        # Cmd_vel monitoring
        self._last_cmd_vel_time: Optional[float] = None
        self._last_cmd_vel_msg: Optional[Twist] = None
        self._last_cmd_vel_source: Optional[str] = None

        # Dispatch and progress watchdogs
        self._dispatch_fail_count = 0
        self._last_goal_dispatch_time: Optional[float] = None
        self._progress_window_start: Optional[float] = None
        self._progress_start_pose: Optional[tuple] = None  # (x, y, yaw)

        # Phase A: State transition cause
        self._last_transition_cause = "init"
        # Last failure reason from _dispatch_box_goal (for mission log)
        self._last_dispatch_fail_reason: Optional[str] = None

        # APPROACH_VEHICLE / INSPECT_TIRE: timeout so we don't wait forever if Nav2 hangs
        self._approach_entered_time: Optional[float] = None
        self._face_tire_start_time: Optional[float] = None
        # Per-tire metrics for mission report (observability_upgrade)
        self._current_tire_approach_start_time: Optional[float] = None
        self._current_tire_nav_retries: int = 0
        self._face_tire_target_yaw: Optional[float] = None

        # INIT: sensor health (aurora_interface/healthy)
        self._sensor_healthy: Optional[bool] = None
        # Depth gate / nav permitted (from aurora_sdk_bridge)
        self._nav_permitted: Optional[bool] = None
        self._last_nav_permitted_time: Optional[float] = None

        # Mission report data
        self._total_tires_captured = 0
        self._tires_skipped: List[Dict[str, Any]] = []  # [{tire_id, tire_position, reason}]; determinism audit 1.3
        self._tires_deferred: List[Dict[str, Any]] = []  # [{position, vehicle_id, tire_position_label}] for return-later retry
        self._return_later_pass_count: int = 0  # number of return-later passes done for current vehicle (0 = first pass)
        self._tire_capture_log: List[Dict[str, Any]] = []  # Per-tire: {tire_index, success, image_path, distance_m, timestamp, reason}
        self._last_capture_distance_m: Optional[float] = None
        self._last_capture_dist_to_tire_m: Optional[float] = None
        self._mission_report = {
            "total_vehicles": 0,
            "total_tires_expected": 0,
            "total_tires_captured": 0,
            "tires_skipped": [],
            "tires_deferred_retried": 0,
            "tires_deferred_still_skipped": 0,
            "error_states_encountered": 0,
        }

        # Mission policy (centralized thresholds and fallbacks)
        self.policy = load_policy_from_node(self)
        self._startup_invariants = StartupInvariants()
        self._startup_detection_wait_start_time: Optional[float] = None
        self._startup_topics_last_log_time: float = 0.0
        self._wait_context = None  # WaitStateContext for WAIT_* states
        self._target_lifecycle = TargetLifecycle.DETECTED.value
        self._last_tf_pose_sample: Optional[tuple] = None
        self._last_segmentation_mode_publish_time: float = 0.0  # periodic "inspection" in WAIT_TIRE_BOX
        self._wheel_switch_during_approach_done: bool = False  # switch to best_fallback.pt when within wheel_detection_switch_distance_m
        self._wheel_wait_start_time: Optional[float] = None  # WAIT_WHEEL_FOR_CAPTURE: when we started waiting
        self._wheel_confirm_count: int = 0  # WAIT_WHEEL_FOR_CAPTURE: frames with wheel in view
        self._wheel_capture_inspection_published: bool = False  # ensure we publish "inspection" once when entering WAIT_WHEEL_FOR_CAPTURE

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        # RELIABLE for Aurora semantic vehicle_boxes (publisher uses default RELIABLE); avoids "topic not alive" at startup.
        qos_reliable = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        detection_topic = self.get_parameter("detection_topic").value
        self.detection_sub = self.create_subscription(
            BoundingBoxes3d, detection_topic, self._detection_cb, qos_profile=qos
        )
        self.get_logger().info(f"Using tire detection topic: {detection_topic}")
        # Vehicle boxes come from semantic topic when configured; otherwise from YOLO vehicle stream.
        vehicle_detection_topic = self.get_parameter("vehicle_detection_topic").value
        vehicle_boxes_topic = self.get_parameter("vehicle_boxes_topic").value
        self._vehicle_boxes_sub = None
        vehicle_topic_to_subscribe = vehicle_boxes_topic if vehicle_boxes_topic else vehicle_detection_topic
        if vehicle_topic_to_subscribe:
            # Use RELIABLE when subscribing to Aurora semantic (vehicle_boxes_topic) so we match publisher and receive reliably.
            vehicle_qos = qos_reliable if vehicle_boxes_topic else qos
            self._vehicle_boxes_sub = self.create_subscription(
                BoundingBoxes3d, vehicle_topic_to_subscribe, self._vehicle_boxes_cb, qos_profile=vehicle_qos
            )
            if vehicle_boxes_topic:
                self.get_logger().info(f"Using semantic vehicle detection: {vehicle_boxes_topic}")
            else:
                self.get_logger().info(f"Using vehicle detection topic: {vehicle_detection_topic}")

        # Publish current FSM state for debugging.
        self.state_pub = self.create_publisher(String, "inspection_state", 10)
        # Mission report at completion
        self.mission_report_pub = self.create_publisher(String, "inspection_manager/mission_report", 10)
        # Phase A: Runtime diagnostics (JSON at 5Hz)
        self.diagnostics_pub = self.create_publisher(String, "inspection_manager/runtime_diagnostics", 10)
        self.approach_markers_pub = self.create_publisher(
            MarkerArray, "inspection_manager/approach_markers", 10
        )
        self.current_goal_pub = self.create_publisher(
            PoseStamped, "inspection_manager/current_goal", 10
        )
        self.centroid_servo_enable_pub = self.create_publisher(
            Bool, "inspection_manager/centroid_servo_enable", 10
        )
        self._publish_mission_snapshot = bool(self.get_parameter("publish_mission_snapshot").value)
        self._mission_snapshot_pub = (
            self.create_publisher(String, "inspection_debug/mission_snapshot", 10)
            if self._publish_mission_snapshot else None
        )

        # Publish segmentation mode to control which model the segmentation node uses
        segmentation_mode_topic = self.get_parameter("segmentation_mode_topic").value
        self.segmentation_mode_pub = self.create_publisher(String, segmentation_mode_topic, 10)
        self.get_logger().info(f"Publishing segmentation mode to: {segmentation_mode_topic}")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        use_fw = bool(self.get_parameter("use_follow_waypoints").value)
        use_batch_wp = bool(self.get_parameter("use_batch_waypoints").value)
        self.follow_waypoints_client = (
            ActionClient(self, FollowWaypoints, "follow_waypoints")
            if (use_fw or use_batch_wp)
            else None
        )
        self._active_follow_waypoints_handle = None
        
        # TF buffer and listener for getting robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Photo capture publisher
        self.photo_capture_pub = self.create_publisher(Bool, self.get_parameter("photo_capture_topic").value, 10)
        # Capture metadata (map pose, timestamp, tire_class) for persistence alongside images
        self.capture_metadata_pub = self.create_publisher(
            String, self.get_parameter("capture_metadata_topic").value, 10
        )
        # Vehicle anchor: when a vehicle is first detected, publish pose in map frame for mission anchoring
        self.vehicle_detected_pub = self.create_publisher(
            String, self.get_parameter("vehicle_detected_topic").value, 10
        )

        self._health_sub = self.create_subscription(
            Bool, "aurora_interface/healthy", self._health_cb, 10
        )
        self._nav_permitted_sub = self.create_subscription(
            Bool,
            self.get_parameter("nav_permitted_topic").value,
            self._nav_permitted_cb,
            10,
        )
        # Subscribe to capture result for VERIFY_CAPTURE state
        self.capture_result_sub = self.create_subscription(
            String,
            self.get_parameter("capture_result_topic").value,
            self._capture_result_cb,
            10,
        )
        # Subscribe to centroid_centered for handoff from centroid servo to WAIT_WHEEL_FOR_CAPTURE
        self.centroid_centered_sub = self.create_subscription(
            Bool,
            "inspection_manager/centroid_centered",
            self._centroid_centered_cb,
            10,
        )
        # Optional: pose covariance gating (e.g. from /odometry/filtered when using robot_localization)
        odom_filtered_topic = self.get_parameter("odometry_filtered_topic").value
        if odom_filtered_topic and self.get_parameter("max_pose_covariance_xy").value > 0:
            self._odom_filtered_sub = self.create_subscription(
                Odometry,
                odom_filtered_topic,
                self._odometry_filtered_cb,
                10,
            )
            self.get_logger().info(
                f"Pose covariance gating: {odom_filtered_topic} (max_pose_covariance_xy={self.get_parameter('max_pose_covariance_xy').value})"
            )
        else:
            self._odom_filtered_sub = None
        # Monitor cmd_vel to detect missing/blocked motion after goal dispatch
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            lambda msg: self._cmd_vel_cb(msg, self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self._cmd_vel_nav_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_nav_topic").value,
            lambda msg: self._cmd_vel_cb(msg, self.get_parameter("cmd_vel_nav_topic").value),
            10,
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("start_mission_topic").value,
            self._start_mission_cb,
            10,
        )
        self._capture_retry_count = 0
        self._verify_capture_start_time = None

        self._tyre_3d_poses = None
        self._tyre_3d_stamp = None
        self.create_subscription(
            PoseArray,
            self.get_parameter("tyre_3d_positions_topic").value,
            self._tyre_3d_cb,
            10,
        )
        demo_nav_topic = str(self.get_parameter("demo_simulate_nav_success_topic").value or "").strip()
        self._demo_nav_sub = None
        if demo_nav_topic:
            self._demo_nav_sub = self.create_subscription(
                Empty, demo_nav_topic, self._on_demo_simulated_nav_success, 10
            )
            self.get_logger().info(
                f"demo_simulate_nav_success_topic={demo_nav_topic} "
                "(Empty message → treat as Nav2 arrival in INSPECT_TIRE when demo_mode is true)"
            )

        self.timer = self.create_timer(1.0, self._tick)
        if self.get_parameter("enable_runtime_diagnostics").value:
            self.diagnostics_timer = self.create_timer(0.2, self._publish_runtime_diagnostics)  # 5Hz
        if self._publish_mission_snapshot and self._mission_snapshot_pub is not None:
            self._mission_snapshot_timer = self.create_timer(1.0, self._publish_mission_snapshot_cb)
        
        use_dynamic = self.get_parameter("use_dynamic_detection").value
        if use_dynamic:
            self.get_logger().info("Using dynamic vehicle detection from bounding boxes (Aurora-based)")
        else:
            trucks = self._load_trucks()
            self.get_logger().info(f"Loaded {len(trucks)} vehicles from YAML file (legacy mode)")

        if self._is_dry_run():
            self.get_logger().info("dry_run=True: goals will be validated but not sent to Nav2; approach will simulate arrival -> WAIT_TIRE_BOX")

        # Log key offsets for runtime verification (PRODUCTION_CONFIG override check)
        ao = float(self.get_parameter("approach_offset").value)
        to = float(self.get_parameter("tire_offset").value)
        self.get_logger().info(f"Loaded approach_offset={ao:.3f}m tire_offset={to:.3f}m (verify PRODUCTION_CONFIG if unexpected)")

        # Publish initial state.
        self._set_state(MissionState.IDLE)
        self._validate_startup_config()

    def _health_cb(self, msg: Bool):
        self._sensor_healthy = msg.data

    def _on_demo_simulated_nav_success(self, _msg: Empty) -> None:
        """Bench: Empty on demo_simulate_nav_success_topic acts like Nav2 arrival (stub motor never reaches goal)."""
        if not self._demo_mode_enabled():
            return
        if self.current_state != MissionState.INSPECT_TIRE:
            return
        if self._active_nav_goal_handle is not None:
            try:
                self._active_nav_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"demo_simulated_nav: cancel_goal_async failed: {e}")
            self._active_nav_goal_handle = None
        self._mission_log_append(
            "demo_simulated_nav_success",
            {"state": str(self.current_state)},
            sync=True,
        )
        self.get_logger().info("demo_simulated_nav_success: treating as Nav2 goal reached")
        self._handle_box_goal_success("demo_simulated_nav_success")

    def _validate_startup_config(self) -> None:
        """Hard-fail on unsafe or inconsistent startup configuration."""
        errors = []
        if float(self.get_parameter("goal_max_age_s").value) <= 0.0:
            errors.append("goal_max_age_s must be > 0")
        dsmas = float(self.get_parameter("detection_stamp_max_age_s").value)
        if dsmas < 0.0:
            errors.append("detection_stamp_max_age_s must be >= 0 (0 = disable strict stamp check)")
        if float(self.get_parameter("approach_offset").value) <= 0.0:
            errors.append("approach_offset must be > 0")
        if float(self.get_parameter("tire_offset").value) <= 0.0:
            errors.append("tire_offset must be > 0")
        tsm = float(self.get_parameter("tire_staging_min_m").value)
        if tsm < 0.0:
            errors.append("tire_staging_min_m must be >= 0")
        if int(self.get_parameter("vehicle_confirmations_required").value) < 1:
            errors.append("vehicle_confirmations_required must be >= 1")
        if int(self.get_parameter("tire_confirmations_required").value) < 1:
            errors.append("tire_confirmations_required must be >= 1")
        if errors:
            self._startup_invariants.config_valid = False
            joined = "; ".join(errors)
            self.get_logger().error(f"Startup config validation failed: {joined}")
            self._mission_log_append(
                "config_validation_failed",
                {"errors": errors},
                sync=True,
            )
            self._mission_report["error_states_encountered"] += 1
            self._set_state(MissionState.ERROR, cause=TransitionReason.CONFIG_VALIDATION_FAILED.value)

    def _nav_permitted_cb(self, msg: Bool):
        self._nav_permitted = msg.data
        self._last_nav_permitted_time = time.time()

    def _centroid_centered_cb(self, msg: Bool):
        """When centroid servo reports centered, transition to WAIT_WHEEL_FOR_CAPTURE."""
        if not msg.data:
            return
        if (
            self.current_state == MissionState.INSPECT_TIRE
            and self._centroid_handoff_initiated
        ):
            self._centroid_handoff_initiated = False
            disable_msg = Bool()
            disable_msg.data = False
            self.centroid_servo_enable_pub.publish(disable_msg)
            self.get_logger().info("Centroid centered; transitioning to WAIT_WHEEL_FOR_CAPTURE.")
            if self.get_parameter("capture_require_wheel_detection").value:
                self._set_state(MissionState.WAIT_WHEEL_FOR_CAPTURE, cause="centroid_centered")
            else:
                self._trigger_tire_capture()

    def _capture_result_cb(self, msg: String):
        """Handle capture_result (SUCCESS|FAILURE,filename,bytes). In VERIFY_CAPTURE state, finish or retry."""
        if self.current_state != MissionState.VERIFY_CAPTURE:
            return
        parts = msg.data.split(",", 2)
        success = len(parts) >= 1 and parts[0].strip().upper() == "SUCCESS"
        filename = parts[1].strip() if len(parts) > 1 else ""
        size_bytes = int(parts[2].strip()) if len(parts) > 2 and parts[2].strip().isdigit() else 0
        if success:
            self._finish_verify_capture_success(filename, size_bytes)
        else:
            max_retries = self.get_parameter("max_capture_retries").value
            if self._capture_retry_count < max_retries:
                self._capture_retry_count += 1
                self.get_logger().warn(f"Capture FAILURE; retry {self._capture_retry_count}/{int(max_retries)}")
                photo_msg = Bool()
                photo_msg.data = True
                self.photo_capture_pub.publish(photo_msg)
                self._verify_capture_start_time = time.time()
            else:
                self._finish_verify_capture_failure(f"capture failed after {int(max_retries)} retries")

    def _finish_verify_capture_success(self, filename: str = "", size_bytes: int = 0):
        """Mark tire capture done and transition to next tire or next vehicle."""
        try:
            save_dir = os.path.expanduser(str(self.get_parameter("save_directory").value))
        except Exception:
            workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
            save_dir = os.path.join(workspace, "tire_inspection_photos")
        image_path = os.path.join(save_dir, filename) if filename else ""
        approach_time_s = (
            (time.time() - self._current_tire_approach_start_time)
            if self._current_tire_approach_start_time else None
        )
        self._tire_capture_log.append({
            "tire_index": self._total_tires_captured,
            "success": True,
            "image_path": image_path,
            "filename": filename,
            "distance_to_goal_m": self._last_capture_distance_m,
            "distance_to_tire_m": self._last_capture_dist_to_tire_m,
            "timestamp": time.time(),
            "size_bytes": size_bytes,
            "approach_time_s": round(approach_time_s, 2) if approach_time_s is not None else None,
            "retries": self._current_tire_nav_retries,
            "goal_source": self._tire_registry[-1].get("goal_source", "") if self._tire_registry else "",
        })
        if self._tire_registry:
            self._tire_registry[-1]["image_captured"] = True
            entry = self._tire_registry[-1]
            self._mission_log_append(
                "photo_captured",
                {
                    "filename": filename,
                    "bytes": size_bytes,
                    "tire_id": entry.get("id"),
                    "vehicle_id": entry.get("vehicle_id"),
                    "tire_position": entry.get("tire_position", ""),
                },
                sync=True,
            )
        self.get_logger().info(f"Capture verified: {filename} ({size_bytes} bytes)")
        exp = int(self.get_parameter("expected_tires_per_vehicle").value)
        # Multi-tire flow: until inspected count reaches expected_tires_per_vehicle, return to WAIT_TIRE_BOX for the next planned/tyre_3d goal (pop in tick).
        if len(self.inspected_tire_positions) >= exp:
            if self._should_retry_deferred_tires():
                self._requeue_deferred_tires()
                self._post_capture_backup_sync()
                self._clear_costmaps()
                self._set_state(MissionState.WAIT_TIRE_BOX, cause="return_later_requeue")
                return
            self.get_logger().info(f"Completed inspection of {len(self.inspected_tire_positions)} tires. Moving to next vehicle.")
            self._set_state(MissionState.NEXT_VEHICLE, cause="verify_success")
        else:
            self._post_capture_backup_sync()
            self._clear_costmaps()
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="verify_success")

    def _finish_verify_capture_failure(self, reason: str):
        """Proceed after capture failure (retries exhausted or timeout); still advance to next tire/vehicle."""
        approach_time_s = (
            (time.time() - self._current_tire_approach_start_time)
            if self._current_tire_approach_start_time else None
        )
        self._tire_capture_log.append({
            "tire_index": self._total_tires_captured,
            "success": False,
            "image_path": "",
            "reason": reason,
            "distance_to_goal_m": self._last_capture_distance_m,
            "distance_to_tire_m": self._last_capture_dist_to_tire_m,
            "timestamp": time.time(),
            "approach_time_s": round(approach_time_s, 2) if approach_time_s is not None else None,
            "retries": self._current_tire_nav_retries,
            "goal_source": self._tire_registry[-1].get("goal_source", "") if self._tire_registry else "",
        })
        self.get_logger().warn(f"Verify capture failure: {reason}; proceeding to next target")
        self._mission_log_append(
            "capture_failure",
            {"reason": reason, "retry_count": self._capture_retry_count},
            sync=True,
        )
        if self._tire_registry:
            self._tire_registry[-1]["image_captured"] = False
        if len(self.inspected_tire_positions) >= self.get_parameter("expected_tires_per_vehicle").value:
            if self._should_retry_deferred_tires():
                self._requeue_deferred_tires()
                self._set_state(MissionState.WAIT_TIRE_BOX, cause="return_later_requeue")
                return
            self._set_state(MissionState.NEXT_VEHICLE, cause="verify_failure")
        else:
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="verify_failure")

    # ----------------------- State Helpers ----------------------- #
    def _is_dry_run(self) -> bool:
        """Return True if dry_run is enabled (accepts bool or string 'true'/'1' from launch)."""
        v = self.get_parameter("dry_run").value
        if isinstance(v, bool):
            return v
        if isinstance(v, str):
            return v.strip().lower() in ("true", "1", "yes")
        return bool(v)

    def _demo_mode_enabled(self) -> bool:
        """stable_viz / bench: bypass photo distance gates (launch may pass string 'true')."""
        v = self.get_parameter("demo_mode").value
        if isinstance(v, bool):
            return v
        if isinstance(v, str):
            return v.strip().lower() in ("true", "1", "yes")
        return bool(v)

    def _stop_robot(self) -> None:
        """Publish zero cmd_vel so the base stops if Nav2/controller left a residual command."""
        try:
            n = int(self.get_parameter("stop_cmd_vel_repeat").value)
        except Exception:
            n = 3
        n = max(1, min(n, 10))
        z = Twist()
        for _ in range(n):
            self._cmd_vel_pub.publish(z)

    def _open_loop_backup_distance(self, distance_m: float, speed_m: float) -> None:
        """Timed reverse motion along base_link +x (negative linear.x). Open-loop; no odometry."""
        if distance_m <= 0 or speed_m <= 0:
            return
        duration = distance_m / speed_m
        t_end = time.time() + duration
        msg = Twist()
        msg.linear.x = -abs(speed_m)
        rate_hz = 20.0
        dt = 1.0 / rate_hz
        while time.time() < t_end:
            self._cmd_vel_pub.publish(msg)
            time.sleep(dt)
        self._stop_robot()

    def _robot_cost_from_global_costmap(self) -> Optional[float]:
        """Footprint cost from global costmap at current map pose, or None if unavailable."""
        try:
            from nav2_msgs.srv import GetCosts
        except Exception:
            return None
        pose = self._get_current_pose_in_map_frame()
        if pose is None:
            return None
        map_frame = self.get_parameter("map_frame").value
        pose.header.frame_id = map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        svc = str(self.get_parameter("robot_pose_costmap_get_cost_service").value)

        def _do_call() -> Optional[float]:
            cli = self.create_client(GetCosts, svc)
            if not cli.wait_for_service(timeout_sec=0.5):
                return None
            req = GetCosts.Request()
            req.use_footprint = True
            req.poses = [pose]
            resp = cli.call(req)
            if not resp.success or not resp.costs:
                return None
            return float(resp.costs[0])

        try:
            with ThreadPoolExecutor(max_workers=1) as ex:
                fut = ex.submit(_do_call)
                return fut.result(timeout=1.5)
        except (FuturesTimeoutError, Exception):
            return None

    def _is_robot_pose_lethal(self) -> Optional[bool]:
        """True if footprint is in lethal/inscribed band; False if clearly free; None if unknown."""
        c = self._robot_cost_from_global_costmap()
        if c is None:
            return None
        return c >= COSTMAP_INSCRIBED_INFLATED

    def escape_lethal_start_if_needed(self) -> bool:
        """Before NavigateToPose: if start pose is lethal on global costmap, back up and retry."""
        if not bool(self.get_parameter("pre_nav_lethal_escape_enabled").value):
            return True
        if self._is_dry_run():
            return True
        max_att = int(self.get_parameter("pre_nav_lethal_max_backup_attempts").value)
        max_att = max(1, min(max_att, 5))
        dist = float(self.get_parameter("pre_nav_lethal_backup_distance_m").value)
        speed = float(self.get_parameter("pre_nav_lethal_backup_speed_m").value)
        settle = float(self.get_parameter("post_capture_costmap_settle_s").value)
        for attempt in range(max_att):
            lethal = self._is_robot_pose_lethal()
            if lethal is False or lethal is None:
                return True
            self.get_logger().warn(
                f"Start pose in lethal global cost; backup attempt {attempt + 1}/{max_att} ({dist:.2f} m)"
            )
            self._mission_log_append(
                "pre_nav_lethal_backup",
                {"attempt": attempt + 1, "max": max_att, "distance_m": dist},
                sync=True,
            )
            self._stop_robot()
            self._open_loop_backup_distance(dist, speed)
            if settle > 0:
                time.sleep(settle)
        still = self._is_robot_pose_lethal()
        if still is True:
            self.get_logger().error(
                "Robot still in lethal cost after backup attempts; refusing nav goal."
            )
            self._mission_log_append(
                "pre_nav_lethal_refuse",
                {"reason": "still_lethal_after_backup"},
                sync=True,
            )
            return False
        return True

    def _post_capture_backup_sync(self) -> None:
        """After a successful tyre photo, reverse slightly before the next Nav2 goal (same vehicle)."""
        if not bool(self.get_parameter("post_capture_backup_enable").value):
            return
        if self._is_dry_run():
            return
        dist = float(self.get_parameter("post_capture_backup_distance_m").value)
        speed = float(self.get_parameter("post_capture_backup_speed_m").value)
        settle = float(self.get_parameter("post_capture_costmap_settle_s").value)
        if dist <= 0:
            return
        self._mission_log_append(
            "post_capture_backup",
            {"distance_m": dist, "speed_m": speed},
            sync=True,
        )
        self._stop_robot()
        self._open_loop_backup_distance(dist, speed)
        if settle > 0:
            time.sleep(settle)

    def _clear_costmaps(self) -> None:
        """Clear global and local costmaps before the next tyre goal (drops stale obstacle marks)."""
        if self._is_dry_run():
            return
        try:
            if not hasattr(self, "_clear_global_costmap_client"):
                self._clear_global_costmap_client = self.create_client(
                    ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap"
                )
                self._clear_local_costmap_client = self.create_client(
                    ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap"
                )
            clear_global = self._clear_global_costmap_client
            clear_local = self._clear_local_costmap_client
            if not clear_global.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("global_costmap clear service not available")
            if not clear_local.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("local_costmap clear service not available")
            clear_global.call_async(ClearEntireCostmap.Request())
            clear_local.call_async(ClearEntireCostmap.Request())
            settle = float(self.get_parameter("costmap_clear_settle_s").value)
            if settle < 0.0:
                settle = 0.0
            time.sleep(settle)
            self.get_logger().info(
                f"Cleared global and local costmaps (settle {settle:.2f}s) after tyre capture backup."
            )
            self._mission_log_append("costmaps_cleared", {"after": "post_capture_backup"}, sync=True)
        except Exception as e:
            self.get_logger().warn(f"Costmap clearing failed: {e}")

    def _set_state(self, new_state: str, cause: str = "unknown"):
        """Update internal state, log transition, and publish for debugging."""
        set_state(self, new_state, cause)

    def _publish_runtime_diagnostics(self) -> None:
        """Phase A: Publish structured JSON diagnostics at 5Hz."""
        try:
            t0 = time.perf_counter()
            world_frame = self.get_parameter("world_frame").value
            base_frame = self.get_parameter("base_frame").value

            robot_pose = None
            tf_latency_ms = None
            tf_valid = False
            try:
                t_start = time.perf_counter()
                transform = self.tf_buffer.lookup_transform(
                    world_frame, base_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                tf_latency_ms = (time.perf_counter() - t_start) * 1000
                tf_valid = True
                self._tf_last_valid_time = time.time()
                robot_pose = {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                }
            except Exception as e:
                if self._tf_last_valid_time is not None and (time.time() - self._tf_last_valid_time) > self.get_parameter("tf_watchdog_timeout").value:
                    pass  # Will be handled in watchdog

            goal_pose = None
            distance_to_goal = None
            if self._current_goal_pose is not None and robot_pose is not None:
                g = self._current_goal_pose.pose.position
                goal_pose = {"x": g.x, "y": g.y, "z": g.z}
                distance_to_goal = math.sqrt(
                    (g.x - robot_pose["x"]) ** 2 + (g.y - robot_pose["y"]) ** 2
                )

            timeout_remaining = None
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout_s = self.get_parameter("detection_timeout").value
                timeout_remaining = max(0.0, timeout_s - elapsed)

            now = time.time()
            detection_stale_s = float(self.get_parameter("detection_stale_s").value)
            vehicle_boxes_stale_s = float(self.get_parameter("vehicle_boxes_stale_s").value)
            tire_detection_age = None if self._last_detection_msg_time is None else (now - self._last_detection_msg_time)
            vehicle_boxes_age = None if self._last_vehicle_boxes_time is None else (now - self._last_vehicle_boxes_time)
            detection_stale = tire_detection_age is None or tire_detection_age > detection_stale_s
            vehicle_boxes_stale = vehicle_boxes_age is None or vehicle_boxes_age > vehicle_boxes_stale_s
            require_det_at_start = self.get_parameter("require_detection_topic_at_startup").value
            if require_det_at_start and detection_stale and (self._last_detection_stale_log_time is None or (now - self._last_detection_stale_log_time) > 5.0):
                self._mission_log_append(
                    "detection_stream_stale",
                    {
                        "age_s": tire_detection_age,
                        "threshold_s": detection_stale_s,
                        "state": self.current_state,
                        "topic": self.get_parameter("detection_topic").value,
                    },
                    sync=True,
                )
                self._last_detection_stale_log_time = now
            if vehicle_boxes_stale and (self._last_vehicle_stale_log_time is None or (now - self._last_vehicle_stale_log_time) > 5.0):
                self._mission_log_append(
                    "vehicle_boxes_stream_stale",
                    {
                        "age_s": vehicle_boxes_age,
                        "threshold_s": vehicle_boxes_stale_s,
                        "state": self.current_state,
                        "topic": self.get_parameter("vehicle_boxes_topic").value,
                    },
                    sync=True,
                )
                self._last_vehicle_stale_log_time = now

            diag = {
                "timestamp": time.time(),
                "current_state": self.current_state,
                "current_vehicle_id": self.current_vehicle_idx if self.detected_vehicles else None,
                "current_tire_id": self._tire_id_counter,
                "robot_pose": robot_pose,
                "goal_pose": goal_pose,
                "distance_to_goal": distance_to_goal,
                "state_timeout_remaining": timeout_remaining,
                "retry_counter": self.rotation_attempts,
                "nav_retry_count": self._nav_retry_count,
                "detection_confidence": self._last_detection_confidence,
                "detection_stream_age_s": tire_detection_age,
                "vehicle_boxes_stream_age_s": vehicle_boxes_age,
                "detection_stream_interval_ema_s": self._detection_interval_ema,
                "vehicle_boxes_stream_interval_ema_s": self._vehicle_boxes_interval_ema,
                "tf_valid": tf_valid,
                "tf_lookup_latency_ms": tf_latency_ms,
                "tf_watchdog_paused": self._tf_watchdog_paused,
                "last_transition_cause": self._last_transition_cause,
                "target_lifecycle": self._target_lifecycle,
                "tires_captured": self._total_tires_captured,
                "vehicles_detected": len(self.detected_vehicles),
                "mission_elapsed_s": (time.time() - self._mission_start_time) if self._mission_start_time else 0,
                "last_cmd_vel": {
                    "linear": {
                        "x": self._last_cmd_vel_msg.linear.x,
                        "y": self._last_cmd_vel_msg.linear.y,
                        "z": self._last_cmd_vel_msg.linear.z,
                    },
                    "angular": {
                        "x": self._last_cmd_vel_msg.angular.x,
                        "y": self._last_cmd_vel_msg.angular.y,
                        "z": self._last_cmd_vel_msg.angular.z,
                    },
                } if self._last_cmd_vel_msg else None,
                "last_cmd_vel_time": self._last_cmd_vel_time,
                "last_cmd_vel_source": self._last_cmd_vel_source,
                "dispatch_fail_count": self._dispatch_fail_count,
                "nav_feedback": (
                    {
                        "distance_remaining": self._last_nav_feedback.distance_remaining,
                        "number_of_recoveries": self._last_nav_feedback.number_of_recoveries,
                        "estimated_time_remaining_s": (
                            self._last_nav_feedback.estimated_time_remaining.sec
                            + self._last_nav_feedback.estimated_time_remaining.nanosec / 1e9
                        )
                        if hasattr(self._last_nav_feedback, "estimated_time_remaining")
                        and self._last_nav_feedback.estimated_time_remaining
                        else None,
                    }
                    if self._last_nav_feedback
                    else None
                ),
                "proximity_gate_passed": self._proximity_gate_passed,
            }
            msg = String()
            msg.data = json.dumps(diag, default=str)
            self.diagnostics_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Diagnostics publish error: {e}")

    def _publish_mission_snapshot_cb(self) -> None:
        """Publish lightweight mission snapshot at 1 Hz (observability_upgrade)."""
        if self._mission_snapshot_pub is None:
            return
        try:
            mission_id = f"{self._mission_start_time:.0f}" if self._mission_start_time else "idle"
            current_goal = None
            if self._current_goal_pose is not None:
                g = self._current_goal_pose.pose.position
                current_goal = {"x": g.x, "y": g.y, "z": g.z}
            snapshot = {
                "mission_id": mission_id,
                "state": self.current_state,
                "current_goal": current_goal,
                "tires_captured": self._total_tires_captured,
                "tires_skipped": len(self._tires_skipped),
                "last_event": self._last_transition_cause,
            }
            msg = String()
            msg.data = json.dumps(snapshot, default=str)
            self._mission_snapshot_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Mission snapshot publish error: {e}")

    def _check_tf_watchdog(self) -> bool:
        """Phase C: Return False if TF invalid for > timeout; mission should pause.
        After tf_unavailable_abort_s seconds of continuous failure, transition to ERROR."""
        world_frame = self.get_parameter("world_frame").value
        base_frame = self.get_parameter("base_frame").value
        timeout = self.get_parameter("tf_watchdog_timeout").value
        abort_s = self.get_parameter("tf_unavailable_abort_s").value
        try:
            self.tf_buffer.lookup_transform(
                world_frame, base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            self._tf_last_valid_time = time.time()
            self._tf_watchdog_paused = False
            self._tf_watchdog_paused_since = None
            self._tf_recently_valid = True
            pose = self._get_current_pose()
            if pose is not None:
                cx = float(pose.pose.position.x)
                cy = float(pose.pose.position.y)
                cyaw = float(yaw_from_quaternion(pose.pose.orientation))
                now = time.time()
                prev = getattr(self, "_last_tf_pose_sample", None)
                self._last_tf_pose_sample = (cx, cy, cyaw, now)
                if prev is not None:
                    px, py, pyaw, ptime = prev
                    dt = max(1e-6, now - ptime)
                    jump_m = math.hypot(cx - px, cy - py)
                    yaw_jump = abs(math.atan2(math.sin(cyaw - pyaw), math.cos(cyaw - pyaw)))
                    jump_m_max = 1.0
                    yaw_jump_max = math.radians(45.0)
                    if jump_m > jump_m_max or yaw_jump > yaw_jump_max:
                        self.get_logger().error(
                            f"TF discontinuity detected jump_m={jump_m:.2f}, yaw_jump_deg={math.degrees(yaw_jump):.1f}; pausing mission."
                        )
                        self._mission_log_append(
                            "tf_discontinuity",
                            {
                                "jump_m": jump_m,
                                "yaw_jump_deg": math.degrees(yaw_jump),
                                "dt_s": dt,
                                "state": self.current_state,
                            },
                            sync=True,
                        )
                        self._tf_watchdog_paused = True
                        self._tf_watchdog_paused_since = now
                        return False
            return True
        except Exception:
            if self._tf_last_valid_time is None:
                self._tf_last_valid_time = time.time()
            if (time.time() - self._tf_last_valid_time) > timeout:
                if not self._tf_watchdog_paused:
                    self.get_logger().error(
                        f"TF watchdog: {world_frame}->{base_frame} invalid > {timeout}s. Pausing mission. "
                        "Is Aurora (slamware_ros_sdk_server_node) running? "
                        f"Check: ros2 run tf2_ros tf2_echo {world_frame} {base_frame}"
                    )
                    self._tf_watchdog_paused = True
                    self._tf_watchdog_paused_since = time.time()
                    self._mission_log_append(
                        "tf_watchdog",
                        {"timeout_s": timeout, "state": self.current_state},
                        sync=True,
                    )
                    # Cancel any in-flight Nav2 goal so robot stops while TF is invalid
                    if self._active_nav_goal_handle is not None:
                        try:
                            self._active_nav_goal_handle.cancel_goal_async()
                            self.get_logger().info(
                                "Cancelled in-flight Nav2 goal due to TF watchdog."
                            )
                        except Exception as e:
                            self.get_logger().warn(f"Failed to cancel Nav2 goal: {e}")
                        self._active_nav_goal_handle = None
                # After prolonged TF outage, abort mission instead of pausing forever
                if (
                    self._tf_watchdog_paused_since is not None
                    and (time.time() - self._tf_watchdog_paused_since) >= abort_s
                ):
                    self.get_logger().error(
                        f"TF unavailable for {abort_s}s. Aborting mission (ERROR). "
                        f"Verify Aurora is connected and publishing TF: "
                        f"ros2 run tf2_ros tf2_echo {world_frame} {base_frame}"
                    )
                    self._mission_log_append(
                        "tf_unavailable_abort",
                        {"abort_s": abort_s, "state": self.current_state},
                        sync=True,
                    )
                    self._mission_report["error_states_encountered"] += 1
                    self._set_state(MissionState.ERROR, cause="tf_unavailable")
                # Throttled log so we know ticks are being skipped (every 5s)
                now = time.time()
                if (
                    self._tf_skip_log_last_time is None
                    or (now - self._tf_skip_log_last_time) >= 5.0
                ):
                    self._tf_skip_log_last_time = now
                    paused_elapsed = (
                        (now - self._tf_watchdog_paused_since)
                        if self._tf_watchdog_paused_since
                        else 0
                    )
                    self._mission_log_append(
                        "tick_skipped_tf_invalid",
                        {
                            "state": self.current_state,
                            "paused_elapsed_s": paused_elapsed,
                            "reason": "tf_lookup_failed",
                            "frame": f"{world_frame}->{base_frame}",
                        },
                        sync=True,
                    )
                return False
            return True

    def _publish_mission_report(self):
        """Publish structured mission report and write JSON to disk for offline review."""
        self._mission_report["total_tires_captured"] = self._total_tires_captured
        self._mission_report["tires_skipped"] = list(self._tires_skipped)
        self._mission_report["mission_end_cause"] = self._last_transition_cause
        expected = self._mission_report["total_tires_expected"]
        captured = self._mission_report["total_tires_captured"]
        skipped = len(self._tires_skipped)
        missing = max(0, expected - captured - skipped)
        all_tires_done = expected > 0 and captured >= expected
        total_time = (time.time() - self._mission_start_time) if self._mission_start_time else 0
        success_flag = expected == captured and self._mission_report["error_states_encountered"] == 0
        deferred_retried = self._mission_report.get("tires_deferred_retried", 0)
        deferred_still_skipped = self._mission_report.get("tires_deferred_still_skipped", 0)
        report = (
            f"MISSION_REPORT\n"
            f"vehicles_detected={self._mission_report['total_vehicles']}\n"
            f"expected_tires={expected}\n"
            f"tires_completed={captured}\n"
            f"tires_skipped={skipped}\n"
            f"tires_deferred_retried={deferred_retried}\n"
            f"tires_deferred_still_skipped={deferred_still_skipped}\n"
            f"missing_tires={missing}\n"
            f"all_tires_captured={str(all_tires_done).upper()}\n"
            f"success_flag={str(success_flag).upper()}\n"
            f"total_time={total_time:.1f}\n"
            f"error_states_encountered={self._mission_report['error_states_encountered']}\n"
        )
        if missing > 0:
            report += f"INCOMPLETE: {missing} tire(s) not captured (e.g. far side). Mission did not finish normally.\n"
        msg = String()
        msg.data = report
        self.mission_report_pub.publish(msg)
        self.get_logger().info(f"Mission report published:\n{report}")
        # Write full report JSON to disk for offline review
        report_path = (self._mission_report_path or "").strip()
        if report_path:
            report_path = os.path.expanduser(os.path.expandvars(report_path))
            try:
                d = os.path.dirname(report_path)
                if d:
                    os.makedirs(d, exist_ok=True)
                report_json = {
                    "timestamp": time.time(),
                    "vehicles_detected": self._mission_report["total_vehicles"],
                    "expected_tires": expected,
                    "tires_captured": captured,
                    "tires_skipped": self._mission_report.get("tires_skipped", []),
                    "tires_deferred_retried": self._mission_report.get("tires_deferred_retried", 0),
                    "tires_deferred_still_skipped": self._mission_report.get("tires_deferred_still_skipped", 0),
                    "tire_capture_log": list(self._tire_capture_log),
                    "tire_goal_sources": [e.get("goal_source", "") for e in self._tire_capture_log],
                    "missing_tires": missing,
                    "all_tires_captured": all_tires_done,
                    "success": success_flag,
                    "total_time_s": total_time,
                    "mission_duration_s": total_time,  # alias for observability_upgrade
                    "error_states_encountered": self._mission_report["error_states_encountered"],
                    "mission_end_cause": self._mission_report.get("mission_end_cause"),
                }
                failures = []
                if self._mission_report.get("error_states_encountered", 0) > 0:
                    failures.append("error_states_encountered")
                if self._mission_report.get("tires_skipped"):
                    failures.append("tires_skipped")
                for e in self._tire_capture_log:
                    if not e.get("success", True) and e.get("reason"):
                        failures.append(f"tire_{e.get('tire_index', '?')}: {e.get('reason')}")
                if self._mission_report.get("mission_end_cause"):
                    failures.append(f"mission_end_cause: {self._mission_report['mission_end_cause']}")
                report_json["failures"] = failures
                if missing > 0:
                    report_json["incomplete"] = True
                    report_json["incomplete_reason"] = "mission_ended_before_all_tires_captured"
                with open(report_path, "w") as f:
                    json.dump(report_json, f, indent=2)
                    f.flush()
                    os.fsync(f.fileno())
                self.get_logger().info(f"Mission report saved to {report_path}")
            except Exception as e:
                self.get_logger().warn(f"Mission report save failed: {e}")

    def _load_trucks(self) -> List[dict]:
        """Load vehicles from YAML file (legacy mode, only used if use_dynamic_detection=False)."""
        trucks_file = self.get_parameter("trucks_file").value
        if not trucks_file:
            # Default to package share config path (works in install and from source)
            try:
                share_dir = get_package_share_directory("inspection_manager")
                trucks_file = os.path.join(share_dir, "config", "trucks.yaml")
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().warn(
                    f"Failed to resolve package share directory: {exc}"
                )
                return []

        if not os.path.exists(trucks_file):
            self.get_logger().warn(f"No vehicles file at {trucks_file}, starting empty.")
            return []

        with open(trucks_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        trucks = data.get("trucks", [])
        return trucks
    
    def _publish_vehicle_detected(self, vehicle_id: int, frame_id: str, x: float, y: float, z: float, yaw: float, confidence: float):
        """Publish vehicle anchor to /inspection_manager/vehicle_detected (JSON). Schema: vehicle_id, frame_id, x, y, z, yaw, confidence, timestamp_sec, timestamp_nanosec."""
        now = self.get_clock().now()
        payload = {
            "vehicle_id": vehicle_id,
            "frame_id": frame_id,
            "x": x, "y": y, "z": z, "yaw": yaw,
            "confidence": confidence,
            "timestamp_sec": int(now.nanoseconds // 1_000_000_000),
            "timestamp_nanosec": int(now.nanoseconds % 1_000_000_000),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.vehicle_detected_pub.publish(msg)
        self.get_logger().info(f"Published vehicle_detected: vehicle_id={vehicle_id} pose=({x:.3f},{y:.3f},{z:.3f}) yaw={math.degrees(yaw):.1f}° conf={confidence:.2f}")
        self.get_logger().info(
            f"DETECTION_EVENT: vehicle_id={vehicle_id} frame={frame_id} "
            f"pos=({x:.3f},{y:.3f},{z:.3f}) yaw_deg={math.degrees(yaw):.1f} conf={confidence:.2f}"
        )
        self._mission_log_append(
            "vehicle_detected",
            {
                "vehicle_id": vehicle_id,
                "frame_id": frame_id,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "confidence": confidence,
            },
            sync=True,
        )

    def _start_mission_cb(self, msg: Bool):
        """When start_mission_on_ready is False, mission starts only after receiving True here."""
        if msg.data:
            self._mission_start_requested = True
            self.get_logger().info("Start mission requested via topic.")

    def _cmd_vel_cb(self, msg: Twist, source: str) -> None:
        """Track last cmd_vel for watchdog diagnostics."""
        self._last_cmd_vel_time = time.time()
        self._last_cmd_vel_msg = msg
        self._last_cmd_vel_source = source

    def _rotate_mission_logs(self) -> None:
        """On mission start: archive existing mission_latest.* to timestamped files; apply retention (keep last 10, gzip older)."""
        archive_dir = None
        for path_param, prefix in [
            (self._mission_log_path, "mission"),
            (self._mission_report_path, "mission_report"),
        ]:
            path = (path_param or "").strip()
            if not path:
                continue
            path = os.path.expanduser(os.path.expandvars(path))
            if not os.path.isfile(path):
                continue
            try:
                d = os.path.dirname(path)
                archive_dir = os.path.join(d, "archive") if d else "archive"
                os.makedirs(archive_dir, exist_ok=True)
                stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
                ext = os.path.splitext(path)[1] or ".jsonl"
                if prefix == "mission_report":
                    ext = ".json"
                archive_name = f"{prefix}_{stamp}{ext}"
                archive_path = os.path.join(archive_dir, archive_name)
                os.rename(path, archive_path)
                self.get_logger().info(f"Archived mission log to {archive_path}")
            except Exception as e:
                self.get_logger().warn(f"Mission log archive failed: {e}")

        # Retention: keep last 10 archives per prefix; gzip older ones
        if archive_dir and os.path.isdir(archive_dir):
            try:
                import glob
                import subprocess
                keep_count = 10
                for pattern in ["mission_*", "mission_report_*"]:
                    files = sorted(
                        [f for f in glob.glob(os.path.join(archive_dir, pattern)) if os.path.isfile(f)],
                        key=os.path.getmtime,
                        reverse=True,
                    )
                    to_compress = [f for f in files[keep_count:] if not f.endswith(".gz")]
                    for f in to_compress:
                        try:
                            subprocess.run(["gzip", "-f", f], check=False, capture_output=True, timeout=5)
                        except Exception:
                            pass
            except Exception as e:
                self.get_logger().debug(f"Log retention/compress skipped: {e}")

    def _mission_log_append(self, event: str, data: Dict[str, Any], sync: bool = False) -> None:
        """Append one JSON line to mission log file if mission_log_path is set."""
        path = (self._mission_log_path or "").strip()
        if not path:
            return
        path = os.path.expanduser(os.path.expandvars(path))
        try:
            d = os.path.dirname(path)
            if d:
                os.makedirs(d, exist_ok=True)
            sync = sync or self._mission_log_fsync
            with open(path, "a") as f:
                f.write(json.dumps({"event": event, "data": data, "t": time.time()}) + "\n")
                if sync:
                    f.flush()
                    os.fsync(f.fileno())
        except Exception as e:
            self.get_logger().warn(f"Mission log append failed: {e}")

    def _publish_approach_markers(
        self,
        target_center: tuple,
        goal_pose: PoseStamped,
        label: str,
    ) -> None:
        """Publish target and approach markers for operator visibility."""
        try:
            now = self.get_clock().now().to_msg()
            marker_array = MarkerArray()

            target_marker = Marker()
            target_marker.header.frame_id = goal_pose.header.frame_id
            target_marker.header.stamp = now
            target_marker.ns = "inspection_target"
            target_marker.id = 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = float(target_center[0])
            target_marker.pose.position.y = float(target_center[1])
            target_marker.pose.position.z = float(target_center[2])
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = 0.20
            target_marker.scale.y = 0.20
            target_marker.scale.z = 0.20
            target_marker.color.r = 1.0
            target_marker.color.g = 0.2
            target_marker.color.b = 0.2
            target_marker.color.a = 0.95
            target_marker.lifetime.sec = 2

            approach_marker = Marker()
            approach_marker.header.frame_id = goal_pose.header.frame_id
            approach_marker.header.stamp = now
            approach_marker.ns = "inspection_approach"
            approach_marker.id = 2
            approach_marker.type = Marker.ARROW
            approach_marker.action = Marker.ADD
            approach_marker.pose = goal_pose.pose
            approach_marker.scale.x = 0.45
            approach_marker.scale.y = 0.08
            approach_marker.scale.z = 0.08
            approach_marker.color.r = 0.1
            approach_marker.color.g = 0.9
            approach_marker.color.b = 0.1
            approach_marker.color.a = 0.95
            approach_marker.lifetime.sec = 2

            text_marker = Marker()
            text_marker.header.frame_id = goal_pose.header.frame_id
            text_marker.header.stamp = now
            text_marker.ns = "inspection_approach_label"
            text_marker.id = 3
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = goal_pose.pose.position.x
            text_marker.pose.position.y = goal_pose.pose.position.y
            text_marker.pose.position.z = goal_pose.pose.position.z + 0.45
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.18
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.95
            text_marker.text = label
            text_marker.lifetime.sec = 2

            marker_array.markers = [target_marker, approach_marker, text_marker]
            self.approach_markers_pub.publish(marker_array)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish approach markers: {e}")

    def _vehicle_box_in_range(self, box: BoundingBox3d) -> bool:
        """Filter vehicle boxes by distance to robot (prevents far/unstable detections)."""
        pose = self._get_current_pose()
        if pose is None:
            return True
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        dx = center_x - pose.pose.position.x
        dy = center_y - pose.pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)
        min_d = float(self.get_parameter("vehicle_min_distance_m").value)
        if (
            self.current_vehicle_box is not None
            or self.current_state
            not in (MissionState.SEARCH_VEHICLE, MissionState.WAIT_VEHICLE_BOX)
        ):
            min_d = float(self.get_parameter("vehicle_min_distance_confirmed_m").value)
        max_d = float(self.get_parameter("vehicle_max_distance_m").value)
        if dist < min_d or dist > max_d:
            self._mission_log_append(
                "vehicle_out_of_range",
                {"distance_m": round(dist, 2), "min_m": min_d, "max_m": max_d},
                sync=True,
            )
            return False
        return True

    def _confirm_vehicle_box(self, box: BoundingBox3d) -> bool:
        required = int(self.get_parameter("vehicle_confirmations_required").value)
        if required <= 1:
            return True
        center = (
            (box.xmin + box.xmax) / 2.0,
            (box.ymin + box.ymax) / 2.0,
            (box.zmin + box.zmax) / 2.0,
        )
        tol = float(self.get_parameter("vehicle_confirm_tolerance_m").value)
        if self._vehicle_confirm_center:
            dx = center[0] - self._vehicle_confirm_center[0]
            dy = center[1] - self._vehicle_confirm_center[1]
            if math.sqrt(dx * dx + dy * dy) <= tol:
                self._vehicle_confirm_count += 1
            else:
                self._vehicle_confirm_center = center
                self._vehicle_confirm_count = 1
        else:
            self._vehicle_confirm_center = center
            self._vehicle_confirm_count = 1
        if self._vehicle_confirm_count >= required:
            self._vehicle_confirm_count = 0
            return True
        return False

    def _confirm_tire_box(self, tire_center: tuple) -> bool:
        required = int(self.get_parameter("tire_confirmations_required").value)
        if required <= 1:
            return True
        tol = float(self.get_parameter("tire_confirm_tolerance_m").value)
        if self._tire_confirm_center:
            dx = tire_center[0] - self._tire_confirm_center[0]
            dy = tire_center[1] - self._tire_confirm_center[1]
            if math.sqrt(dx * dx + dy * dy) <= tol:
                self._tire_confirm_count += 1
            else:
                self._tire_confirm_center = tire_center
                self._tire_confirm_count = 1
        else:
            self._tire_confirm_center = tire_center
            self._tire_confirm_count = 1
        if self._tire_confirm_count >= required:
            self._tire_confirm_count = 0
            return True
        return False

    def _dispatch_patrol_goal(self) -> bool:
        """Dispatch a short patrol goal to search for vehicles."""
        pose = self._get_current_pose_in_map_frame()
        if pose is None:
            self.get_logger().warn("Patrol: cannot get pose in map frame.")
            return False
        map_frame = self.get_parameter("map_frame").value
        if pose.header.frame_id != map_frame and self.get_parameter("require_goal_transform").value:
            self.get_logger().warn(
                f"Patrol: pose frame {pose.header.frame_id} != {map_frame}; cannot dispatch."
            )
            self._mission_log_append(
                "patrol_dispatch_blocked",
                {"reason": "map_frame_unavailable", "frame": pose.header.frame_id},
                sync=True,
            )
            return False
        if self.get_parameter("require_nav_permitted").value and self._nav_permitted is not True:
            self._last_dispatch_fail_reason = "nav_permitted_blocked"
            self._mission_log_append(
                "nav_gate_blocked_dispatch",
                {
                    "state": self.current_state,
                    "nav_permitted": self._nav_permitted,
                    "last_nav_permitted_time": self._last_nav_permitted_time,
                    "topic": self.get_parameter("nav_permitted_topic").value,
                },
                sync=True,
            )
            return False
        current_yaw = yaw_from_quaternion(pose.pose.orientation)
        turn_deg = float(self.get_parameter("patrol_turn_deg").value)
        step = float(self.get_parameter("patrol_step_m").value)
        new_yaw = current_yaw + math.radians(turn_deg * self._patrol_direction)
        self._patrol_direction *= -1

        goal = PoseStamped()
        goal.header.frame_id = map_frame if pose.header.frame_id == map_frame else pose.header.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = pose.pose.position.x + step * math.cos(new_yaw)
        goal.pose.position.y = pose.pose.position.y + step * math.sin(new_yaw)
        goal.pose.position.z = pose.pose.position.z
        goal.pose.orientation = quaternion_from_yaw(new_yaw)

        sent = send_nav_goal(self, goal, self._on_patrol_done, feedback_cb=self._on_nav_feedback)
        self._mission_log_append(
            "patrol_dispatched",
            {
                "sent": sent,
                "step_m": step,
                "turn_deg": turn_deg,
                "goal_x": goal.pose.position.x,
                "goal_y": goal.pose.position.y,
            },
            sync=True,
        )
        return sent

    def _refine_target_from_detection(self, target_pos: tuple) -> Optional[tuple]:
        """If a wheel detection is within refinement_max_distance_m of target_pos, return its 3D center (x,y,z); else None.
        Applies EMA smoothing (refinement_ema_alpha) when cache has prior value to reduce goal flicker from detection jitter.
        Assumes _last_tire_boxes are in the same frame as target_pos (e.g. map/slamware_map)."""
        if len(target_pos) < 2:
            return None
        boxes = self._last_tire_boxes
        if not boxes:
            return None
        tire_label = self.get_parameter("tire_label").value
        min_prob = float(self.get_parameter("min_tire_probability").value)
        max_dist = float(self.get_parameter("refinement_max_distance_m").value)
        tx, ty = target_pos[0], target_pos[1]
        best_pos: Optional[tuple] = None
        best_d = max_dist
        for b in boxes:
            if b.object_name.lower() != tire_label.lower() or b.probability < min_prob:
                continue
            cx = (b.xmin + b.xmax) / 2.0
            cy = (b.ymin + b.ymax) / 2.0
            cz = (b.zmin + b.zmax) / 2.0
            d = math.hypot(cx - tx, cy - ty)
            if d < best_d:
                best_d = d
                best_pos = (cx, cy, cz)
        if best_pos is None:
            return None
        # EMA smoothing to reduce flicker when detection jitters between frames
        key = (round(tx, 1), round(ty, 1))
        alpha = float(self.get_parameter("refinement_ema_alpha").value)
        alpha = max(0.0, min(1.0, alpha))
        if alpha > 0 and key in self._refinement_ema_cache:
            prev = self._refinement_ema_cache[key]
            # Only blend if new detection is within 0.3 m of cached (avoid blending wrong tire)
            if math.hypot(best_pos[0] - prev[0], best_pos[1] - prev[1]) < 0.3:
                cx = alpha * best_pos[0] + (1 - alpha) * prev[0]
                cy = alpha * best_pos[1] + (1 - alpha) * prev[1]
                cz = alpha * best_pos[2] + (1 - alpha) * prev[2]
                best_pos = (cx, cy, cz)
        self._refinement_ema_cache[key] = best_pos
        return best_pos

    def _detection_near_expected_tire(self, expected_pos: tuple) -> bool:
        """Return True if any tire detection in _last_tire_boxes is within
        capture_max_distance_detection_to_expected_tire_m of expected_pos (x,y) or (x,y,z)."""
        if not self._last_tire_boxes:
            return False
        ex = expected_pos[0]
        ey = expected_pos[1]
        max_dist = float(self.get_parameter("capture_max_distance_detection_to_expected_tire_m").value)
        tire_label = self.get_parameter("tire_label").value
        min_prob = float(self.get_parameter("min_tire_probability").value)
        for box in self._last_tire_boxes:
            if box.object_name.lower() != tire_label.lower() or box.probability < min_prob:
                continue
            cx = (box.xmin + box.xmax) / 2.0
            cy = (box.ymin + box.ymax) / 2.0
            if math.hypot(cx - ex, cy - ey) <= max_dist:
                return True
        return False

    def _vehicle_position_valid(self, vehicle_pos: tuple) -> bool:
        """Return False if _mission_start_robot_position is set and distance from it to vehicle_pos
        exceeds vehicle_max_distance_from_start_m, else True. vehicle_pos is (x, y) or (x, y, z)."""
        if self._mission_start_robot_position is None:
            return True
        vx, vy = vehicle_pos[0], vehicle_pos[1]
        sx, sy = self._mission_start_robot_position[0], self._mission_start_robot_position[1]
        dist = math.hypot(vx - sx, vy - sy)
        max_from_start = float(self.get_parameter("vehicle_max_distance_from_start_m").value)
        if dist > max_from_start:
            self.get_logger().warn(
                f"Rejecting vehicle at {dist:.1f}m from start (max {max_from_start}m)"
            )
            return False
        return True

    def _tyre_3d_cb(self, msg: PoseArray) -> None:
        self._tyre_3d_poses = msg.poses
        self._tyre_3d_stamp = msg.header.stamp
        self._update_tyre_geometry_from_poses(msg.poses)
        if msg.poses and bool(self.get_parameter("use_tyre_3d_positions").value):
            self.get_logger().info(
                f"/tyre_3d_positions: {len(msg.poses)} pose(s), frame={msg.header.frame_id}",
                throttle_duration_sec=5.0,
            )

    def _update_tyre_geometry_from_poses(self, poses) -> None:
        """Recompute tyre-centric vehicle geometry when new tyre poses arrive."""
        if not bool(self.get_parameter("use_tyre_geometry").value):
            self._tyre_geometry = None
            self._last_tyre_geometry_pose_count = 0
            return
        if len(poses) < 2:
            self._tyre_geometry = None
            self._last_tyre_geometry_pose_count = len(poses)
            self.get_logger().debug(
                f"use_tyre_geometry: need >=2 tyre poses (got {len(poses)}); geometry cleared.",
                throttle_duration_sec=5.0,
            )
            return
        try:
            geom = tyre_geometry_from_poses(poses, min_points=2)
            self._tyre_geometry = geom
            self._last_tyre_geometry_pose_count = len(poses)
            self.get_logger().info(
                f"Tyre geometry: n={len(poses)} center=({geom.mean[0]:.2f},{geom.mean[1]:.2f}) "
                f"yaw_deg={math.degrees(geom.orientation):.1f} L={geom.length:.2f}m W={geom.width:.2f}m",
                throttle_duration_sec=4.0,
            )
        except Exception as e:
            self._tyre_geometry = None
            self.get_logger().warn(f"use_tyre_geometry: failed to compute geometry: {e}")

    def _tyre_3d_stamp_age_s(self) -> Optional[float]:
        """Seconds since tyre_3d message stamp (0 if stamp is in the future vs ROS time — still treat as fresh)."""
        if self._tyre_3d_stamp is None:
            return None
        try:
            now = self.get_clock().now()
            dt = (now - rclpy.time.Time.from_msg(self._tyre_3d_stamp)).nanoseconds / 1e9
            return max(0.0, dt)
        except Exception:
            return None

    def _tyre_3d_fresh(self) -> bool:
        if not self._tyre_3d_poses:
            return False
        if self._tyre_3d_stamp is None:
            return False
        try:
            stale_s = float(self.get_parameter("tyre_3d_stale_s").value)
            age = self._tyre_3d_stamp_age_s()
            if age is None:
                return False
            return age <= stale_s
        except Exception:
            return False

    def _tyre_3d_unavailable_reason(self) -> str:
        """Human-readable reason `_tyre_3d_pick_nearest_target` would return None (for logging)."""
        if not self._tyre_3d_poses:
            return "no_poses"
        if self._tyre_3d_stamp is None:
            return "no_header_stamp"
        stale_s = float(self.get_parameter("tyre_3d_stale_s").value)
        age = self._tyre_3d_stamp_age_s()
        if age is None:
            return "stamp_age_unavailable"
        if age > stale_s:
            return f"stale age={age:.2f}s max={stale_s:.2f}s"
        robot = self._get_current_pose()
        if robot is None:
            return "no_robot_tf"
        rx, ry = robot.pose.position.x, robot.pose.position.y
        best_d = float("inf")
        for p in self._tyre_3d_poses:
            d = math.hypot(p.position.x - rx, p.position.y - ry)
            if d < best_d:
                best_d = d
        if best_d is float("inf"):
            return "empty_pose_list"
        min_r = float(self.get_parameter("tyre_3d_min_range_m").value)
        max_r = float(self.get_parameter("tyre_3d_max_range_m").value)
        if not (min_r <= best_d <= max_r):
            return f"nearest_d={best_d:.2f}m outside [{min_r:.2f},{max_r:.2f}]"
        return "unknown_should_pick"

    def _tyre_3d_pick_nearest_target(self) -> Optional[Tuple[Tuple[float, float, float], float]]:
        """Return ((x,y,z), distance_m) for nearest tyre in range from fresh /tyre_3d_positions, or None."""
        if not self._tyre_3d_poses:
            return None
        if not self._tyre_3d_fresh():
            return None
        robot = self._get_current_pose()
        if robot is None:
            return None
        rx, ry = robot.pose.position.x, robot.pose.position.y
        best = None
        best_d = float("inf")
        for p in self._tyre_3d_poses:
            px, py = p.position.x, p.position.y
            d = math.hypot(px - rx, py - ry)
            if d < best_d:
                best_d = d
                best = p
        if best is None:
            return None
        min_r = float(self.get_parameter("tyre_3d_min_range_m").value)
        max_r = float(self.get_parameter("tyre_3d_max_range_m").value)
        if not (min_r <= best_d <= max_r):
            return None
        target = (best.position.x, best.position.y, best.position.z)
        return (target, best_d)

    def _prune_planned_tire_near_xy(self, tx: float, ty: float, radius_m: float) -> None:
        """Remove the planned corner closest to (tx,ty) if within radius_m (after tyre_3d navigation)."""
        if not self._planned_tire_positions or radius_m <= 0.0:
            return
        best_i = None
        best_d = float("inf")
        for i, p in enumerate(self._planned_tire_positions):
            d = math.hypot(p[0] - tx, p[1] - ty)
            if d < best_d:
                best_d = d
                best_i = i
        if best_i is not None and best_d <= radius_m:
            removed = self._planned_tire_positions.pop(best_i)
            if self.current_vehicle_idx < len(self.detected_vehicles):
                self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
            self.get_logger().info(
                f"tyre_3d: pruned planned corner at ({removed[0]:.2f},{removed[1]:.2f}) "
                f"(d={best_d:.2f}m to tyre_3d) — {len(self._planned_tire_positions)} planned remaining"
            )
            self._mission_log_append(
                "tyre_3d_pruned_planned_corner",
                {"removed": list(removed)[:3], "distance_m": best_d, "remaining": len(self._planned_tire_positions)},
                sync=True,
            )

    def _try_dispatch_from_tyre_3d(self) -> bool:
        """Navigate to nearest tyre from tyre_3d_projection_node (map/slamware_map poses)."""
        if not bool(self.get_parameter("use_tyre_3d_positions").value):
            return False
        picked = self._tyre_3d_pick_nearest_target()
        if picked is None:
            if bool(self.get_parameter("use_tyre_3d_positions").value) and self._tyre_3d_poses and not self._tyre_3d_fresh():
                self.get_logger().warn(
                    "/tyre_3d_positions is stale or has no valid stamp; waiting for fresh poses",
                    throttle_duration_sec=5.0,
                )
            elif bool(self.get_parameter("use_tyre_3d_positions").value) and self._tyre_3d_poses:
                robot = self._get_current_pose()
                if robot is None:
                    self.get_logger().warn(
                        "tyre_3d: robot pose unavailable (TF); cannot compute nav goal",
                        throttle_duration_sec=5.0,
                    )
                else:
                    # In range filter failed — log occasionally
                    self.get_logger().info(
                        "tyre_3d: nearest tyre outside tyre_3d_min/max_range_m — no goal",
                        throttle_duration_sec=5.0,
                    )
            return False
        target, best_d = picked
        sent, _ = self._dispatch_planned_tire_goal(target, tyre_3d_direct=True)
        if not sent:
            reason = getattr(self, "_last_dispatch_fail_reason", None)
            self.get_logger().warn(
                f"tyre_3d: dispatch failed at ({target[0]:.2f},{target[1]:.2f}) dist={best_d:.2f}m "
                f"(reason={reason})",
                throttle_duration_sec=3.0,
            )
            return False
        self._set_state(MissionState.INSPECT_TIRE, cause="tyre_3d_direct")
        self.get_logger().info(
            f"tyre_3d: navigating to nearest tyre at ({target[0]:.2f},{target[1]:.2f}) dist={best_d:.2f}m"
        )
        return True

    def _try_dispatch_tyre_3d_in_wait_tire_box(self) -> bool:
        """WAIT_TIRE_BOX: prefer fresh /tyre_3d_positions (standoff via tyre_3d_direct) over planned box corners."""
        if not bool(self.get_parameter("use_tyre_3d_positions").value):
            return False
        if not bool(self.get_parameter("prefer_tyre_3d_in_wait_tire_box").value):
            return False
        picked = self._tyre_3d_pick_nearest_target()
        if picked is None:
            why = self._tyre_3d_unavailable_reason()
            self.get_logger().info(
                f"tyre_3d WAIT_TIRE_BOX: skip direct goal ({why}); world_frame={self.get_parameter('world_frame').value}",
                throttle_duration_sec=2.0,
            )
            self._mission_log_append(
                "tyre_3d_wait_tire_box_skip",
                {
                    "reason": why,
                    "pose_count": len(self._tyre_3d_poses) if self._tyre_3d_poses else 0,
                    "stamp_age_s": self._tyre_3d_stamp_age_s(),
                },
                sync=False,
            )
            return False
        target, best_d = picked
        tx, ty, tz = target[0], target[1], target[2]
        robot_pose = self._get_current_pose()
        robot_pos = (
            (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z)
            if robot_pose
            else None
        )
        tire_pos_label = (
            tire_position_label(target, self._committed_vehicle_center, robot_pos)
            if self._committed_vehicle_center and robot_pos
            else "tyre_3d"
        )
        self.inspected_tire_positions.append(target)
        self._tire_id_counter += 1
        self._tire_registry.append(
            {
                "id": self._tire_id_counter,
                "vehicle_id": self.current_vehicle_idx,
                "position": target,
                "tire_position": tire_pos_label or "tyre_3d",
                "visited": False,
                "image_captured": False,
                "goal_source": "tyre_3d_direct",
            }
        )
        self._mission_log_append(
            "tyre_3d_wait_tire_box_dispatch",
            {
                "target": [tx, ty, tz],
                "distance_m": best_d,
                "remaining_planned_before": len(self._planned_tire_positions),
            },
            sync=True,
        )
        sent, _ = self._dispatch_planned_tire_goal(target, tyre_3d_direct=True)
        if sent:
            prune_r = float(self.get_parameter("tyre_3d_prune_planned_radius_m").value)
            self._prune_planned_tire_near_xy(tx, ty, prune_r)
            self._set_state(MissionState.INSPECT_TIRE, cause="tyre_3d_direct_wait_tire_box")
            self.get_logger().info(
                f"tyre_3d (WAIT_TIRE_BOX): nav to standoff near tyre at ({tx:.2f},{ty:.2f}) dist={best_d:.2f}m"
            )
            return True
        self.inspected_tire_positions.pop()
        self._tire_registry.pop()
        self._tire_id_counter -= 1
        reason = getattr(self, "_last_dispatch_fail_reason", None)
        self.get_logger().warn(
            f"tyre_3d (WAIT_TIRE_BOX): dispatch failed at ({tx:.2f},{ty:.2f}) (reason={reason}); will retry planned/cached path",
            throttle_duration_sec=3.0,
        )
        return False

    def _maybe_tyre_perimeter_waypoints(self, target_pos: tuple) -> Optional[List[PoseStamped]]:
        """If tyre_perimeter_bridge_enabled, return 1+ map poses (bridge + standoff).

        Prefers tyre-derived footprint from /tyre_3d_positions when use_tyre_geometry; else Aurora box.
        """
        if not bool(self.get_parameter("tyre_perimeter_bridge_enabled").value):
            return None
        robot = self._get_current_pose()
        if robot is None:
            return None
        map_frame = self.get_parameter("map_frame").value
        stamp = self.get_clock().now().to_msg()
        clearance = float(self.get_parameter("vehicle_perimeter_clearance_m").value)
        lat_eps = float(self.get_parameter("tyre_perimeter_lateral_eps_m").value)
        offset_m = float(self._last_tire_offset or self.get_parameter("tire_offset").value)

        if bool(self.get_parameter("use_tyre_geometry").value) and self._tyre_geometry is not None:
            g = self._tyre_geometry
            cx, cy = float(g.mean[0]), float(g.mean[1])
            fwd_x = float(g.longitudinal_axis[0])
            fwd_y = float(g.longitudinal_axis[1])
            right_x = float(g.right_axis[0])
            right_y = float(g.right_axis[1])
            return build_tyre_approach_waypoints(
                map_frame,
                stamp,
                (robot.pose.position.x, robot.pose.position.y),
                target_pos,
                (cx, cy),
                (fwd_x, fwd_y),
                (right_x, right_y),
                float(g.half_length),
                float(g.half_width),
                offset_m,
                clearance,
                lat_eps,
            )

        if self._committed_vehicle_box is None:
            return None
        fp = get_vehicle_footprint(
            self._committed_vehicle_box,
            (robot.pose.position.x, robot.pose.position.y, robot.pose.position.z),
        )
        if fp is None:
            return None
        cx, cy, L, W, yaw = fp
        half_len = L * 0.5
        half_wid = W * 0.5
        fwd_x = math.cos(yaw)
        fwd_y = math.sin(yaw)
        right_x = -fwd_y
        right_y = fwd_x
        return build_tyre_approach_waypoints(
            map_frame,
            stamp,
            (robot.pose.position.x, robot.pose.position.y),
            target_pos,
            (cx, cy),
            (fwd_x, fwd_y),
            (right_x, right_y),
            half_len,
            half_wid,
            offset_m,
            clearance,
            lat_eps,
        )

    def _dispatch_planned_tire_goal(self, target_pos: tuple, tyre_3d_direct: bool = False) -> tuple:
        """Dispatch a goal to a planned tire position. When a wheel detection is near the planned position,
        use the detection center as the goal (refinement). Returns (sent: bool, used_refined: bool).

        If tyre_3d_direct is True (goal from tyre_3d_projection_node), standoff uses **robot→tyre** direction so
        bad 1×1 m vehicle boxes do not skew the goal (vehicle_center offset is skipped)."""
        used_refined = False
        if self.get_parameter("require_nav_permitted").value and self._nav_permitted is not True:
            self._last_dispatch_fail_reason = "nav_permitted_blocked"
            self._mission_log_append(
                "nav_gate_blocked_dispatch",
                {
                    "state": self.current_state,
                    "nav_permitted": self._nav_permitted,
                    "last_nav_permitted_time": self._last_nav_permitted_time,
                    "topic": self.get_parameter("nav_permitted_topic").value,
                },
                sync=True,
            )
            return (False, False)
        if len(target_pos) != 3 or not all(math.isfinite(x) for x in target_pos):
            self.get_logger().warn(
                "Planned tire: target_pos invalid (must be length 3, all finite); cannot dispatch."
            )
            self._mission_log_append(
                "planned_tire_dispatch_blocked",
                {"reason": "target_pos_invalid", "target_pos": list(target_pos) if hasattr(target_pos, "__iter__") else str(target_pos)},
                sync=True,
            )
            return (False, False)
        map_frame = self.get_parameter("map_frame").value
        if self.get_parameter("require_goal_transform").value:
            try:
                self.tf_buffer.lookup_transform(
                    map_frame, self.get_parameter("world_frame").value, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
            except Exception:
                self.get_logger().warn("Planned tire: required map transform unavailable; cannot dispatch.")
                self._mission_log_append(
                    "planned_tire_dispatch_blocked",
                    {"reason": "map_transform_unavailable", "frame": map_frame},
                    sync=True,
                )
                return (False, False)
        # Refine goal from wheel detection when one is within refinement_max_distance_m of planned tire.
        # Skip for tyre_3d_direct: goal already comes from /tyre_3d_positions (refinement could snap back toward a bad vehicle box).
        if not tyre_3d_direct:
            refined = self._refine_target_from_detection(target_pos)
            if refined is not None:
                dist_m = math.hypot(refined[0] - target_pos[0], refined[1] - target_pos[1])
                target_pos = refined
                used_refined = True
                self.get_logger().info(
                    f"Refining goal: planned → detection ({dist_m:.2f}m)"
                )
                self._mission_log_append(
                    "tire_goal_refined_from_detection",
                    {"position": list(target_pos), "refinement_max_distance_m": float(self.get_parameter("refinement_max_distance_m").value)},
                    sync=True,
                )
        to = float(self.get_parameter("tire_offset").value)
        staging = float(self.get_parameter("tire_staging_min_m").value)
        self._last_tire_offset = max(to, staging) if staging > 0 else to

        # Tyre 3D pose from projection: stand off along (robot - tyre) so we stop in front of the real tyre,
        # not along (tyre - vehicle_center) which is wrong when the committed box is tiny or mis-sized.
        if tyre_3d_direct:
            robot = self._get_current_pose()
            if robot is None:
                self.get_logger().warn("tyre_3d_direct: robot pose unavailable; cannot compute standoff goal.")
                self._last_dispatch_fail_reason = "no_robot_pose_tyre_3d"
                return (False, used_refined)
            self._pending_perimeter_nav_queue.clear()
            rx, ry = robot.pose.position.x, robot.pose.position.y
            tx, ty, tz = target_pos[0], target_pos[1], target_pos[2]
            offset_m = self._last_tire_offset
            map_frame = self.get_parameter("map_frame").value
            stamp = self.get_clock().now().to_msg()
            perimeter_wp = self._maybe_tyre_perimeter_waypoints(target_pos)
            if perimeter_wp is not None and perimeter_wp:
                waypoints_to_send = perimeter_wp
            else:
                gx, gy, heading, d = standoff_goal_robot_tyre_xy(rx, ry, tx, ty, offset_m)
                waypoints_to_send = [
                    pose_stamped_from_standoff_xy(map_frame, stamp, gx, gy, tz, heading)
                ]
            goal = waypoints_to_send[0]
            gx = goal.pose.position.x
            gy = goal.pose.position.y
            heading = yaw_from_quaternion(goal.pose.orientation)
            d = math.hypot(rx - tx, ry - ty)
            if len(waypoints_to_send) > 1:
                self._pending_perimeter_nav_queue = list(waypoints_to_send[1:])
                self._mission_log_append(
                    "tyre_perimeter_waypoints",
                    {
                        "segments": len(waypoints_to_send),
                        "queued": len(self._pending_perimeter_nav_queue),
                    },
                    sync=True,
                )
            if not (math.isfinite(gx) and math.isfinite(gy)):
                self._last_dispatch_fail_reason = "goal_position_not_finite"
                return (False, used_refined)
            self._mission_log_append(
                "goal_computed",
                {
                    "frame": map_frame,
                    "x": gx,
                    "y": gy,
                    "z": tz,
                    "yaw_deg": math.degrees(heading),
                    "offset": offset_m,
                    "distance_to_object": d,
                    "state": self.current_state,
                    "transform_ok": True,
                    "world_frame": map_frame,
                    "tyre_3d_robot_standoff": True,
                    "used_refined": used_refined,
                    "goal_world": {"x": gx, "y": gy, "z": tz},
                    "perimeter_segments": len(waypoints_to_send),
                },
                sync=True,
            )
            self._current_goal_pose = goal
            self.current_goal_pub.publish(goal)
            if self._is_dry_run():
                self.get_logger().info("Dry run: tyre_3d standoff goal validated, not sending to Nav2")
                self._last_dispatch_fail_reason = None
                return (True, used_refined)
            ok = send_nav_goal(self, goal, self._on_box_goal_done, feedback_cb=self._on_nav_feedback)
            if ok:
                self._mission_log_append(
                    "nav_command_sent",
                    {
                        "frame": map_frame,
                        "x": gx,
                        "y": gy,
                        "z": tz,
                        "yaw_deg": math.degrees(heading),
                        "state": self.current_state,
                        "tyre_3d_robot_standoff": True,
                        "perimeter_segments": len(waypoints_to_send),
                    },
                    sync=True,
                )
                self.get_logger().info(
                    f"NAV_COMMAND_SENT (tyre_3d robot standoff): pos=({gx:.3f},{gy:.3f},{tz:.3f}) "
                    f"yaw_deg={math.degrees(heading):.1f} segments={len(waypoints_to_send)}"
                )
            else:
                self._last_dispatch_fail_reason = "nav_send_failed"
                self._dispatch_fail_count += 1
                self._pending_perimeter_nav_queue.clear()
            return (ok, used_refined)

        extent = float(self.get_parameter("planned_tire_box_extent_m").value)
        tire_label = self.get_parameter("tire_label").value
        box = BoundingBox3d()
        box.object_name = tire_label
        box.probability = 0.5
        box.xmin = target_pos[0] - extent
        box.xmax = target_pos[0] + extent
        box.ymin = target_pos[1] - extent
        box.ymax = target_pos[1] + extent
        box.zmin = target_pos[2] - extent
        box.zmax = target_pos[2] + extent
        self._last_tire_box = box
        self._mission_log_append(
            "tire_planned_fallback",
            {"position": target_pos, "extent": extent},
            sync=True,
        )
        # Research (Nav2 #4299, goal_generator): For planned tires with a committed vehicle box,
        # ALWAYS use vehicle_center -> tire_center direction for goal placement. This guarantees
        # goal is outside the vehicle (avoids goal_inside_vehicle_box). Works for both near-side
        # and far-side tires: goal = tire + offset * (tire - vehicle_center) / |tire - vehicle_center|.
        anchor = self._committed_vehicle_anchor_pose
        # Fallback: derive anchor from committed vehicle box if anchor_pose missing (e.g. legacy data)
        if (anchor is None or "x" not in anchor or "y" not in anchor) and self._committed_vehicle_box is not None:
            vb = self._committed_vehicle_box
            anchor = {
                "x": (vb.xmin + vb.xmax) / 2.0,
                "y": (vb.ymin + vb.ymax) / 2.0,
                "z": (vb.zmin + vb.zmax) / 2.0,
            }
        use_vehicle_center_goal = (
            self._committed_vehicle_box is not None
            and anchor is not None
            and "x" in anchor
            and "y" in anchor
        )
        if use_vehicle_center_goal:
            vcx = float(anchor["x"])
            vcy = float(anchor["y"])
            tx, ty, tz = target_pos[0], target_pos[1], target_pos[2]
            offset_m = self._last_tire_offset
            out = standoff_goal_vehicle_center_tire_xy(vcx, vcy, tx, ty, offset_m)
            if out is not None:
                gx, gy, heading, d = out
                map_frame = self.get_parameter("map_frame").value
                goal = pose_stamped_from_standoff_xy(
                    map_frame, self.get_clock().now().to_msg(), gx, gy, tz, heading
                )
                if not (math.isfinite(gx) and math.isfinite(gy)):
                    self._last_dispatch_fail_reason = "goal_position_not_finite"
                    self._dispatch_fail_count += 1
                    return (False, used_refined)
                self._mission_log_append(
                    "goal_computed",
                    {
                        "frame": map_frame,
                        "x": gx,
                        "y": gy,
                        "z": tz,
                        "yaw_deg": math.degrees(heading),
                        "offset": offset_m,
                        "distance_to_object": d,
                        "state": self.current_state,
                        "transform_ok": True,
                        "world_frame": map_frame,
                        "box_frame_id": map_frame,
                        "planned_tire_vehicle_center_goal": True,
                        "goal_world": {"x": gx, "y": gy, "z": tz},
                    },
                    sync=True,
                )
                self._current_goal_pose = goal
                self.current_goal_pub.publish(goal)
                if self._is_dry_run():
                    self.get_logger().info("Dry run: planned tire (far-side) goal validated, not sending to Nav2")
                    self._last_dispatch_fail_reason = None
                    return (True, used_refined)
                ok = send_nav_goal(self, goal, self._on_box_goal_done, feedback_cb=self._on_nav_feedback)
                if ok:
                    self._mission_log_append(
                        "nav_command_sent",
                        {
                            "frame": map_frame,
                            "x": gx,
                            "y": gy,
                            "z": tz,
                            "yaw_deg": math.degrees(heading),
                            "state": self.current_state,
                        },
                        sync=True,
                    )
                    self.get_logger().info(
                        f"NAV_COMMAND_SENT (planned tire far-side): pos=({gx:.3f},{gy:.3f},{tz:.3f}) yaw_deg={math.degrees(heading):.1f}"
                    )
                else:
                    self._last_dispatch_fail_reason = "nav_send_failed"
                    self._dispatch_fail_count += 1
                return (ok, used_refined)
        # Planned/committed tire position: skip stamp-age check (position is fixed; stamp would often be stale from segment_3d latency).
        sent = self._dispatch_box_goal(
            box,
            offset=self._last_tire_offset,
            detection_stamp=None,
            vehicle_box=self._committed_vehicle_box,
        )
        return (sent, used_refined)

    def _build_planned_tire_poses(
        self, positions: List[tuple]
    ) -> List[PoseStamped]:
        """Build PoseStamped list from planned tire positions (far-side placement)."""
        map_frame = self.get_parameter("map_frame").value
        offset_m = max(
            float(self.get_parameter("tire_offset").value),
            float(self.get_parameter("tire_staging_min_m").value or 0),
        )
        anchor = self._committed_vehicle_anchor_pose
        if (anchor is None or "x" not in anchor) and self._committed_vehicle_box is not None:
            vb = self._committed_vehicle_box
            anchor = {
                "x": (vb.xmin + vb.xmax) / 2.0,
                "y": (vb.ymin + vb.ymax) / 2.0,
                "z": (vb.zmin + vb.zmax) / 2.0,
            }
        if anchor is None or "x" not in anchor or "y" not in anchor:
            return []
        vcx, vcy = float(anchor["x"]), float(anchor["y"])
        poses = []
        for target_pos in positions:
            if len(target_pos) != 3 or not all(math.isfinite(x) for x in target_pos):
                continue
            tx, ty, tz = target_pos[0], target_pos[1], target_pos[2]
            out = standoff_goal_vehicle_center_tire_xy(vcx, vcy, tx, ty, offset_m)
            if out is None:
                continue
            gx, gy, heading, _d = out
            goal = pose_stamped_from_standoff_xy(
                map_frame, self.get_clock().now().to_msg(), gx, gy, tz, heading
            )
            if math.isfinite(gx) and math.isfinite(gy):
                poses.append(goal)
        return poses

    def _transform_pose_to_target_frame(
        self, pose_stamped: PoseStamped, target_frame: str
    ) -> Optional[PoseStamped]:
        """Transform a PoseStamped into ``target_frame`` (e.g. Nav2 ``map``). Returns None on TF failure."""
        return tf_transform_pose(
            self.tf_buffer,
            pose_stamped,
            target_frame,
            timeout_s=2.0,
            logger=self.get_logger(),
        )

    def _cancel_active_navigation_if_any(self) -> None:
        """Cancel NavigateToPose / FollowWaypoints before sending a new navigation command."""
        h = getattr(self, "_active_nav_goal_handle", None)
        if h is not None:
            try:
                h.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"cancel_active_navigation: nav goal cancel failed: {e}")
            self._active_nav_goal_handle = None
        fh = getattr(self, "_active_follow_waypoints_handle", None)
        if fh is not None:
            try:
                if hasattr(fh, "cancel_goal_async"):
                    fh.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"cancel_active_navigation: follow_waypoints cancel failed: {e}")
            self._active_follow_waypoints_handle = None

    def _vehicle_geometry_for_batch_waypoints_in_map(self) -> Optional[Dict[str, Any]]:
        """Return vehicle center, axes, half-extents in ``map`` frame for ``build_tyre_approach_waypoints``."""
        world_frame = self.get_parameter("world_frame").value
        map_frame = self.get_parameter("map_frame").value
        robot_w = self._get_current_pose()
        if robot_w is None:
            return None

        def _norm_xy(x: float, y: float) -> Tuple[float, float]:
            n = math.hypot(x, y)
            if n < 1e-9:
                return (1.0, 0.0)
            return (x / n, y / n)

        if bool(self.get_parameter("use_tyre_geometry").value) and self._tyre_geometry is not None:
            g = self._tyre_geometry
            cx, cy = float(g.mean[0]), float(g.mean[1])
            ps = PoseStamped()
            ps.header.frame_id = world_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = cx
            ps.pose.position.y = cy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            pc = tf_transform_pose(
                self.tf_buffer, ps, map_frame, timeout_s=2.0, logger=self.get_logger()
            )
            if pc is None:
                return None
            fx, fy = float(g.longitudinal_axis[0]), float(g.longitudinal_axis[1])
            rx, ry = float(g.right_axis[0]), float(g.right_axis[1])
            fwd_m = tf_transform_vector_xy(
                self.tf_buffer, fx, fy, world_frame, map_frame, logger=self.get_logger()
            )
            right_m = tf_transform_vector_xy(
                self.tf_buffer, rx, ry, world_frame, map_frame, logger=self.get_logger()
            )
            if fwd_m is None or right_m is None:
                return None
            fwd_m = _norm_xy(fwd_m[0], fwd_m[1])
            right_m = _norm_xy(right_m[0], right_m[1])
            return {
                "center_xy": (float(pc.pose.position.x), float(pc.pose.position.y)),
                "fwd_xy": fwd_m,
                "right_xy": right_m,
                "half_length_m": float(g.half_length),
                "half_width_m": float(g.half_width),
            }

        if self._committed_vehicle_box is None:
            return None
        fp = get_vehicle_footprint(
            self._committed_vehicle_box,
            (robot_w.pose.position.x, robot_w.pose.position.y, robot_w.pose.position.z),
        )
        if fp is None:
            return None
        cx, cy, L, W, yaw = fp
        fw, fyaw = math.cos(yaw), math.sin(yaw)
        rw, rry = -fyaw, fw
        ps = PoseStamped()
        ps.header.frame_id = world_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(cx)
        ps.pose.position.y = float(cy)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        pc = tf_transform_pose(
            self.tf_buffer, ps, map_frame, timeout_s=2.0, logger=self.get_logger()
        )
        if pc is None:
            return None
        fwd_m = tf_transform_vector_xy(
            self.tf_buffer, fw, fyaw, world_frame, map_frame, logger=self.get_logger()
        )
        right_m = tf_transform_vector_xy(
            self.tf_buffer, rw, rry, world_frame, map_frame, logger=self.get_logger()
        )
        if fwd_m is None or right_m is None:
            return None
        fwd_m = _norm_xy(fwd_m[0], fwd_m[1])
        right_m = _norm_xy(right_m[0], right_m[1])
        half_len = max(float(L) * 0.5, 0.1)
        half_wid = max(float(W) * 0.5, 0.1)
        return {
            "center_xy": (float(pc.pose.position.x), float(pc.pose.position.y)),
            "fwd_xy": fwd_m,
            "right_xy": right_m,
            "half_length_m": half_len,
            "half_width_m": half_wid,
        }

    def _build_full_waypoint_list(self) -> List[PoseStamped]:
        """Ordered Nav2 waypoints in ``map`` frame: optional perimeter polyline + standoff per planned tyre."""
        out: List[PoseStamped] = []
        if not self._planned_tire_positions:
            return out
        map_frame = self.get_parameter("map_frame").value
        world_frame = self.get_parameter("world_frame").value
        robot_pose_map = self._get_current_pose_in_map_frame()
        if robot_pose_map is None:
            self.get_logger().warn("batch waypoints: robot pose in map frame unavailable")
            return out
        geom = self._vehicle_geometry_for_batch_waypoints_in_map()
        if geom is None:
            self.get_logger().warn(
                "batch waypoints: vehicle geometry in map unavailable "
                "(need use_tyre_geometry + tyre poses, or a valid committed vehicle box)"
            )
            return out

        to = float(self.get_parameter("tire_offset").value)
        staging = float(self.get_parameter("tire_staging_min_m").value)
        standoff_m = max(to, staging if staging > 0 else 0.0)
        clearance = float(self.get_parameter("vehicle_perimeter_clearance_m").value)
        lat_eps = float(self.get_parameter("tyre_perimeter_lateral_eps_m").value)
        bridge = bool(self.get_parameter("tyre_perimeter_bridge_enabled").value)
        expected = int(self.get_parameter("expected_tires_per_vehicle").value)
        positions = list(self._planned_tire_positions)
        if expected >= 1 and len(positions) > expected:
            positions = positions[:expected]

        rx_m = float(robot_pose_map.pose.position.x)
        ry_m = float(robot_pose_map.pose.position.y)
        stamp = self.get_clock().now().to_msg()

        for target_xyz in positions:
            if len(target_xyz) != 3 or not all(math.isfinite(float(x)) for x in target_xyz):
                continue
            tx, ty, tz = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
            ps = PoseStamped()
            ps.header.frame_id = world_frame
            ps.header.stamp = stamp
            ps.pose.position.x = tx
            ps.pose.position.y = ty
            ps.pose.position.z = tz
            ps.pose.orientation.w = 1.0
            tyre_map = self._transform_pose_to_target_frame(ps, map_frame)
            if tyre_map is None:
                self.get_logger().warn(
                    f"batch waypoints: skip tyre at ({tx:.2f},{ty:.2f}) — TF to {map_frame} failed"
                )
                continue
            tmx = float(tyre_map.pose.position.x)
            tmy = float(tyre_map.pose.position.y)
            tmz = float(tyre_map.pose.position.z)

            if bridge:
                segment = build_tyre_approach_waypoints(
                    map_frame,
                    stamp,
                    (rx_m, ry_m),
                    (tmx, tmy, tmz),
                    geom["center_xy"],
                    geom["fwd_xy"],
                    geom["right_xy"],
                    geom["half_length_m"],
                    geom["half_width_m"],
                    standoff_m,
                    clearance,
                    lat_eps,
                )
            else:
                gx, gy, heading, _ = standoff_goal_robot_tyre_xy(rx_m, ry_m, tmx, tmy, standoff_m)
                segment = [
                    pose_stamped_from_standoff_xy(map_frame, stamp, gx, gy, tmz, heading)
                ]
            if not segment:
                continue
            out.extend(segment)
            rx_m = float(segment[-1].pose.position.x)
            ry_m = float(segment[-1].pose.position.y)

        return out

    def _sync_planned_tires_for_current_vehicle(self) -> None:
        """Load planned tires from current vehicle data into active list."""
        if self.current_vehicle_idx < len(self.detected_vehicles):
            planned = self.detected_vehicles[self.current_vehicle_idx].get("planned_tires", [])
            self._planned_tire_positions = list(planned) if planned else []
        else:
            self._planned_tire_positions = []

    def _is_box_valid_for_commit(self, box) -> bool:
        """Reject boxes with bad extent or aspect ratio before save/commit."""
        ex = abs(getattr(box, "xmax", 0) - getattr(box, "xmin", 0))
        ey = abs(getattr(box, "ymax", 0) - getattr(box, "ymin", 0))
        if not (math.isfinite(ex) and math.isfinite(ey)):
            return False
        min_ext = self.get_parameter("commit_min_extent_m").value
        max_ext = self.get_parameter("commit_max_extent_m").value
        max_ar = self.get_parameter("commit_max_aspect_ratio").value
        if ex < min_ext or ey < min_ext:
            self.get_logger().warn(
                f"Rejecting box: extent ({ex:.2f}, {ey:.2f}) < min {min_ext} m"
            )
            return False
        if ex > max_ext or ey > max_ext:
            self.get_logger().warn(
                f"Rejecting box: extent ({ex:.2f}, {ey:.2f}) > max {max_ext} m"
            )
            return False
        lo, hi = min(ex, ey), max(ex, ey)
        if lo > 0 and hi / lo > max_ar:
            self.get_logger().warn(
                f"Rejecting box: aspect ratio {hi / lo:.1f} > max {max_ar}"
            )
            return False
        return True

    def _get_geometry_for_vehicle(self, box) -> tuple:
        """Return (wheelbase_m, track_m) for box.object_name; truck/bus use overrides when > 0."""
        default_wb = float(self.get_parameter("vehicle_wheelbase_m").value)
        default_tr = float(self.get_parameter("vehicle_track_m").value)
        name = (getattr(box, "object_name", None) or "").lower()
        if "truck" in name:
            wb = float(self.get_parameter("vehicle_wheelbase_truck_m").value)
            tr = float(self.get_parameter("vehicle_track_truck_m").value)
            return (wb, tr) if wb > 0 and tr > 0 else (default_wb, default_tr)
        if "bus" in name:
            wb = float(self.get_parameter("vehicle_wheelbase_bus_m").value)
            tr = float(self.get_parameter("vehicle_track_bus_m").value)
            return (wb, tr) if wb > 0 and tr > 0 else (default_wb, default_tr)
        return (default_wb, default_tr)

    def _clear_committed_vehicle(self) -> None:
        """Clear committed plan (call when switching to next vehicle)."""
        self._committed_vehicle_box = None
        self._committed_vehicle_center = None
        self._committed_vehicle_planned_tires = None
        self._committed_vehicle_anchor_pose = None
        self._committed_vehicle_id = None
        self._pending_perimeter_nav_queue.clear()

    def _commit_current_vehicle(self) -> bool:
        """Lock in vehicle position and tire plan from detected_vehicles[current_vehicle_idx].
        All subsequent goals for this vehicle use this committed state only (no live box jitter).
        Returns True if commit succeeded, False if no vehicle data.
        """
        if self.current_vehicle_idx >= len(self.detected_vehicles):
            self._clear_committed_vehicle()
            return False
        v = self.detected_vehicles[self.current_vehicle_idx]
        box = v.get("box")
        if box is None or not self._is_box_valid_for_commit(box):
            self.get_logger().warn("Cannot commit vehicle: box missing or fails extent/aspect validation.")
            return False
        planned = v.get("planned_tires", [])
        if not planned or len(planned) != 4:
            self.get_logger().warn(
                "Cannot commit vehicle: planned_tires missing or not 4; will use live box for approach only."
            )
        self._committed_vehicle_box = v["box"]  # use same reference; box is not updated after save
        self._committed_vehicle_center = tuple(v["position"])
        self._committed_vehicle_planned_tires = list(planned) if planned else []
        anchor_pose = dict(v.get("anchor_pose", {}))
        # Fallback: derive anchor from box center if anchor_pose missing (ROS Answers 416578, Nav2 #4299)
        if not anchor_pose or "x" not in anchor_pose or "y" not in anchor_pose:
            b = v["box"]
            anchor_pose = {
                "x": (b.xmin + b.xmax) / 2.0,
                "y": (b.ymin + b.ymax) / 2.0,
                "z": (b.zmin + b.zmax) / 2.0,
                "frame_id": self.get_parameter("map_frame").value,
                "yaw": 0.0,
            }
        self._committed_vehicle_anchor_pose = anchor_pose
        self._committed_vehicle_id = v.get("vehicle_id")
        self._planned_tire_positions = list(self._committed_vehicle_planned_tires)
        # Single-tire test mode: only inspect the one nearest tire (e.g. robot 2 m in front of one tire)
        expected_tires = self.get_parameter("expected_tires_per_vehicle").value
        if expected_tires == 1 and len(self._planned_tire_positions) >= 1:
            robot_pose = self._get_current_pose()
            if robot_pose:
                rx, ry = robot_pose.pose.position.x, robot_pose.pose.position.y
                self._planned_tire_positions.sort(key=lambda t: math.hypot(t[0] - rx, t[1] - ry))
                self._planned_tire_positions = self._planned_tire_positions[:1]
        else:
            mode = str(self.get_parameter("tyre_inspection_order_mode").value).strip().lower()
            if mode == "vehicle_side" and len(self._committed_vehicle_planned_tires) == 4:
                robot_pose = self._get_current_pose()
                if robot_pose is not None:
                    rx = robot_pose.pose.position.x
                    ry = robot_pose.pose.position.y
                    ordered_from_tyres = False
                    if (
                        bool(self.get_parameter("use_tyre_geometry").value)
                        and self._tyre_3d_poses
                        and len(self._tyre_3d_poses) == 4
                    ):
                        try:
                            geom = tyre_geometry_from_poses(self._tyre_3d_poses, min_points=4)
                            if geom is not None:
                                idx_order = geom.visit_order_pose_indices(rx, ry)
                                self._planned_tire_positions = []
                                for j in idx_order:
                                    pos = self._tyre_3d_poses[j].position
                                    self._planned_tire_positions.append(
                                        (float(pos.x), float(pos.y), float(pos.z))
                                    )
                                ordered_from_tyres = True
                                self.get_logger().info(
                                    f"tyre_inspection_order_mode=vehicle_side: visit order from tyre geometry "
                                    f"(pose_indices={idx_order})"
                                )
                                self._mission_log_append(
                                    "tyre_visit_order_tyre_geometry",
                                    {"pose_indices": list(idx_order)},
                                    sync=True,
                                )
                        except Exception as ex:
                            self.get_logger().warn(
                                f"tyre geometry visit order failed ({ex}); falling back to vehicle box footprint."
                            )
                    if not ordered_from_tyres and self._committed_vehicle_box is not None:
                        fp = get_vehicle_footprint(
                            self._committed_vehicle_box,
                            (rx, ry, robot_pose.pose.position.z),
                        )
                        if fp is not None:
                            cx, cy, _L, _W, yaw = fp
                            fwd_x = math.cos(yaw)
                            fwd_y = math.sin(yaw)
                            right_x = -fwd_y
                            right_y = fwd_x
                            lon, lat = robot_lon_lat((rx, ry), (cx, cy), (fwd_x, fwd_y), (right_x, right_y))
                            side = classify_robot_side(lon, lat)
                            order = inspection_order_indices(side)
                            committed = [tuple(p) for p in self._committed_vehicle_planned_tires]
                            self._planned_tire_positions = [committed[i] for i in order]
                            self.get_logger().info(
                                f"tyre_inspection_order_mode=vehicle_side (box): slot visit order {order} "
                                f"(robot side={side.name})"
                            )
                            self._mission_log_append(
                                "tyre_visit_order_vehicle_side",
                                {"robot_side": side.name, "slot_order": list(order)},
                                sync=True,
                            )
        if self.current_vehicle_idx < len(self.detected_vehicles):
            self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
        self._mission_log_append(
            "vehicle_plan_committed",
            {
                "vehicle_id": self._committed_vehicle_id,
                "vehicle_idx": self.current_vehicle_idx,
                "center": list(self._committed_vehicle_center),
                "planned_tires": [list(p) for p in self._committed_vehicle_planned_tires],
                "anchor_pose": self._committed_vehicle_anchor_pose,
            },
            sync=True,
        )
        n_tires = len(self._planned_tire_positions)
        self.get_logger().info(
            f"Committed vehicle {self._committed_vehicle_id}: center={self._committed_vehicle_center}, "
            f"{n_tires} tire(s) in plan; all goals will use this plan."
        )
        self._ordered_tyre_indices = list(range(len(self._planned_tire_positions)))
        return True

    def _save_vehicle_position(self, box: BoundingBox3d):
        """Save detected vehicle position to detected_vehicles list and publish vehicle_detected anchor (map frame).

        Stores: center, full box extent, front/rear extreme points, orientation, and 4 planned tire
        positions inferred from box dimensions (longer dimension = car length, front = end closer to robot).
        """
        if not self._is_box_valid_for_commit(box):
            return
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        center_z = (box.zmin + box.zmax) / 2.0
        extent_x = abs(box.xmax - box.xmin)
        extent_y = abs(box.ymax - box.ymin)
        robot_pose = self._get_current_pose()
        robot_pos = (
            (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z)
            if robot_pose else None
        )
        front_point, rear_point = box_front_rear_points(box, robot_pos)

        # Check if this vehicle was already detected (avoid duplicates)
        tolerance = self.get_parameter("vehicle_duplicate_tolerance").value
        for existing in self.detected_vehicles:
            existing_pos = existing["position"]
            dist = math.sqrt(
                (center_x - existing_pos[0])**2 +
                (center_y - existing_pos[1])**2 +
                (center_z - existing_pos[2])**2
            )
            if dist < tolerance:
                self.get_logger().debug(f"Vehicle at ({center_x:.2f}, {center_y:.2f}) already detected, skipping duplicate")
                return

        # Resolve vehicle center in map frame and yaw (vehicle "facing" robot) for anchoring
        world_frame = self.get_parameter("world_frame").value
        map_frame = self.get_parameter("map_frame").value
        pose_slam = PoseStamped()
        pose_slam.header.frame_id = world_frame
        pose_slam.header.stamp = self.get_clock().now().to_msg()
        pose_slam.pose.position.x = center_x
        pose_slam.pose.position.y = center_y
        pose_slam.pose.position.z = center_z
        pose_slam.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame, world_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
            )
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_slam, transform)
            anchor_x = pose_map.pose.position.x
            anchor_y = pose_map.pose.position.y
            anchor_z = pose_map.pose.position.z
        except (TransformException, Exception):
            pose_map = pose_slam
            anchor_x, anchor_y, anchor_z = center_x, center_y, center_z
        if not self._vehicle_position_valid((anchor_x, anchor_y, anchor_z)):
            return
        robot_pose = self._get_current_pose()
        if robot_pose:
            rx = robot_pose.pose.position.x
            ry = robot_pose.pose.position.y
            dx = rx - center_x
            dy = ry - center_y
            d = math.sqrt(dx * dx + dy * dy)
            anchor_yaw = math.atan2(dy, dx) if d > 0.01 else 0.0
        else:
            anchor_yaw = 0.0

        vehicle_id = len(self.detected_vehicles) + 1
        anchor_pose = {"frame_id": map_frame, "x": anchor_x, "y": anchor_y, "z": anchor_z, "yaw": anchor_yaw}

        vehicle_data = {
            "box": box,
            "position": (center_x, center_y, center_z),
            "extent_x": extent_x,
            "extent_y": extent_y,
            "front_point": front_point,
            "rear_point": rear_point,
            "inspected": False,
            "class": box.object_name,
            "probability": box.probability,
            "vehicle_id": vehicle_id,
            "anchor_pose": anchor_pose,
        }
        self.detected_vehicles.append(vehicle_data)
        self._mission_report["total_vehicles"] = len(self.detected_vehicles)
        self._mission_report["total_tires_expected"] = (
            len(self.detected_vehicles) * self.get_parameter("expected_tires_per_vehicle").value
        )

        self._publish_vehicle_detected(
            vehicle_id=vehicle_id,
            frame_id=map_frame,
            x=anchor_x, y=anchor_y, z=anchor_z,
            yaw=anchor_yaw,
            confidence=float(box.probability),
        )
        # Plan tire positions from box dimensions (longer dimension = car length, front = end closer to robot)
        wheelbase, track = self._get_geometry_for_vehicle(box)
        planned = estimate_tire_positions_from_box(
            box,
            robot_pos,
            wheelbase_m=wheelbase,
            track_m=track,
        )
        if planned:
            vehicle_data["planned_tires"] = list(planned)
            if self.current_vehicle_idx >= len(self.detected_vehicles):
                self._planned_tire_positions = list(planned)
            self._mission_log_append(
                "tire_plan",
                {
                    "vehicle_id": vehicle_id,
                    "positions": planned,
                    "extent_x_m": round(extent_x, 3),
                    "extent_y_m": round(extent_y, 3),
                    "front_point": [round(front_point[0], 3), round(front_point[1], 3), round(front_point[2], 3)],
                    "rear_point": [round(rear_point[0], 3), round(rear_point[1], 3), round(rear_point[2], 3)],
                    "wheelbase_m": wheelbase,
                    "track_m": track,
                },
                sync=True,
            )
        self.get_logger().info(
            f"Saved vehicle: {box.object_name} center=({center_x:.2f},{center_y:.2f}) "
            f"extent=({extent_x:.2f}x{extent_y:.2f})m front=({front_point[0]:.2f},{front_point[1]:.2f}) "
            f"rear=({rear_point[0]:.2f},{rear_point[1]:.2f}) prob={box.probability:.2f} "
            f"planned_tires=4 Total vehicles={len(self.detected_vehicles)}"
        )

    # ----------------------- State Machine ----------------------- #
    def _tick(self):
        if self.current_state in (MissionState.DONE, MissionState.ERROR):
            return

        # Continuous Nav2 lifecycle/availability monitoring
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self._mission_log_append(
                "nav2_unavailable_runtime",
                {"state": self.current_state},
                sync=True,
            )
            if self.current_state not in (MissionState.IDLE,):
                self.get_logger().error("Nav2 unavailable during mission execution; pausing mission tick.")
                return

        # Mission heartbeat: every 10s log state and key flags for forensics
        now = time.time()
        if self._mission_start_time is not None and (self._mission_heartbeat_last_time is None or (now - self._mission_heartbeat_last_time) >= 10.0):
            self._mission_heartbeat_last_time = now
            tf_valid = False
            try:
                wf = self.get_parameter("world_frame").value
                bf = self.get_parameter("base_frame").value
                self.tf_buffer.lookup_transform(
                    wf, bf, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                tf_valid = True
            except Exception:
                pass
            self._mission_log_append(
                "mission_heartbeat",
                {
                    "state": self.current_state,
                    "tf_valid": tf_valid,
                    "tf_watchdog_paused": self._tf_watchdog_paused,
                    "tf_paused_elapsed_s": (now - self._tf_watchdog_paused_since) if self._tf_watchdog_paused_since else 0,
                    "detected_vehicles": len(self.detected_vehicles),
                    "has_current_vehicle_box": self.current_vehicle_box is not None,
                    "dispatched_approach_this_wait": self._dispatched_approach_this_wait,
                    "elapsed_since_start": round(now - self._mission_start_time, 1),
                },
                sync=True,
            )

        # Phase C: TF watchdog - do not progress if TF invalid
        if not self._check_tf_watchdog():
            return

        # Depth gate: if navigation is not permitted, do not dispatch goals
        if self.get_parameter("require_nav_permitted").value:
            if self._nav_permitted is not True:
                self._mission_log_append(
                    "nav_gate_blocked",
                    {
                        "state": self.current_state,
                        "nav_permitted": self._nav_permitted,
                        "last_nav_permitted_time": self._last_nav_permitted_time,
                        "topic": self.get_parameter("nav_permitted_topic").value,
                    },
                    sync=True,
                )
                return

        # Fail hard after repeated dispatch failures
        if self._dispatch_fail_count >= self.get_parameter("dispatch_fail_abort_count").value:
            self.get_logger().error(
                f"Dispatch failure count {self._dispatch_fail_count} exceeded threshold; aborting mission."
            )
            self._mission_log_append(
                "dispatch_fail_abort",
                {
                    "count": self._dispatch_fail_count,
                    "state": self.current_state,
                    "reason": self._last_dispatch_fail_reason,
                },
                sync=True,
            )
            self._mission_report["error_states_encountered"] += 1
            self._set_state(MissionState.ERROR, cause="dispatch_failures")
            return

        # Watchdog: goal sent but no cmd_vel seen
        if self._last_goal_dispatch_time is not None:
            cmd_timeout = self.get_parameter("cmd_vel_timeout_s").value
            now = time.time()
            if self._last_cmd_vel_time is None or (now - self._last_cmd_vel_time) > cmd_timeout:
                if (now - self._last_goal_dispatch_time) > cmd_timeout:
                    self.get_logger().error(
                        f"No cmd_vel within {cmd_timeout}s after goal dispatch; likely motion blocked."
                    )
                    self._mission_log_append(
                        "cmd_vel_timeout",
                        {
                            "timeout_s": cmd_timeout,
                            "state": self.current_state,
                            "last_goal_dispatch_time": self._last_goal_dispatch_time,
                            "last_cmd_vel_time": self._last_cmd_vel_time,
                            "last_cmd_vel_source": self._last_cmd_vel_source,
                        },
                        sync=True,
                    )
                    # Clear to avoid spamming every tick
                    self._last_goal_dispatch_time = None

        # Proximity gating: when distance_remaining < threshold, log "approaching" (throttled) and set gate passed
        if self.current_state in (MissionState.APPROACH_VEHICLE, MissionState.INSPECT_TIRE):
            prox_gate = float(self.get_parameter("proximity_gate_distance_m").value)
            if prox_gate > 0 and self._active_nav_goal_handle is not None and self._last_nav_feedback is not None:
                dist_rem = getattr(self._last_nav_feedback, "distance_remaining", None)
                if dist_rem is not None and dist_rem < prox_gate:
                    self._proximity_gate_passed = True
                    if self._last_proximity_log_time is None or (now - self._last_proximity_log_time) >= 5.0:
                        self._last_proximity_log_time = now
                        self.get_logger().info(
                            f"Proximity gate passed: distance_remaining={dist_rem:.2f}m < {prox_gate}m"
                        )

        # Centroid servo handoff: when close enough, cancel Nav2 and let centroid servo fine-position.
        # Optional spatial filter: only hand off when a tire detection is near the expected tire position (reduces false captures).
        if (
            self.current_state == MissionState.INSPECT_TIRE
            and not self._centroid_handoff_initiated
            and not self._recovery_skip_initiated
            and self.get_parameter("centroid_servo_enabled").value
            and self._active_nav_goal_handle is not None
            and self._last_nav_feedback is not None
        ):
            dist_rem = getattr(self._last_nav_feedback, "distance_remaining", None)
            centroid_prox = float(self.get_parameter("centroid_servo_proximity_m").value)
            if dist_rem is not None and dist_rem < centroid_prox:
                allow_handoff = True
                if self.get_parameter("centroid_servo_require_detection_near_expected").value and self._tire_registry:
                    expected_xy = (self._tire_registry[-1]["position"][0], self._tire_registry[-1]["position"][1])
                    max_dist = float(self.get_parameter("capture_max_distance_detection_to_expected_tire_m").value)
                    has_near_detection = self._detection_near_expected_tire(expected_xy)
                    if not has_near_detection:
                        timeout_s = float(self.get_parameter("centroid_servo_wait_for_detection_timeout_s").value)
                        elapsed = (now - self._approach_entered_time) if self._approach_entered_time else 0
                        if elapsed < timeout_s:
                            allow_handoff = False
                            if getattr(self, "_centroid_spatial_skip_log_time", None) is None or (now - getattr(self, "_centroid_spatial_skip_log_time", 0)) >= 5.0:
                                self._centroid_spatial_skip_log_time = now
                                self.get_logger().info(
                                    f"Centroid handoff deferred: no tire detection within {max_dist}m of expected tire; "
                                    f"elapsed={elapsed:.1f}s (timeout={timeout_s}s)."
                                )
                if allow_handoff:
                    self.get_logger().info(
                        f"Centroid handoff: distance_remaining={dist_rem:.2f}m < {centroid_prox}m; cancelling Nav2."
                    )
                    self._centroid_handoff_initiated = True
                    self._mission_log_append(
                        "centroid_handoff",
                        {"distance_remaining": dist_rem, "threshold_m": centroid_prox},
                        sync=True,
                    )
                    try:
                        self._active_nav_goal_handle.cancel_goal_async()
                    except Exception as e:
                        self.get_logger().error(f"Failed to cancel goal for centroid handoff: {e}")
                        self._centroid_handoff_initiated = False
                    else:
                        msg = Bool()
                        msg.data = True
                        self.centroid_servo_enable_pub.publish(msg)

        # Recovery-aware skip: cancel and skip tire when too many recoveries and still far
        if self.current_state == MissionState.INSPECT_TIRE and not self._recovery_skip_initiated:
            max_rec = int(self.get_parameter("max_recoveries_before_skip").value)
            if max_rec > 0 and self._active_nav_goal_handle is not None and self._last_nav_feedback is not None:
                num_rec = getattr(self._last_nav_feedback, "number_of_recoveries", 0) or 0
                dist_rem = getattr(self._last_nav_feedback, "distance_remaining", None)
                min_time = float(self.get_parameter("recovery_skip_min_nav_time_s").value)
                dist_thresh = float(self.get_parameter("recovery_skip_distance_threshold_m").value)
                nav_elapsed = (now - self._approach_entered_time) if self._approach_entered_time else 0
                if (
                    num_rec >= max_rec
                    and nav_elapsed >= min_time
                    and (dist_rem is None or dist_rem > dist_thresh)
                ):
                    self.get_logger().warn(
                        f"Recovery-aware skip: number_of_recoveries={num_rec} >= {max_rec}, "
                        f"nav_time={nav_elapsed:.0f}s >= {min_time}s, distance_remaining={dist_rem}; cancelling goal."
                    )
                    self._recovery_skip_initiated = True
                    self._mission_log_append(
                        "recovery_skip_triggered",
                        {
                            "number_of_recoveries": num_rec,
                            "nav_elapsed_s": round(nav_elapsed, 1),
                            "distance_remaining": dist_rem,
                        },
                        sync=True,
                    )
                    try:
                        self._active_nav_goal_handle.cancel_goal_async()
                    except Exception as e:
                        self.get_logger().error(f"Failed to cancel goal for recovery skip: {e}")
                        self._recovery_skip_initiated = False

        # Progress watchdog: detect stall or spin during active motion
        if self.current_state in (MissionState.APPROACH_VEHICLE, MissionState.INSPECT_TIRE, MissionState.FACE_TIRE):
            pose = self._get_current_pose()
            yaw = self._get_current_yaw()
            now = time.time()
            if pose and yaw is not None:
                if self._progress_window_start is None:
                    self._progress_window_start = now
                    self._progress_start_pose = (pose.pose.position.x, pose.pose.position.y, yaw)
                else:
                    elapsed = now - self._progress_window_start
                    if elapsed >= self.get_parameter("progress_stall_timeout_s").value:
                        sx, sy, syaw = self._progress_start_pose
                        dx = pose.pose.position.x - sx
                        dy = pose.pose.position.y - sy
                        dist = math.sqrt(dx * dx + dy * dy)
                        yaw_delta = abs(math.degrees(yaw - syaw))
                        if yaw_delta > 180.0:
                            yaw_delta = 360.0 - yaw_delta
                        goal_tol = self._goal_tolerance_for_state()
                        distance_to_goal = (
                            self._distance_to_current_goal() if goal_tol is not None else None
                        )
                        if (
                            goal_tol is not None
                            and distance_to_goal is not None
                            and distance_to_goal <= goal_tol
                        ):
                            self.get_logger().warn(
                                f"Progress stall but within {goal_tol:.2f}m of goal; treating as success."
                            )
                            self._mission_log_append(
                                "progress_stall_near_goal",
                                {
                                    "state": self.current_state,
                                    "distance_to_goal": round(distance_to_goal, 3),
                                    "tolerance_m": goal_tol,
                                },
                                sync=True,
                            )
                            if self._active_nav_goal_handle is not None:
                                try:
                                    self._active_nav_goal_handle.cancel_goal_async()
                                except Exception as e:
                                    self.get_logger().warn(f"Cancel goal after near-goal stall: {e}")
                                self._active_nav_goal_handle = None
                            cause = (
                                "approach_reached_by_distance"
                                if self.current_state == MissionState.APPROACH_VEHICLE
                                else "tire_reached_by_distance"
                            )
                            self._handle_box_goal_success(cause)
                            return
                        if self.current_state == MissionState.APPROACH_VEHICLE:
                            anchor_tol = float(self.get_parameter("vehicle_anchor_reach_distance_m").value)
                            anchor_dist = self._distance_to_vehicle_anchor()
                            if anchor_dist is not None and anchor_dist <= anchor_tol:
                                self.get_logger().warn(
                                    f"Progress stall but within {anchor_tol:.2f}m of vehicle anchor; treating as success."
                                )
                                self._mission_log_append(
                                    "progress_stall_near_anchor",
                                    {
                                        "state": self.current_state,
                                        "distance_to_anchor": round(anchor_dist, 3),
                                        "tolerance_m": anchor_tol,
                                    },
                                    sync=True,
                                )
                                if self._active_nav_goal_handle is not None:
                                    try:
                                        self._active_nav_goal_handle.cancel_goal_async()
                                    except Exception as e:
                                        self.get_logger().warn(f"Cancel goal after anchor stall: {e}")
                                    self._active_nav_goal_handle = None
                                self._handle_box_goal_success("approach_reached_by_anchor")
                                return
                        if dist < self.get_parameter("progress_min_delta_m").value:
                            self._mission_log_append(
                                "progress_stall",
                                {
                                    "state": self.current_state,
                                    "elapsed_s": round(elapsed, 1),
                                    "distance_m": round(dist, 3),
                                    "yaw_delta_deg": round(yaw_delta, 1),
                                },
                                sync=True,
                            )
                            self._progress_stall_count += 1
                            if self._progress_stall_count >= 3:
                                self.get_logger().warn(
                                    "Repeated progress stalls; cancelling goal and retrying."
                                )
                                if self._active_nav_goal_handle is not None:
                                    try:
                                        self._active_nav_goal_handle.cancel_goal_async()
                                    except Exception as e:
                                        self.get_logger().warn(f"Cancel goal after stall: {e}")
                                    self._active_nav_goal_handle = None
                                if self.current_state == MissionState.APPROACH_VEHICLE:
                                    self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="progress_stall")
                                elif self.current_state == MissionState.INSPECT_TIRE:
                                    self._set_state(MissionState.WAIT_TIRE_BOX, cause="progress_stall")
                                self._progress_stall_count = 0
                        if (
                            self.current_state != MissionState.FACE_TIRE
                            and
                            elapsed >= self.get_parameter("spin_detect_min_time_s").value
                            and dist < self.get_parameter("spin_detect_distance_m").value
                            and yaw_delta >= self.get_parameter("spin_detect_yaw_deg").value
                        ):
                            self.get_logger().error("Spin detected without progress; dumping diagnostics.")
                            self._mission_log_append(
                                "spin_detected",
                                {
                                    "state": self.current_state,
                                    "elapsed_s": round(elapsed, 1),
                                    "distance_m": round(dist, 3),
                                    "yaw_delta_deg": round(yaw_delta, 1),
                                },
                                sync=True,
                            )
                        # reset window
                        self._progress_window_start = now
                        self._progress_start_pose = (pose.pose.position.x, pose.pose.position.y, yaw)

        # Phase J: Hard mission timeout
        if self._mission_start_time is not None:
            elapsed = time.time() - self._mission_start_time
            if elapsed > self.get_parameter("hard_mission_timeout").value:
                self.get_logger().error(f"Hard mission timeout ({elapsed:.0f}s). Aborting.")
                self._last_transition_cause = "hard_mission_timeout"
                self._mission_log_append(
                    "hard_timeout",
                    {"elapsed_s": elapsed, "state": self.current_state},
                    sync=True,
                )
                self._mission_report["error_states_encountered"] += 1
                self._publish_mission_report()
                self._set_state(MissionState.ERROR, cause="hard_mission_timeout")
                return

        use_dynamic = self.get_parameter("use_dynamic_detection").value

        if self.current_state == MissionState.IDLE:
            if not self.get_parameter("start_mission_on_ready").value and not self._mission_start_requested:
                return
            # Full-system-on-launch: wait for Nav2 navigate_to_pose before starting (skip in dry_run)
            if not self._is_dry_run():
                if self._nav2_wait_start_time is None:
                    self._nav2_wait_start_time = time.time()
                nav2_timeout = self.get_parameter("nav2_wait_timeout").value
                if not self.nav_client.wait_for_server(timeout_sec=1.0):
                    elapsed = time.time() - self._nav2_wait_start_time
                    if elapsed > nav2_timeout:
                        self.get_logger().error(
                            f"Nav2 navigate_to_pose not available after {nav2_timeout}s. Mission not started. "
                            "Ensure full_bringup has started Nav2 and lifecycle has completed."
                        )
                        self._mission_log_append(
                            "nav2_unavailable",
                            {"timeout_s": nav2_timeout},
                            sync=True,
                        )
                        self._mission_report["error_states_encountered"] += 1
                        self._set_state(MissionState.ERROR, cause="nav2_unavailable")
                        self._nav2_wait_start_time = None
                        return
                    elif elapsed - self._nav2_wait_last_log_elapsed >= 10.0 or self._nav2_wait_last_log_elapsed == 0.0:
                        self.get_logger().info(
                            f"Waiting for Nav2 navigate_to_pose action server... ({elapsed:.0f}s / {nav2_timeout}s)"
                        )
                        self._nav2_wait_last_log_elapsed = elapsed
                        self._mission_log_append(
                            "idle_wait_nav2",
                            {"elapsed_s": round(elapsed, 1), "timeout_s": nav2_timeout},
                            sync=True,
                        )
                    return
                # Nav2 ready
                if self._nav2_wait_start_time is not None:
                    self.get_logger().info("Nav2 navigate_to_pose available. Starting mission.")
                self._nav2_wait_start_time = None
                self._nav2_wait_last_log_elapsed = 0.0
                # When use_follow_waypoints / use_batch_waypoints: also wait for follow_waypoints action server
                if self.follow_waypoints_client and not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().warn(
                        "FollowWaypoints action server not available; use_follow_waypoints/use_batch_waypoints set but server missing. "
                        "Will fall back to sequential navigate_to_pose if batch path not taken."
                    )
            # Require TF (slamware_map->base_link) valid and stable before starting so we don't enter SEARCH_VEHICLE then immediately pause
            world_frame = self.get_parameter("world_frame").value
            base_frame = self.get_parameter("base_frame").value
            map_frame = self.get_parameter("map_frame").value
            tf_wait_timeout = self.get_parameter("tf_wait_timeout").value
            tf_stable_s = self.get_parameter("tf_stable_s").value
            now_tf = time.time()
            try:
                self.tf_buffer.lookup_transform(
                world_frame, base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
                if self._tf_stable_since is None:
                    self._tf_stable_since = now_tf
                stable_elapsed = now_tf - self._tf_stable_since
                if stable_elapsed < tf_stable_s:
                    self._mission_log_append(
                        "idle_wait_tf_stable",
                        {"stable_elapsed_s": round(stable_elapsed, 1), "required_s": tf_stable_s},
                        sync=True,
                    )
                    self.get_logger().info(
                        f"TF valid; waiting for stability ({stable_elapsed:.1f}s / {tf_stable_s}s)"
                    )
                    return
                self._tf_wait_start_time = None
                self._tf_wait_last_log_elapsed = 0.0
                self._tf_stable_since = None
            except Exception:
                self._tf_stable_since = None
                if self._tf_wait_start_time is None:
                    self._tf_wait_start_time = time.time()
                elapsed = time.time() - self._tf_wait_start_time
                if elapsed > tf_wait_timeout:
                    self.get_logger().error(
                        f"TF ({world_frame}->{base_frame}) not available after {tf_wait_timeout}s. "
                        "Mission not started. Ensure Aurora is on and publishing TF."
                    )
                    self._mission_log_append(
                        "tf_unavailable_at_start",
                        {"timeout_s": tf_wait_timeout},
                        sync=True,
                    )
                    self._mission_report["error_states_encountered"] += 1
                    self._set_state(MissionState.ERROR, cause="tf_unavailable_at_start")
                    self._tf_wait_start_time = None
                    return
                if elapsed - self._tf_wait_last_log_elapsed >= 10.0 or self._tf_wait_last_log_elapsed == 0.0:
                    self.get_logger().info(
                        f"Waiting for TF {world_frame}->{base_frame}... ({elapsed:.0f}s / {tf_wait_timeout}s)"
                    )
                    self._tf_wait_last_log_elapsed = elapsed
                    self._mission_log_append(
                        "idle_wait_tf",
                        {
                            "elapsed_s": round(elapsed, 1),
                            "timeout_s": tf_wait_timeout,
                            "frame": f"{world_frame}->{base_frame}",
                        },
                        sync=True,
                    )
                return
            if self.get_parameter("require_goal_transform").value:
                try:
                    self.tf_buffer.lookup_transform(
                    map_frame, world_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                except Exception:
                    if self._tf_wait_start_time is None:
                        self._tf_wait_start_time = time.time()
                    elapsed = time.time() - self._tf_wait_start_time
                    if elapsed > tf_wait_timeout:
                        self.get_logger().error(
                            f"TF ({map_frame}->{world_frame}) not available after {tf_wait_timeout}s. "
                            "Mission not started. Ensure Nav2 is running and map frame exists."
                        )
                        self._mission_log_append(
                            "map_tf_unavailable_at_start",
                            {"timeout_s": tf_wait_timeout, "frame": f"{map_frame}->{world_frame}"},
                            sync=True,
                        )
                        self._mission_report["error_states_encountered"] += 1
                        self._set_state(MissionState.ERROR, cause="map_tf_unavailable_at_start")
                        self._tf_wait_start_time = None
                        return
                    if elapsed - self._tf_wait_last_log_elapsed >= 10.0 or self._tf_wait_last_log_elapsed == 0.0:
                        self.get_logger().info(
                            f"Waiting for TF {map_frame}->{world_frame}... ({elapsed:.0f}s / {tf_wait_timeout}s)"
                        )
                        self._tf_wait_last_log_elapsed = elapsed
                        self._mission_log_append(
                            "idle_wait_map_tf",
                            {
                                "elapsed_s": round(elapsed, 1),
                                "timeout_s": tf_wait_timeout,
                                "frame": f"{map_frame}->{world_frame}",
                            },
                            sync=True,
                        )
                    return
                # Reset TF wait timer on success
                self._tf_wait_start_time = None
                self._tf_wait_last_log_elapsed = 0.0
            # Startup invariants: required topics must have published at least once
            startup_timeout = self.get_parameter("startup_detection_wait_timeout_s").value
            if self._startup_detection_wait_start_time is None:
                self._startup_detection_wait_start_time = time.time()
            elapsed_startup = time.time() - self._startup_detection_wait_start_time
            detection_alive = self._last_detection_msg_time is not None
            vehicle_boxes_topic = self.get_parameter("vehicle_boxes_topic").value
            vehicle_topic = vehicle_boxes_topic if vehicle_boxes_topic else self.get_parameter("vehicle_detection_topic").value
            vehicle_boxes_alive = self._last_vehicle_boxes_time is not None
            require_nav = self.get_parameter("require_nav_permitted").value
            nav_permitted_alive = (
                not require_nav
                or self._last_nav_permitted_time is not None
            )
            require_det_at_start = self.get_parameter("require_detection_topic_at_startup").value
            startup_ok = vehicle_boxes_alive and nav_permitted_alive and (detection_alive or not require_det_at_start)
            if not startup_ok:
                if elapsed_startup > startup_timeout:
                    self.get_logger().error(
                        f"Startup invariants failed: topics not alive within {startup_timeout}s. "
                        f"detection_alive={detection_alive}, vehicle_boxes_alive={vehicle_boxes_alive}, "
                        f"nav_permitted_alive={nav_permitted_alive}, require_detection_at_startup={require_det_at_start}. Aborting mission."
                    )
                    self._mission_log_append(
                        "startup_topic_not_alive",
                        {
                            "timeout_s": startup_timeout,
                            "detection_alive": detection_alive,
                            "vehicle_boxes_alive": vehicle_boxes_alive,
                            "nav_permitted_alive": nav_permitted_alive,
                            "detection_topic": self.get_parameter("detection_topic").value,
                            "vehicle_boxes_topic": vehicle_topic,
                            "nav_permitted_topic": self.get_parameter("nav_permitted_topic").value,
                        },
                        sync=True,
                    )
                    self._mission_report["error_states_encountered"] += 1
                    self._set_state(MissionState.ERROR, cause=TransitionReason.TOPIC_NOT_ALIVE.value)
                    self._startup_detection_wait_start_time = None
                    return
                if elapsed_startup > 1.0 and (elapsed_startup - self._startup_topics_last_log_time) >= 10.0:
                    self._startup_topics_last_log_time = elapsed_startup
                    self.get_logger().info(
                        f"Waiting for required topics ({elapsed_startup:.0f}s / {startup_timeout}s): "
                        f"detection={detection_alive}, vehicle_boxes={vehicle_boxes_alive}, "
                        f"nav_permitted={nav_permitted_alive}"
                    )
                    self._mission_log_append(
                        "idle_wait_topics",
                        {
                            "elapsed_s": round(elapsed_startup, 1),
                            "timeout_s": startup_timeout,
                            "detection_alive": detection_alive,
                            "vehicle_boxes_alive": vehicle_boxes_alive,
                            "nav_permitted_alive": nav_permitted_alive,
                        },
                        sync=True,
                    )
                return
            self._startup_detection_wait_start_time = None
            self._mission_log_append(
                "idle_passed",
                {"nav2_ok": True, "tf_ok": True, "tf_stable_s": tf_stable_s, "topics_alive": True},
                sync=True,
            )
            if use_dynamic:
                self._rotate_mission_logs()
                self._mission_start_time = time.time()
                start_pose = self._get_current_pose_in_map_frame()
                if start_pose is not None:
                    self._mission_start_robot_position = (
                        start_pose.pose.position.x,
                        start_pose.pose.position.y,
                        start_pose.pose.position.z,
                    )
                else:
                    self._mission_start_robot_position = None
                self._last_transition_cause = "user_start"
                self._mission_log_append(
                    "mission_start",
                    {
                        "reason": "user_start",
                        "require_sensor_health": self.get_parameter("require_sensor_health").value,
                    },
                    sync=True,
                )
                if self.get_parameter("require_sensor_health").value:
                    self.get_logger().info("Starting mission: INIT (awaiting sensor health).")
                    self.wait_start_time = time.time()
                    self._set_state(MissionState.INIT)
                else:
                    self.get_logger().info("Starting mission with dynamic vehicle detection (Aurora-based).")
                    self.get_logger().info("Robot will search for vehicles (car/truck) and save their positions.")
                    self._set_state(MissionState.SEARCH_VEHICLE)
            else:
                trucks = self._load_trucks()
                if not trucks:
                    self.get_logger().warn("No vehicles configured; mission done.")
                    self._set_state(MissionState.DONE, cause="no_vehicles_configured")
                    return
                self.get_logger().info("Starting mission with YAML vehicle list (legacy mode).")
                self._dispatch_standoff_goal()
            return

        if self.current_state == MissionState.INIT:
            timeout = self.get_parameter("sensor_health_timeout").value
            elapsed = time.time() - (self.wait_start_time or time.time())
            if self._sensor_healthy is True:
                self.get_logger().info("Sensor health OK. Proceeding to SEARCH_VEHICLE.")
                self._set_state(MissionState.SEARCH_VEHICLE, cause="sensor_healthy")
            elif elapsed > timeout:
                self.get_logger().warn(
                    f"Sensor health timeout ({timeout}s). Proceeding to SEARCH_VEHICLE anyway."
                )
                self._set_state(MissionState.SEARCH_VEHICLE, cause="sensor_health_timeout")
            return

        if self.current_state == MissionState.SEARCH_VEHICLE:
            if bool(self.get_parameter("use_tyre_3d_positions").value):
                if self._try_dispatch_from_tyre_3d():
                    return
            # In dynamic mode, wait for vehicle detection to populate detected_vehicles list
            # Once we have vehicles, move to first vehicle
            if len(self.detected_vehicles) > 0:
                # Find first un-inspected vehicle
                un_inspected = [v for v in self.detected_vehicles if not v["inspected"]]
                if un_inspected:
                    self.current_vehicle_idx = self.detected_vehicles.index(un_inspected[0])
                    self.current_vehicle_box = un_inspected[0]["box"]
                    self._commit_current_vehicle()  # lock plan so all goals use this exact position and tire list
                    self.get_logger().info(
                        f"Found {len(self.detected_vehicles)} vehicle(s). "
                        f"Starting inspection of vehicle {self.current_vehicle_idx + 1} ({un_inspected[0]['class']})."
                    )
                    self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="vehicles_available")
                else:
                    self.get_logger().info("All detected vehicles have been inspected. Mission complete.")
                    self._mission_report["total_vehicles"] = len(self.detected_vehicles)
                    self._mission_report["total_tires_expected"] = (
                        len(self.detected_vehicles) * self.get_parameter("expected_tires_per_vehicle").value
                    )
                    self._publish_mission_report()
                    self._set_state(MissionState.DONE, cause="all_detected_vehicles_inspected")
                return
            # No vehicles detected: patrol/rotate with timeout
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("detection_timeout").value
                search_timeout = self.get_parameter("vehicle_search_timeout_s").value
                max_rot = self.get_parameter("max_rotation_attempts").value
                if self._vehicle_search_start_time is not None:
                    total_elapsed = time.time() - self._vehicle_search_start_time
                    if total_elapsed > search_timeout:
                        self.get_logger().warn(
                            f"Vehicle search timeout ({total_elapsed:.0f}s). Ending mission."
                        )
                        self._mission_report["total_vehicles"] = len(self.detected_vehicles) if self.detected_vehicles else 0
                        self._mission_report["total_tires_expected"] = (
                            len(self.detected_vehicles) * self.get_parameter("expected_tires_per_vehicle").value
                        )
                        self._publish_mission_report()
                        self._set_state(MissionState.DONE, cause="vehicle_search_timeout")
                        return
                if elapsed > timeout:
                    # Attempt slow patrol step
                    now = time.time()
                    patrol_interval = self.get_parameter("vehicle_patrol_interval_s").value
                    if (
                        self._last_patrol_time is None
                        or (now - self._last_patrol_time) >= patrol_interval
                    ):
                        if self._patrol_attempts < int(self.get_parameter("patrol_max_attempts").value):
                            self._patrol_attempts += 1
                            self._last_patrol_time = now
                            self.get_logger().info(
                                f"No vehicles after {elapsed:.1f}s. Patrol step {self._patrol_attempts}."
                            )
                            self._dispatch_patrol_goal()
                            self._set_state(MissionState.PATROL_SEARCH, cause="patrol_search")
                            return
                    if self.get_parameter("rotation_search_enabled").value and self.rotation_attempts < max_rot:
                        self.get_logger().info(
                            f"No vehicle detected after {elapsed:.1f}s. "
                            f"Rotating to search (attempt {self.rotation_attempts + 1}/{max_rot})."
                        )
                        self._dispatch_rotation_goal(is_vehicle=False, is_search=True)
                    else:
                        self.get_logger().warn(
                            "Max search rotations reached with no vehicles detected. Mission complete."
                        )
                        self._mission_report["total_vehicles"] = len(self.detected_vehicles) if self.detected_vehicles else 0
                        self._mission_report["total_tires_expected"] = (
                            len(self.detected_vehicles) * self.get_parameter("expected_tires_per_vehicle").value
                        )
                        self._publish_mission_report()
                        self._set_state(MissionState.DONE, cause="vehicle_search_exhausted")
            return

        if self.current_state == MissionState.TURN_IN_PLACE_SEARCH:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.FOLLOW_WAYPOINTS_BATCH:
            return  # waiting for FollowWaypoints result callback

        if self.current_state == MissionState.PATROL_SEARCH:
            return  # waiting on patrol nav to complete

        if self.current_state == MissionState.WAIT_VEHICLE_BOX:
            if bool(self.get_parameter("use_tyre_3d_positions").value):
                if self._try_dispatch_from_tyre_3d():
                    return
            # Use committed vehicle plan only: no live box jitter. Commit once per vehicle.
            if not self._dispatched_approach_this_wait and self.current_vehicle_box is not None:
                if self._committed_vehicle_box is None:
                    self._commit_current_vehicle()
                approach_box = self._committed_vehicle_box if self._committed_vehicle_box is not None else self.current_vehicle_box
                offset = self.get_parameter("approach_offset").value
                self._last_approach_box = approach_box
                self._last_approach_offset = offset
                log_bounding_box(self.get_logger(), approach_box, f"VEHICLE_{approach_box.object_name.upper()}")

                # approach_nearest_corner: go to nearest corner (tire) instead of nearest face (bumper).
                # Bounding box = costmap; corners are long-side vertices where tires are.
                if bool(self.get_parameter("approach_nearest_corner").value):
                    expected_tires = self.get_parameter("expected_tires_per_vehicle").value
                    # Refill only when we have no tires, or expect 4 and have fewer (single-tire mode keeps 1 at commit)
                    need_refill = not self._planned_tire_positions or (
                        expected_tires >= 4 and len(self._planned_tire_positions) < 4
                    )
                    if need_refill:
                        robot_pose = self._get_current_pose()
                        robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z) if robot_pose else None
                        wheelbase, track = self._get_geometry_for_vehicle(approach_box)
                        planned = estimate_tire_positions_from_box(approach_box, robot_pos, wheelbase_m=wheelbase, track_m=track)
                        if planned:
                            self._planned_tire_positions = list(planned)
                            if expected_tires == 1 and robot_pose:
                                rx, ry = robot_pose.pose.position.x, robot_pose.pose.position.y
                                self._planned_tire_positions.sort(key=lambda t: math.hypot(t[0] - rx, t[1] - ry))
                                self._planned_tire_positions = self._planned_tire_positions[:1]
                            if self.current_vehicle_idx < len(self.detected_vehicles):
                                self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
                    if self._planned_tire_positions:
                        # Sort by distance to robot (nearest corner first) unless batch waypoints (fixed inspection order)
                        robot_pose = self._get_current_pose()
                        if robot_pose and not bool(
                            self.get_parameter("use_batch_waypoints").value
                        ):
                            rx, ry = robot_pose.pose.position.x, robot_pose.pose.position.y
                            self._planned_tire_positions.sort(
                                key=lambda t: math.hypot(t[0] - rx, t[1] - ry)
                            )
                        use_batch = bool(self.get_parameter("use_batch_waypoints").value)
                        use_fw = bool(self.get_parameter("use_follow_waypoints").value)
                        expected_tires = int(self.get_parameter("expected_tires_per_vehicle").value)
                        n_plan = len(self._planned_tire_positions)
                        # Full approach list in map frame (perimeter + standoff per tyre)
                        if use_batch and n_plan >= 1 and expected_tires >= 1:
                            self._cancel_active_navigation_if_any()
                            poses = self._build_full_waypoint_list()
                            if poses:
                                self._last_follow_waypoints_pose_count = len(poses)
                                self._last_batch_waypoints_perimeter = bool(
                                    self.get_parameter("tyre_perimeter_bridge_enabled").value
                                )
                                if send_follow_waypoints(self, poses, self._on_follow_waypoints_done):
                                    self._dispatched_approach_this_wait = True
                                    if self._wait_context:
                                        self._wait_context.dispatch_retry_count = 0
                                    self.get_logger().info(
                                        f"FollowWaypoints batch (full list): sent {len(poses)} poses in map frame."
                                    )
                                    self._mission_log_append(
                                        "follow_waypoints_batch_dispatched",
                                        {
                                            "poses_count": len(poses),
                                            "vehicle_id": self._committed_vehicle_id,
                                            "mode": "use_batch_waypoints",
                                        },
                                        sync=True,
                                    )
                                    self._set_state(
                                        MissionState.FOLLOW_WAYPOINTS_BATCH,
                                        cause="batch_follow_waypoints",
                                    )
                                    return
                            self.get_logger().warn(
                                "use_batch_waypoints: empty waypoint list; falling back to sequential nav."
                            )
                        # Legacy batch: four standoff poses only (no perimeter polyline)
                        if (
                            use_fw
                            and not use_batch
                            and n_plan >= 4
                            and expected_tires >= 4
                        ):
                            poses = self._build_planned_tire_poses(
                                self._planned_tire_positions[:4]
                            )
                            if poses and send_follow_waypoints(
                                self, poses, self._on_follow_waypoints_done
                            ):
                                self._last_follow_waypoints_pose_count = len(poses)
                                self._last_batch_waypoints_perimeter = False
                                self._dispatched_approach_this_wait = True
                                if self._wait_context:
                                    self._wait_context.dispatch_retry_count = 0
                                self.get_logger().info(
                                    "FollowWaypoints batch: sent 4 tire standoff poses (waypoint task plugin captures at each)."
                                )
                                self._mission_log_append(
                                    "follow_waypoints_batch_dispatched",
                                    {"poses_count": len(poses), "vehicle_id": self._committed_vehicle_id},
                                    sync=True,
                                )
                                self._set_state(
                                    MissionState.FOLLOW_WAYPOINTS_BATCH,
                                    cause="follow_waypoints_batch",
                                )
                                return
                        target = self._planned_tire_positions.pop(0)
                        if self.current_vehicle_idx < len(self.detected_vehicles):
                            self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
                        self._last_tire_offset = self.get_parameter("tire_offset").value
                        # Register first tire so it is counted: inspected_tire_positions and _tire_registry
                        # (later tires are registered in _select_tire_in_wait_state; this path skips that)
                        self.inspected_tire_positions.append(target)
                        robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z) if robot_pose else None
                        tire_pos_label = tire_position_label(
                            target,
                            self._committed_vehicle_center,
                            robot_pos,
                        ) if self._committed_vehicle_center and robot_pos else "nearest"
                        self._tire_id_counter += 1
                        self._tire_registry.append({
                            "id": self._tire_id_counter,
                            "vehicle_id": self.current_vehicle_idx,
                            "position": target,
                            "tire_position": tire_pos_label or "nearest",
                            "visited": False,
                            "image_captured": False,
                            "goal_source": "planned",
                        })
                        self.get_logger().info(
                            f"Vehicle plan committed; dispatching to nearest corner (costmap = vehicle)."
                        )
                        self._mission_log_append(
                            "mission_plan_dispatch",
                            {
                                "phase": "approach_nearest_corner",
                                "vehicle_id": self._committed_vehicle_id,
                                "target_position": list(target),
                                "offset_m": self._last_tire_offset,
                                "remaining_corners": len(self._planned_tire_positions),
                            },
                            sync=True,
                        )
                        sent, used_refined = self._dispatch_planned_tire_goal(target)
                        if used_refined and self._tire_registry:
                            self._tire_registry[-1]["goal_source"] = "detection_refined"
                        if sent:
                            self._dispatched_approach_this_wait = True
                            if self._wait_context:
                                self._wait_context.dispatch_retry_count = 0
                            if self._is_dry_run():
                                self.get_logger().info("Dry run: simulating approach success, transitioning to WAIT_TIRE_BOX")
                                self._simulate_approach_success()
                            else:
                                self._set_state(MissionState.INSPECT_TIRE, cause=TransitionReason.APPROACH_DISPATCHED.value)
                            return
                        if self._last_dispatch_fail_reason == "goal_inside_vehicle_box":
                            self._planned_tire_positions.insert(0, target)
                    # Fallback to nearest-face approach if no planned tires
                    self.get_logger().warn("approach_nearest_corner: no planned tires; falling back to nearest-face approach.")

                self.get_logger().info(
                    f"Vehicle plan committed; dispatching approach to nearest face (exact goal from committed plan)."
                )
                self._mission_log_append(
                    "mission_plan_dispatch",
                    {
                        "phase": "approach_vehicle",
                        "vehicle_id": self._committed_vehicle_id,
                        "target_center": list(self._committed_vehicle_center) if self._committed_vehicle_center else None,
                        "offset_m": offset,
                        "committed": self._committed_vehicle_box is not None,
                    },
                    sync=True,
                )
                # Use committed plan: skip detection_stamp age check so we don't reject when stamp is >0.5s old (tick runs between vehicle_boxes messages).
                detection_stamp_for_goal = None if self._committed_vehicle_box is not None else self._last_vehicle_boxes_stamp
                sent = self._dispatch_box_goal(
                    approach_box,
                    offset=offset,
                    detection_stamp=detection_stamp_for_goal,
                )
                self._mission_log_append(
                    "approach_dispatch_attempt",
                    {
                        "sent": sent,
                        "reason": self._last_dispatch_fail_reason if not sent else None,
                        "state": self.current_state,
                    },
                    sync=True,
                )
                if sent:
                    self._dispatched_approach_this_wait = True
                    if self._wait_context:
                        self._wait_context.dispatch_retry_count = 0
                    if self._is_dry_run():
                        self.get_logger().info("Dry run: simulating approach success, transitioning to WAIT_TIRE_BOX")
                        self._simulate_approach_success()
                    else:
                        self._set_state(MissionState.APPROACH_VEHICLE, cause=TransitionReason.APPROACH_DISPATCHED.value)
                    return
                # Dispatch failed: retry or recover
                if self._wait_context:
                    self._wait_context.dispatch_retry_count += 1
                    max_retries = getattr(
                        self._wait_context, "max_dispatch_retries_before_recover", 5
                    )
                    if self._wait_context.dispatch_retry_count >= max_retries:
                        self.get_logger().warn(
                            f"Dispatch failed {self._wait_context.dispatch_retry_count} times; "
                            "triggering recovery rotation."
                        )
                        self._dispatch_rotation_goal(is_vehicle=True)
                        self._set_state(
                            MissionState.TURN_IN_PLACE_VEHICLE,
                            cause=TransitionReason.NAV_RETRY.value,
                        )
                        return
                self.get_logger().warn("Failed to send approach goal (pose or Nav2); will retry next tick.")
            # Check for timeout and trigger recovery if needed
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("detection_timeout").value
                if elapsed > timeout:
                    self._mission_log_append(
                        "wait_vehicle_timeout",
                        {
                            "elapsed_s": round(elapsed, 1),
                            "timeout_s": timeout,
                            "rotation_attempts": self.rotation_attempts,
                            "dispatched_approach_this_wait": self._dispatched_approach_this_wait,
                            "has_current_vehicle_box": self.current_vehicle_box is not None,
                        },
                        sync=True,
                    )
                    if self.get_parameter("turn_in_place_enabled").value and self.rotation_attempts < self.get_parameter("max_rotation_attempts").value:
                        self.get_logger().warn(
                            f"No vehicle detection after {elapsed:.1f}s. Attempting recovery rotation {self.rotation_attempts + 1}."
                        )
                        self._dispatch_rotation_goal(is_vehicle=True)
                    else:
                        self.get_logger().warn(
                            f"Max rotation attempts reached. Moving to next vehicle or completing mission."
                        )
                        self._set_state(MissionState.NEXT_VEHICLE, cause=TransitionReason.NEXT_VEHICLE.value)
            return  # waiting on detections

        if self.current_state == MissionState.TURN_IN_PLACE_VEHICLE:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.APPROACH_VEHICLE:
            # Delayed approach retry: send the next goal only after delay (avoids preemption storm; controller needs time)
            now_t = time.time()
            if self._approach_retry_pending and self._approach_retry_at is not None and now_t >= self._approach_retry_at:
                self._approach_retry_pending = False
                self._approach_retry_at = None
                budget = self.get_parameter("nav_retry_budget").value
                if self._nav_retry_count <= budget and self._last_approach_box is not None:
                    if (
                        self.get_parameter("reuse_approach_goal_on_retry").value
                        and self._current_goal_pose is not None
                    ):
                        if send_nav_goal(self, self._current_goal_pose, self._on_box_goal_done, feedback_cb=self._on_nav_feedback):
                            self.get_logger().info("Approach retry (after delay): sent reused goal.")
                            return
                    self._current_goal_pose = None
                    ds_retry = None if self._committed_vehicle_box is not None else self._last_vehicle_boxes_stamp
                    self._dispatch_box_goal(
                        self._last_approach_box,
                        offset=self._last_approach_offset,
                        detection_stamp=ds_retry,
                    )
                    self.get_logger().info("Approach retry (after delay): dispatched box goal.")
                return
            # Switch to best_fallback.pt (wheel detection) when within wheel_detection_switch_distance_m of vehicle edge.
            # Bounding box and costmap both represent the vehicle; goals use edges/corners, not center.
            if not self._wheel_switch_during_approach_done:
                dist_to_edge = self._distance_to_nearest_point_on_committed_box()
                switch_dist = float(self.get_parameter("wheel_detection_switch_distance_m").value)
                if switch_dist > 0 and dist_to_edge is not None and dist_to_edge <= switch_dist:
                    mode_msg = String()
                    mode_msg.data = "inspection"
                    self.segmentation_mode_pub.publish(mode_msg)
                    self._wheel_switch_during_approach_done = True
                    self.get_logger().info(
                        "Within %.2fm of vehicle edge; switched to wheel detection (best_fallback.pt).",
                        switch_dist,
                    )
            # Keep cache clean while driving; tire detections are updated asynchronously in _detection_cb.
            self._prune_tire_cache()
            # Timeout: if Nav2 is stuck, cancel and return to WAIT_VEHICLE_BOX so we can retry or rotate
            if self._approach_entered_time is not None:
                timeout_s = self.get_parameter("approach_timeout_s").value
                if (time.time() - self._approach_entered_time) > timeout_s:
                    self.get_logger().warn(
                        f"Approach timeout ({timeout_s}s); cancelling goal and returning to WAIT_VEHICLE_BOX."
                    )
                    self._mission_log_append(
                        "approach_timeout",
                        {"timeout_s": timeout_s, "state": self.current_state},
                        sync=True,
                    )
                    if self._active_nav_goal_handle is not None:
                        try:
                            self._active_nav_goal_handle.cancel_goal_async()
                        except Exception as e:
                            self.get_logger().warn(f"Cancel goal on timeout: {e}")
                        self._active_nav_goal_handle = None
                    self._approach_entered_time = None
                    self._approach_retry_pending = False
                    self._approach_retry_at = None
                    self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="approach_timeout")
            return  # waiting on navigation to complete

        if self.current_state == MissionState.WAIT_TIRE_BOX:
            # Single-tire mode and already at tire (no more planned tires): proceed to capture instead of waiting
            expected_tires = self.get_parameter("expected_tires_per_vehicle").value
            if expected_tires == 1 and not self._planned_tire_positions and len(self.inspected_tire_positions) >= 1:
                self.get_logger().info(
                    "Single-tire mode: at tire with no more planned tires; proceeding to WAIT_WHEEL_FOR_CAPTURE."
                )
                if self.get_parameter("capture_require_wheel_detection").value:
                    self._set_state(MissionState.WAIT_WHEEL_FOR_CAPTURE, cause="single_tire_at_goal")
                else:
                    self._trigger_tire_capture()
                return
            # Prefer fresh /tyre_3d_positions over planned 1×1 m vehicle corners (avoids goals inside inflated vehicle cells).
            if bool(self.get_parameter("use_tyre_3d_positions").value) and bool(
                self.get_parameter("prefer_tyre_3d_in_wait_tire_box").value
            ):
                if self._try_dispatch_tyre_3d_in_wait_tire_box():
                    return
            # Re-publish segmentation mode periodically so ultralytics_node receives "inspection"
            # even if it started after we entered WAIT_TIRE_BOX (avoids "Waiting for subscriber")
            now = time.time()
            if (now - self._last_segmentation_mode_publish_time) >= 3.0:
                mode_msg = String()
                mode_msg.data = "inspection"
                self.segmentation_mode_pub.publish(mode_msg)
                self._last_segmentation_mode_publish_time = now
                self.get_logger().debug("Published segmentation mode: inspection (periodic)")
            # Strict planned order: always take next tire from pop(0) (2nd→3rd→4th nearest). No detection reordering.
            if (
                bool(self.get_parameter("strict_planned_tire_order").value)
                and self._planned_tire_positions
                and bool(self.get_parameter("planned_tire_fallback_enabled").value)
            ):
                target = self._planned_tire_positions.pop(0)
                if self.current_vehicle_idx < len(self.detected_vehicles):
                    self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
                self.inspected_tire_positions.append(target)
                robot_pose = self._get_current_pose()
                robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z) if robot_pose else None
                tire_pos_label = (
                    tire_position_label(target, self._committed_vehicle_center, robot_pos)
                    if self._committed_vehicle_center and robot_pos else "nearest"
                )
                self._tire_id_counter += 1
                self._tire_registry.append({
                    "id": self._tire_id_counter,
                    "vehicle_id": self.current_vehicle_idx,
                    "position": target,
                    "tire_position": tire_pos_label or "nearest",
                    "visited": False,
                    "image_captured": False,
                    "goal_source": "planned",
                })
                self.get_logger().info(
                    f"Strict planned order: next tire = 2nd/3rd/4th nearest; dispatching to {list(target)[:2]}."
                )
                self._mission_log_append(
                    "mission_plan_dispatch",
                    {
                        "phase": "tire_strict_planned_order",
                        "target_position": list(target),
                        "offset_m": self._last_tire_offset,
                        "remaining_planned": len(self._planned_tire_positions),
                    },
                    sync=True,
                )
                sent, used_refined = self._dispatch_planned_tire_goal(target)
                if used_refined and self._tire_registry:
                    self._tire_registry[-1]["goal_source"] = "detection_refined"
                if sent:
                    self._set_state(MissionState.INSPECT_TIRE, cause=TransitionReason.PLANNED_TIRE_FALLBACK.value)
                    return
                if self._last_dispatch_fail_reason == "goal_inside_vehicle_box":
                    self._planned_tire_positions.insert(0, target)
                    if self.current_vehicle_idx < len(self.detected_vehicles):
                        self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
                self.inspected_tire_positions.pop()
                self._tire_registry.pop()
                self._tire_id_counter -= 1
            # Cache-first tire selection: if live stream flickers, keep progress using recent tire candidates.
            # (Skipped when strict_planned_tire_order is True; tick uses pop(0) only.)
            if not bool(self.get_parameter("strict_planned_tire_order").value) and bool(self.get_parameter("prefer_cached_tires").value):
                tire_label = self.get_parameter("tire_label").value
                min_tire_prob = self.get_parameter("min_tire_probability").value
                cached_boxes = self._cached_tire_boxes(tire_label, min_tire_prob)
                if cached_boxes and self._select_tire_in_wait_state(cached_boxes, source="cache_tick"):
                    return
            # Check for timeout and trigger recovery if needed
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("tire_search_timeout_s").value
                if elapsed > timeout:
                    if self.get_parameter("turn_in_place_enabled").value and self.rotation_attempts < self.get_parameter("max_rotation_attempts").value:
                        self.get_logger().warn(
                            f"No un-inspected tire detection after {elapsed:.1f}s. "
                            f"Attempting recovery rotation {self.rotation_attempts + 1}. "
                            f"Already inspected {len(self.inspected_tire_positions)} tires."
                        )
                        self._dispatch_rotation_goal(is_vehicle=False)
                    else:
                        # Planned tire fallback before moving on
                        if (
                            self.get_parameter("planned_tire_fallback_enabled").value
                            and self._planned_tire_positions
                        ):
                            target = self._planned_tire_positions.pop(0)
                            if self.current_vehicle_idx < len(self.detected_vehicles):
                                self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
                            # Register tire so it is counted (planned fallback skips _select_tire_in_wait_state)
                            self.inspected_tire_positions.append(target)
                            robot_pose = self._get_current_pose()
                            robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z) if robot_pose else None
                            tire_pos_label = (
                                tire_position_label(target, self._committed_vehicle_center, robot_pos)
                                if self._committed_vehicle_center and robot_pos else "nearest"
                            )
                            self._tire_id_counter += 1
                            self._tire_registry.append({
                                "id": self._tire_id_counter,
                                "vehicle_id": self.current_vehicle_idx,
                                "position": target,
                                "tire_position": tire_pos_label or "nearest",
                                "visited": False,
                                "image_captured": False,
                                "goal_source": "planned",
                            })
                            self.get_logger().warn(
                                f"No tire boxes after {elapsed:.1f}s. Using planned tire fallback: {target}."
                            )
                            self._mission_log_append(
                                "planned_tire_fallback_used",
                                {
                                    "reason": "tire_search_timeout",
                                    "elapsed_s": round(elapsed, 1),
                                    "timeout_s": timeout,
                                    "target": list(target),
                                    "remaining_planned": len(self._planned_tire_positions),
                                },
                                sync=True,
                            )
                            self._mission_log_append(
                                "mission_plan_dispatch",
                                {
                                    "phase": "tire_planned_fallback",
                                    "target_position": list(target),
                                    "offset_m": self._last_tire_offset,
                                    "committed": True,
                                },
                                sync=True,
                            )
                            sent, used_refined = self._dispatch_planned_tire_goal(target)
                            if used_refined and self._tire_registry:
                                self._tire_registry[-1]["goal_source"] = "detection_refined"
                            if sent:
                                self._set_state(MissionState.INSPECT_TIRE, cause=TransitionReason.PLANNED_TIRE_FALLBACK.value)
                                return
                            if self._last_dispatch_fail_reason == "goal_inside_vehicle_box":
                                self._planned_tire_positions.append(target)
                        max_reacquire = int(self.get_parameter("max_vehicle_reacquire_attempts").value)
                        if (
                            self.current_vehicle_idx < len(self.detected_vehicles)
                            and self._vehicle_reacquire_attempts < max_reacquire
                        ):
                            self._vehicle_reacquire_attempts += 1
                            self.get_logger().warn(
                                f"Tire detection stale; reacquiring vehicle context from anchor "
                                f"(attempt {self._vehicle_reacquire_attempts}/{max_reacquire})."
                            )
                            self._mission_log_append(
                                "reacquire_from_anchor",
                                {
                                    "vehicle_idx": self.current_vehicle_idx,
                                    "attempt": self._vehicle_reacquire_attempts,
                                    "max_attempts": max_reacquire,
                                },
                                sync=True,
                            )
                            self._dispatched_approach_this_wait = False  # allow re-dispatch of approach with committed plan
                            self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="reacquire_from_anchor")
                            return
                        # Check if we have enough tires inspected (typically 4 per vehicle)
                        inspected_count = len(self.inspected_tire_positions)
                        if self._should_retry_deferred_tires():
                            self._requeue_deferred_tires()
                            self.get_logger().info(
                                f"Return later: retrying deferred tires before moving to next vehicle."
                            )
                            return
                        self.get_logger().warn(
                            f"Max rotation attempts reached. Inspected {inspected_count} tires. "
                            f"Moving to next vehicle."
                        )
                        self._set_state(MissionState.NEXT_VEHICLE, cause=TransitionReason.NEXT_VEHICLE.value)
            return

        if self.current_state == MissionState.TURN_IN_PLACE_TIRE:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.INSPECT_TIRE:
            # Timeout: if Nav2 is stuck driving to tire, cancel and return to WAIT_TIRE_BOX
            if self._approach_entered_time is not None:
                timeout_s = self.get_parameter("approach_timeout_s").value
                if (time.time() - self._approach_entered_time) > timeout_s:
                    self.get_logger().warn(
                        f"Tire approach timeout ({timeout_s}s); cancelling goal and returning to WAIT_TIRE_BOX."
                    )
                    self._mission_log_append(
                        "tire_approach_timeout",
                        {"timeout_s": timeout_s, "state": self.current_state},
                        sync=True,
                    )
                    if self._active_nav_goal_handle is not None:
                        try:
                            self._active_nav_goal_handle.cancel_goal_async()
                        except Exception as e:
                            self.get_logger().warn(f"Cancel goal on timeout: {e}")
                        self._active_nav_goal_handle = None
                    self._approach_entered_time = None
                    self._set_state(MissionState.WAIT_TIRE_BOX, cause="tire_approach_timeout")
            return  # waiting on navigation and photo capture

        if self.current_state == MissionState.FACE_TIRE:
            if self._face_tire_start_time is not None:
                timeout_s = float(self.get_parameter("face_tire_timeout_s").value)
                if (time.time() - self._face_tire_start_time) > timeout_s:
                    self.get_logger().warn(
                        f"Face tire timeout ({timeout_s}s); cancelling rotation and returning to WAIT_TIRE_BOX."
                    )
                    self._mission_log_append(
                        "face_tire_timeout",
                        {"timeout_s": timeout_s, "state": self.current_state},
                        sync=True,
                    )
                    if self._active_nav_goal_handle is not None:
                        try:
                            self._active_nav_goal_handle.cancel_goal_async()
                        except Exception as e:
                            self.get_logger().warn(f"Cancel face_tire goal on timeout: {e}")
                        self._active_nav_goal_handle = None
                    self._face_tire_start_time = None
                    self._face_tire_target_yaw = None
                    self._set_state(MissionState.WAIT_TIRE_BOX, cause="face_tire_timeout")
            return

        if self.current_state == MissionState.WAIT_WHEEL_FOR_CAPTURE:
            if not self._wheel_capture_inspection_published:
                mode_msg = String()
                mode_msg.data = "inspection"
                self.segmentation_mode_pub.publish(mode_msg)
                self._wheel_capture_inspection_published = True
                self.get_logger().info("WAIT_WHEEL_FOR_CAPTURE: switched to inspection (wheel) model; waiting for wheel in view.")
            required = int(self.get_parameter("capture_wheel_required_frames").value)
            timeout_s = float(self.get_parameter("capture_wheel_wait_timeout_s").value)
            elapsed = time.time() - self._wheel_wait_start_time if self._wheel_wait_start_time else 0.0
            if self._wheel_confirm_count >= required:
                self.get_logger().info(f"Wheel in view ({self._wheel_confirm_count} frames); triggering photo capture.")
                self._trigger_tire_capture()
                return
            if elapsed >= timeout_s:
                if self.get_parameter("capture_on_wheel_timeout").value:
                    self.get_logger().info(
                        f"No wheel confirmed within {timeout_s}s (count={self._wheel_confirm_count}); capturing anyway and proceeding."
                    )
                    self._mission_log_append(
                        "capture_on_wheel_timeout",
                        {"timeout_s": timeout_s, "elapsed_s": elapsed, "wheel_confirm_count": self._wheel_confirm_count},
                        sync=True,
                    )
                    self._trigger_tire_capture()
                else:
                    self.get_logger().warn(
                        f"No wheel detected within {timeout_s}s at capture position; skipping this tire (count={self._wheel_confirm_count})"
                    )
                    self._mission_log_append(
                        "capture_skipped_no_wheel",
                        {"timeout_s": timeout_s, "elapsed_s": elapsed, "wheel_confirm_count": self._wheel_confirm_count},
                        sync=True,
                    )
                    self._set_state(MissionState.WAIT_TIRE_BOX, cause="capture_skipped_no_wheel")
            return

        if self.current_state == MissionState.VERIFY_CAPTURE:
            timeout_s = self.get_parameter("capture_verify_timeout_s").value
            if self._verify_capture_start_time is not None and (time.time() - self._verify_capture_start_time) > timeout_s:
                self.get_logger().warn(f"No capture_result within {timeout_s}s; treating as failure")
                self._finish_verify_capture_failure("timeout")
            return

        if self.current_state == MissionState.NEXT_VEHICLE:
            # Mark current vehicle as inspected
            if self.current_vehicle_idx < len(self.detected_vehicles):
                self.detected_vehicles[self.current_vehicle_idx]["inspected"] = True
            
            # Find next un-inspected vehicle
            un_inspected = [v for v in self.detected_vehicles if not v["inspected"]]
            if un_inspected:
                self._clear_committed_vehicle()
                self.current_vehicle_idx = self.detected_vehicles.index(un_inspected[0])
                self.current_vehicle_box = un_inspected[0]["box"]
                self.current_tire_idx = 0
                self.inspected_tire_positions = []  # Reset for next vehicle
                self._tire_registry = []  # Reset so current tire is always _tire_registry[-1]
                self._vehicle_reacquire_attempts = 0
                self._commit_current_vehicle()  # lock plan for this vehicle; all goals use committed state
                self.rotation_attempts = 0  # Reset rotation attempts
                self.get_logger().info(
                    f"Moving to vehicle {self.current_vehicle_idx + 1}/{len(self.detected_vehicles)} "
                    f"({un_inspected[0]['class']}); plan committed."
                )
                self._set_state(MissionState.WAIT_VEHICLE_BOX)
            else:
                self.get_logger().info("All vehicles inspected. Mission complete.")
                self._mission_report["total_vehicles"] = len(self.detected_vehicles)
                self._mission_report["total_tires_expected"] = (
                    len(self.detected_vehicles) * self.get_parameter("expected_tires_per_vehicle").value
                )
                self._publish_mission_report()
                self._set_state(MissionState.DONE, cause="all_vehicles_inspected")
            return

    # ----------------------- Detections ----------------------- #
    def _vehicle_boxes_cb(self, msg: BoundingBoxes3d):
        """Vehicle boxes callback (semantic or YOLO vehicle stream)."""
        now = time.time()
        try:
            self._last_vehicle_boxes_stamp = rclpy.time.Time(
                seconds=msg.header.stamp.sec,
                nanoseconds=msg.header.stamp.nanosec,
            )
        except Exception:
            self._last_vehicle_boxes_stamp = None
        if self._last_vehicle_boxes_time is not None:
            interval = now - self._last_vehicle_boxes_time
            if interval >= 0:
                if self._vehicle_boxes_interval_ema is None:
                    self._vehicle_boxes_interval_ema = interval
                else:
                    self._vehicle_boxes_interval_ema = (0.2 * interval) + (0.8 * self._vehicle_boxes_interval_ema)
        self._last_vehicle_boxes_time = now
        n = len(msg.bounding_boxes)
        if n > 0:
            self.get_logger().info(
                f"Received {n} vehicle box(es): {[b.object_name for b in msg.bounding_boxes]}",
                throttle_duration_sec=2.0,
            )
            vehicle_labels = parse_vehicle_labels(self.get_parameter("vehicle_labels").value)
            min_vehicle_prob = self.get_parameter("min_vehicle_probability").value
            self._update_vehicle_cache(msg.bounding_boxes, vehicle_labels, min_vehicle_prob)
        self._process_vehicle_boxes(msg.bounding_boxes)

    @staticmethod
    def _box_center(box: BoundingBox3d) -> tuple:
        return (
            (box.xmin + box.xmax) / 2.0,
            (box.ymin + box.ymax) / 2.0,
            (box.zmin + box.zmax) / 2.0,
        )

    def _update_vehicle_cache(self, boxes: list, vehicle_labels: List[str], min_vehicle_prob: float) -> None:
        vehicle_box = find_vehicle_box(boxes, vehicle_labels, min_vehicle_prob)
        if vehicle_box is None:
            return
        now = time.time()
        confidence = float(vehicle_box.probability)
        if self._vehicle_cache:
            confidence = (0.2 * confidence) + (0.8 * float(self._vehicle_cache.get("confidence_ema", confidence)))
        self._vehicle_cache = {
            "box": vehicle_box,
            "center": self._box_center(vehicle_box),
            "last_seen": now,
            "confidence_ema": confidence,
        }

    def _update_tire_cache(self, boxes: list, tire_label: str, min_tire_prob: float) -> None:
        now = time.time()
        match_tol = float(self.get_parameter("tire_cache_match_tolerance_m").value)
        for box in boxes:
            if box.object_name.lower() != tire_label.lower() or box.probability < min_tire_prob:
                continue
            center = self._box_center(box)
            matched_entry = None
            for entry in self._tire_cache:
                cx, cy, cz = entry["center"]
                dist = math.sqrt((center[0] - cx) ** 2 + (center[1] - cy) ** 2 + (center[2] - cz) ** 2)
                if dist <= match_tol:
                    matched_entry = entry
                    break
            if matched_entry is None:
                self._tire_cache.append(
                    {
                        "box": box,
                        "center": center,
                        "last_seen": now,
                        "confidence_ema": float(box.probability),
                    }
                )
            else:
                matched_entry["box"] = box
                matched_entry["center"] = center
                matched_entry["last_seen"] = now
                matched_entry["confidence_ema"] = (0.2 * float(box.probability)) + (
                    0.8 * float(matched_entry.get("confidence_ema", float(box.probability)))
                )
        self._prune_tire_cache()

    def _prune_tire_cache(self) -> None:
        stale_s = float(self.get_parameter("tire_cache_stale_s").value)
        now = time.time()
        self._tire_cache = [e for e in self._tire_cache if (now - float(e.get("last_seen", 0.0))) <= stale_s]

    def _cached_tire_boxes(self, tire_label: str, min_tire_prob: float) -> List[BoundingBox3d]:
        self._prune_tire_cache()
        result: List[BoundingBox3d] = []
        stale_s = float(self.get_parameter("tire_cache_stale_s").value)
        now = time.time()
        for entry in self._tire_cache:
            box = entry["box"]
            if box.object_name.lower() != tire_label.lower():
                continue
            if float(entry.get("confidence_ema", box.probability)) < min_tire_prob:
                continue
            if (now - float(entry.get("last_seen", 0.0))) > stale_s:
                continue
            result.append(box)
        return result

    def _select_tire_in_wait_state(self, candidate_boxes: List[BoundingBox3d], source: str) -> bool:
        if self.current_state != MissionState.WAIT_TIRE_BOX or not candidate_boxes:
            return False
        robot_pose = self._get_current_pose()
        robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z) if robot_pose else None
        vehicle_pos = None
        if self.current_vehicle_box:
            vehicle_pos = self._box_center(self.current_vehicle_box)
        elif self.current_vehicle_idx < len(self.detected_vehicles):
            vehicle_pos = self.detected_vehicles[self.current_vehicle_idx]["position"]
        elif self._vehicle_cache:
            cache_age = time.time() - float(self._vehicle_cache.get("last_seen", 0.0))
            if cache_age <= float(self.get_parameter("vehicle_cache_stale_s").value):
                vehicle_pos = self._vehicle_cache.get("center")
            else:
                self._vehicle_cache = {}
        if vehicle_pos is None and self.current_vehicle_idx < len(self.detected_vehicles):
            anchor = self.detected_vehicles[self.current_vehicle_idx].get("anchor_pose")
            if anchor:
                vehicle_pos = (anchor.get("x", 0.0), anchor.get("y", 0.0), anchor.get("z", 0.0))
        target_pos = self._planned_tire_positions[0] if self._planned_tire_positions else None
        tire_box = find_tire_for_inspection(
            candidate_boxes,
            self.get_parameter("tire_label").value,
            self.get_parameter("min_tire_probability").value,
            self.inspected_tire_positions,
            self.get_parameter("tire_position_tolerance").value,
            vehicle_pos,
            self.get_parameter("max_tire_distance_from_vehicle").value,
            robot_pos,
            target_pos,
            self.get_logger(),
        )
        if tire_box is None:
            return False
        tire_num = len(self.inspected_tire_positions) + 1
        log_bounding_box(self.get_logger(), tire_box, f"TIRE_{tire_num}")
        tire_center = self._box_center(tire_box)
        if not self._confirm_tire_box(tire_center):
            self._mission_log_append(
                "tire_confirmation_pending",
                {"state": self.current_state, "required": self.get_parameter("tire_confirmations_required").value},
                sync=True,
            )
            return False
        # Hardened: drive to committed tire position only (live detection only picks which slot)
        vehicle_center = self._box_center(self.current_vehicle_box) if self.current_vehicle_box else (
            self.detected_vehicles[self.current_vehicle_idx]["position"] if self.current_vehicle_idx < len(self.detected_vehicles) else None
        )
        tire_pos_label = tire_position_label(tire_center, vehicle_center, robot_pos)
        committed_pos = None
        if self._planned_tire_positions:
            tire_xy = (tire_center[0], tire_center[1])
            closest_idx = min(
                range(len(self._planned_tire_positions)),
                key=lambda i: (self._planned_tire_positions[i][0] - tire_xy[0]) ** 2
                + (self._planned_tire_positions[i][1] - tire_xy[1]) ** 2,
            )
            committed_pos = self._planned_tire_positions.pop(closest_idx)
            if self.current_vehicle_idx < len(self.detected_vehicles):
                self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
        self.get_logger().info(
            f"Tire {tire_num} selected from {source} at ({tire_center[0]:.2f}, {tire_center[1]:.2f}); "
            f"driving to committed position {committed_pos if committed_pos else tire_center}"
        )
        self._last_detection_confidence = tire_box.probability
        self.inspected_tire_positions.append(committed_pos if committed_pos else tire_center)
        self._tire_id_counter += 1
        self._tire_registry.append({
            "id": self._tire_id_counter,
            "vehicle_id": self.current_vehicle_idx,
            "position": committed_pos if committed_pos else tire_center,
            "tire_position": tire_pos_label,
            "visited": False,
            "image_captured": False,
            "goal_source": "detection",
        })
        to = float(self.get_parameter("tire_offset").value)
        staging = float(self.get_parameter("tire_staging_min_m").value)
        self._last_tire_offset = max(to, staging) if staging > 0 else to
        # _last_tire_box for face_tire yaw: use synthetic box at committed position so heading is deterministic
        extent = float(self.get_parameter("planned_tire_box_extent_m").value)
        if committed_pos:
            self._last_tire_box = BoundingBox3d()
            self._last_tire_box.object_name = self.get_parameter("tire_label").value
            self._last_tire_box.probability = tire_box.probability
            self._last_tire_box.xmin = committed_pos[0] - extent
            self._last_tire_box.xmax = committed_pos[0] + extent
            self._last_tire_box.ymin = committed_pos[1] - extent
            self._last_tire_box.ymax = committed_pos[1] + extent
            self._last_tire_box.zmin = committed_pos[2] - extent
            self._last_tire_box.zmax = committed_pos[2] + extent
        else:
            self._last_tire_box = tire_box
        self._target_lifecycle = TargetLifecycle.CONFIRMED.value
        self._mission_log_append(
            "tire_approach",
            {
                "source": source,
                "tire_id": self._tire_id_counter,
                "vehicle_id": self.current_vehicle_idx,
                "position": list(committed_pos if committed_pos else tire_center),
                "tire_position": tire_pos_label,
                "committed": committed_pos is not None,
            },
            sync=True,
        )
        self._mission_log_append(
            "mission_plan_dispatch",
            {
                "phase": "tire",
                "tire_id": self._tire_id_counter,
                "target_position": list(committed_pos if committed_pos else tire_center),
                "offset_m": self._last_tire_offset,
                "committed": committed_pos is not None,
            },
            sync=True,
        )
        if committed_pos is not None:
            sent, used_refined = self._dispatch_planned_tire_goal(committed_pos)
            if used_refined and self._tire_registry:
                self._tire_registry[-1]["goal_source"] = "detection_refined"
            if not sent and self._last_dispatch_fail_reason == "goal_inside_vehicle_box":
                self._planned_tire_positions.append(committed_pos)
        else:
            sent = self._dispatch_box_goal(
                tire_box,
                offset=self._last_tire_offset,
                detection_stamp=self._last_detection_stamp,
                vehicle_box=self._committed_vehicle_box,
            )
        if sent:
            self._current_tire_approach_start_time = time.time()
            self._set_state(MissionState.INSPECT_TIRE, cause=f"tire_detected_{source}")
        return sent

    def _process_vehicle_boxes(self, boxes: list):
        """Save vehicle positions and handle WAIT_VEHICLE_BOX dispatch. Used by detection_topic (YOLO) or vehicle_boxes_topic (semantic)."""
        use_dynamic = self.get_parameter("use_dynamic_detection").value
        vehicle_labels = parse_vehicle_labels(self.get_parameter("vehicle_labels").value)
        min_vehicle_prob = self.get_parameter("min_vehicle_probability").value

        if use_dynamic:
            for box in boxes:
                if box.object_name.lower() in [label.lower() for label in vehicle_labels]:
                    if box.probability >= min_vehicle_prob and self._vehicle_box_in_range(box):
                        if self._confirm_vehicle_box(box):
                            self._save_vehicle_position(box)

        if self.current_state == MissionState.SEARCH_VEHICLE:
            return
        if not boxes:
            if self.current_state == MissionState.WAIT_VEHICLE_BOX and self._vehicle_cache:
                cache_age = time.time() - float(self._vehicle_cache.get("last_seen", 0.0))
                if cache_age <= float(self.get_parameter("vehicle_cache_stale_s").value):
                    cached_box = self._vehicle_cache.get("box")
                    if cached_box is not None and self._vehicle_box_in_range(cached_box):
                        self.current_vehicle_box = cached_box
                        self._last_approach_offset = self.get_parameter("approach_offset").value
                        # Use committed plan if we have it (exact goal, no jitter)
                        approach_box = self._committed_vehicle_box if self._committed_vehicle_box is not None else cached_box
                        self._last_approach_box = approach_box
                        self._mission_log_append(
                            "vehicle_cache_hit",
                            {"state": self.current_state, "cache_age_s": cache_age, "using_committed": self._committed_vehicle_box is not None},
                            sync=True,
                        )
                        # Committed plan: skip stamp-age/TF-at-stamp check so dispatch succeeds
                        ds = None if self._committed_vehicle_box is not None else self._last_vehicle_boxes_stamp
                        if self._dispatch_box_goal(
                            approach_box,
                            offset=self._last_approach_offset,
                            detection_stamp=ds,
                        ):
                            self._dispatched_approach_this_wait = True
                            self._set_state(MissionState.APPROACH_VEHICLE, cause="vehicle_cache")
                            return
            if self.current_state == MissionState.WAIT_VEHICLE_BOX:
                self.get_logger().error(
                    "DETECTION_MISSING: no vehicle boxes received while in WAIT_VEHICLE_BOX"
                )
                self._mission_log_append(
                    "detection_missing",
                    {
                        "state": self.current_state,
                        "topic": self.get_parameter("vehicle_boxes_topic").value,
                        "labels": vehicle_labels,
                    },
                    sync=True,
                )
            return
        if self.current_state == MissionState.WAIT_VEHICLE_BOX:
            target_pos = None
            if self.current_vehicle_idx < len(self.detected_vehicles):
                target_pos = self.detected_vehicles[self.current_vehicle_idx].get("position")
            vehicle_box = find_vehicle_box(boxes, vehicle_labels, min_vehicle_prob, target_position=target_pos)
            if vehicle_box:
                if not self._vehicle_box_in_range(vehicle_box):
                    return
                if not self._confirm_vehicle_box(vehicle_box):
                    self._mission_log_append(
                        "vehicle_confirmation_pending",
                        {"state": self.current_state, "required": self.get_parameter("vehicle_confirmations_required").value},
                        sync=True,
                    )
                    return
                log_bounding_box(self.get_logger(), vehicle_box, f"VEHICLE_{vehicle_box.object_name.upper()}")
                self.current_vehicle_box = vehicle_box
                self._target_lifecycle = TargetLifecycle.CONFIRMED.value
                if use_dynamic:
                    self._save_vehicle_position(vehicle_box)
                self._last_detection_confidence = vehicle_box.probability
                if self._committed_vehicle_box is None:
                    self._commit_current_vehicle()
                # When approach_nearest_corner is True, dispatch must happen in tick() so that we sort
                # tires by distance and go to the nearest tire first. If we dispatch here we would
                # send the robot to the vehicle face (nearest face), not the nearest tire, and
                # _planned_tire_positions would never be sorted/popped, breaking the tire order.
                if bool(self.get_parameter("approach_nearest_corner").value):
                    self.get_logger().info(
                        f"Vehicle confirmed in callback (approach_nearest_corner=True); dispatch in tick to nearest tire."
                    )
                    return
                approach_box = self._committed_vehicle_box if self._committed_vehicle_box is not None else vehicle_box
                self._last_approach_box = approach_box
                self._last_approach_offset = self.get_parameter("approach_offset").value
                self.get_logger().info(
                    f"Vehicle detected ({vehicle_box.object_name}, prob: {vehicle_box.probability:.3f}); approaching (committed plan)."
                )
                self._mission_log_append(
                    "mission_plan_dispatch",
                    {
                        "phase": "approach_vehicle",
                        "vehicle_id": self._committed_vehicle_id,
                        "target_center": list(self._committed_vehicle_center) if self._committed_vehicle_center else None,
                        "offset_m": self._last_approach_offset,
                        "committed": self._committed_vehicle_box is not None,
                    },
                    sync=True,
                )
                # Committed plan: skip stamp-age/TF-at-detection-time check so dispatch succeeds
                ds_cb = None if self._committed_vehicle_box is not None else self._last_vehicle_boxes_stamp
                sent = self._dispatch_box_goal(
                    approach_box,
                    offset=self._last_approach_offset,
                    detection_stamp=ds_cb,
                )
                if not sent:
                    self.get_logger().warn("Vehicle callback: dispatch failed (TF/Nav2); will retry on next tick or callback.")
                    return
                self._dispatched_approach_this_wait = True
                if self._is_dry_run():
                    # Simulate nav arrival so state machine progresses to WAIT_TIRE_BOX (same as full launch)
                    self.get_logger().info("Dry run: simulating approach success, transitioning to WAIT_TIRE_BOX")
                    self._simulate_approach_success()
                else:
                    self._set_state(MissionState.APPROACH_VEHICLE, cause="vehicle_detected")
            else:
                self.get_logger().error(
                    f"DETECTION_FILTERED: boxes received but none passed filters "
                    f"(labels={vehicle_labels}, min_prob={min_vehicle_prob:.2f})"
                )
                self._mission_log_append(
                    "detection_filtered",
                    {
                        "state": self.current_state,
                        "labels": vehicle_labels,
                        "min_vehicle_probability": min_vehicle_prob,
                    },
                    sync=True,
                )
            return

    def _odometry_filtered_cb(self, msg: Odometry):
        """Gate goal dispatch when pose covariance exceeds max_pose_covariance_xy (x,y variance)."""
        threshold = float(self.get_parameter("max_pose_covariance_xy").value)
        if threshold <= 0 or len(msg.pose.covariance) < 8:
            self._pose_covariance_ok = True
            return
        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        self._pose_covariance_ok = var_x <= threshold and var_y <= threshold

    def _detection_cb(self, msg: BoundingBoxes3d):
        now = time.time()
        try:
            self._last_detection_stamp = rclpy.time.Time(
                seconds=msg.header.stamp.sec,
                nanoseconds=msg.header.stamp.nanosec,
            )
        except Exception:
            self._last_detection_stamp = None
        if self._last_detection_msg_time is not None:
            interval = now - self._last_detection_msg_time
            if interval >= 0:
                if self._detection_interval_ema is None:
                    self._detection_interval_ema = interval
                else:
                    self._detection_interval_ema = (0.2 * interval) + (0.8 * self._detection_interval_ema)
        self._last_detection_msg_time = now
        self._last_tire_boxes = list(msg.bounding_boxes) if msg.bounding_boxes else []
        tire_label = self.get_parameter("tire_label").value
        min_tire_prob = self.get_parameter("min_tire_probability").value
        self._update_tire_cache(msg.bounding_boxes, tire_label, min_tire_prob)

        if self.current_state in (MissionState.SEARCH_VEHICLE, MissionState.WAIT_VEHICLE_BOX, MissionState.APPROACH_VEHICLE):
            return

        if self.current_state == MissionState.WAIT_TIRE_BOX:
            # When strict_planned_tire_order is True, tick() uses pop(0) only; do not let detection reorder.
            if not bool(self.get_parameter("strict_planned_tire_order").value):
                if self._select_tire_in_wait_state(msg.bounding_boxes, source="live"):
                    return
                if bool(self.get_parameter("prefer_cached_tires").value):
                    cached = self._cached_tire_boxes(tire_label, min_tire_prob)
                    if cached and self._select_tire_in_wait_state(cached, source="cache"):
                        self._last_tire_cache_log_time = now
                        return
            return

        if self.current_state == MissionState.WAIT_WHEEL_FOR_CAPTURE:
            required = int(self.get_parameter("capture_wheel_required_frames").value)
            wheel_boxes = [b for b in msg.bounding_boxes if b.object_name.lower() == tire_label.lower() and b.probability >= min_tire_prob]
            if wheel_boxes:
                self._wheel_confirm_count = min(self._wheel_confirm_count + 1, required)

    def _find_box(self, boxes: List[BoundingBox3d], label: str, index: Optional[int] = None) -> Optional[BoundingBox3d]:
        """Find a box matching the label (legacy method, kept for compatibility)."""
        filtered = [b for b in boxes if b.object_name.lower() == label.lower()]
        if not filtered:
            return None
        if index is not None and index < len(filtered):
            return filtered[index]
        return sorted(filtered, key=lambda b: b.probability, reverse=True)[0]

    # ----------------------- Goals ----------------------- #
    def _dispatch_standoff_goal(self):
        """Dispatch standoff goal (legacy mode only - uses YAML file)."""
        trucks = self._load_trucks()
        if self.current_vehicle_idx >= len(trucks):
            return
        truck = trucks[self.current_vehicle_idx]
        standoff = self.get_parameter("standoff_distance").value
        yaw = truck.get("yaw", 0.0)
        goal_pose = self._pose_from_truck(truck, standoff, yaw)
        self.get_logger().info(
            f"Vehicle {self.current_vehicle_idx+1}/{len(trucks)}: navigating to standoff (legacy mode)."
        )
        send_nav_goal(self, goal_pose, self._on_standoff_done, feedback_cb=self._on_nav_feedback)
        # Note: Legacy mode uses old state names - this is for backward compatibility only

    def _dispatch_box_goal(
        self, box: BoundingBox3d, offset: float, detection_stamp=None, vehicle_box=None
    ) -> bool:
        """
        Dispatch navigation goal to approach detected object.
        CRITICAL: Bounding box coordinates are in slamware_map frame (from segmentation_processor).
        Robot pose must be looked up in the SAME frame.
        detection_stamp: optional rclpy.time.Time from message header; used for TF validity and staleness checks.
        vehicle_box: optional; when dispatching tire goals, pass committed vehicle box for far-side placement.
        Returns True if the goal was sent to Nav2, False otherwise (caller can retry).
        """
        if (
            self.get_parameter("require_nav_permitted").value
            and self._nav_permitted is not True
        ):
            self._last_dispatch_fail_reason = "nav_permitted_blocked"
            self._mission_log_append(
                "nav_gate_blocked_dispatch",
                {
                    "state": self.current_state,
                    "nav_permitted": self._nav_permitted,
                    "last_nav_permitted_time": self._last_nav_permitted_time,
                    "topic": self.get_parameter("nav_permitted_topic").value,
                },
                sync=True,
            )
            return False
        if (
            self.get_parameter("max_pose_covariance_xy").value > 0
            and not self._pose_covariance_ok
        ):
            self._last_dispatch_fail_reason = "pose_covariance_high"
            self._mission_log_append(
                "pose_covariance_blocked_dispatch",
                {
                    "state": self.current_state,
                    "odometry_filtered_topic": self.get_parameter("odometry_filtered_topic").value,
                    "max_pose_covariance_xy": self.get_parameter("max_pose_covariance_xy").value,
                },
                sync=True,
            )
            return False
        goal_info = compute_box_goal(
            self, box, offset, detection_stamp=detection_stamp, vehicle_box=vehicle_box
        )
        if goal_info is None:
            world_frame = self.get_parameter("world_frame").value
            self.get_logger().error(
                f"Cannot get robot pose in {world_frame} frame! Cannot compute goal."
            )
            self._last_dispatch_fail_reason = DispatchFailureCode.ROBOT_POSE_UNAVAILABLE.value
            self._dispatch_fail_count += 1
            return False
        if goal_info.get("failure_code") is not None:
            failure_code = str(goal_info.get("failure_code"))
            self._last_dispatch_fail_reason = failure_code
            self._dispatch_fail_count += 1
            self.get_logger().warn(
                f"Goal generation failed with failure_code={failure_code}"
            )
            self._mission_log_append(
                "goal_generation_failed",
                {
                    "failure_code": failure_code,
                    "state": self.current_state,
                    "detection_age_s": goal_info.get("detection_age_s"),
                },
                sync=True,
            )
            return False

        goal = goal_info["goal"]
        if not (
            math.isfinite(goal.pose.position.x)
            and math.isfinite(goal.pose.position.y)
            and math.isfinite(goal.pose.position.z)
        ):
            self.get_logger().error(
                "Goal position has non-finite value; rejecting dispatch."
            )
            self._mission_log_append(
                "goal_generation_failed",
                {
                    "failure_code": "goal_position_not_finite",
                    "state": self.current_state,
                    "x": goal.pose.position.x,
                    "y": goal.pose.position.y,
                    "z": goal.pose.position.z,
                },
                sync=True,
            )
            self._last_dispatch_fail_reason = "goal_position_not_finite"
            self._dispatch_fail_count += 1
            return False

        # Reject goal if it lies inside the committed vehicle box (would require driving through the car).
        goal_world = goal_info["goal_world"]
        if self._committed_vehicle_box is not None:
            gx, gy = goal_world[0], goal_world[1]
            vb = self._committed_vehicle_box
            if vb.xmin <= gx <= vb.xmax and vb.ymin <= gy <= vb.ymax:
                self._last_dispatch_fail_reason = "goal_inside_vehicle_box"
                # Do NOT increment _dispatch_fail_count: geometric rejection, not system failure.
                # Caller will re-queue tire or skip; mission continues (Nav2 Waypoint Follower stop_on_failure: false).
                self.get_logger().warn(
                    f"Goal ({gx:.2f}, {gy:.2f}) inside vehicle box [X={vb.xmin:.2f},{vb.xmax:.2f} Y={vb.ymin:.2f},{vb.ymax:.2f}]; rejecting (would drive through car)."
                )
                self._mission_log_append(
                    "goal_generation_failed",
                    {
                        "failure_code": "goal_inside_vehicle_box",
                        "state": self.current_state,
                        "goal_xy": [round(gx, 3), round(gy, 3)],
                    },
                    sync=True,
                )
                return False

        heading = goal_info["heading"]
        dist = goal_info["distance_to_object"]
        center_x, center_y, center_z = goal_info["center"]
        robot_x, robot_y, robot_z = goal_info["robot_pos"]
        world_frame = goal_info["world_frame"]
        transform_ok = goal_info["transform_ok"]

        self.get_logger().info(
            f"Computing navigation goal:\n"
            f"  Bounding box frame: {world_frame}\n"
            f"  Object center (in {world_frame}): ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})"
        )
        self.get_logger().info(
            f"Navigation goal calculation:\n"
            f"  Object center ({world_frame}): ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})\n"
            f"  Robot position ({world_frame}): ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})\n"
            f"  Vector to object: ({center_x - robot_x:.3f}, {center_y - robot_y:.3f})\n"
            f"  Distance to object: {dist:.3f}m\n"
            f"  Heading: {math.degrees(heading):.2f}°"
        )
        if not transform_ok:
            self.get_logger().warn(
                f"Could not transform goal from {world_frame} to map. Using map frame with world coords."
            )

        self.get_logger().info(
            f"Sending navigation goal:\n"
            f"  Frame: {goal.header.frame_id}\n"
            f"  Goal position: ({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}, {goal.pose.position.z:.3f})\n"
            f"  Orientation (yaw): {math.degrees(heading):.2f}°\n"
            f"  Offset from object: {offset:.3f}m\n"
            f"  Distance robot->goal: {math.sqrt((goal.pose.position.x - robot_x)**2 + (goal.pose.position.y - robot_y)**2):.3f}m"
        )
        self.get_logger().info(
            f"GOAL_COMPUTED: frame={goal.header.frame_id} "
            f"pos=({goal.pose.position.x:.3f},{goal.pose.position.y:.3f},{goal.pose.position.z:.3f}) "
            f"yaw_deg={math.degrees(heading):.1f} offset={offset:.3f} dist_to_object={dist:.3f}"
        )
        self._mission_log_append(
            "goal_computed",
            {
                "frame": goal.header.frame_id,
                "x": goal.pose.position.x,
                "y": goal.pose.position.y,
                "z": goal.pose.position.z,
                "yaw_deg": math.degrees(heading),
                "offset": offset,
                "distance_to_object": dist,
                "state": self.current_state,
                "transform_ok": transform_ok,
                "world_frame": world_frame,
                "box_frame_id": world_frame,
                "detection_age_s": goal_info.get("detection_age_s"),
                "detection_stamp_age_s": goal_info.get("detection_stamp_age_s"),
                "detector_confidence": getattr(box, "probability", None),
                "goal_world": {"x": goal_world[0], "y": goal_world[1], "z": goal_world[2]},
            },
            sync=True,
        )
        self._target_lifecycle = TargetLifecycle.DISPATCHED.value
        self._publish_approach_markers(
            target_center=(center_x, center_y, center_z),
            goal_pose=goal,
            label=f"{box.object_name}:{self._target_lifecycle}",
        )

        # Phase D: Goal validation - distance check (use goal_to_surface_m when present; else distance to center)
        goal_to_surface = goal_info.get("goal_to_surface_m")
        if goal_to_surface is None:
            goal_to_surface = math.sqrt((goal.pose.position.x - center_x)**2 + (goal.pose.position.y - center_y)**2)
        offset_error = abs(goal_to_surface - offset)
        reject_threshold = float(self.get_parameter("reject_goal_offset_error_m").value)
        if reject_threshold > 0 and offset_error > reject_threshold:
            self.get_logger().warn(
                f"Goal validation rejected: |goal_to_surface - offset| = {offset_error:.3f}m > {reject_threshold:.3f}m "
                f"(goal_to_surface={goal_to_surface:.3f}m offset={offset:.3f}m)"
            )
            self._last_dispatch_fail_reason = "goal_offset_validation_failed"
            self._dispatch_fail_count += 1
            self._mission_log_append(
                "goal_offset_validation_failed",
                {
                    "goal_to_surface_m": goal_to_surface,
                    "offset": offset,
                    "offset_error": offset_error,
                    "reject_threshold": reject_threshold,
                    "state": self.current_state,
                },
                sync=True,
            )
            return False
        if offset_error > 0.1:
            self.get_logger().warn(f"Goal validation: goal_to_surface {goal_to_surface:.3f}m vs offset {offset:.3f}m")

        if self._is_dry_run():
            self.get_logger().info("Dry run: goal validated, not sending to Nav2")
            self._last_dispatch_fail_reason = None
            return True  # caller still transitions state in dry run

        goal.header.frame_id = self.get_parameter("map_frame").value  # Nav2 expects map frame
        goal.header.stamp = self.get_clock().now().to_msg()  # Nav2 best practice: current timestamp to avoid tf extrapolation
        self._current_goal_pose = goal
        self.current_goal_pub.publish(goal)  # So operators/monitors always know where the robot is heading
        ok = send_nav_goal(self, goal, self._on_box_goal_done, feedback_cb=self._on_nav_feedback)
        if not ok:
            self.get_logger().error(
                f"NAV_COMMAND_FAILED: goal_frame={goal.header.frame_id} "
                f"pos=({goal.pose.position.x:.3f},{goal.pose.position.y:.3f},{goal.pose.position.z:.3f})"
            )
            self._mission_log_append(
                "nav_command_failed",
                {
                    "frame": goal.header.frame_id,
                    "x": goal.pose.position.x,
                    "y": goal.pose.position.y,
                    "z": goal.pose.position.z,
                    "state": self.current_state,
                },
                sync=True,
            )
            self._last_dispatch_fail_reason = "nav2_unavailable"
            self._dispatch_fail_count += 1
        else:
            self._last_dispatch_fail_reason = None
            self._dispatch_fail_count = 0
            self._last_goal_dispatch_time = time.time()
            self.get_logger().info(
                f"NAV_COMMAND_SENT: goal_frame={goal.header.frame_id} "
                f"pos=({goal.pose.position.x:.3f},{goal.pose.position.y:.3f},{goal.pose.position.z:.3f}) "
                f"yaw_deg={math.degrees(heading):.1f}"
            )
            self._mission_log_append(
                "nav_command_sent",
                {
                    "frame": goal.header.frame_id,
                    "x": goal.pose.position.x,
                    "y": goal.pose.position.y,
                    "z": goal.pose.position.z,
                    "yaw_deg": math.degrees(heading),
                    "state": self.current_state,
                },
                sync=True,
            )
        return ok

    def _simulate_approach_success(self) -> None:
        """Dry run: apply same state updates as nav arrival at vehicle (so flow matches full launch)."""
        self._current_goal_pose = None
        self.current_tire_idx = 0
        self.inspected_tire_positions = []
        self.rotation_attempts = 0
        self._set_state(MissionState.WAIT_TIRE_BOX, cause="dry_run_nav_arrived")

    def _send_nav_goal(self, pose: PoseStamped, done_cb) -> bool:
        """Deprecated: use navigation_controller.send_nav_goal."""
        return send_nav_goal(self, pose, done_cb, feedback_cb=self._on_nav_feedback)

    def _get_current_yaw(self) -> Optional[float]:
        """Get current robot yaw from TF."""
        pose = self._get_current_pose()
        if pose is None:
            return None
        return yaw_from_quaternion(pose.pose.orientation)

    def _get_current_pose(self) -> Optional[PoseStamped]:
        """Get current robot pose from TF."""
        world_frame = self.get_parameter("world_frame").value
        base_frame = self.get_parameter("base_frame").value
        return tf_get_current_pose(
            self.tf_buffer,
            world_frame,
            base_frame,
            timeout_s=0.2,
            logger=self.get_logger(),
        )

    def _add_tire_to_deferred(self, position: tuple, tire_position_label: str) -> None:
        """Add skipped tire to deferred list for return-later retry. Only on first pass."""
        if not bool(self.get_parameter("return_later_enabled").value):
            return
        if self._return_later_pass_count != 0:
            return  # During return-later pass, do not add (avoid infinite retry loop)
        max_passes = int(self.get_parameter("max_return_later_passes").value)
        if self._return_later_pass_count >= max_passes:
            return
        self._tires_deferred.append({
            "position": position,
            "vehicle_id": self.current_vehicle_idx,
            "tire_position_label": tire_position_label,
        })

    def _should_retry_deferred_tires(self) -> bool:
        """Return True if we should retry deferred tires before moving to NEXT_VEHICLE."""
        if not bool(self.get_parameter("return_later_enabled").value):
            return False
        if not self._tires_deferred:
            return False
        max_passes = int(self.get_parameter("max_return_later_passes").value)
        if self._return_later_pass_count >= max_passes:
            return False
        # All deferred must belong to current vehicle
        for d in self._tires_deferred:
            if d.get("vehicle_id") != self.current_vehicle_idx:
                return False
        return True

    def _requeue_deferred_tires(self) -> None:
        """Add deferred tires back to planned positions, sorted by distance to robot."""
        robot_pose = self._get_current_pose()
        rx = robot_pose.pose.position.x if robot_pose else 0.0
        ry = robot_pose.pose.position.y if robot_pose else 0.0
        for d in self._tires_deferred:
            if d.get("vehicle_id") == self.current_vehicle_idx:
                pos = d.get("position")
                if pos is not None:
                    self._planned_tire_positions.append(pos)
        self._planned_tire_positions.sort(key=lambda t: math.hypot(t[0] - rx, t[1] - ry))
        n = len([d for d in self._tires_deferred if d.get("vehicle_id") == self.current_vehicle_idx])
        self._tires_deferred = [d for d in self._tires_deferred if d.get("vehicle_id") != self.current_vehicle_idx]
        self._return_later_pass_count += 1
        self._mission_report["tires_deferred_retried"] = (
            self._mission_report.get("tires_deferred_retried", 0) + n
        )
        self.get_logger().info(f"Return later: retrying {n} deferred tires (pass {self._return_later_pass_count}).")
        self._mission_log_append(
            "return_later_requeue",
            {"count": n, "pass": self._return_later_pass_count, "remaining_planned": len(self._planned_tire_positions)},
            sync=True,
        )
        if self.current_vehicle_idx < len(self.detected_vehicles):
            self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)

    def _get_current_pose_in_map_frame(self) -> Optional[PoseStamped]:
        """Get current robot pose in map frame (for Nav2 and capture metadata)."""
        pose = self._get_current_pose()
        if pose is None:
            return None
        map_frame = self.get_parameter("map_frame").value
        transformed = tf_transform_pose(
            self.tf_buffer,
            pose,
            map_frame,
            timeout_s=2.0,
            logger=self.get_logger(),
        )
        return transformed or pose  # fallback: use world_frame pose (identity map/slamware_map)

    def _distance_to_current_goal(self) -> Optional[float]:
        """Return distance from robot to current goal in map frame, or None."""
        if self._current_goal_pose is None:
            return None
        pose_map = self._get_current_pose_in_map_frame()
        if pose_map is None:
            return None
        dx = pose_map.pose.position.x - self._current_goal_pose.pose.position.x
        dy = pose_map.pose.position.y - self._current_goal_pose.pose.position.y
        return math.hypot(dx, dy)

    def _distance_to_nearest_point_on_committed_box(self) -> Optional[float]:
        """Return 2D distance from robot to nearest point on committed vehicle box (edge), or None.
        Bounding box and costmap both represent the vehicle; this distance approximates distance to costmap occupied area.
        """
        box = self._committed_vehicle_box if self._committed_vehicle_box is not None else self.current_vehicle_box
        if box is None:
            return None
        pose = self._get_current_pose()
        if pose is None:
            return None
        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y
        np_x = max(box.xmin, min(box.xmax, robot_x))
        np_y = max(box.ymin, min(box.ymax, robot_y))
        dx = robot_x - np_x
        dy = robot_y - np_y
        return math.hypot(dx, dy)

    def _distance_to_vehicle_anchor(self) -> Optional[float]:
        """Return distance from robot to current vehicle anchor, or None."""
        # Prefer saved anchor pose in map frame (stable across box jitter).
        anchor_pose = None
        if self.current_vehicle_idx < len(self.detected_vehicles):
            anchor_pose = self.detected_vehicles[self.current_vehicle_idx].get("anchor_pose")
        pose_map = self._get_current_pose_in_map_frame()
        if anchor_pose and pose_map is not None:
            dx = pose_map.pose.position.x - anchor_pose["x"]
            dy = pose_map.pose.position.y - anchor_pose["y"]
            return math.hypot(dx, dy)
        # Fallback: use current vehicle box center in world frame.
        if self.current_vehicle_box is None:
            return None
        pose_world = self._get_current_pose()
        if pose_world is None:
            return None
        center_x = (self.current_vehicle_box.xmin + self.current_vehicle_box.xmax) / 2.0
        center_y = (self.current_vehicle_box.ymin + self.current_vehicle_box.ymax) / 2.0
        dx = pose_world.pose.position.x - center_x
        dy = pose_world.pose.position.y - center_y
        return math.hypot(dx, dy)

    def _goal_tolerance_for_state(self) -> Optional[float]:
        if self.current_state == MissionState.APPROACH_VEHICLE:
            return float(self.get_parameter("approach_goal_tolerance_m").value)
        if self.current_state == MissionState.INSPECT_TIRE:
            return float(self.get_parameter("tire_goal_tolerance_m").value)
        return None

    def _handle_box_goal_success(self, cause: str) -> None:
        """Handle success for approach/tire goals (used by Nav2 or fallback)."""
        self._nav_retry_count = 0
        self._progress_stall_count = 0
        self._progress_window_start = None
        self._progress_start_pose = None
        if self.current_state == MissionState.APPROACH_VEHICLE:
            self._current_goal_pose = None
            self.current_tire_idx = 0
            self.inspected_tire_positions = []
            self.rotation_attempts = 0
            self._vehicle_reacquire_attempts = 0
            self._sync_planned_tires_for_current_vehicle()
            self._set_state(MissionState.WAIT_TIRE_BOX, cause=cause)
            return
        if self.current_state == MissionState.INSPECT_TIRE:
            if self.get_parameter("face_tire_final_yaw").value:
                outcome = self._dispatch_face_tire_goal()
                if outcome == "sent":
                    self._set_state(MissionState.FACE_TIRE, cause=cause)
                elif outcome == "failed":
                    # Still at tire; proceed to wheel wait / capture instead of stalling in WAIT_TIRE_BOX
                    if self.get_parameter("capture_require_wheel_detection").value:
                        self._set_state(MissionState.WAIT_WHEEL_FOR_CAPTURE, cause="face_tire_dispatch_failed")
                    else:
                        self._trigger_tire_capture()
                return
            if self.get_parameter("capture_require_wheel_detection").value:
                self._set_state(MissionState.WAIT_WHEEL_FOR_CAPTURE, cause=cause)
            else:
                self._trigger_tire_capture()
            return

    def _publish_capture_metadata(self, tire_class: str, tire_position: str = ""):
        """Publish JSON metadata for the next photo capture (map pose, timestamp, tire_class, vehicle_id, tire_number)."""
        pose = self._get_current_pose_in_map_frame()
        if pose is None:
            self.get_logger().warn("Cannot get pose in map frame for capture metadata; publishing minimal metadata.")
        now = self.get_clock().now()
        tire_number = len(self._tire_registry) if self._tire_registry else (self._total_tires_captured + 1)
        meta = {
            "frame_id": pose.header.frame_id if pose else "map",
            "x": float(pose.pose.position.x) if pose else 0.0,
            "y": float(pose.pose.position.y) if pose else 0.0,
            "z": float(pose.pose.position.z) if pose else 0.0,
            "yaw": float(yaw_from_quaternion(pose.pose.orientation)) if pose else 0.0,
            "tire_class": tire_class,
            "tire_position": tire_position,
            "tire_number": tire_number,
            "tire_goal_source": self._tire_registry[-1].get("goal_source", "") if self._tire_registry else "",
            "timestamp_sec": int(now.nanoseconds // 1_000_000_000),
            "timestamp_nanosec": int(now.nanoseconds % 1_000_000_000),
        }
        # Include vehicle anchor (pose at detection time) when available
        if self.current_vehicle_idx < len(self.detected_vehicles):
            v = self.detected_vehicles[self.current_vehicle_idx]
            meta["vehicle_id"] = v.get("vehicle_id", self.current_vehicle_idx + 1)
            if "anchor_pose" in v:
                meta["vehicle_anchor"] = v["anchor_pose"]
        msg = String()
        msg.data = json.dumps(meta)
        self.capture_metadata_pub.publish(msg)

    def _dispatch_rotation_goal(self, is_vehicle: bool = True, is_search: bool = False):
        """Dispatch a rotation goal to turn in place by rotation_angle.
        
        To ensure Nav2 actually executes the rotation, we add a small position offset
        in the direction of the new heading. This forces Nav2 to rotate to align
        with the goal orientation.
        """
        current_pose = self._get_current_pose_in_map_frame()
        if current_pose is None:
            self.get_logger().error("Cannot get current pose for rotation. Aborting recovery.")
            if is_vehicle:
                self._set_state(MissionState.NEXT_VEHICLE)
            else:
                self.current_tire_idx += 1
                if self.current_tire_idx >= self.get_parameter("expected_tires_per_vehicle").value:
                    self._set_state(MissionState.NEXT_VEHICLE)
                else:
                    self._set_state(MissionState.WAIT_TIRE_BOX)
            return
        map_frame = self.get_parameter("map_frame").value
        if current_pose.header.frame_id != map_frame and self.get_parameter("require_goal_transform").value:
            self.get_logger().warn(
                f"Rotation: pose frame {current_pose.header.frame_id} != {map_frame}; cannot dispatch."
            )
            self._mission_log_append(
                "rotation_dispatch_blocked",
                {"reason": "map_frame_unavailable", "frame": current_pose.header.frame_id},
                sync=True,
            )
            return
        if self.get_parameter("require_nav_permitted").value and self._nav_permitted is not True:
            self._last_dispatch_fail_reason = "nav_permitted_blocked"
            self._mission_log_append(
                "nav_gate_blocked_dispatch",
                {
                    "state": self.current_state,
                    "nav_permitted": self._nav_permitted,
                    "last_nav_permitted_time": self._last_nav_permitted_time,
                    "topic": self.get_parameter("nav_permitted_topic").value,
                },
                sync=True,
            )
            return

        current_yaw = yaw_from_quaternion(current_pose.pose.orientation)
        rotation_angle = self.get_parameter("rotation_angle").value
        new_yaw = current_yaw + rotation_angle
        
        # Normalize yaw to [-pi, pi]
        new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
        
        # Calculate actual yaw difference (accounting for wrap-around)
        yaw_diff = new_yaw - current_yaw
        # Normalize to [-pi, pi]
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Add a small position offset in the direction of the new heading
        # This forces Nav2 to actually execute the rotation instead of
        # thinking it's already at the goal (due to orientation tolerance)
        offset = self.get_parameter("rotation_position_offset").value
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = map_frame if current_pose.header.frame_id == map_frame else current_pose.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()  # Use current time
        # Add small forward offset along new heading to force rotation execution
        goal_pose.pose.position.x = current_pose.pose.position.x + offset * math.cos(new_yaw)
        goal_pose.pose.position.y = current_pose.pose.position.y + offset * math.sin(new_yaw)
        goal_pose.pose.position.z = current_pose.pose.position.z
        goal_pose.pose.orientation = quaternion_from_yaw(new_yaw)
        
        self.rotation_attempts += 1
        current_yaw_deg = math.degrees(current_yaw)
        new_yaw_deg = math.degrees(new_yaw)
        yaw_diff_deg = math.degrees(yaw_diff)
        self.get_logger().info(
            f"Rotation goal (attempt {self.rotation_attempts}):\n"
            f"  Current yaw: {current_yaw_deg:.2f}° ({current_yaw:.3f} rad)\n"
            f"  Target yaw: {new_yaw_deg:.2f}° ({new_yaw:.3f} rad)\n"
            f"  Yaw difference: {yaw_diff_deg:.2f}° ({yaw_diff:.3f} rad)\n"
            f"  Position offset: {offset:.3f}m\n"
            f"  Goal position: ({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})"
        )

        if self._is_dry_run():
            self.get_logger().info("Dry run: rotation goal validated, not sending to Nav2")
            if is_search:
                self._set_state(MissionState.SEARCH_VEHICLE, cause="dry_run_rotation")
            elif is_vehicle:
                self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="dry_run_rotation")
            else:
                self._set_state(MissionState.WAIT_TIRE_BOX, cause="dry_run_rotation")
            return

        goal_type = "search" if is_search else ("vehicle" if is_vehicle else "tire")
        if is_search:
            if send_nav_goal(self, goal_pose, self._on_rotation_done_search, feedback_cb=self._on_nav_feedback):
                self._mission_log_append(
                    "rotation_dispatched",
                    {"goal_type": goal_type, "rotation_attempts": self.rotation_attempts},
                    sync=True,
                )
                self._set_state(MissionState.TURN_IN_PLACE_SEARCH)
            else:
                # Stay in SEARCH_VEHICLE; reset wait so we retry after detection_timeout
                self.wait_start_time = time.time()
        elif is_vehicle:
            if send_nav_goal(self, goal_pose, self._on_rotation_done_vehicle, feedback_cb=self._on_nav_feedback):
                self._mission_log_append(
                    "rotation_dispatched",
                    {"goal_type": goal_type, "rotation_attempts": self.rotation_attempts},
                    sync=True,
                )
                self._set_state(MissionState.TURN_IN_PLACE_VEHICLE)
            else:
                self._set_state(MissionState.NEXT_VEHICLE)
        else:
            if send_nav_goal(self, goal_pose, self._on_rotation_done_tire, feedback_cb=self._on_nav_feedback):
                self._mission_log_append(
                    "rotation_dispatched",
                    {"goal_type": goal_type, "rotation_attempts": self.rotation_attempts},
                    sync=True,
                )
                self._set_state(MissionState.TURN_IN_PLACE_TIRE)
            else:
                self.current_tire_idx += 1
                if self.current_tire_idx >= self.get_parameter("expected_tires_per_vehicle").value:
                    self._set_state(MissionState.NEXT_VEHICLE)
                else:
                    self._set_state(MissionState.WAIT_TIRE_BOX)

    # ----------------------- Callbacks ----------------------- #
    def _on_standoff_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Standoff goal rejected.")
            self._stop_robot()
            self._mission_log_append(
                "nav_rejected",
                {"goal": "standoff", "reason": "goal_handle.accepted=False"},
                sync=True,
            )
            return
        self._active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_standoff_result)

    def _on_standoff_result(self, future):
        self._active_nav_goal_handle = None
        status = future.result().status
        self.get_logger().info(f"Standoff result status: {status}")
        # Regardless of status we move to waiting for vehicle box; you may gate this on success.
        self._set_state(MissionState.WAIT_VEHICLE_BOX)

    def _on_follow_waypoints_done(self, future):
        """Handle FollowWaypoints goal acceptance; request result."""
        goal_handle = future.result()
        self._active_follow_waypoints_handle = goal_handle if goal_handle.accepted else None
        if not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal rejected.")
            self._stop_robot()
            self._mission_log_append(
                "follow_waypoints_rejected",
                {"state": self.current_state},
                sync=True,
            )
            self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="follow_waypoints_rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_follow_waypoints_result)

    def _on_follow_waypoints_result(self, future):
        """Process FollowWaypoints result; update captures/skipped, transition to NEXT_VEHICLE."""
        self._active_follow_waypoints_handle = None
        result = future.result().result
        missed = list(result.missed_waypoints) if result else []
        n_poses = int(getattr(self, "_last_follow_waypoints_pose_count", 0) or 0)
        if n_poses <= 0:
            n_poses = 4
        succeeded = max(0, n_poses - len(missed))
        self._total_tires_captured += succeeded
        planned = self._committed_vehicle_planned_tires or []
        use_perimeter = bool(getattr(self, "_last_batch_waypoints_perimeter", False))
        for idx in missed:
            tire_label = f"waypoint_{idx}"
            self._tires_skipped.append({
                "waypoint_index": int(idx),
                "reason": "unreachable",
                "tire_position": tire_label,
            })
            if (
                not use_perimeter
                and planned
                and 0 <= int(idx) < len(planned)
            ):
                self._add_tire_to_deferred(planned[int(idx)], tire_label)
        self._mission_report["tires_skipped"] = list(self._tires_skipped)
        self.get_logger().info(
            f"FollowWaypoints complete: {succeeded} succeeded, {len(missed)} missed (indices {missed})."
        )
        self._mission_log_append(
            "follow_waypoints_result",
            {
                "succeeded": succeeded,
                "missed_waypoints": missed,
                "total_poses": n_poses,
                "perimeter_batch": use_perimeter,
            },
            sync=True,
        )
        self._planned_tire_positions = []
        if self._should_retry_deferred_tires():
            self._requeue_deferred_tires()
            self.get_logger().info("Return later: retrying deferred tires from FollowWaypoints.")
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="return_later_requeue")
            return
        self._set_state(MissionState.NEXT_VEHICLE, cause="follow_waypoints_complete")

    def _on_nav_feedback(self, msg) -> None:
        """Store latest Nav2 feedback for proximity gating and recovery-aware skip."""
        self._last_nav_feedback = getattr(msg, "feedback", msg)

    def _on_box_goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Box goal rejected by Nav2; returning to WAIT so we can retry.")
            self._stop_robot()
            self._mission_log_append(
                "nav_rejected",
                {"goal": "box", "state": self.current_state},
                sync=True,
            )
            if self.current_state == MissionState.APPROACH_VEHICLE:
                self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="nav_goal_rejected")
            elif self.current_state == MissionState.INSPECT_TIRE:
                self._set_state(MissionState.WAIT_TIRE_BOX, cause="nav_goal_rejected")
            return
        self._active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_box_result)

    def _on_box_result(self, future):
        self._active_nav_goal_handle = None
        result = future.result()
        status = result.status
        # Handle centroid handoff: we cancelled to hand off to centroid servo; waiting for centroid_centered
        if status == GoalStatus.STATUS_CANCELED and self._centroid_handoff_initiated:
            self.get_logger().info("Nav2 goal cancelled for centroid handoff; waiting for centroid_centered.")
            return
        # Handle recovery-aware skip: we cancelled the goal; treat as tire skipped
        if status == GoalStatus.STATUS_CANCELED and self._recovery_skip_initiated:
            self._recovery_skip_initiated = False
            tire_label = "unknown"
            if self._tire_registry:
                last_tire = self._tire_registry[-1]
                tire_label = last_tire.get("tire_position", "unknown")
                self._tires_skipped.append({
                    "tire_id": last_tire.get("id"),
                    "tire_position": tire_label,
                    "reason": "recovery_limit_exceeded",
                    "nav_status": int(status),
                })
                self._mission_report["tires_skipped"] = list(self._tires_skipped)
                if self._return_later_pass_count > 0:
                    self._mission_report["tires_deferred_still_skipped"] = (
                        self._mission_report.get("tires_deferred_still_skipped", 0) + 1
                    )
                else:
                    self._add_tire_to_deferred(last_tire.get("position", (0, 0, 0)), tire_label)
                self._tire_registry[-1]["visited"] = True
                self._tire_registry[-1]["image_captured"] = False
            self.get_logger().warn(
                f"Tire skipped (recovery limit): {tire_label}. Continuing to next tire."
            )
            self._mission_log_append(
                "tire_skipped",
                {"tire_position": tire_label, "reason": "recovery_limit_exceeded", "status": int(status)},
                sync=True,
            )
            self._nav_retry_count = 0
            self._last_tire_box = None
            self._last_tire_offset = None
            self._current_goal_pose = None
            self._approach_entered_time = None
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="recovery_skip")
            return
        nav_succeeded = status == 4  # SUCCEEDED
        distance_to_goal = self._distance_to_current_goal()
        distance_to_anchor = None
        if self.current_state == MissionState.APPROACH_VEHICLE:
            distance_to_anchor = self._distance_to_vehicle_anchor()
        forced_success = False
        if not nav_succeeded:
            goal_tol = self._goal_tolerance_for_state()
            if (
                goal_tol is not None
                and distance_to_goal is not None
                and distance_to_goal <= goal_tol
            ):
                forced_success = True
                nav_succeeded = True
                self.get_logger().warn(
                    f"Nav2 reported failure but robot is within {goal_tol:.2f}m of goal; treating as success."
                )
                self._mission_log_append(
                    "nav_forced_success",
                    {
                        "state": self.current_state,
                        "distance_to_goal": round(distance_to_goal, 3),
                        "tolerance_m": goal_tol,
                        "status": int(status),
                    },
                    sync=True,
                )
            if self.current_state == MissionState.APPROACH_VEHICLE:
                anchor_tol = float(self.get_parameter("vehicle_anchor_reach_distance_m").value)
                if (
                    distance_to_anchor is not None
                    and distance_to_anchor <= anchor_tol
                ):
                    forced_success = True
                    nav_succeeded = True
                    self.get_logger().warn(
                        f"Nav2 reported failure but robot is within {anchor_tol:.2f}m of vehicle anchor; treating as success."
                    )
                    self._mission_log_append(
                        "nav_forced_success_anchor",
                        {
                            "state": self.current_state,
                            "distance_to_anchor": round(distance_to_anchor, 3),
                            "tolerance_m": anchor_tol,
                            "status": int(status),
                        },
                        sync=True,
                    )
        self.get_logger().info(
            f"Box goal result:\n"
            f"  Status: {status} ({'SUCCESS' if nav_succeeded else 'FAILED' if status == 2 else 'OTHER'})\n"
            f"  Current state: {self.current_state}"
        )
        self.get_logger().info(
            f"NAV_FEEDBACK: goal=box status={status} success={nav_succeeded} state={self.current_state}"
        )
        self._mission_log_append(
            "nav_result",
            {
                "goal": "box",
                "status": int(status),
                "success": nav_succeeded,
                "state": self.current_state,
                "nav_retry_count": self._nav_retry_count,
                "distance_to_goal": distance_to_goal,
                "distance_to_anchor": distance_to_anchor,
                "forced_success": forced_success,
            },
            sync=True,
        )
        self._target_lifecycle = (
            TargetLifecycle.REACHED.value if nav_succeeded else TargetLifecycle.DISPATCHED.value
        )

        if not nav_succeeded:
            self._stop_robot()
            self._mission_log_append("nav_failure_stop_cmd_vel", {"status": int(status)}, sync=True)
            self._nav_retry_count += 1
            budget = self.get_parameter("nav_retry_budget").value
            self._progress_stall_count += 1
            if self.current_state == MissionState.APPROACH_VEHICLE:
                if self._nav_retry_count <= budget and self._last_approach_box is not None:
                    delay_s = float(self.get_parameter("approach_nav_retry_delay_s").value)
                    self._approach_retry_pending = True
                    self._approach_retry_at = time.time() + delay_s
                    self.get_logger().warn(
                        f"Approach nav failed; retry {self._nav_retry_count}/{int(budget)} in {delay_s:.1f}s (delay avoids preemption storm)."
                    )
                    self._mission_log_append(
                        "nav_retry_scheduled",
                        {
                            "state": self.current_state,
                            "nav_retry_count": self._nav_retry_count,
                            "delay_s": delay_s,
                        },
                        sync=True,
                    )
                    return
                self.get_logger().error(
                    f"Approach nav failed after {self._nav_retry_count} attempts; returning to WAIT_VEHICLE_BOX."
                )
                self._nav_retry_count = 0
                self._current_goal_pose = None
                self._approach_retry_pending = False
                self._approach_retry_at = None
                self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="nav_failed_approach")
                return
            if self.current_state == MissionState.INSPECT_TIRE:
                self._pending_perimeter_nav_queue.clear()
                skip_on_fail = bool(self.get_parameter("skip_tire_on_nav_failure").value)
                if skip_on_fail:
                    # Skip unreachable tire immediately (research: stop_on_failure: false; determinism 1.3)
                    reason = "unreachable" if status == 2 else "aborted" if status == 3 else "timeout"
                    tire_label = "unknown"
                    if self._tire_registry:
                        last_tire = self._tire_registry[-1]
                        tire_label = last_tire.get("tire_position", "unknown")
                        self._tires_skipped.append({
                            "tire_id": last_tire.get("id"),
                            "tire_position": tire_label,
                            "reason": reason,
                            "nav_status": int(status),
                        })
                        self._mission_report["tires_skipped"] = list(self._tires_skipped)
                        if self._return_later_pass_count > 0:
                            self._mission_report["tires_deferred_still_skipped"] = (
                                self._mission_report.get("tires_deferred_still_skipped", 0) + 1
                            )
                        else:
                            self._add_tire_to_deferred(last_tire.get("position", (0, 0, 0)), tire_label)
                    self.get_logger().warn(
                        f"Tire nav failed; skipping tire ({tire_label}, reason={reason}). "
                        "skip_tire_on_nav_failure=True."
                    )
                    self._mission_log_append(
                        "tire_skipped",
                        {"tire_position": tire_label, "reason": reason, "status": int(status)},
                        sync=True,
                    )
                    self._nav_retry_count = 0
                    self._last_tire_box = None
                    self._last_tire_offset = None
                    if self._tire_registry:
                        self._tire_registry[-1]["visited"] = True
                        self._tire_registry[-1]["image_captured"] = False
                    self._current_goal_pose = None
                    self._set_state(MissionState.WAIT_TIRE_BOX, cause="tire_skipped_unreachable")
                    return
                budget_ok = self._nav_retry_count <= budget
                try_next = (
                    bool(self.get_parameter("try_next_corner_on_nav_fail").value)
                    and self._planned_tire_positions
                    and budget_ok
                )
                if try_next:
                    target = self._planned_tire_positions.pop(0)
                    if self.current_vehicle_idx < len(self.detected_vehicles):
                        self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(
                            self._planned_tire_positions
                        )
                    # Register this tire so it is counted (same as first-tire from WAIT_VEHICLE_BOX)
                    self.inspected_tire_positions.append(target)
                    robot_pose = self._get_current_pose()
                    robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z) if robot_pose else None
                    tire_pos_label = (
                        tire_position_label(target, self._committed_vehicle_center, robot_pos)
                        if self._committed_vehicle_center and robot_pos else "nearest"
                    )
                    self._tire_id_counter += 1
                    self._tire_registry.append({
                        "id": self._tire_id_counter,
                        "vehicle_id": self.current_vehicle_idx,
                        "position": target,
                        "tire_position": tire_pos_label or "nearest",
                        "visited": False,
                        "image_captured": False,
                        "goal_source": "planned",
                    })
                    self.get_logger().warn(
                        f"Tire nav failed; trying next nearest corner ({self._nav_retry_count}/{int(budget)})."
                    )
                    self._mission_log_append(
                        "nav_try_next_corner",
                        {
                            "nav_retry_count": self._nav_retry_count,
                            "remaining_corners": len(self._planned_tire_positions),
                        },
                        sync=True,
                    )
                    self._current_tire_approach_start_time = time.time()
                    sent, used_refined = self._dispatch_planned_tire_goal(target)
                    if used_refined and self._tire_registry:
                        self._tire_registry[-1]["goal_source"] = "detection_refined"
                    if sent:
                        return
                    if self._last_dispatch_fail_reason == "goal_inside_vehicle_box":
                        self._planned_tire_positions.append(target)
                elif budget_ok and self._last_tire_box is not None:
                    self.get_logger().warn(
                        f"Tire nav failed; retry same goal {self._nav_retry_count}/{int(budget)}."
                    )
                    self._current_goal_pose = None
                    self._dispatch_box_goal(
                        self._last_tire_box,
                        offset=self._last_tire_offset,
                        detection_stamp=self._last_detection_stamp,
                        vehicle_box=self._committed_vehicle_box,
                    )
                    return
                self.get_logger().error(
                    f"Tire nav failed after {self._nav_retry_count} attempts; skipping to next tire."
                )
                self._nav_retry_count = 0
                self._last_tire_box = None
                self._last_tire_offset = None
                if self._tire_registry:
                    last_tire = self._tire_registry[-1]
                    last_tire["visited"] = True
                    last_tire["image_captured"] = False
                    if self._return_later_pass_count > 0:
                        self._mission_report["tires_deferred_still_skipped"] = (
                            self._mission_report.get("tires_deferred_still_skipped", 0) + 1
                        )
                    else:
                        self._add_tire_to_deferred(last_tire.get("position", (0, 0, 0)), last_tire.get("tire_position", "unknown"))
                self._current_goal_pose = None
                self._set_state(MissionState.WAIT_TIRE_BOX, cause="nav_failed_tire")
                return

        if (
            nav_succeeded
            and self.current_state == MissionState.INSPECT_TIRE
            and self._pending_perimeter_nav_queue
        ):
            next_pose = self._pending_perimeter_nav_queue.pop(0)
            self._mission_log_append(
                "perimeter_nav_segment",
                {"remaining": len(self._pending_perimeter_nav_queue)},
                sync=True,
            )
            if send_nav_goal(self, next_pose, self._on_box_goal_done, feedback_cb=self._on_nav_feedback):
                self._current_goal_pose = next_pose
                self.current_goal_pub.publish(next_pose)
                return
            self._pending_perimeter_nav_queue.clear()
            self.get_logger().warn("perimeter_nav: failed to send next segment; completing tire arrival.")

        if self.current_state == MissionState.APPROACH_VEHICLE:
            self._handle_box_goal_success("nav_arrived_vehicle")
            return

        if self.current_state == MissionState.INSPECT_TIRE:
            self._current_tire_nav_retries = self._nav_retry_count  # capture before reset (observability)
            self._handle_box_goal_success("nav_arrived_tire")
            return

    def _compute_face_tire_yaw(self) -> Optional[float]:
        """Compute target yaw to face the current tire center."""
        pose = self._get_current_pose()
        if pose is None:
            return None
        tire_box = self._last_tire_box
        if tire_box is None:
            return yaw_from_quaternion(pose.pose.orientation)
        center_x = (tire_box.xmin + tire_box.xmax) / 2.0
        center_y = (tire_box.ymin + tire_box.ymax) / 2.0
        dx = center_x - pose.pose.position.x
        dy = center_y - pose.pose.position.y
        if math.hypot(dx, dy) < 0.01:
            return yaw_from_quaternion(pose.pose.orientation)
        return math.atan2(dy, dx)

    def _dispatch_face_tire_goal(self) -> str:
        """Rotate in place to face the tire before capture. Returns: sent | capture | failed."""
        if self.get_parameter("require_nav_permitted").value and self._nav_permitted is not True:
            self._mission_log_append(
                "nav_gate_blocked_dispatch",
                {
                    "state": self.current_state,
                    "nav_permitted": self._nav_permitted,
                    "last_nav_permitted_time": self._last_nav_permitted_time,
                    "topic": self.get_parameter("nav_permitted_topic").value,
                },
                sync=True,
            )
            return "failed"
        target_yaw = self._compute_face_tire_yaw()
        if target_yaw is None:
            self.get_logger().warn("Face tire: cannot compute target yaw.")
            self._mission_log_append(
                "face_tire_failed",
                {"reason": "target_yaw_unavailable", "state": self.current_state},
                sync=True,
            )
            return "failed"
        current_yaw = self._get_current_yaw()
        tol = float(self.get_parameter("face_tire_yaw_tolerance").value)
        if current_yaw is not None:
            yaw_diff = abs(math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw)))
            if yaw_diff <= tol:
                self._mission_log_append(
                    "face_tire_aligned",
                    {"yaw_diff": yaw_diff, "tolerance": tol},
                    sync=True,
                )
                self._trigger_tire_capture()
                return "capture"
        pose_map = self._get_current_pose_in_map_frame()
        if pose_map is None:
            self.get_logger().warn("Face tire: cannot get pose in map frame.")
            return "failed"
        map_frame = self.get_parameter("map_frame").value
        if pose_map.header.frame_id != map_frame and self.get_parameter("require_goal_transform").value:
            self.get_logger().warn(
                f"Face tire: pose frame {pose_map.header.frame_id} != {map_frame}; cannot dispatch."
            )
            return "failed"
        goal = PoseStamped()
        goal.header.frame_id = map_frame if pose_map.header.frame_id == map_frame else pose_map.header.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position = pose_map.pose.position
        goal.pose.orientation = quaternion_from_yaw(target_yaw)
        pos_ok = (
            math.isfinite(goal.pose.position.x)
            and math.isfinite(goal.pose.position.y)
            and math.isfinite(goal.pose.position.z)
        )
        ori = goal.pose.orientation
        ori_ok = (
            math.isfinite(ori.x) and math.isfinite(ori.y)
            and math.isfinite(ori.z) and math.isfinite(ori.w)
        )
        if not (pos_ok and ori_ok):
            self.get_logger().warn("Face tire: goal pose has non-finite value; cannot dispatch.")
            self._mission_log_append(
                "face_tire_failed",
                {"reason": "goal_pose_not_finite", "state": self.current_state},
                sync=True,
            )
            return "failed"
        if self._is_dry_run():
            self.get_logger().info("Dry run: face_tire rotation validated, not sending to Nav2")
            self._trigger_tire_capture()
            return "capture"
        if send_nav_goal(self, goal, self._on_face_tire_done, feedback_cb=self._on_nav_feedback):
            self._face_tire_start_time = time.time()
            self._face_tire_target_yaw = target_yaw
            self._mission_log_append(
                "face_tire_dispatched",
                {"yaw_deg": math.degrees(target_yaw), "frame": goal.header.frame_id},
                sync=True,
            )
            return "sent"
        self.get_logger().warn("Face tire: Nav2 goal dispatch failed.")
        return "failed"

    def _trigger_tire_capture(self) -> None:
        """Trigger photo capture after tire approach (with distance gate)."""
        trigger_threshold = float(self.get_parameter("photo_trigger_distance").value)
        distance_to_goal = None
        if self._current_goal_pose is not None:
            pose_map = self._get_current_pose_in_map_frame()
            if pose_map is not None:
                dx = pose_map.pose.position.x - self._current_goal_pose.pose.position.x
                dy = pose_map.pose.position.y - self._current_goal_pose.pose.position.y
                distance_to_goal = math.sqrt(dx * dx + dy * dy)
        # Secondary gate: use Nav2 feedback distance_remaining when TF-based distance is unavailable
        if distance_to_goal is None and self._last_nav_feedback is not None:
            dist_rem = getattr(self._last_nav_feedback, "distance_remaining", None)
            if dist_rem is not None:
                distance_to_goal = dist_rem
        demo = self._demo_mode_enabled()
        self._mission_log_append(
            "photo_trigger_distance_check",
            {
                "distance_to_goal": distance_to_goal,
                "threshold": trigger_threshold,
                "state": self.current_state,
                "demo_mode": demo,
            },
            sync=True,
        )
        if not demo:
            if not should_trigger_photo(distance_to_goal, trigger_threshold):
                self.get_logger().error(
                    f"Photo trigger blocked: distance_to_goal={distance_to_goal} > threshold={trigger_threshold}"
                )
                self._mission_log_append(
                    "photo_trigger_blocked",
                    {
                        "distance_to_goal": distance_to_goal,
                        "threshold": trigger_threshold,
                        "state": self.current_state,
                    },
                    sync=True,
                )
                self._set_state(MissionState.WAIT_TIRE_BOX, cause="photo_trigger_distance")
                return
            # Optional: require close to tire center (avoids capturing when goal was wrong or we never reached tire)
            max_to_tire = float(self.get_parameter("capture_max_distance_to_tire_m").value)
            if max_to_tire > 0 and self._last_tire_box is not None:
                pose_map = self._get_current_pose_in_map_frame()
                if pose_map is not None:
                    cx = (self._last_tire_box.xmin + self._last_tire_box.xmax) / 2.0
                    cy = (self._last_tire_box.ymin + self._last_tire_box.ymax) / 2.0
                    dx = pose_map.pose.position.x - cx
                    dy = pose_map.pose.position.y - cy
                    dist_to_tire = math.sqrt(dx * dx + dy * dy)
                    if dist_to_tire > max_to_tire:
                        self.get_logger().error(
                            f"Photo trigger blocked: distance_to_tire={dist_to_tire:.2f}m > max={max_to_tire}m (not at tire)"
                        )
                        self._mission_log_append(
                            "photo_trigger_blocked",
                            {
                                "distance_to_tire": round(dist_to_tire, 3),
                                "max_distance_to_tire_m": max_to_tire,
                                "state": self.current_state,
                            },
                            sync=True,
                        )
                        self._set_state(MissionState.WAIT_TIRE_BOX, cause="photo_trigger_distance_to_tire")
                        return
            # Point-cloud distance check: require close to nearest tire box face (adds robustness if TF/odom drifts)
            max_to_face = float(self.get_parameter("capture_max_distance_to_tire_face_m").value)
            if max_to_face > 0 and self._last_tire_box is not None:
                pose_map = self._get_current_pose_in_map_frame()
                if pose_map is not None:
                    dist_to_face = distance_point_to_aabb_2d(
                        pose_map.pose.position.x,
                        pose_map.pose.position.y,
                        self._last_tire_box.xmin,
                        self._last_tire_box.xmax,
                        self._last_tire_box.ymin,
                        self._last_tire_box.ymax,
                    )
                    if dist_to_face > max_to_face:
                        self.get_logger().error(
                            f"Photo trigger blocked: distance_to_tire_face={dist_to_face:.2f}m > max={max_to_face}m (not close enough)"
                        )
                        self._mission_log_append(
                            "photo_trigger_blocked",
                            {
                                "distance_to_tire_face": round(dist_to_face, 3),
                                "max_distance_to_tire_face_m": max_to_face,
                                "state": self.current_state,
                            },
                            sync=True,
                        )
                        self._set_state(MissionState.WAIT_TIRE_BOX, cause="photo_trigger_distance_to_tire_face")
                        return
        else:
            self.get_logger().info(
                "demo_mode: skipping photo distance gates (photo_trigger_distance, capture_max_distance_to_tire_m, face)"
            )
            self._mission_log_append(
                "photo_trigger_demo_mode",
                {
                    "distance_to_goal": distance_to_goal,
                    "bypass": "distance_gates",
                },
                sync=True,
            )
        if self._tire_registry:
            self._tire_registry[-1]["visited"] = True
        # Store distance for mission report before clearing goal
        self._last_capture_distance_m = distance_to_goal
        pose_map = self._get_current_pose_in_map_frame()
        if pose_map is not None and self._last_tire_box is not None:
            cx = (self._last_tire_box.xmin + self._last_tire_box.xmax) / 2.0
            cy = (self._last_tire_box.ymin + self._last_tire_box.ymax) / 2.0
            dx = pose_map.pose.position.x - cx
            dy = pose_map.pose.position.y - cy
            self._last_capture_dist_to_tire_m = math.sqrt(dx * dx + dy * dy)
        else:
            self._last_capture_dist_to_tire_m = None
        self._current_goal_pose = None
        tire_class = self.get_parameter("tire_label").value
        tire_position = (self._tire_registry[-1].get("tire_position", "")) if self._tire_registry else ""
        self._publish_capture_metadata(tire_class, tire_position)
        self._total_tires_captured += 1
        self._target_lifecycle = TargetLifecycle.VERIFIED.value
        self._capture_retry_count = 0
        self._verify_capture_start_time = time.time()
        photo_msg = Bool()
        photo_msg.data = True
        self.photo_capture_pub.publish(photo_msg)
        self.get_logger().info(f"Photo capture triggered for tire {self._total_tires_captured}; entering VERIFY_CAPTURE")
        self._set_state(MissionState.VERIFY_CAPTURE, cause="photo_triggered")

    def _on_face_tire_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Face tire goal rejected.")
            self._stop_robot()
            self._mission_log_append("nav_rejected", {"goal": "face_tire"}, sync=True)
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="face_tire_rejected")
            return
        self._active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_face_tire_result)

    def _on_face_tire_result(self, future):
        self._active_nav_goal_handle = None
        status = future.result().status
        nav_succeeded = status == 4
        self._mission_log_append(
            "face_tire_result",
            {"status": int(status), "success": nav_succeeded},
            sync=True,
        )
        target_yaw = self._face_tire_target_yaw  # capture before reset
        self._face_tire_start_time = None
        self._face_tire_target_yaw = None
        if not nav_succeeded:
            # PAL Robotics / Nav2: check if we're already aligned despite Nav2 status (e.g. status 6 OTHER)
            tol = float(self.get_parameter("face_tire_yaw_tolerance").value)
            current_yaw = self._get_current_yaw()
            if target_yaw is not None and current_yaw is not None:
                yaw_diff = abs(math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw)))
                if yaw_diff <= tol:
                    self.get_logger().info(
                        f"Face tire Nav2 status={status} but robot aligned (yaw_diff={math.degrees(yaw_diff):.1f}° <= {math.degrees(tol):.1f}°); capturing."
                    )
                    self._mission_log_append(
                        "face_tire_aligned_fallback",
                        {"status": int(status), "yaw_diff_deg": math.degrees(yaw_diff), "tolerance_deg": math.degrees(tol)},
                        sync=True,
                    )
                    if self.get_parameter("capture_require_wheel_detection").value:
                        self._set_state(MissionState.WAIT_WHEEL_FOR_CAPTURE, cause="face_tire_aligned_fallback")
                    else:
                        self._trigger_tire_capture()
                    return
            self.get_logger().warn("Face tire rotation failed; returning to WAIT_TIRE_BOX.")
            self._stop_robot()
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="face_tire_failed")
            return
        if self.get_parameter("capture_require_wheel_detection").value:
            self._set_state(MissionState.WAIT_WHEEL_FOR_CAPTURE, cause="face_tire_done")
        else:
            self._trigger_tire_capture()

    def _on_rotation_done_vehicle(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Rotation goal rejected.")
            self._stop_robot()
            self._mission_log_append("nav_rejected", {"goal": "rotation_vehicle"}, sync=True)
            self._set_state(MissionState.NEXT_VEHICLE)
            return
        self._active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_vehicle)

    def _on_rotation_result_vehicle(self, future):
        self._active_nav_goal_handle = None
        status = future.result().status
        self.get_logger().info(f"Rotation result status: {status}")
        # Reset wait timer to give more time after rotation
        self.wait_start_time = time.time()
        # Return to waiting for vehicle box
        self._set_state(MissionState.WAIT_VEHICLE_BOX)

    def _on_rotation_done_tire(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Rotation goal rejected.")
            self._stop_robot()
            self._mission_log_append("nav_rejected", {"goal": "rotation_tire"}, sync=True)
            # Check if we have enough tires
            if len(self.inspected_tire_positions) >= self.get_parameter("expected_tires_per_vehicle").value:
                if self._should_retry_deferred_tires():
                    self._requeue_deferred_tires()
                    self._set_state(MissionState.WAIT_TIRE_BOX, cause="return_later_requeue")
                    return
                self._set_state(MissionState.NEXT_VEHICLE)
            else:
                self._set_state(MissionState.WAIT_TIRE_BOX)
            return
        self._active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_tire)

    def _on_rotation_result_tire(self, future):
        self._active_nav_goal_handle = None
        status = future.result().status
        self.get_logger().info(f"Rotation result status: {status}")
        # Reset wait timer to give more time after rotation
        self.wait_start_time = time.time()
        # Single-tire mode and no more tires to visit: go straight to capture (avoid stall in WAIT_TIRE_BOX)
        expected_tires = self.get_parameter("expected_tires_per_vehicle").value
        if (
            status == 4
            and expected_tires == 1
            and not self._planned_tire_positions
            and len(self.inspected_tire_positions) >= 1
        ):
            self.get_logger().info(
                "Single-tire mode: rotation done at tire; proceeding to WAIT_WHEEL_FOR_CAPTURE."
            )
            if self.get_parameter("capture_require_wheel_detection").value:
                self._set_state(MissionState.WAIT_WHEEL_FOR_CAPTURE, cause="rotation_done_single_tire")
            else:
                self._trigger_tire_capture()
            return
        # Return to waiting for tire box (or next tire)
        self._set_state(MissionState.WAIT_TIRE_BOX)

    def _on_patrol_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Patrol goal rejected; returning to SEARCH_VEHICLE.")
            self._stop_robot()
            self._set_state(MissionState.SEARCH_VEHICLE, cause="patrol_rejected")
            return
        self._active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_patrol_result)

    def _on_patrol_result(self, future):
        self._active_nav_goal_handle = None
        status = future.result().status
        self.get_logger().info(f"Patrol result status: {status}")
        pause_s = float(self.get_parameter("patrol_pause_s").value)
        if pause_s > 0:
            time.sleep(pause_s)
        self.wait_start_time = time.time()
        self._set_state(MissionState.SEARCH_VEHICLE, cause="patrol_complete")

    def _on_rotation_done_search(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Search rotation goal rejected.")
            self._stop_robot()
            self._mission_log_append("nav_rejected", {"goal": "rotation_search"}, sync=True)
            self._mission_report["error_states_encountered"] += 1
            self._publish_mission_report()
            self._set_state(MissionState.DONE, cause="rotation_search_rejected")
            return
        self._active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_search)

    def _on_rotation_result_search(self, future):
        self._active_nav_goal_handle = None
        status = future.result().status
        self.get_logger().info(f"Search rotation result status: {status}")
        self.wait_start_time = time.time()
        self._set_state(MissionState.SEARCH_VEHICLE, cause="rotation_search_complete")

    # ----------------------- Helpers ----------------------- #
    def _pose_from_truck(self, truck: dict, standoff: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.get_parameter("world_frame").value

        # Stand off along -x of truck heading.
        pose.pose.position.x = truck["x"] - standoff * math.cos(yaw)
        pose.pose.position.y = truck["y"] - standoff * math.sin(yaw)
        pose.pose.position.z = truck.get("z", 0.0)

        pose.pose.orientation = quaternion_from_yaw(yaw)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInspectionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

