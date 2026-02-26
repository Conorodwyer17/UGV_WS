import json
import math
import os
import time
from typing import List, Optional, Dict, Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Header, String
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from ament_index_python.packages import get_package_share_directory
import yaml
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs

from .utils import should_trigger_photo
from .mission_state_machine import MissionState, set_state
from .geometry_utils import quaternion_from_yaw, yaw_from_quaternion
from .perception_handler import (
    parse_vehicle_labels,
    log_bounding_box,
    find_vehicle_box,
    tire_position_label,
    find_tire_for_inspection,
)
from .goal_generator import compute_box_goal
from .navigation_controller import send_nav_goal
from .vehicle_modeler import estimate_tire_positions
from .transformer import get_current_pose as tf_get_current_pose, transform_pose as tf_transform_pose

class VehicleInspectionManager(Node):
    """Mission manager for autonomous vehicle tire inspection using Aurora SLAM and detection."""

    def __init__(self):
        super().__init__("inspection_manager")

        # Vehicle detection parameters - support both car and truck
        self.declare_parameter("vehicle_labels", "car,truck")  # Comma-separated string of vehicle class names (e.g., "car,truck")
        self.declare_parameter("tire_label", "car-tire")  # Must match model output and segment_3d interested_classes
        self.declare_parameter("standoff_distance", 2.0)  # Distance to stop before vehicle
        self.declare_parameter("approach_offset", 0.5)  # Offset when approaching vehicle
        self.declare_parameter("tire_offset", 0.4)  # Offset when approaching tire
        self.declare_parameter("detection_topic", "/darknet_ros_3d/bounding_boxes")  # Aurora detection topic (tires + vehicles when not using semantic)
        self.declare_parameter("vehicle_boxes_topic", "")  # Optional: use semantic vehicle boxes (e.g. /aurora_semantic/vehicle_bounding_boxes); tires still from detection_topic
        self.declare_parameter("world_frame", "slamware_map")  # Must match bounding box frame from segmentation_processor
        self.declare_parameter("base_frame", "base_link")  # Robot base frame
        self.declare_parameter("map_frame", "map")  # Nav2 map frame
        self.declare_parameter("require_goal_transform", True)  # Require world->map transform for goals
        self.declare_parameter("detection_timeout", 5.0)  # seconds to wait before recovery (legacy)
        self.declare_parameter("detection_stale_s", 2.0)  # seconds without detection_topic messages -> stale
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
        self.declare_parameter("planned_tire_fallback_enabled", True)  # allow planned tire goals if no tire boxes
        self.declare_parameter("planned_tire_box_extent_m", 0.25)  # half-extent for planned tire box
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
        self.declare_parameter("photo_capture_topic", "/inspection_manager/capture_photo")  # Topic to trigger photo capture
        self.declare_parameter("capture_metadata_topic", "/inspection_manager/capture_metadata")  # JSON metadata for each capture (pose, tire_class, etc.)
        self.declare_parameter("capture_result_topic", "/inspection_manager/capture_result")  # SUCCESS|FAILURE,filename,bytes from photo_capture_service
        self.declare_parameter("photo_trigger_distance", 0.0)  # meters; 0 disables distance gate before capture
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
        self.declare_parameter("vehicle_wheelbase_m", 2.7)  # meters
        self.declare_parameter("vehicle_track_m", 1.6)  # meters
        self.declare_parameter("tf_watchdog_timeout", 0.2)  # seconds - pause mission if TF fails > this
        self.declare_parameter("tf_unavailable_abort_s", 60.0)  # after this many seconds of continuous TF failure, transition to ERROR
        self.declare_parameter("tf_wait_timeout", 60.0)  # max seconds to wait for TF (slamware_map->base_link) in IDLE before starting mission
        self.declare_parameter("tf_stable_s", 3.0)  # require TF valid for this many seconds in IDLE before starting (avoids start-then-immediate-pause)
        self.declare_parameter("hard_mission_timeout", 1800.0)  # seconds - max mission duration
        self.declare_parameter("nav_retry_budget", 3)  # max nav failures before abort
        self.declare_parameter("max_capture_retries", 2)  # retry photo capture this many times on failure
        self.declare_parameter("capture_verify_timeout_s", 10.0)  # timeout waiting for capture_result in VERIFY_CAPTURE
        self.declare_parameter("dry_run", False)  # validate goals without sending to Nav2
        self.declare_parameter("enable_runtime_diagnostics", True)  # publish /inspection_manager/runtime_diagnostics at 5Hz
        self.declare_parameter("require_sensor_health", False)  # wait for aurora_interface/healthy before SEARCH_VEHICLE
        self.declare_parameter("sensor_health_timeout", 30.0)  # seconds to wait for sensor health in INIT
        self.declare_parameter("start_mission_on_ready", True)  # if False, stay in IDLE until /inspection_manager/start_mission is True
        self.declare_parameter("start_mission_topic", "/inspection_manager/start_mission")
        self.declare_parameter("nav2_wait_timeout", 90.0)  # max seconds to wait for Nav2 navigate_to_pose before starting mission (full-system-on-launch)
        self.declare_parameter("approach_timeout_s", 120.0)  # max seconds in APPROACH_VEHICLE before cancelling and retrying (avoids stuck nav)
        self.declare_parameter("mission_log_path", "")  # JSONL event log (mission_start, state_transition, nav_result, etc.)
        self.declare_parameter("mission_report_path", "")  # full report JSON at mission end
        self.declare_parameter("mission_log_fsync", True)  # fsync each log entry for power-loss safety

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

        # Spin protection: prevent infinite loops
        self._last_state: Optional[str] = None
        self._state_repeat_count = 0

        # Phase C: TF watchdog
        self._tf_last_valid_time: Optional[float] = None
        self._tf_watchdog_paused = False
        self._tf_watchdog_paused_since: Optional[float] = None  # when we first paused due to TF

        # Phase J: Safety locks
        self._mission_start_time: Optional[float] = None
        self._nav_retry_count = 0
        # Store last dispatched box/offset for nav retry on failure
        self._last_approach_box: Optional[BoundingBox3d] = None
        self._last_approach_offset: Optional[float] = None
        self._last_tire_box: Optional[BoundingBox3d] = None
        self._last_tire_offset: Optional[float] = None
        # Detection confirmation
        self._vehicle_confirm_count = 0
        self._vehicle_confirm_center: Optional[tuple] = None
        self._tire_confirm_count = 0
        self._tire_confirm_center: Optional[tuple] = None
        # Progress stall tracking
        self._progress_stall_count = 0

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
        # Phase A: Last detection confidence for diagnostics
        self._last_detection_confidence: Optional[float] = None
        # Phase A: Detection stream health
        self._last_detection_msg_time: Optional[float] = None
        self._last_vehicle_boxes_time: Optional[float] = None
        self._detection_interval_ema: Optional[float] = None
        self._vehicle_boxes_interval_ema: Optional[float] = None
        self._last_detection_stale_log_time: Optional[float] = None
        self._last_vehicle_stale_log_time: Optional[float] = None

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
        self._face_tire_target_yaw: Optional[float] = None

        # INIT: sensor health (aurora_interface/healthy)
        self._sensor_healthy: Optional[bool] = None
        # Depth gate / nav permitted (from aurora_sdk_bridge)
        self._nav_permitted: Optional[bool] = None
        self._last_nav_permitted_time: Optional[float] = None

        # Mission report data
        self._total_tires_captured = 0
        self._mission_report = {
            "total_vehicles": 0,
            "total_tires_expected": 0,
            "total_tires_captured": 0,
            "error_states_encountered": 0,
        }

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        detection_topic = self.get_parameter("detection_topic").value
        self.detection_sub = self.create_subscription(
            BoundingBoxes3d, detection_topic, self._detection_cb, qos_profile=qos
        )
        # Optional: semantic vehicle boxes (Aurora 2.11) — vehicles from semantic, tires from detection_topic
        vehicle_boxes_topic = self.get_parameter("vehicle_boxes_topic").value
        self._vehicle_boxes_sub = None
        if vehicle_boxes_topic:
            self._vehicle_boxes_sub = self.create_subscription(
                BoundingBoxes3d, vehicle_boxes_topic, self._vehicle_boxes_cb, qos_profile=qos
            )
            self.get_logger().info(f"Using semantic vehicle detection: {vehicle_boxes_topic}")

        # Publish current FSM state for debugging.
        self.state_pub = self.create_publisher(String, "inspection_state", 10)
        # Mission report at completion
        self.mission_report_pub = self.create_publisher(String, "inspection_manager/mission_report", 10)
        # Phase A: Runtime diagnostics (JSON at 5Hz)
        self.diagnostics_pub = self.create_publisher(String, "inspection_manager/runtime_diagnostics", 10)
        
        # Publish segmentation mode to control which model the segmentation node uses
        segmentation_mode_topic = self.get_parameter("segmentation_mode_topic").value
        self.segmentation_mode_pub = self.create_publisher(String, segmentation_mode_topic, 10)
        self.get_logger().info(f"Publishing segmentation mode to: {segmentation_mode_topic}")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
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
        self.create_subscription(
            Bool,
            self.get_parameter("start_mission_topic").value,
            self._start_mission_cb,
            10,
        )
        self._capture_retry_count = 0
        self._verify_capture_start_time = None

        self.timer = self.create_timer(1.0, self._tick)
        if self.get_parameter("enable_runtime_diagnostics").value:
            self.diagnostics_timer = self.create_timer(0.2, self._publish_runtime_diagnostics)  # 5Hz
        
        use_dynamic = self.get_parameter("use_dynamic_detection").value
        if use_dynamic:
            self.get_logger().info("Using dynamic vehicle detection from bounding boxes (Aurora-based)")
        else:
            trucks = self._load_trucks()
            self.get_logger().info(f"Loaded {len(trucks)} vehicles from YAML file (legacy mode)")

        if self._is_dry_run():
            self.get_logger().info("dry_run=True: goals will be validated but not sent to Nav2; approach will simulate arrival -> WAIT_TIRE_BOX")

        # Publish initial state.
        self._set_state(MissionState.IDLE)

    def _health_cb(self, msg: Bool):
        self._sensor_healthy = msg.data

    def _nav_permitted_cb(self, msg: Bool):
        self._nav_permitted = msg.data
        self._last_nav_permitted_time = time.time()

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
        if len(self.inspected_tire_positions) >= self.get_parameter("expected_tires_per_vehicle").value:
            self.get_logger().info(f"Completed inspection of {len(self.inspected_tire_positions)} tires. Moving to next vehicle.")
            self._set_state(MissionState.NEXT_VEHICLE, cause="verify_success")
        else:
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="verify_success")

    def _finish_verify_capture_failure(self, reason: str):
        """Proceed after capture failure (retries exhausted or timeout); still advance to next tire/vehicle."""
        self.get_logger().warn(f"Verify capture failure: {reason}; proceeding to next target")
        self._mission_log_append(
            "capture_failure",
            {"reason": reason, "retry_count": self._capture_retry_count},
            sync=True,
        )
        if self._tire_registry:
            self._tire_registry[-1]["image_captured"] = False
        if len(self.inspected_tire_positions) >= self.get_parameter("expected_tires_per_vehicle").value:
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
                    world_frame, base_frame, rclpy.time.Time()
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
            detection_age = None if self._last_detection_msg_time is None else (now - self._last_detection_msg_time)
            vehicle_boxes_age = None if self._last_vehicle_boxes_time is None else (now - self._last_vehicle_boxes_time)
            detection_stale = detection_age is None or detection_age > detection_stale_s
            vehicle_boxes_stale = (
                bool(self.get_parameter("vehicle_boxes_topic").value)
                and (vehicle_boxes_age is None or vehicle_boxes_age > vehicle_boxes_stale_s)
            )
            if detection_stale and (self._last_detection_stale_log_time is None or (now - self._last_detection_stale_log_time) > 5.0):
                self._mission_log_append(
                    "detection_stream_stale",
                    {
                        "age_s": detection_age,
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
                "detection_stream_age_s": detection_age,
                "vehicle_boxes_stream_age_s": vehicle_boxes_age,
                "detection_stream_interval_ema_s": self._detection_interval_ema,
                "vehicle_boxes_stream_interval_ema_s": self._vehicle_boxes_interval_ema,
                "tf_valid": tf_valid,
                "tf_lookup_latency_ms": tf_latency_ms,
                "tf_watchdog_paused": self._tf_watchdog_paused,
                "last_transition_cause": self._last_transition_cause,
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
            }
            msg = String()
            msg.data = json.dumps(diag, default=str)
            self.diagnostics_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Diagnostics publish error: {e}")

    def _check_tf_watchdog(self) -> bool:
        """Phase C: Return False if TF invalid for > timeout; mission should pause.
        After tf_unavailable_abort_s seconds of continuous failure, transition to ERROR."""
        world_frame = self.get_parameter("world_frame").value
        base_frame = self.get_parameter("base_frame").value
        timeout = self.get_parameter("tf_watchdog_timeout").value
        abort_s = self.get_parameter("tf_unavailable_abort_s").value
        try:
            self.tf_buffer.lookup_transform(world_frame, base_frame, rclpy.time.Time())
            self._tf_last_valid_time = time.time()
            self._tf_watchdog_paused = False
            self._tf_watchdog_paused_since = None
            self._tf_recently_valid = True
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
        self._mission_report["mission_end_cause"] = self._last_transition_cause
        expected = self._mission_report["total_tires_expected"]
        captured = self._mission_report["total_tires_captured"]
        missing = max(0, expected - captured)
        total_time = (time.time() - self._mission_start_time) if self._mission_start_time else 0
        success_flag = expected == captured and self._mission_report["error_states_encountered"] == 0
        report = (
            f"MISSION_REPORT\n"
            f"vehicles_detected={self._mission_report['total_vehicles']}\n"
            f"expected_tires={expected}\n"
            f"tires_completed={captured}\n"
            f"missing_tires={missing}\n"
            f"success_flag={str(success_flag).upper()}\n"
            f"total_time={total_time:.1f}\n"
            f"error_states_encountered={self._mission_report['error_states_encountered']}\n"
        )
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
                    "missing_tires": missing,
                    "success": success_flag,
                    "total_time_s": total_time,
                    "error_states_encountered": self._mission_report["error_states_encountered"],
                    "mission_end_cause": self._mission_report.get("mission_end_cause"),
                }
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
        """On mission start: archive existing mission_latest.* to timestamped files so each run has a clear log."""
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

        sent = send_nav_goal(self, goal, self._on_patrol_done)
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

    def _dispatch_planned_tire_goal(self, target_pos: tuple) -> bool:
        """Fallback: dispatch a goal to a planned tire position when detection is missing."""
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
        map_frame = self.get_parameter("map_frame").value
        if self.get_parameter("require_goal_transform").value:
            try:
                self.tf_buffer.lookup_transform(map_frame, self.get_parameter("world_frame").value, rclpy.time.Time())
            except Exception:
                self.get_logger().warn("Planned tire: required map transform unavailable; cannot dispatch.")
                self._mission_log_append(
                    "planned_tire_dispatch_blocked",
                    {"reason": "map_transform_unavailable", "frame": map_frame},
                    sync=True,
                )
                return False
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
        self._last_tire_offset = self.get_parameter("tire_offset").value
        self._mission_log_append(
            "tire_planned_fallback",
            {"position": target_pos, "extent": extent},
            sync=True,
        )
        return self._dispatch_box_goal(box, offset=self._last_tire_offset)

    def _sync_planned_tires_for_current_vehicle(self) -> None:
        """Load planned tires from current vehicle data into active list."""
        if self.current_vehicle_idx < len(self.detected_vehicles):
            planned = self.detected_vehicles[self.current_vehicle_idx].get("planned_tires", [])
            self._planned_tire_positions = list(planned) if planned else []
        else:
            self._planned_tire_positions = []

    def _save_vehicle_position(self, box: BoundingBox3d):
        """Save detected vehicle position to detected_vehicles list and publish vehicle_detected anchor (map frame)."""
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        center_z = (box.zmin + box.zmax) / 2.0
        
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
        # Plan tire positions for this vehicle (used to prefer closer detections)
        robot_pose = self._get_current_pose()
        robot_pos = (
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
            robot_pose.pose.position.z,
        ) if robot_pose else None
        wheelbase = float(self.get_parameter("vehicle_wheelbase_m").value)
        track = float(self.get_parameter("vehicle_track_m").value)
        planned = estimate_tire_positions(
            (center_x, center_y, center_z),
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
                    "wheelbase_m": wheelbase,
                    "track_m": track,
                },
                sync=True,
            )
        self.get_logger().info(
            f"Saved vehicle position: {box.object_name} at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f}) "
            f"(probability: {box.probability:.3f}). Total vehicles: {len(self.detected_vehicles)}"
        )

    # ----------------------- State Machine ----------------------- #
    def _tick(self):
        if self.current_state in (MissionState.DONE, MissionState.ERROR):
            return

        # Mission heartbeat: every 10s log state and key flags for forensics
        now = time.time()
        if self._mission_start_time is not None and (self._mission_heartbeat_last_time is None or (now - self._mission_heartbeat_last_time) >= 10.0):
            self._mission_heartbeat_last_time = now
            tf_valid = False
            try:
                wf = self.get_parameter("world_frame").value
                bf = self.get_parameter("base_frame").value
                self.tf_buffer.lookup_transform(wf, bf, rclpy.time.Time())
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
            # Require TF (slamware_map->base_link) valid and stable before starting so we don't enter SEARCH_VEHICLE then immediately pause
            world_frame = self.get_parameter("world_frame").value
            base_frame = self.get_parameter("base_frame").value
            map_frame = self.get_parameter("map_frame").value
            tf_wait_timeout = self.get_parameter("tf_wait_timeout").value
            tf_stable_s = self.get_parameter("tf_stable_s").value
            now_tf = time.time()
            try:
                self.tf_buffer.lookup_transform(world_frame, base_frame, rclpy.time.Time())
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
                    self.tf_buffer.lookup_transform(map_frame, world_frame, rclpy.time.Time())
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
            self._mission_log_append(
                "idle_passed",
                {"nav2_ok": True, "tf_ok": True, "tf_stable_s": tf_stable_s},
                sync=True,
            )
            if use_dynamic:
                self._rotate_mission_logs()
                self._mission_start_time = time.time()
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
            # In dynamic mode, wait for vehicle detection to populate detected_vehicles list
            # Once we have vehicles, move to first vehicle
            if len(self.detected_vehicles) > 0:
                # Find first un-inspected vehicle
                un_inspected = [v for v in self.detected_vehicles if not v["inspected"]]
                if un_inspected:
                    self.current_vehicle_idx = self.detected_vehicles.index(un_inspected[0])
                    self.current_vehicle_box = un_inspected[0]["box"]
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

        if self.current_state == MissionState.PATROL_SEARCH:
            return  # waiting on patrol nav to complete

        if self.current_state == MissionState.WAIT_VEHICLE_BOX:
            # Whenever we have current_vehicle_box, dispatch approach on first tick (stops spin loop; works after SEARCH, NEXT_VEHICLE, or TURN_IN_PLACE_VEHICLE)
            if not self._dispatched_approach_this_wait and self.current_vehicle_box is not None:
                offset = self.get_parameter("approach_offset").value
                self._last_approach_box = self.current_vehicle_box
                self._last_approach_offset = offset
                log_bounding_box(self.get_logger(), self.current_vehicle_box, f"VEHICLE_{self.current_vehicle_box.object_name.upper()}")
                self.get_logger().info(
                    f"Vehicle box known; dispatching approach to vehicle (no wait for next detection)."
                )
                sent = self._dispatch_box_goal(self.current_vehicle_box, offset=offset)
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
                    if self._is_dry_run():
                        self.get_logger().info("Dry run: simulating approach success, transitioning to WAIT_TIRE_BOX")
                        self._simulate_approach_success()
                    else:
                        self._set_state(MissionState.APPROACH_VEHICLE, cause="vehicle_box_known")
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
                        self._set_state(MissionState.NEXT_VEHICLE)
            return  # waiting on detections

        if self.current_state == MissionState.TURN_IN_PLACE_VEHICLE:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.APPROACH_VEHICLE:
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
                    self._set_state(MissionState.WAIT_VEHICLE_BOX, cause="approach_timeout")
            return  # waiting on navigation to complete

        if self.current_state == MissionState.WAIT_TIRE_BOX:
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
                            self.get_logger().warn(
                                f"No tire boxes after {elapsed:.1f}s. Using planned tire fallback: {target}."
                            )
                            if self._dispatch_planned_tire_goal(target):
                                self._set_state(MissionState.INSPECT_TIRE, cause="planned_tire_fallback")
                                return
                        # Check if we have enough tires inspected (typically 4 per vehicle)
                        inspected_count = len(self.inspected_tire_positions)
                        self.get_logger().warn(
                            f"Max rotation attempts reached. Inspected {inspected_count} tires. "
                            f"Moving to next vehicle."
                        )
                        self._set_state(MissionState.NEXT_VEHICLE)
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
                self.current_vehicle_idx = self.detected_vehicles.index(un_inspected[0])
                self.current_vehicle_box = un_inspected[0]["box"]
                self.current_tire_idx = 0
                self.inspected_tire_positions = []  # Reset for next vehicle
                self._sync_planned_tires_for_current_vehicle()
                self.rotation_attempts = 0  # Reset rotation attempts
                self.get_logger().info(
                    f"Moving to vehicle {self.current_vehicle_idx + 1}/{len(self.detected_vehicles)} "
                    f"({un_inspected[0]['class']})."
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
        """Semantic vehicle boxes (Aurora 2.11). Vehicles from this topic; tires from detection_topic."""
        now = time.time()
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
                f"Received {n} vehicle box(es) from semantic topic: {[b.object_name for b in msg.bounding_boxes]}",
                throttle_duration_sec=2.0,
            )
        self._process_vehicle_boxes(msg.bounding_boxes)

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
                if use_dynamic:
                    self._save_vehicle_position(vehicle_box)
                self._last_detection_confidence = vehicle_box.probability
                self.get_logger().info(
                    f"Vehicle detected ({vehicle_box.object_name}, prob: {vehicle_box.probability:.3f}); approaching."
                )
                self._last_approach_box = vehicle_box
                self._last_approach_offset = self.get_parameter("approach_offset").value
                sent = self._dispatch_box_goal(vehicle_box, offset=self._last_approach_offset)
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

    def _detection_cb(self, msg: BoundingBoxes3d):
        now = time.time()
        if self._last_detection_msg_time is not None:
            interval = now - self._last_detection_msg_time
            if interval >= 0:
                if self._detection_interval_ema is None:
                    self._detection_interval_ema = interval
                else:
                    self._detection_interval_ema = (0.2 * interval) + (0.8 * self._detection_interval_ema)
        self._last_detection_msg_time = now
        vehicle_boxes_topic = self.get_parameter("vehicle_boxes_topic").value
        vehicle_labels = parse_vehicle_labels(self.get_parameter("vehicle_labels").value)
        tire_label = self.get_parameter("tire_label").value
        min_vehicle_prob = self.get_parameter("min_vehicle_probability").value
        min_tire_prob = self.get_parameter("min_tire_probability").value

        # Vehicle logic: from vehicle_boxes_topic (semantic) or detection_topic (YOLO)
        if not vehicle_boxes_topic:
            self._process_vehicle_boxes(msg.bounding_boxes)

        if self.current_state == MissionState.SEARCH_VEHICLE:
            return

        # Handle tire detection state (always from detection_topic)
        if self.current_state == MissionState.WAIT_TIRE_BOX:
            robot_pose = self._get_current_pose()
            robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z) if robot_pose else None
            vehicle_pos = None
            if self.current_vehicle_box:
                vehicle_pos = (
                    (self.current_vehicle_box.xmin + self.current_vehicle_box.xmax) / 2.0,
                    (self.current_vehicle_box.ymin + self.current_vehicle_box.ymax) / 2.0,
                    (self.current_vehicle_box.zmin + self.current_vehicle_box.zmax) / 2.0
                )
            elif self.current_vehicle_idx < len(self.detected_vehicles):
                vehicle_pos = self.detected_vehicles[self.current_vehicle_idx]["position"]
            target_pos = self._planned_tire_positions[0] if self._planned_tire_positions else None
            tire_box = find_tire_for_inspection(
                msg.bounding_boxes,
                tire_label,
                min_tire_prob,
                self.inspected_tire_positions,
                self.get_parameter("tire_position_tolerance").value,
                vehicle_pos,
                self.get_parameter("max_tire_distance_from_vehicle").value,
                robot_pos,
                target_pos,
                self.get_logger(),
            )
            if tire_box:
                # Log bounding box details
                tire_num = len(self.inspected_tire_positions) + 1
                log_bounding_box(self.get_logger(), tire_box, f"TIRE_{tire_num}")
                tire_center = (
                    (tire_box.xmin + tire_box.xmax) / 2.0,
                    (tire_box.ymin + tire_box.ymax) / 2.0,
                    (tire_box.zmin + tire_box.zmax) / 2.0
                )
                if not self._confirm_tire_box(tire_center):
                    self._mission_log_append(
                        "tire_confirmation_pending",
                        {"state": self.current_state, "required": self.get_parameter("tire_confirmations_required").value},
                        sync=True,
                    )
                    return
                self.get_logger().info(
                    f"Tire {tire_num} detected at ({tire_center[0]:.2f}, {tire_center[1]:.2f}, {tire_center[2]:.2f}); "
                    f"moving to inspect. Already inspected: {len(self.inspected_tire_positions)}"
                )
                # Store tire position BEFORE navigating to avoid detecting it again while navigating
                self._last_detection_confidence = tire_box.probability
                self.inspected_tire_positions.append(tire_center)
                # Vehicle center and robot pos for FL/FR/RL/RR label
                vehicle_center = None
                if self.current_vehicle_box:
                    vehicle_center = (
                        (self.current_vehicle_box.xmin + self.current_vehicle_box.xmax) / 2.0,
                        (self.current_vehicle_box.ymin + self.current_vehicle_box.ymax) / 2.0,
                        (self.current_vehicle_box.zmin + self.current_vehicle_box.zmax) / 2.0,
                    )
                tire_pos_label = tire_position_label(tire_center, vehicle_center, robot_pos)
                # Phase F: register tire with unique ID and position label
                self._tire_id_counter += 1
                self._tire_registry.append({
                    "id": self._tire_id_counter,
                    "vehicle_id": self.current_vehicle_idx,
                    "position": tire_center,
                    "tire_position": tire_pos_label,
                    "visited": False,
                    "image_captured": False,
                })
                self._last_tire_box = tire_box
                self._last_tire_offset = self.get_parameter("tire_offset").value
                self._mission_log_append(
                    "tire_approach",
                    {
                        "tire_id": self._tire_id_counter,
                        "vehicle_id": self.current_vehicle_idx,
                        "position": tire_center,
                        "tire_position": tire_pos_label,
                    },
                    sync=True,
                )
                # Remove planned target closest to this tire (if available)
                if self._planned_tire_positions:
                    tire_xy = (tire_center[0], tire_center[1])
                    def _plan_dist_sq(p):
                        return (p[0] - tire_xy[0]) ** 2 + (p[1] - tire_xy[1]) ** 2
                    closest_idx = min(range(len(self._planned_tire_positions)), key=lambda i: _plan_dist_sq(self._planned_tire_positions[i]))
                    self._planned_tire_positions.pop(closest_idx)
                    if self.current_vehicle_idx < len(self.detected_vehicles):
                        self.detected_vehicles[self.current_vehicle_idx]["planned_tires"] = list(self._planned_tire_positions)
                self._dispatch_box_goal(tire_box, offset=self._last_tire_offset)
                self._set_state(MissionState.INSPECT_TIRE, cause="tire_detected")
            else:
                # Log why no tire was selected
                all_tires = [b for b in msg.bounding_boxes 
                            if b.object_name.lower() == tire_label.lower() 
                            and b.probability >= min_tire_prob]
                if all_tires:
                    self.get_logger().debug(
                        f"Found {len(all_tires)} tire(s) but none selected. "
                        f"Already inspected: {len(self.inspected_tire_positions)}"
                    )
            return

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
        send_nav_goal(self, goal_pose, self._on_standoff_done)
        # Note: Legacy mode uses old state names - this is for backward compatibility only

    def _dispatch_box_goal(self, box: BoundingBox3d, offset: float) -> bool:
        """
        Dispatch navigation goal to approach detected object.
        CRITICAL: Bounding box coordinates are in slamware_map frame (from segmentation_processor).
        Robot pose must be looked up in the SAME frame.
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
        goal_info = compute_box_goal(self, box, offset)
        if goal_info is None:
            world_frame = self.get_parameter("world_frame").value
            self.get_logger().error(
                f"Cannot get robot pose in {world_frame} frame! Cannot compute goal."
            )
            self._last_dispatch_fail_reason = "robot_pose_or_transform_unavailable"
            self._dispatch_fail_count += 1
            return False

        goal = goal_info["goal"]
        heading = goal_info["heading"]
        dist = goal_info["distance_to_object"]
        center_x, center_y, center_z = goal_info["center"]
        robot_x, robot_y, robot_z = goal_info["robot_pos"]
        world_frame = goal_info["world_frame"]
        goal_world = goal_info["goal_world"]
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
                "goal_world": {"x": goal_world[0], "y": goal_world[1], "z": goal_world[2]},
            },
            sync=True,
        )

        # Phase D: Goal validation - distance check
        goal_dist = math.sqrt((goal.pose.position.x - center_x)**2 + (goal.pose.position.y - center_y)**2)
        if abs(goal_dist - offset) > 0.1:
            self.get_logger().warn(f"Goal validation: distance to object {goal_dist:.3f}m vs offset {offset:.3f}m")

        if self._is_dry_run():
            self.get_logger().info("Dry run: goal validated, not sending to Nav2")
            self._last_dispatch_fail_reason = None
            return True  # caller still transitions state in dry run

        self._current_goal_pose = goal
        ok = send_nav_goal(self, goal, self._on_box_goal_done)
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
        return send_nav_goal(self, pose, done_cb)

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

    def _publish_capture_metadata(self, tire_class: str, tire_position: str = ""):
        """Publish JSON metadata for the next photo capture (map pose, timestamp, tire_class, vehicle_id, vehicle_anchor)."""
        pose = self._get_current_pose_in_map_frame()
        if pose is None:
            self.get_logger().warn("Cannot get pose in map frame for capture metadata; publishing minimal metadata.")
        now = self.get_clock().now()
        meta = {
            "frame_id": pose.header.frame_id if pose else "map",
            "x": float(pose.pose.position.x) if pose else 0.0,
            "y": float(pose.pose.position.y) if pose else 0.0,
            "z": float(pose.pose.position.z) if pose else 0.0,
            "yaw": float(yaw_from_quaternion(pose.pose.orientation)) if pose else 0.0,
            "tire_class": tire_class,
            "tire_position": tire_position,
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
            if send_nav_goal(self, goal_pose, self._on_rotation_done_search):
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
            if send_nav_goal(self, goal_pose, self._on_rotation_done_vehicle):
                self._mission_log_append(
                    "rotation_dispatched",
                    {"goal_type": goal_type, "rotation_attempts": self.rotation_attempts},
                    sync=True,
                )
                self._set_state(MissionState.TURN_IN_PLACE_VEHICLE)
            else:
                self._set_state(MissionState.NEXT_VEHICLE)
        else:
            if send_nav_goal(self, goal_pose, self._on_rotation_done_tire):
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

    def _on_box_goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Box goal rejected by Nav2; returning to WAIT so we can retry.")
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
        nav_succeeded = status == 4  # SUCCEEDED
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
            },
            sync=True,
        )

        if not nav_succeeded:
            self._nav_retry_count += 1
            budget = self.get_parameter("nav_retry_budget").value
            self._progress_stall_count += 1
            if self.current_state == MissionState.APPROACH_VEHICLE:
                if self._nav_retry_count <= budget and self._last_approach_box is not None:
                    self.get_logger().warn(
                        f"Approach nav failed; retry {self._nav_retry_count}/{int(budget)}."
                    )
                    self._current_goal_pose = None
                    self._dispatch_box_goal(self._last_approach_box, offset=self._last_approach_offset)
                    return
                self.get_logger().error(
                    f"Approach nav failed after {self._nav_retry_count} attempts; moving to next vehicle."
                )
                self._nav_retry_count = 0
                self._last_approach_box = None
                self._last_approach_offset = None
                self._current_goal_pose = None
                self._set_state(MissionState.NEXT_VEHICLE, cause="nav_failed_approach")
                return
            if self.current_state == MissionState.INSPECT_TIRE:
                if self._nav_retry_count <= budget and self._last_tire_box is not None:
                    self.get_logger().warn(
                        f"Tire nav failed; retry {self._nav_retry_count}/{int(budget)}."
                    )
                    self._current_goal_pose = None
                    self._dispatch_box_goal(self._last_tire_box, offset=self._last_tire_offset)
                    return
                self.get_logger().error(
                    f"Tire nav failed after {self._nav_retry_count} attempts; skipping to next tire."
                )
                self._nav_retry_count = 0
                self._last_tire_box = None
                self._last_tire_offset = None
                if self._tire_registry:
                    self._tire_registry[-1]["visited"] = True
                    self._tire_registry[-1]["image_captured"] = False
                self._current_goal_pose = None
                self._set_state(MissionState.WAIT_TIRE_BOX, cause="nav_failed_tire")
                return

        self._nav_retry_count = 0
        self._progress_stall_count = 0

        if self.current_state == MissionState.APPROACH_VEHICLE:
            self._current_goal_pose = None
            self.current_tire_idx = 0
            self.inspected_tire_positions = []
            self.rotation_attempts = 0
            self._sync_planned_tires_for_current_vehicle()
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="nav_arrived_vehicle")
            return

        if self.current_state == MissionState.INSPECT_TIRE:
            if self.get_parameter("face_tire_final_yaw").value:
                outcome = self._dispatch_face_tire_goal()
                if outcome == "sent":
                    self._set_state(MissionState.FACE_TIRE, cause="face_tire_rotation")
                elif outcome == "failed":
                    self._set_state(MissionState.WAIT_TIRE_BOX, cause="face_tire_dispatch_failed")
                return
            self._trigger_tire_capture()
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
        if self._is_dry_run():
            self.get_logger().info("Dry run: face_tire rotation validated, not sending to Nav2")
            self._trigger_tire_capture()
            return "capture"
        if send_nav_goal(self, goal, self._on_face_tire_done):
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
        self._mission_log_append(
            "photo_trigger_distance_check",
            {
                "distance_to_goal": distance_to_goal,
                "threshold": trigger_threshold,
                "state": self.current_state,
            },
            sync=True,
        )
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
        if self._tire_registry:
            self._tire_registry[-1]["visited"] = True
        self._current_goal_pose = None
        tire_class = self.get_parameter("tire_label").value
        tire_position = (self._tire_registry[-1].get("tire_position", "")) if self._tire_registry else ""
        self._publish_capture_metadata(tire_class, tire_position)
        self._total_tires_captured += 1
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
        self._face_tire_start_time = None
        self._face_tire_target_yaw = None
        if not nav_succeeded:
            self.get_logger().warn("Face tire rotation failed; returning to WAIT_TIRE_BOX.")
            self._set_state(MissionState.WAIT_TIRE_BOX, cause="face_tire_failed")
            return
        self._trigger_tire_capture()

    def _on_rotation_done_vehicle(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Rotation goal rejected.")
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
            self._mission_log_append("nav_rejected", {"goal": "rotation_tire"}, sync=True)
            # Check if we have enough tires
            if len(self.inspected_tire_positions) >= self.get_parameter("expected_tires_per_vehicle").value:
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
        # Return to waiting for tire box
        self._set_state(MissionState.WAIT_TIRE_BOX)

    def _on_patrol_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Patrol goal rejected; returning to SEARCH_VEHICLE.")
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

