# System Re-Audit Report — Autonomous Tyre Inspection

Full architectural audit of the current system. Risk classification per subsystem: Safe | Probably Safe | Fragile | Non-deterministic | Hidden Failure Risk.

---

## Executive Summary

The system has been hardened with transform_tolerance 0.5 s, origin_x/y on local costmap, skip-on-failure logic, and mission report determinism. Remaining gaps: no goal-in-costmap precheck, callback/threading audit incomplete, no pose re-validation before photo capture, and limited observability metrics.

---

## 1. Nav2 Configuration

| Component | Current State | Risk | Notes |
|-----------|---------------|------|-------|
| transform_tolerance | 0.5 s (controller, behavior, recoveries) | **Probably Safe** | Aligned with PATH_FORWARD; costmaps may inherit |
| local_costmap global_frame | odom | **Safe** | Nav2 #4299; rolling window correct |
| local_costmap origin_x/y | 2.5, 2.5 | **Safe** | ROS Answers 416578 |
| inflation_radius | 0.6 local, 0.35 global | **Safe** | IEEE 10606581; above robot half-width |
| obstacle_layer | No voxel on local | **Safe** | Avoids sensor origin out of bounds |
| controller_frequency | 20 Hz | **Probably Safe** | 10 Hz recommended when CPU contended |
| goal_checker stateful | True | **Safe** | Prevents false success |
| xy/yaw_goal_tolerance | 0.25 | **Safe** | Coarse staging; reduces oscillation |
| waypoint_follower stop_on_failure | false | **Safe** | Skip unreachable tyre |

**Costmap boundaries:** Local 5×5 m, resolution 0.05; robot footprint 253×243 mm. Sensor origin centered. **Probably Safe** — no runtime validation of sensor origin vs bounds.

---

## 2. Transform Tree and Temporal Guarantees

| Check | Implementation | Risk | Notes |
|-------|----------------|------|-------|
| TF lookup timeout | transformer.py: timeout_s=0.2 | **Probably Safe** | Bounded; 0.2 s may be tight under load |
| can_transform before goal | check_tf_validity | **Safe** | goal_generator rejects if TF invalid |
| detection_stamp_max_age_s | 0.5 s | **Safe** | Rejects stale detections |
| TF watchdog | tf_watchdog_timeout 0.2 s | **Safe** | Cancels goal on TF loss |
| Future timestamp handling | None | **Fragile** | No explicit fallback to now-epsilon |
| map→slamware_map identity | Static publisher | **Safe** | Aurora map frame |

**Hidden Failure Risk:** Aurora may publish TF with future timestamps; Nav2 transform_tolerance 0.5 s helps but no programmatic fallback exists in inspection_manager.

---

## 3. Perception Pipeline

| Component | Current State | Risk | Notes |
|-----------|---------------|------|-------|
| aurora_semantic_fusion | Median depth over mask | **Safe** | Robust to centroid noise |
| segmentation_processor | VoxelGrid + Euclidean | **Safe** | Best practice flow |
| voxel_leaf_size | Configurable (0 = adaptive) | **Safe** | 0.02–0.05 for vehicles |
| Passthrough | z in [-5, 2] | **Probably Safe** | No range filter for >10 m |
| segmentationCallback | Synchronous; heavy compute | **Fragile** | Blocks executor; no callback group |
| Memory allocation | PCL clouds per object | **Probably Safe** | Bounded by object count |
| tf_max_age_ms | 250 ms | **Safe** | Rejects old transforms |
| Cluster size | min 1 (wheel), 50 (vehicle) | **Safe** | Relaxed for sparse depth |

**Fragile:** segmentationCallback does full clustering in callback; can block other callbacks. No temporal smoothing on bounding boxes.

---

## 4. Goal Generation and Filtering

| Check | Implementation | Risk | Notes |
|-------|----------------|------|-------|
| _is_finite_box | goal_generator | **Safe** | Rejects NaN, degenerate |
| check_tf_validity | transformer | **Safe** | Before goal creation |
| Far-side placement | vehicle_center → tire_center | **Safe** | Avoids goal inside vehicle |
| MIN_SAFE_OFFSET | 0.5 vehicle, 0.25 tire | **Safe** | Prevents goal in obstacle |
| Goal-in-costmap check | None | **Hidden Failure Risk** | No query to costmap; goal could be in occupied cell |
| nav_goal_min_interval_s | 1.0 s | **Safe** | Prevents flooding |
| goal-in-flight guard | send_nav_goal | **Safe** | One goal at a time |

---

## 5. Mission State Machine and Concurrency

| Check | Implementation | Risk | Notes |
|-------|----------------|------|-------|
| max_state_repeats | 3 → ERROR | **Safe** | Spin protection |
| approach_timeout_s | 120 s | **Safe** | Cancel and retry |
| skip_tire_on_nav_failure | true | **Safe** | Skip unreachable |
| Executor | Default (SingleThreaded) | **Probably Safe** | Python rclpy.spin |
| Callback blocking | segmentationCallback heavy | **Fragile** | Can delay _tick |
| Mutex for shared state | None in inspection_manager | **Probably Safe** | Single-threaded; action callbacks on same executor |
| _tires_skipped, _mission_report | List/dict mutation | **Probably Safe** | No concurrent writers |

---

## 6. Photo Capture and Arrival Verification

| Check | Implementation | Risk | Notes |
|-------|----------------|------|-------|
| Photo trigger | After goal SUCCEEDED | **Safe** | No race |
| capture_require_wheel_detection | Optional gate | **Safe** | Wheel in view |
| capture_verify_timeout_s | Timeout | **Safe** | Prevents indefinite wait |
| Pose re-validation before photo | should_trigger_photo + _distance_to_current_goal | **Safe** | Blocks capture if distance > photo_trigger_distance |
| Metadata with pose | JSON sidecar | **Safe** | Records visit |

---

## 7. Parameter Defaults for Production

| Parameter | Default | Risk | Notes |
|-----------|---------|------|-------|
| detection_stamp_max_age_s | 0.5 | **Safe** | Strict |
| approach_timeout_s | 120 | **Safe** | Adequate |
| nav_retry_budget | 3 | **Safe** | Retry limit |
| tf_watchdog_timeout | 0.2 | **Probably Safe** | May need 0.3 under load |
| controller_frequency | 20 | **Probably Safe** | Consider 10 on Jetson |

---

## 8. Risk Summary by Subsystem

| Subsystem | Classification | Primary Concern |
|-----------|----------------|-----------------|
| Nav2 config | Probably Safe | Controller 20 Hz under CPU load |
| Costmap | Probably Safe | No runtime sensor-origin validation |
| TF | Fragile | No future-timestamp fallback |
| Perception | Fragile | segmentationCallback blocks |
| Goal generation | Hidden Failure Risk | No costmap precheck |
| Mission logic | Probably Safe | Single-threaded; timeouts present |
| Photo capture | Safe | Pose re-validation via should_trigger_photo |
| Parameters | Probably Safe | Production defaults reasonable |

---

## 9. Recommended Actions (Priority Order)

1. **High:** Add pose re-validation before photo trigger (distance to goal < tolerance).
2. **High:** Add goal-in-costmap-free-space precheck (optional; Nav2 replans but adds safety).
3. **Medium:** Move segmentation heavy compute to dedicated callback group or thread.
4. **Medium:** Add TF lookup fallback with now-epsilon when stamp fails.
5. **Low:** Add runtime costmap bounds validation script.
6. **Low:** Consider controller_frequency 10 Hz in PRODUCTION_CONFIG for Jetson.
