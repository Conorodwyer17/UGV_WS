# Mission Determinism Audit — Yes/No Checklist

Guarantees that the autonomous tyre inspection mission is deterministic and failure-resistant. Each item must be verifiable.

---

## 1. Tyre Visit Verification

| # | Criterion | Pass | Notes |
|---|-----------|------|-------|
| 1.1 | Every tyre visit is explicitly recorded (e.g. mission_report, photo metadata) | ☑ | tire_inspection_photos/*.json; mission_report tires_captured |
| 1.2 | Tyre order is deterministic (2nd→3rd→4th nearest) | ☑ | strict_planned_tire_order |
| 1.3 | Skipped tyres are logged with reason (unreachable, timeout) | ☑ | _tires_skipped; tires_skipped in mission_report; skip_tire_on_nav_failure |
| 1.4 | Total tyres expected vs captured is reported at mission end | ☑ | total_tires_expected, total_tires_captured, tires_skipped in mission_report |

---

## 2. Goal Success Verification

| # | Criterion | Pass | Notes |
|---|-----------|------|-------|
| 2.1 | Goal success cannot silently occur without proximity check | ☑ | goal_checker stateful; Nav2 result |
| 2.2 | Proximity to goal is verified before photo trigger | ☑ | xy_goal_tolerance, capture_require_wheel_detection |
| 2.3 | Failed goals are not counted as success | ☑ | Nav2 FAILED/ABORTED; no photo; skip_tire_on_nav_failure |
| 2.4 | Goal pose is validated (finite, in map bounds) before send | ☑ | _is_finite_box, check_tf_validity |

---

## 3. Navigation–Photo Race Conditions

| # | Criterion | Pass | Notes |
|---|-----------|------|-------|
| 3.1 | No race between navigation result and photo capture | ☑ | processAtWaypoint only after SUCCEEDED; sequential flow |
| 3.2 | Photo capture is triggered only when at waypoint | ☑ | goal reached; WAIT_WHEEL_FOR_CAPTURE → VERIFY_CAPTURE |
| 3.3 | No blocking wait in detection callback during capture | ☑ | Async capture; no spin in callback |
| 3.4 | Capture service timeout prevents indefinite wait | ☑ | capture_verify_timeout_s |

---

## 4. Callback and Thread Safety

| # | Criterion | Pass | Notes |
|---|-----------|------|-------|
| 4.1 | No blocking operations in ROS callbacks | ☑ | inspection_manager: single-threaded; no spin in callbacks; segmentation heavy but bounded |
| 4.2 | No unbounded memory allocation in hot loops | ☑ | PCL clouds bounded by object count; no unbounded vector growth |
| 4.3 | TF lookup uses timeout (no infinite wait) | ☑ | transformer.py timeout_s=0.2; segmentation canTransform 5s; lookupTransform bounded |
| 4.4 | Mutex used where shared state is updated from multiple threads | ☑ | inspection_manager single-threaded executor; no concurrent writers; segmentation single callback |

---

## 5. Transform Validation

| # | Criterion | Pass | Notes |
|---|-----------|------|-------|
| 5.1 | All transforms validated before use | ☑ | check_tf_validity, canTransform |
| 5.2 | Detection stamp age checked before goal creation | ☑ | detection_stamp_max_age_s |
| 5.3 | TF tree (map→odom→base_link) verified at startup | ☑ | tf2_tools view_frames; manual |
| 5.4 | Frame IDs consistent (slamware_map vs map) | ☑ | Static identity transform; transform_tolerance 0.5 |

---

## 6. Topic and Action Validation

| # | Criterion | Pass | Notes |
|---|-----------|------|-------|
| 6.1 | All required topics validated at startup | ☑ | detection_topic, vehicle_boxes_topic |
| 6.2 | Action servers (navigate_to_pose, follow_waypoints) waited for | ☑ | wait_for_server; follow_waypoints when use_follow_waypoints |
| 6.3 | Action result (SUCCEEDED/FAILED/ABORTED) always checked | ☑ | resultCallback, future.result(); _on_box_result, _on_follow_waypoints_result |
| 6.4 | Empty waypoint list rejected before send | ☑ | send_follow_waypoints checks poses empty |

---

## 7. Mission Logic Determinism

| # | Criterion | Pass | Notes |
|---|-----------|------|-------|
| 7.1 | State machine has no unbounded loops | ☑ | max_state_repeats → ERROR |
| 7.2 | Timeouts exist for every wait state | ☑ | detection_timeout, approach_timeout_s, capture_verify_timeout_s, etc. |
| 7.3 | Skip-to-next logic on failure (not abort) | ☑ | stop_on_failure: false; skip_tire_on_nav_failure; _tires_skipped |
| 7.4 | Vehicle re-detection strategy defined | ☑ | TURN_IN_PLACE, PATROL_SEARCH |

---

## 8. External Evidence (from Research Repos)

| Source | Pattern | Audit Item |
|--------|---------|------------|
| nav2_waypoint_follower | processAtWaypoint only on SUCCEEDED | 3.1 |
| nav2_waypoint_follower | missed_waypoints in result | 1.3 |
| goal_generator | _is_finite_box, check_tf_validity | 2.4, 5.1 |
| l2i_fusion_detection | mutex for bbox updates | 4.4 |
| gb_visual_detection_3d | TF lookup with timeout | 4.3 |

---

## Verification Commands

```bash
# TF tree
ros2 run tf2_tools view_frames

# Topic rates
ros2 topic hz /darknet_ros_3d/vehicle_bounding_boxes
ros2 topic hz /slamware_ros_sdk_server_node/odom

# Action server
ros2 action list
ros2 action info /follow_waypoints
```
