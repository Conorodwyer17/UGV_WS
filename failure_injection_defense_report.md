# Failure Mode Injection Defense Report

Simulated failure conditions and defenses. For each: expected behavior, current behavior, patches, logging, guarding, timeout.

---

## 1. TF Delays

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Reject goal or retry after bounded wait | TF lookup timeout 0.2 s; returns None | **Defended** |
| Logging | Log TF failure with frame pair | transformer returns None; caller logs | **Present** |
| Guarding | Do not send goal without valid TF | check_tf_validity; goal_generator returns failure_code | **Present** |
| Timeout | Bounded wait | timeout_s=0.2 in lookup_transform, check_tf_validity | **Present** |

**Patch:** Increase tf_watchdog_timeout to 0.3 s in PRODUCTION_CONFIG if TF latency > 200 ms observed. No code change required.

---

## 2. TF Future Timestamps

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Fallback to now or now-epsilon; avoid extrapolation exception | tf2 throws on future; lookup returns None | **Partial** |
| Logging | Log "TF future stamp" | Exception caught; no specific message | **Missing** |
| Guarding | Retry with latest available | None | **Missing** |
| Timeout | Bounded | Yes | **Present** |

**Patch:** In `transformer.py`, add fallback: if lookup at stamp fails with extrapolation, retry with `rclpy.time.Time()` (latest). Log `TF_future_stamp_fallback` when used.

---

## 3. Depth Image Dropout for 300 ms

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Use last valid pointcloud; no crash | segmentation_processor uses latest_pointcloud_ | **Defended** |
| Logging | Log dropout duration | "No PointCloud2 message available" | **Present** |
| Guarding | Skip processing if no cloud | Early return in segmentationCallback | **Present** |
| Timeout | N/A | N/A | N/A |

**Patch:** Add `pointcloud_max_age_s` param; reject if cloud stamp older than threshold. Default 0.5 s. Log `pointcloud_stale` when rejected.

---

## 4. YOLO False Positives

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Filter by min_valid_points; reject low-confidence | min_valid_points (5), min_valid_points_wheel (3); minimum_probability 0.3 | **Defended** |
| Logging | Log filtered count | Per-object filtering; no aggregate | **Partial** |
| Guarding | Reject degenerate boxes | _is_finite_box, box validity | **Present** |
| Timeout | N/A | N/A | N/A |

**Patch:** Add optional `max_objects_per_class` to cap processing; log `yolo_filtered_count` in segmentation_processor. Low priority.

---

## 5. Partial Occlusion of One Tire

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Skip or retry; do not block mission | skip_tire_on_nav_failure; min_valid_points_wheel=3 | **Defended** |
| Logging | Log tire skipped with reason | _tires_skipped; tires_skipped in mission_report | **Present** |
| Guarding | Reject if points < min | segmentation_processor filters | **Present** |
| Timeout | approach_timeout_s | 120 s | **Present** |

**Patch:** None. System handles via skip logic.

---

## 6. Vehicle Slightly Rotated Between Detections

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Use latest vehicle box; far-side placement adapts | compute_box_goal uses vehicle_box; nearest-face logic | **Defended** |
| Logging | Log vehicle box age | detection_stamp_max_age_s rejects stale | **Present** |
| Guarding | Reject stale detections | detection_stamp_max_age_s 0.5 s | **Present** |
| Timeout | goal_max_age_s | Param | **Present** |

**Patch:** None. Far-side placement and stamp age handle rotation.

---

## 7. Costmap Inflation Over-Blocking Path

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Planner finds alternate; or skip tyre | inflation 0.6 local, 0.35 global; Nav2 replans | **Defended** |
| Logging | Log nav failure | _on_box_result; skip_tire_on_nav_failure | **Present** |
| Guarding | Goal-in-costmap precheck | None | **Missing** |
| Timeout | approach_timeout_s | 120 s | **Present** |

**Patch:** Optional: add costmap query before goal send (nav2_costmap_2d getCost). If goal in occupied cell, reject and skip. Medium priority.

---

## 8. Planner Oscillation

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Recovery; spin protection; skip | Oscillation critic; spin_detect; max_state_repeats | **Defended** |
| Logging | Log spin_detected | _mission_log_append | **Present** |
| Guarding | max_state_repeats → ERROR | mission_state_machine | **Present** |
| Timeout | spin_detect_min_time_s | 3.0 s | **Present** |

**Patch:** None. DWB Oscillation critic + spin protection sufficient.

---

## 9. FollowWaypoints Partial Success with Missed Index

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Continue to next; report missed | stop_on_failure: false; missed_waypoints in result | **Defended** |
| Logging | Log missed indices | _on_follow_waypoints_result | **Needs check** |
| Guarding | Reject empty poses | send_follow_waypoints | **Present** |
| Timeout | Per-waypoint | waypoint_follower | **Present** |

**Patch:** Ensure _on_follow_waypoints_result logs `missed_waypoints` from result. Add to mission_report tires_skipped when index in missed_waypoints.

---

## 10. Single Tire Unreachable Due to Obstacle

| Aspect | Expected | Current | Status |
|--------|----------|---------|--------|
| Behavior | Skip tyre; continue; log | skip_tire_on_nav_failure; _tires_skipped | **Defended** |
| Logging | Log unreachable with tire_id | _tires_skipped list | **Present** |
| Guarding | approach_timeout_s | 120 s | **Present** |
| Timeout | approach_timeout_s | 120 s | **Present** |

**Patch:** None. Skip logic handles.

---

## Summary of Required Patches

| # | Condition | Patch | Priority |
|---|-----------|-------|----------|
| 2 | TF future timestamps | Fallback to latest in transformer.py | High |
| 3 | Depth dropout 300 ms | pointcloud_max_age_s param | Medium |
| 7 | Costmap over-blocking | Optional goal-in-costmap precheck | Medium |
| 9 | FollowWaypoints missed | Log missed_waypoints; map to tires_skipped | Medium |

---

## Implementation Notes

- **TF future fallback:** In `lookup_transform_at_stamp`, catch `TransformException`; if message indicates extrapolation (future), retry with `rclpy.time.Time()` and log.
- **pointcloud_max_age_s:** In segmentation_processor, before processing, check `(now - cloud->header.stamp) > max_age`; return early and log.
- **missed_waypoints:** nav2_msgs/FollowWaypointsResult has `missed_waypoints` (uint32[]). Iterate and add to _tires_skipped with reason "waypoint_missed".
