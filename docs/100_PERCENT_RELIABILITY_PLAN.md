# 100% Reliable Autonomous Tire Inspection — Master Plan

**Date:** 2026-03-04
**Status:** In Progress
**Goal:** Achieve deterministic, error-free four-tire inspection for every vehicle, every time.

---

## Executive Summary

This document outlines a systematic plan to achieve 100% mission success for the Autonomous Tire Inspection Robot. It identifies gaps, proposes solutions, and prioritizes implementations. The system is already functional; the focus is on hardening against edge cases, tuning parameters, and ensuring robustness under real-world conditions.

---

## 1. Tire Detection — 100% Reliable

### 1.1 Current State
- **Model:** `best_fallback.pt` (YOLO segmentation, class `wheel` id=22)
- **Pipeline:** ultralytics_tire → segmentation_processor_tire → tire_detector_pcl (fallback) → tire_merger → inspection_manager
- **TensorRT:** Engine exists but has invalid class indices (28, 37, 45); use PyTorch until re-exported

### 1.2 Gaps & Solutions

| Gap | Impact | Solution | Priority |
|-----|--------|----------|----------|
| **Wrong detection topic** | PRODUCTION_CONFIG uses `/darknet_ros_3d/bounding_boxes` (doesn't exist); full_bringup defaults to `/darknet_ros_3d/tire_bounding_boxes` instead of merged | Use `/tire_bounding_boxes_merged` in PRODUCTION_CONFIG; full_bringup passes it to inspection_manager | **P0** |
| **Segmentation processor too strict** | `minimum_probability: 0.8` filters out valid wheels (0.35–0.8); PRODUCTION_CONFIG expects 0.35 | Lower to 0.35–0.5 to align with min_tire_probability | **P0** |
| **TensorRT invalid indices** | Engine returns out-of-range class IDs (37, 39); ultralytics_node filters them | ultralytics_node now filters `class_index < 0 or class_index >= num_classes`; detections skipped silently. Re-export engine for clean output. | **P1** (mitigated) |
| **Inference throttling** | `inference_interval_s: 1.5` may miss wheels during fast approach | Consider 1.0 s when in INSPECT_TIRE; or adaptive throttle based on distance_remaining | **P2** |
| **Lighting/occlusion** | Model may miss wheels in poor conditions | PCL fallback already handles; ensure `yolo_stale_s: 0.5` for fast fallback; consider retraining with augmented data | **P2** |

### 1.3 Parameter Alignment
- **ultralytics_tire:** `wheel_confidence` 0.5 (launch) — keep
- **segmentation_processor_tire:** `minimum_probability` 0.8 → **0.35** (match PRODUCTION_CONFIG)
- **inspection_manager:** `min_tire_probability` 0.35 — keep
- **tire_merger:** `yolo_stale_s` 0.5 — good for fast PCL fallback

---

## 2. Goal Generation & Refinement — Always Correct

### 2.1 Current State
- **Vehicle modeler:** `estimate_tire_positions_from_box()` uses wheelbase 2.7 m, track 1.6 m; infers front from robot position
- **Goal placement:** Far-side tires use vehicle_center → tire vector; near-side use nearest-point-on-box
- **Refinement:** `_refine_target_from_detection()` replaces planned goal with detection center when within 0.6 m

### 2.2 Gaps & Solutions

| Gap | Impact | Solution | Priority |
|-----|--------|----------|----------|
| **Detection flicker** | Refinement picks latest detection; flicker can cause goal jumps | Use temporal smoothing: require detection stable for 2–3 frames before refining; or use EMA of last N detections | **P1** |
| **Refinement distance** | 0.6 m may be too large for far-side (wrong tire) or too small for near-side | Add per-tire refinement: near-side 0.6 m, far-side 0.4 m; or use planned-only for far-side until detection confirmed | **P2** |
| **Aurora 1×1 m boxes** | Square boxes cause orientation ambiguity | vehicle_modeler uses robot direction; already handled | Done |
| **Truck/bus dimensions** | Default wheelbase 2.7 m may be wrong for trucks | PRODUCTION_CONFIG has vehicle_wheelbase_truck_m, vehicle_track_truck_m; ensure vehicle_modeler uses them when box extent suggests truck | **P2** |

### 2.3 Robustness
- **goal_costmap_precheck:** Rejects goals in occupied cells — keep enabled
- **detection_stamp_max_age_s:** 0.5 s — strict; avoid stale TF
- **refinement_max_distance_m:** 0.6 m — reasonable; document edge cases

---

## 3. Navigation — Precise and Collision-Free

### 3.1 Current State
- **Nav2:** DWB + RotationShimController; xy_goal_tolerance 0.15, yaw 0.25
- **Costmap:** Local 5×5 m, inflation 0.6 m; scan + depth point cloud
- **Planner:** NavfnPlanner, use_final_approach_orientation True

### 3.2 Gaps & Solutions

| Gap | Impact | Solution | Priority |
|-----|--------|----------|----------|
| **Costmap blocks far-side** | Vehicle inflated; path around may be blocked | Consider temporary "known obstacle" for vehicle footprint; or reduce inflation for tire approach (research: dynamic layers) | **P2** |
| **No path found** | Planner fails; mission retries or skips | skip_tire_on_nav_failure already handles; ensure try_next_corner_on_nav_fail used | Done |
| **TF extrapolation** | Occasional TF lag causes goal rejection | transform_tolerance 2.0 in Nav2; tf_max_age_ms 250 in segmentation_processor | Done |

### 3.3 Tuning
- **approach_timeout_s:** 120 s — adequate
- **max_recoveries_before_skip:** 3 — reasonable
- **recovery_skip_distance_threshold_m:** 0.5 — only skip when still far

---

## 4. Final Approach — Perfect Standoff

### 4.1 Current State
- **Centroid servo:** Enabled when distance_remaining < 0.5 m; P-controller centers tire in image
- **Handoff:** Optional spatial filter (detection within 0.5 m of expected tire); 10 s timeout to hand off without detection
- **Standoff:** tire_offset 0.4 m; MIN_SAFE_OFFSET_TIRE_M 0.25 m

### 4.2 Gaps & Solutions

| Gap | Impact | Solution | Priority |
|-----|--------|----------|----------|
| **Centroid image size** | centroid_servo uses 416×224 (Aurora depth); ObjectsSegment from left_image_raw may differ | Verify left_image_raw resolution; centroid_servo.yaml has 416×224 — correct for Aurora | Done |
| **Proximity gate vs centroid** | proximity_gate_distance_m 0.3 (log); centroid_servo_proximity_m 0.5 (handoff) | Align: handoff at 0.5 m is correct; log at 0.3 is fine | Done |
| **No wheel at goal** | centroid_servo_require_detection_near_expected can block | centroid_servo_wait_for_detection_timeout_s 10 s allows handoff anyway | Done |
| **Overshoot** | Robot may overshoot 0.3 m standoff | Centroid servo: distance_gain_scaling reduces Kp when distance_remaining < 0.3 m; min 30% gain to avoid dead zone | **P2** (mitigated) |

---

## 5. Mission Logic — Deterministic and Error-Proof

### 5.1 Current State
- **States:** IDLE → INIT → SEARCH_VEHICLE → WAIT_VEHICLE_BOX → APPROACH_VEHICLE → WAIT_TIRE_BOX → INSPECT_TIRE → FACE_TIRE → WAIT_WHEEL_FOR_CAPTURE → VERIFY_CAPTURE → (next tire/vehicle) → DONE/ERROR
- **Spin protection:** max_state_repeats 3; cycle detection prevents infinite loops
- **Return-later:** Deferred tires requeued; max_return_later_passes 1

### 5.2 Gaps & Solutions

| Gap | Impact | Solution | Priority |
|-----|--------|----------|----------|
| **require_nav_permitted** | PRODUCTION_CONFIG has true; /stereo/navigation_permitted may not exist in headless | Add require_nav_permitted: false for runs without depth gate; or ensure topic always published | **P0** |
| **Aurora disconnect** | TF stops; mission pauses | tf_unavailable_abort_s 60 s transitions to ERROR; document recovery procedure | Done |
| **Motor driver failure** | cmd_vel not reaching motors | cmd_vel_timeout_s 2 s; consider heartbeat/watchdog | **P2** |

### 5.3 Robustness
- **capture_on_wheel_timeout:** True — take photo even if wheel not detected (avoids blocking)
- **skip_tire_on_nav_failure:** True — continue to next tire
- **return_later_enabled:** True — retry skipped tires

---

## 6. Performance — No Bottlenecks

### 6.1 Current State
- **Controller:** 10 Hz (Jetson-tuned)
- **Inference:** Throttled 1.5 s; PyTorch (TensorRT when fixed)
- **PCL:** process_rate_hz 2.0

### 6.2 Recommendations
- **GPU monitoring:** Add optional GPU utilization log at 1 Hz when > 80%
- **Callback groups:** inspection_manager uses default; consider MutuallyExclusive for heavy callbacks
- **NMS:** max_det 100, imgsz 640; reduce to 50/480 if NMS time limit exceeded

---

## 7. Robustness — Real-World Conditions

### 7.1 Checklist
- [x] Different vehicle types (car, truck, bus) — vehicle_modeler has wheelbase/track overrides
- [x] Lighting — PCL fallback; consider retraining with augmentation
- [ ] Wet/dirty wheels — Model may need retraining
- [x] Other objects nearby — vehicle_duplicate_tolerance, tire_position_tolerance
- [x] Starting position variation — planned_tire_fallback_enabled

---

## 8. Implementation Priority

### Phase 1 — Critical (Do First) ✅ IMPLEMENTED
1. **Fix detection_topic** — PRODUCTION_CONFIG: `/tire_bounding_boxes_merged` ✅
2. **Fix full_bringup** — Pass tire_detection_topic to inspection_manager ✅
3. **Lower segmentation_processor_tire minimum_probability** — 0.8 → 0.35 ✅
4. **require_nav_permitted** — Launch arg added; use `require_nav_permitted:=false` for headless ✅

### Phase 2 — High Value
5. **Detection flicker** — EMA smoothing in _refine_target_from_detection ✅ (refinement_ema_alpha 0.3)
6. **TensorRT re-export** — Script exists: `./scripts/export_tensorrt.sh`; run on Jetson
7. **Documentation** — Update RUNBOOK, TROUBLESHOOTING with new params

### Phase 3 — Polish
8. **Truck/bus wheelbase** — Use vehicle_modeler overrides when box extent suggests truck
9. **Inference throttle** — Adaptive based on state
10. **GPU monitoring** — Optional diagnostic

---

## 9. Verification Commands

```bash
# Pre-mission
ros2 topic list | grep -E "vehicle_bounding_boxes|tire_bounding_boxes|left_image_raw"
ros2 topic echo /tire_bounding_boxes_merged --once
ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once

# During mission
ros2 topic echo /inspection_state
ros2 topic echo /inspection_manager/runtime_diagnostics

# Post-mission
cat ~/ugv_ws/logs/mission_report_latest.json
```

---

## 10. References

- `PRODUCTION_CONFIG.yaml` — Mission parameters
- `nav_aurora.yaml` — Nav2 config
- `segment_3d.launch.py` — Perception pipeline
- `docs/BEST_FALLBACK_MODEL_CLASSES.md` — Model class reference
- `docs/TIRE_DETECTION_TROUBLESHOOTING.md` — Perception debugging
