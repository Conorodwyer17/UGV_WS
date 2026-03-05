# Acceptance Criteria for Tire Inspection Mission

This document defines acceptance criteria for validating the tire inspection mission on a real Aurora robot with physical cars.

---

## Acceptance Run (Basic)

**Setup:**
- One car in view (static, no occlusion)
- Aurora robot with full stack running (aurora_bringup, segment_3d, Nav2, inspection_manager, photo_capture_service, motor driver)
- PRODUCTION_CONFIG loaded with `approach_offset: 0.7`, `tire_offset: 0.5`, `dry_run: false`

**Success criteria:**

| # | Criterion | Pass condition |
|---|-----------|----------------|
| 1 | Vehicle detected | Robot transitions SEARCH_VEHICLE → WAIT_VEHICLE_BOX within 30 s |
| 2 | Approach goal dispatched | Approach goal sent to Nav2; robot drives toward vehicle |
| 3 | Stop distance (vehicle) | Robot stops at least approach_offset (0.7 m) from nearest face of vehicle; MIN_SAFE_OFFSET_M (0.5 m) floor in code |
| 4 | No drive into car | Robot never drives into the car; costmap and offsets prevent collision |
| 5 | Tire inspection | Robot visits 4 tires in distance order (nearest first, then 2nd, 3rd, 4th nearest); stops ~0.5 m in front of each |
| 6 | Photo capture | Photos captured for each tire; VERIFY_CAPTURE succeeds 4 times |
| 7 | Mission completion | Mission reaches DONE or NEXT_VEHICLE after 4 tires |
| 8 | No prolonged spin | No spin > 90° without progress for more than 2 s (spin_detect_min_time_s) |
| 9 | No stuck timeout | If Nav2 stalls, mission cancels and retries within approach_timeout_s (120 s) |

---

## Validation Commands

**Before run:**
```bash
bash scripts/pre_mission_verify.sh
```

**During/after run:**
- Mission log: `~/ugv_ws/logs/mission_latest.jsonl`
- Report: `~/ugv_ws/logs/mission_report_latest.json`
- Forensics: grep for `spin_detected`, `progress_stall`, `goal_offset_validation_failed`, `planned_tire_fallback_used`

---

## Known Limitations

- Aurora semantic uses 1×1 m fixed box; orientation inference uses robot direction when box is square
- Planned tire fallback used when no wheel detection within tire_search_timeout_s (90 s)
- Nav2 goal tolerance (xy 0.25 m, yaw 0.25 rad) may cause oscillation near goal on some hardware
