# Post-Mission Forensic Diagnostic Report
**Date:** 2026-02-24  
**Mission intent:** Search → detect vehicle → log pose → navigate to vehicle → tire inspection  
**Observed behavior:** Robot spun in place in search state; no transition to approach or inspection.

---

## 1. Executive Summary

| Question | Answer |
|----------|--------|
| **Single root cause** | **TF chain invalid:** lookup `slamware_map` → `base_link` failed. The mission state machine pauses every tick when TF is invalid and never proceeds to vehicle detection logic or search rotation. |
| **Did the system claim it detected a vehicle?** | **No.** No `vehicle_detected` or `Saved vehicle position` in logs; `detected_vehicles` remained empty. |
| **Why did the robot spin indefinitely?** | The robot either (a) appeared to spin due to some other source of cmd_vel, or (b) remained stuck in **SEARCH_VEHICLE** with **TF watchdog pausing the mission** every second. Because `_tick()` returns early when `_check_tf_watchdog()` is false, **no search rotation was ever dispatched** and **no detection handling ran**. The observable “spin” may have been from a prior goal or from the user’s description of the intended search behavior that never actually ran. |

**Conclusion:** The mission stalled because the **TF transform from `slamware_map` to `base_link` was invalid** (Aurora SDK not publishing `odom` → `base_link`, or Aurora not connected). The inspection manager’s TF watchdog correctly paused the mission but had no timeout to abort, so the mission stayed paused indefinitely.

---

## 2. Timeline Reconstruction

Ordered event sequence from mission launch to stall (from `~/ugv_ws/logs/mission_latest.jsonl` and archive):

| Time (epoch) | Event | Source |
|--------------|--------|--------|
| 1771945109.54 | `mission_start` (reason: user_start, require_sensor_health: false) | mission_latest.jsonl |
| 1771945109.55 | State transition: **IDLE → SEARCH_VEHICLE** (cause: unknown) | mission_latest.jsonl |
| 1771945110.54 | **tf_watchdog** (timeout_s: 0.2, state: SEARCH_VEHICLE) | mission_latest.jsonl |

No further events: no `vehicle_detected`, no `state_transition` to WAIT_VEHICLE_BOX or APPROACH_VEHICLE, no `nav_result`, no rotation. So within ~1 s of entering SEARCH_VEHICLE, the TF watchdog fired and the mission remained paused for the rest of the run.

---

## 3. Detection Analysis

| Question | Finding |
|----------|--------|
| **Was vehicle perception functional?** | **Unknown from logs.** No detection events were logged. With TF invalid, the mission never reached the code paths that would log detection; vehicle callback could have run but state machine never acted. |
| **Confidence scores and thresholds** | N/A — no vehicle_detected events. |
| **Vehicle source in production** | PRODUCTION_CONFIG sets `vehicle_boxes_topic: /aurora_semantic/vehicle_bounding_boxes`. Vehicles are taken only from that topic (inspection_manager does not use detection_topic for vehicles when vehicle_boxes_topic is set). So vehicles must come from **aurora_semantic_fusion** (Aurora semantic + depth or YOLO fallback after 10 s). |

Even if the perception pipeline had published vehicle boxes, the inspection manager would not have processed them for state transitions because **every `_tick()` returned early** when TF watchdog failed.

---

## 4. State Machine Analysis

| Item | Result |
|------|--------|
| **Exact state where mission stalled** | **SEARCH_VEHICLE** |
| **Condition preventing transition** | **TF watchdog:** `_check_tf_watchdog()` returns `False` when `slamware_map` → `base_link` lookup fails for longer than `tf_watchdog_timeout` (0.2 s). In `_tick()`, the first check is `if not self._check_tf_watchdog(): return`, so no later logic (vehicle check, timeout rotation, or DONE) runs. |
| **Guard / timeout** | The guard is “TF valid”. No timeout was present to abort the mission after prolonged TF failure, so the FSM stayed in SEARCH_VEHICLE and paused forever. |

Code reference: `inspection_manager_node.py` — `_tick()` (early return on TF watchdog), `_check_tf_watchdog()`.

---

## 5. Navigation Analysis

| Question | Answer |
|----------|--------|
| **Was a navigation goal issued?** | **No.** No rotation or approach goal is sent while TF is invalid; the tick returns before any goal dispatch. |
| **If blocked, why?** | TF watchdog blocks all progress, so navigation is never requested. |

---

## 6. Code-Level Findings

### 6.1 Root cause (TF)

- **Requirement:** Inspection manager needs `world_frame` → `base_frame` (default `slamware_map` → `base_link`) for pose and goals.
- **Chain in this setup:** `slamware_map` (static) → `odom` (Aurora SDK) → `base_link` (Aurora SDK). Aurora SDK (`slamware_ros_sdk_server_node`) must publish `odom` → `base_link`. If the device is unreachable or not publishing, the lookup fails.
- **Behavior:** On failure, the watchdog pauses the mission but did not transition to ERROR, so the mission could run indefinitely with no progress.

### 6.2 Logic verified (no defect in detection/transition logic for this run)

- **Vehicle source:** When `vehicle_boxes_topic` is set, only `_vehicle_boxes_cb` adds to `detected_vehicles`; `_detection_cb` does not call `_process_vehicle_boxes` for vehicles (by design).
- **SEARCH_VEHICLE → WAIT_VEHICLE_BOX:** Triggered when `len(self.detected_vehicles) > 0` and an un-inspected vehicle exists; this was never reached because TF watchdog returned before this block.
- **Rotation on timeout:** Same tick path that checks `detected_vehicles` and timeout also runs only when TF is valid; it was never executed.

### 6.3 Architectural / operational weakness

- **TF dependency not obvious at mission start:** If Aurora is down or slow, the mission starts anyway and then pauses with only a generic “TF watchdog” log, which is easy to miss.
- **No abort on prolonged TF loss:** Pausing indefinitely can look like “robot spinning” or “stuck in search” without a clear ERROR or runbook hint.

---

## 7. Failure Classification

**Primary classification:** **F. State machine failed to transition** — with the underlying cause that **TF was invalid**, so the state machine never left SEARCH_VEHICLE.

**Supporting classifications:**

- **L. Topic / namespace / frame issue:** The TF frame chain (`slamware_map` → `odom` → `base_link`) was broken (Aurora not publishing or not connected).
- **O. Silent exception / swallowed behavior:** TF lookup exceptions in `_check_tf_watchdog()` result in pausing without a clear “Aurora required” message and without transitioning to ERROR after a defined timeout.

**Not the cause in this run:** Vehicle never detected (A), confidence threshold (B), pose computation (C), pose not written (D/E), navigation goal not issued (G), goal rejected (H), navigation aborted (I), inspection manager blocking (J), launch config (K), parameter misconfiguration (M), race (N) — none of these were the primary blocker; TF invalidity prevented any of that logic from running.

---

## 8. Fixes Applied

1. **inspection_manager_node.py**
   - **Parameter:** `tf_unavailable_abort_s` (default 60.0 s). After this many seconds of continuous TF failure, the mission transitions to **ERROR** with cause `tf_unavailable` instead of pausing forever.
   - **TF watchdog message:** Error log now includes: “Is Aurora (slamware_ros_sdk_server_node) running? Check: ros2 run tf2_ros tf2_echo &lt;world_frame&gt; &lt;base_frame&gt;”.
   - **Tracking:** `_tf_watchdog_paused_since` records when the pause started so the abort timeout can be applied.

2. **PRODUCTION_CONFIG.yaml**
   - Added `tf_unavailable_abort_s: 60.0` under timeouts.

3. **RUNBOOK.md**
   - Troubleshooting: TF lookup failure now suggests running `ros2 run tf2_ros tf2_echo slamware_map base_link`.
   - New row: “Mission spins / never leaves SEARCH_VEHICLE” → check `mission_latest.jsonl` for `tf_watchdog` and fix Aurora connectivity.

---

## 9. Remaining Risks

| Risk | Mitigation |
|------|------------|
| Aurora not connected at mission start | Pre-flight checklist and RUNBOOK already require Aurora; consider failing mission start if TF is not valid within a short window. |
| Aurora drops TF mid-mission | TF watchdog already pauses; with the new change, mission will abort after 60 s and report ERROR. |
| Semantic vehicle topic not publishing | If `/aurora_semantic/vehicle_bounding_boxes` never has data (no Aurora semantic and fallback not yet active), vehicles will never be added; runbook already mentions checking semantic_labels and fallback. |

---

## 10. References

- Mission log (current run): `~/ugv_ws/logs/mission_latest.jsonl`
- Archive: `~/ugv_ws/logs/archive/mission_2026-02-24_14-58-29.jsonl`, `mission_2026-02-24_14-44-35.jsonl`
- State machine: `MISSION_FLOW_DIAGRAM.md`, `docs/MISSION_PRODUCTION_AUDIT.md`
- Inspection manager: `src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/inspection_manager_node.py`
- Aurora bringup / TF: `src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/launch/aurora_bringup.launch.py`, `nav_aurora.launch.py`
- Runbook: `RUNBOOK.md`
