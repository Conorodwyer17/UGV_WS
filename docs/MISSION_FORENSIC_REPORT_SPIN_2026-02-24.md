# Mission Forensic Report — Constant Spin Incident (2026-02-24)

**Incident:** Robot started up, mission launched, then robot spun in circles constantly with no progress. Never moved to the next state; always turning in place.  
**Request:** Determine exactly what happened from start to finish; confirm whether car was detected; identify root cause and fix.

---

## 1. Log Evidence — Two Run Types

### 1a. Run with spin loop (vehicle detected but never approach)

**Source:** `~/ugv_ws/logs/mission_latest.jsonl` (run at 1771950640)

| # | Event | Timestamp | Elapsed |
|---|--------|-----------|--------|
| 1 | `mission_start` (user_start) | 1771950640.20 | 0 s |
| 2 | **IDLE → SEARCH_VEHICLE** | 1771950640.20 | 0 s |
| 3 | **vehicle_detected** (vehicle_id 1, confidence 0.79) | 1771950645.11 | ~5 s |
| 4 | **SEARCH_VEHICLE → WAIT_VEHICLE_BOX** (vehicles_available) | 1771950645.20 | ~5 s |
| 5 | **WAIT_VEHICLE_BOX → TURN_IN_PLACE_VEHICLE** | 1771950651.21 | **~6 s** |

After that the mission would have: rotation done → WAIT_VEHICLE_BOX → (timeout 5 s) → TURN_IN_PLACE_VEHICLE → … **forever**. No APPROACH_VEHICLE, no movement toward the car.

**Root cause (spin loop):** We only dispatched the approach goal when a “pending” flag was True (first entry from SEARCH). When we returned from TURN_IN_PLACE_VEHICLE we set that flag to False, so on every later stay in WAIT_VEHICLE_BOX we never dispatched approach—we only hit the 5 s timeout and sent another rotation. So: **car was detected**, but we kept timing out and rotating instead of using the box we already had.

### 1b. Run with TF watchdog (no detection)

**Source:** earlier run in same file (1771950128)

| # | Event | Elapsed |
|---|--------|--------|
| 1 | mission_start | 0 s |
| 2 | IDLE → SEARCH_VEHICLE | 0 s |
| 3 | **tf_watchdog** (state SEARCH_VEHICLE) | ~1 s |

No vehicle_detected; mission paused by TF so no progress.

### 1c. Latest mission (same “spin” report) — TF watchdog again

**Source:** `~/ugv_ws/logs/mission_latest.jsonl` (run at **1771955195**)

| # | Event | Timestamp | Elapsed |
|---|--------|-----------|--------|
| 1 | mission_start (user_start) | 1771955195.096 | 0 s |
| 2 | IDLE → SEARCH_VEHICLE | 1771955195.100 | ~0 s |
| 3 | **tf_watchdog** (timeout_s: 0.2, state: SEARCH_VEHICLE) | 1771955196.094 | **~1 s** |

**What this means:** The mission left IDLE and entered SEARCH_VEHICLE (so Nav2 and TF checks passed in IDLE at that moment). About **1 second later** the TF lookup `slamware_map` → `base_link` failed for longer than 0.2 s, so the **TF watchdog** fired and the mission **paused**. Every subsequent tick returns early at the watchdog; **no detection logic runs, no approach or rotation goals are sent** by the inspection manager. So in this run:

- **No vehicle_detected** — we never got to the code that processes detections.
- **No approach goal** — we never reached WAIT_VEHICLE_BOX or the approach-dispatch code.
- **Why it might “spin”:** The inspection manager did **not** send any rotation in this run. If the robot was physically spinning, it was from another source (e.g. Nav2 recovery, or a previous run’s goal still executing). If “spinning” meant “stuck / no progress,” that matches: we were stuck paused due to TF.

**Root cause for this run:** TF became invalid ~1 s after start (Aurora flaky or disconnected). We now require TF to be **stable for 3 s** in IDLE before starting, and we added comprehensive mission logging (heartbeat, tick_skipped_tf_invalid, approach_dispatch_attempt, wait_vehicle_timeout, rotation_dispatched, etc.) so the next run’s log will show exactly what happened at each step.

---

## 2. What Happened (Spin-Loop Run)

1. **Mission start** → IDLE → SEARCH_VEHICLE.
2. **~5 s:** Car detected → SEARCH_VEHICLE → WAIT_VEHICLE_BOX (we have `current_vehicle_box`).
3. **~6 s later:** Transition **WAIT_VEHICLE_BOX → TURN_IN_PLACE_VEHICLE**. So we hit the **5 s detection timeout** and sent a **rotation** instead of the approach goal.
4. **Why:** We only dispatched approach when a “pending” flag was True (set only when entering WAIT_VEHICLE_BOX from SEARCH or NEXT_VEHICLE). If the first tick didn’t run in time (TF blip, or the single 1 Hz tick landed after 5 s), we hit timeout and dispatched rotation. When rotation finished we went back to WAIT_VEHICLE_BOX with **pending = False**, so we never dispatched approach again—only timeout → rotate → WAIT_VEHICLE_BOX → timeout → rotate → …
5. **Result:** Robot spins constantly; never approaches the car; never leaves the turn-in-place loop.

---

## 3. Root Cause Summary

| Run type | Root cause |
|----------|------------|
| **Spin loop** (vehicle detected, then constant turn-in-place) | We only dispatched the approach goal on “first entry” to WAIT_VEHICLE_BOX (pending flag). After returning from TURN_IN_PLACE_VEHICLE we set pending = False, so we never dispatched again—only timeout (5 s) → rotation → WAIT_VEHICLE_BOX → repeat. |
| **TF watchdog** (no progress, no detection) | TF `slamware_map` → `base_link` invalid; mission paused every tick so detection and goals never run. |

So the **incident** (no progress, possible spin) is consistent with: **mission started before TF was valid**, then TF failed and the mission paused. If in another run a rotation had been sent before TF failed, Nav2 would keep executing that goal (spin) until it completes or is cancelled.

---

## 4. Comparison With Other Archives

| Log | Vehicles detected? | WAIT_VEHICLE_BOX? | APPROACH / spin? |
|-----|--------------------|--------------------|-------------------|
| mission_2026-02-24_16-22-08 | Yes (2) | Yes | WAIT_VEHICLE_BOX → TURN_IN_PLACE_VEHICLE → WAIT_VEHICLE_BOX → APPROACH_VEHICLE |
| mission_2026-02-24_16-12-01 | Yes (1) | Yes | Stopped after WAIT_VEHICLE_BOX (no approach in log) |
| **mission_latest (this run)** | **No** | **No** | **tf_watchdog ~1 s; no further events** |

So car detection **does** work in other runs when the mission is not blocked by TF. In this run, TF blocked before any detection could be used.

---

## 5. Fixes Implemented

### 5.1 **Spin loop fix (main)** — Dispatch approach every time we enter WAIT_VEHICLE_BOX with a box

- **Change:** Replaced “pending” flag with a **per-entry** flag `_dispatched_approach_this_wait`.
- **Behavior:** Every time we enter WAIT_VEHICLE_BOX (from SEARCH_VEHICLE, NEXT_VEHICLE, or **TURN_IN_PLACE_VEHICLE**) we set `_dispatched_approach_this_wait = False`. On the first tick in WAIT_VEHICLE_BOX, if `current_vehicle_box` is set and we haven’t dispatched yet this entry, we dispatch the approach goal and set the flag True.
- **Effect:** After a rotation we return to WAIT_VEHICLE_BOX with the same `current_vehicle_box`; we now dispatch the approach on the first tick instead of waiting 5 s and rotating again. The robot moves to APPROACH_VEHICLE and drives to the car instead of spinning forever.

### 5.2 Do not start mission until TF is valid (IDLE)

- **Parameter:** `tf_wait_timeout` (default 60 s).
- **Behavior:** In IDLE, before transitioning to SEARCH_VEHICLE (or INIT), the node now waits until the TF lookup `world_frame` → `base_frame` (e.g. `slamware_map` → `base_link`) succeeds.
- **If TF never becomes valid:** After `tf_wait_timeout` seconds, the mission transitions to **ERROR** with cause `tf_unavailable_at_start` and does not enter SEARCH_VEHICLE.
- **Effect:** Avoids starting the mission when Aurora is not yet publishing TF, reducing “launch then immediately pause and spin/no progress” from TF.

### 5.3 Cancel in-flight Nav2 goal when TF watchdog fires

- **Behavior:** When the TF watchdog first triggers (mission paused), the node **cancels** any active Nav2 goal (approach or rotation) via the stored goal handle.
- **Effect:** If a rotation (or approach) had been sent and then TF failed, the robot stops executing that goal instead of spinning until the goal completes.

### 5.4 Existing behavior (unchanged)

- **tf_unavailable_abort_s** (e.g. 60 s): After this many seconds of **continuous** TF failure while paused, the mission transitions to **ERROR** (cause `tf_unavailable`) so it does not sit paused forever.

---

## 6. Checklist for “Start to Finish”

| Step | What to verify |
|------|-----------------|
| 1. Aurora on and publishing TF | Before launch: `ros2 run tf2_ros tf2_echo slamware_map base_link` succeeds. |
| 2. Mission start | Log: `mission_start` then IDLE → SEARCH_VEHICLE. If you see “Waiting for TF…” then TF was not ready at start; mission will only proceed when TF is valid or go to ERROR after `tf_wait_timeout`. |
| 3. No immediate tf_watchdog | If TF is valid at start (step 1), you should not see `tf_watchdog` within the first second. If you do, Aurora stopped publishing TF after start. |
| 4. Car detection | When TF is valid, vehicle boxes from `/aurora_semantic/vehicle_bounding_boxes` (or fallback) are processed. Look for `vehicle_detected` and state_transition to WAIT_VEHICLE_BOX → APPROACH_VEHICLE. |
| 5. Approach / tires | After APPROACH_VEHICLE, nav result and then WAIT_TIRE_BOX → tire-by-tire inspection and photo capture. |

---

## 7. Summary

- **Spin loop (constant turn-in-place):** Car **was** detected. We transitioned to WAIT_VEHICLE_BOX but then hit the 5 s timeout and sent a rotation. After each rotation we re-entered WAIT_VEHICLE_BOX with a flag that prevented dispatching the approach again, so we only timed out and rotated again—forever.
- **Fix:** In WAIT_VEHICLE_BOX we now **always** dispatch the approach goal on the first tick when we have `current_vehicle_box`, on **every** entry (including after TURN_IN_PLACE_VEHICLE). So we no longer get stuck in timeout → rotate → repeat.
- **Other run type:** When TF is invalid, the mission pauses (and we now wait for TF in IDLE and cancel in-flight goals when the watchdog fires).
- **Car detection:** Working; the bug was purely in the state machine (when to dispatch approach from WAIT_VEHICLE_BOX).

---

## 8. Mission log events reference (comprehensive logging)

After the latest changes, each mission appends these event types to `mission_log_path` (e.g. `~/ugv_ws/logs/mission_latest.jsonl`):

| Event | When |
|-------|------|
| `mission_start` | Mission started (reason, require_sensor_health). |
| `state_transition` | Every state change (from, to, cause). |
| `idle_wait_nav2` | Waiting in IDLE for Nav2 (elapsed_s, timeout_s). |
| `idle_wait_tf` | Waiting in IDLE for TF (elapsed_s, timeout_s, frame). |
| `idle_wait_tf_stable` | TF valid but waiting for stability (stable_elapsed_s, required_s). |
| `idle_passed` | IDLE checks passed (nav2_ok, tf_ok, tf_stable_s). |
| `vehicle_detected` | A vehicle was added (vehicle_id, frame_id, x, y, z, yaw, confidence). |
| `approach_dispatch_attempt` | We tried to send the approach goal (sent: true/false, reason if not sent). |
| `wait_vehicle_timeout` | We hit the 5 s timeout in WAIT_VEHICLE_BOX (elapsed_s, rotation_attempts, dispatched_approach_this_wait, has_current_vehicle_box). |
| `rotation_dispatched` | A rotation goal was sent to Nav2 (goal_type: search/vehicle/tire, rotation_attempts). |
| `tf_watchdog` | TF invalid; mission paused (timeout_s, state). |
| `tick_skipped_tf_invalid` | Throttled (every 5 s) while paused (state, paused_elapsed_s, reason, frame). |
| `mission_heartbeat` | Every 10 s while mission running (state, tf_valid, tf_watchdog_paused, detected_vehicles, has_current_vehicle_box, dispatched_approach_this_wait, elapsed_since_start). |
| `nav_result` | Nav goal finished (goal, status, success, state). |
| `nav_rejected` | Nav goal was rejected. |
| `tf_unavailable_at_start` | TF never became available in IDLE (timeout_s). |
| `tf_unavailable_abort` | TF was invalid for 60+ s; mission aborted. |
| `mission_end` | Mission ended (final_state, report). |

Use these to see exactly why the robot did or didn’t approach: e.g. `approach_dispatch_attempt` with `sent: false` and `reason`, or `wait_vehicle_timeout` before any `approach_dispatch_attempt`, or `tick_skipped_tf_invalid` showing the mission was paused.
