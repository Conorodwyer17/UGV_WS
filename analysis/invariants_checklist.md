# Mission Invariants Checklist
#
# Purpose: confirm each invariant is enforced or fails fast.

## Frames and TF
- `slamware_map -> base_link` must be valid and stable before mission starts.
  - Enforced in `inspection_manager_node.py` IDLE: waits for TF, requires `tf_stable_s`, aborts after `tf_wait_timeout`.
- `map -> slamware_map` must be available when `require_goal_transform` is true.
  - Enforced in IDLE: waits for map transform, aborts on timeout.
  - Enforced on goal dispatch: `compute_box_goal` returns `None` if transform missing.
  - Rotation/patrol/planned-tire dispatches are blocked if map transform is unavailable.

## Nav2 readiness and safety gating
- Nav2 `/navigate_to_pose` action server must be available before mission starts.
  - Enforced in IDLE: waits up to `nav2_wait_timeout`, aborts if unavailable.
- Motion must be blocked when `nav_permitted` is false.
  - Enforced for box goals, face-tire rotation, patrol goals, rotation recovery, and planned-tire fallback.

## Perception validity
- Vehicle detections require confirmation before use.
  - Enforced via `vehicle_confirmations_required` and distance tolerance.
- Tire detections require confirmation before use.
  - Enforced via `tire_confirmations_required` and distance tolerance.
- Detection streams are monitored for staleness.
  - Logged via `detection_stream_stale` and `vehicle_boxes_stream_stale` with thresholds.

## Deterministic navigation and timeouts
- Approaches must not hang indefinitely.
  - Enforced with `approach_timeout_s` for both vehicle and tire approaches.
- Progress stalls and spin loops are detected and recovered.
  - Enforced with progress window checks and spin protection state repeat limits.
- Mission must terminate on hard timeout.
  - Enforced via `hard_mission_timeout` -> ERROR.

## Capture correctness
- Final yaw alignment to face tire before capture (RViz-style).
  - Enforced by `FACE_TIRE` state and yaw tolerance.
- Photo trigger distance check.
  - Enforced via `photo_trigger_distance` gate before capture.
