# Diagnosis (2026-02-25 Mission)

## Executive summary
The 2026-02-25 mission failed to approach a vehicle because the Aurora SDK failed to connect, semantic/depth streams were missing, and the TF tree was disconnected (`slamware_map` ↔ `base_link`). These conditions prevent 3D vehicle boxes from being produced and block navigation goal computation. The system then entered repeated search rotations; a Nav2 spin recovery was configured and executed in nearby logs, which can explain the observed in-place spin, but no `/cmd_vel` evidence was captured for the exact 180° spin.

## Root causes with evidence
### 1) Aurora SDK connection failures → no semantic/depth streams (High confidence)
- Evidence:
  - `Failed to connect to the selected device` — `/home/conor/.cursor/worktrees/ugv_ws__SSH__jetson_Thesis_WIFI_/rxd/analysis/journal_ugv_mission_2026-02-25_1240-1310.log` line 69.
  - `No semantic or depth received after 5s...` — same file lines 77, 81, 125 (repeated warnings).
- Impact:
  - Semantic fusion produces no vehicle boxes; segmentation processor cannot compute 3D boxes.

### 2) TF tree disconnected (`slamware_map` ↔ `base_link`, `base_link` ↔ `odom`) (High confidence)
- Evidence:
  - `Could not get current pose: ... slamware_map ... base_link ... not part of the same tree` — `journal_ugv_mission_2026-02-25_1240-1310.log` line 308.
  - `TF watchdog: slamware_map->base_link invalid > 0.2s. Pausing mission.` — line 311.
  - Nav2 local costmap TF timeouts — `/home/conor/.ros/log/controller_server_4108_1772029284102.log` lines 44–81.
- Impact:
  - `_get_current_pose()` fails, so `_dispatch_box_goal()` returns `False` and no navigation goal is sent.

### 3) Perception pipeline does not produce 3D vehicle boxes (Medium confidence)
- Evidence:
  - `No 3D bounding boxes computed from 3 segmented objects` — `journal_ugv_mission_2026-02-25_1240-1310.log` line 1146.
- Impact:
  - Inspection manager never receives vehicle boxes; mission stays in search loop.

### 4) Nav2 spin recovery executed (Low–Medium confidence)
- Evidence:
  - `behavior_server: Running spin` / `Turning 1.57 for spin behavior.` — `/home/conor/.ros/log/behavior_server_3097_1772023011378.log` lines 23–24.
- Impact:
  - Spin recovery can cause in-place rotation; direct linkage to the reported 180° spin is not proven without `/cmd_vel`.

### 5) Motion gating via `depth_gate` (Low confidence)
- Evidence:
  - `depth_gate: cmd_vel_nav -> cmd_vel gated by /stereo/navigation_permitted` — `journal_ugv_mission_2026-02-25_1240-1310.log` line 66.
- Impact:
  - If `/stereo/navigation_permitted` is false, forward motion can be blocked even after goal dispatch. No evidence shows the gate state at the spin moment.

## Subsystem state sequence (evidence + confidence)
- **Power / system**: UGV mission service launched (processes started).
  - Evidence: `slamware_ros_sdk_server_node` and other nodes started — `journal_ugv_mission_2026-02-25_1240-1310.log` lines 11, 47, 54, 68.
  - Confidence: High.
- **Sensors**: RGB image stream appears active (ultralytics receives images), but semantic/depth missing.
  - Evidence: `ultralytics_segmentation` publishing objects (lines 1138–1141) vs `No semantic or depth received` warnings (lines 77+).
  - Confidence: Medium.
- **Localization / TF**: TF unavailable between `slamware_map` and `base_link`.
  - Evidence: TF error and watchdog pause (lines 308–311).
  - Confidence: High.
- **Perception**: Segmentation processor does not compute 3D boxes.
  - Evidence: `No 3D bounding boxes computed...` (line 1146).
  - Confidence: High.
- **Planner / Nav2**: Rotation goal attempted and rejected; Nav2 spin recovery configured.
  - Evidence: rotation goal rejected (lines 1072–1081) and behavior_server spin plugin (lines 205–207).
  - Confidence: Medium.
- **Motion controller**: Command path is gated (depth_gate).
  - Evidence: `cmd_vel_nav -> cmd_vel gated by /stereo/navigation_permitted` (line 66).
  - Confidence: Low (gate state unknown).
- **Safety**: Spin recovery aborted due to collision ahead in another run.
  - Evidence: `Collision Ahead - Exiting Spin` — `/home/conor/.ros/log/behavior_server_3097_1772023011378.log` line 25.
  - Confidence: Low–Medium (adjacent run evidence).

## Chain breakpoint summary
The detection → goal → motion chain breaks **before goal computation**, because TF is invalid and Aurora semantic/depth streams are missing. Without valid TF and 3D vehicle boxes, `_dispatch_box_goal()` cannot compute a valid `map` goal, and no `/cmd_vel` is produced.

## Fixes applied (code/config) + confidence
- **Structured, fail-hard logging & watchdogs** (High): Added `MISSION_STATE`, `DETECTION_EVENT`, `GOAL_COMPUTED`, `NAV_COMMAND_SENT`, `NAV_FEEDBACK`, detection-missing/filtered logs, dispatch-fail abort, cmd_vel timeout, and progress/spin detection. Evidence: `inspection_manager_node.py` emits these events in `_set_state`, `_publish_vehicle_detected`, `_dispatch_box_goal`, and `_on_box_result`.
- **TF/goal dispatch resilience** (High): `TF watchdog` now aborts after prolonged failures; goal dispatch failure counts tracked. Evidence: `_check_tf_watchdog()` and `_dispatch_box_goal()` in `inspection_manager_node.py`.
- **Photo trigger distance guard** (Medium): optional `photo_trigger_distance` gating with logs to avoid silent capture failures. Evidence: parameter in `PRODUCTION_CONFIG.yaml` and `photo_trigger_distance_check` in `_on_box_result()`.
- **Test added** (High): Unit tests for photo-trigger threshold logic (`pytest`). Evidence: `test/test_photo_trigger.py` (4 passing tests).

## Remaining gaps
- No log file from the visualization host that shows bounding boxes (RViz / AORA UI).
- No `/cmd_vel` or `/tf` rosbag for the run; spin attribution cannot be proven.
- No acceptance run after fixes; mission success not yet verified.
