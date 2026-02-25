# Detection → Navigation Trace (2026-02-25)

## Expected chain (code path)
- `inspection_manager_node.py`: `_vehicle_boxes_cb()` → `_process_vehicle_boxes()` → `_dispatch_box_goal()` → `_send_nav_goal()` → `_on_box_result()`.
- Vehicle detection is expected to publish semantic 3D boxes on `/aurora_semantic/vehicle_bounding_boxes`, which the inspection manager uses to compute a goal in `slamware_map` and transform to `map`.

Code references:
- `_vehicle_boxes_cb()` / `_process_vehicle_boxes()` (around L1265–1349).
- `_dispatch_box_goal()` (around L1622–1823).
- `_get_current_pose()` (around L1848–1866).
- `_on_box_result()` (around L2038–2067).

### New structured logs added (to trace the chain in future runs)
- `MISSION_STATE: <state>` — emitted on every state transition.
- `DETECTION_EVENT: ...` — emitted when a vehicle anchor is published.
- `GOAL_COMPUTED: ...` — emitted on goal computation with pose, yaw, and offset.
- `NAV_COMMAND_SENT` / `NAV_COMMAND_FAILED` — emitted on goal dispatch attempt.
- `NAV_FEEDBACK: ...` — emitted on Nav2 result.
- `DETECTION_MISSING` / `DETECTION_FILTERED` — emitted when detection data is absent or filtered out.

## Observed chain (from accessible logs)
1. **Aurora SDK fails to connect** → no semantic/depth streams.
   - `Failed to connect to the selected device` (journald line 69).
   - `No semantic or depth received after 5s...` (line 77 and repeated).
2. **Semantic fusion configured but starved** (no inputs for boxes).
   - `semantic=/slamware_ros_sdk_server_node/semantic_labels + depth=/slamware_ros_sdk_server_node/depth_image_raw -> /aurora_semantic/vehicle_bounding_boxes` (lines 64–65).
3. **Segmentation processor runs but computes no 3D boxes**.
   - `No 3D bounding boxes computed from 3 segmented objects` (line 1146).
4. **Inspection manager starts mission; TF fails immediately**.
   - `State transition: IDLE -> SEARCH_VEHICLE` (line 307).
   - `Could not get current pose ... slamware_map and base_link ... not part of the same tree` (line 308).
   - `TF watchdog ... Pausing mission` (line 311).
5. **Search rotation attempted; goal rejected** (no vehicle detections).
   - `No vehicle detected after 255.0s. Rotating to search (attempt 1/8).`
   - `Search rotation goal rejected.` (lines 1072–1081).
6. **Nav2 spin behavior is configured and executed** (can produce in-place rotations).
   - `behavior_server: Creating behavior plugin spin...` (lines 205–207).
   - `behavior_server: Running spin / Turning 1.57 for spin behavior.` (from `behavior_server_3097` log).
7. **Motion command path is gated** (`cmd_vel_nav` → `cmd_vel`), so forward motion can be blocked if `/stereo/navigation_permitted` is false.
   - `depth_gate: cmd_vel_nav -> cmd_vel gated by /stereo/navigation_permitted` (line 66).

## Breakpoint (most likely)
The chain breaks **before goal computation** due to:
- Aurora SDK connection failures → no semantic/depth → no 3D boxes.
- TF tree disconnected (`slamware_map` ↔ `base_link`) → `_get_current_pose()` fails, so `_dispatch_box_goal()` returns `False`.

## Evidence snippets (logs)
```
69:2026-02-25T13:00:51+0000 ... [slamware ros sdk server]: Failed to connect to the selected device
77:2026-02-25T13:00:55+0000 ... [aurora_semantic_fusion]: No semantic or depth received after 5s...
308:2026-02-25T13:01:52+0000 ... [inspection_manager]: Could not get current pose: ... slamware_map ... base_link ...
311:2026-02-25T13:01:53+0000 ... [inspection_manager]: TF watchdog: slamware_map->base_link invalid > 0.2s. Pausing mission.
1146:2026-02-25T13:06:14+0000 ... [segmentation_processor_node]: No 3D bounding boxes computed from 3 segmented objects
1072:2026-02-25T13:06:07+0000 ... [inspection_manager]: No vehicle detected after 255.0s. Rotating to search (attempt 1/8).
1081:2026-02-25T13:06:07+0000 ... [inspection_manager]: Search rotation goal rejected.
```

## Parameters in effect
- `PRODUCTION_CONFIG.yaml`:
  - `world_frame: slamware_map`
  - `base_frame: base_link`
  - `vehicle_boxes_topic: /aurora_semantic/vehicle_bounding_boxes`
  - `min_vehicle_probability: 0.5`

## Confidence
**Medium**: TF failures and Aurora connection failures are explicit in logs. Bounding box overlays were seen by the user but not persisted here, so the exact detection-to-goal breakpoint for that specific visual run must be correlated with logs from the visualization host.
