# Correct Log Identification (2026-02-25)

## Accessible logs for 2026-02-25
- `/home/conor/ugv_ws/logs/archive/mission_2026-02-25_13-01-52.jsonl`
  - Size: 12493 bytes (stat)
  - Contains `mission_start` and repeated `SEARCH_VEHICLE` ↔ `TURN_IN_PLACE_SEARCH`
  - No `vehicle_detected` events
- `/home/conor/ugv_ws/logs/archive/mission_2026-02-25_12-37-38.jsonl`
  - Repeated `SEARCH_VEHICLE` ↔ `TURN_IN_PLACE_SEARCH`
  - No `vehicle_detected` events
- `/home/conor/ugv_ws/logs/archive/mission_2026-02-25_12-20-41.jsonl`
  - Repeated `SEARCH_VEHICLE` ↔ `TURN_IN_PLACE_SEARCH`
  - No `vehicle_detected` events
- `/home/conor/.ros/log/2026-02-25-13-00-39-393228-conor-desktop-3525/launch.log`
  - Launch-only log; no detection entries
- `/home/conor/.ros/log/python3_3266_1772023057280.log`
  - Inspection manager runtime log for 12:37 run (rotation attempts logged)
- `/home/conor/.ros/log/python3_5277_1772027647034.log`
  - Inspection manager runtime log for 13:54 run (rotation attempts logged)
- `/home/conor/.ros/log/python3_3699_1772024459135.log`
  - Ultralytics segmentation log (publishing segmented objects)
- `/home/conor/.ros/log/segmentation_processor_node_3780_1772024451896.log`
  - 3D bounding box computation log (no 3D boxes in this run)
- `journalctl -u ugv_mission` (captured to `analysis/journal_ugv_mission_2026-02-25_1240-1310.log`)
  - Shows segmentation outputs but not car/truck detections

## Evidence snippets
- Mission JSONL shows only state transitions, no vehicle detections:
  - `{"event": "mission_start", ... "t": 1772023058.7168078}`
  - `{"event": "state_transition", "data": {"from": "IDLE", "to": "SEARCH_VEHICLE"...}}`
- Journald shows segmentation outputs with non-vehicle classes and no 3D boxes:
  - `ultralytics_node`: chair/couch/clock detections
  - `segmentation_processor_node`: “No 3D bounding boxes computed”
- Journald shows Aurora semantic/depth missing during mission window:
  - `2026-02-25T13:00:55+0000 ... [aurora_semantic_fusion] No semantic or depth received after 5s...`
  - `2026-02-25T13:01:51+0000 ... [aurora_semantic_fusion] No semantic or depth received after 5s...`
- AORA SDK label vocabulary includes `car` (label map, not a detection):
  - `2026-02-25T13:06:06+0000 ... [slamware_ros_sdk_server_node] Label ID: 3, Name: car`

## Bounding box drawing evidence (missing)
No `bbox|bounding|draw_box|rectangle|overlay` entries were found in the accessible logs for 2026-02-25 (`~/.ros/log` and `analysis/journal_ugv_mission_2026-02-25_1240-1310.log`). If boxes were visible in RViz, those visuals were not persisted to file here.

## Persistence check
Only `launch.log` exists under `/home/conor/.ros/log/2026-02-25-13-00-39-393228-conor-desktop-3525`, indicating runtime node logs were captured by journald and not per-node ROS log files.

## Evidence of vehicle detections (adjacent day)
- `/home/conor/ugv_ws/logs/archive/mission_2026-02-24_15-40-47.jsonl`
  - Contains `vehicle_detected` events and `WAIT_VEHICLE_BOX` → `APPROACH_VEHICLE` transitions.
  - Example:
    - `{"event": "vehicle_detected", ... "confidence": 0.9295}`
    - `{"event": "state_transition", "data": {"from": "WAIT_VEHICLE_BOX", "to": "APPROACH_VEHICLE"...}}`

## Likely missing sources
- AORA device logs (semantic fusion / visualization)
- RViz logs or screen capture on operator workstation
- ROS log directory on the AORA host if separate from Jetson

## Next log-hunting steps (remediation)
- Check alternate ROS log dirs: `echo $ROS_LOG_DIR`, `ls -la ~/.ros/log`.
- Search on AORA/Jetson: `rg -n "bbox|bounding|car|truck|draw_box" ~/.ros/log /var/log`.
