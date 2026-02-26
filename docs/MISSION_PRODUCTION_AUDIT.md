# Mission Production Audit — Tire Inspection

## 1. Mission specification (authoritative)

Every time the robot starts and the mission runs with a car in view, the system **must**:

| # | Requirement | Success criteria |
|---|-------------|------------------|
| 1 | **Detect the car** | At least one vehicle (car/truck) appears in 3D detections; vehicle is added to `detected_vehicles` and optionally logged. |
| 2 | **Save / log the car** | Vehicle pose (frame, x, y, z, yaw) is published on `/inspection_manager/vehicle_detected` and persisted in mission log when enabled. |
| 3 | **Know where the car is** | Vehicle position is stored in `slamware_map`; approach and tire goals are computed from the same frame. |
| 4 | **Drive to each tire** | Robot navigates to every tire of the current vehicle (expected 4 per vehicle), with retries on nav failure. |
| 5 | **Take a photo of every tire** | For each tire stop, a photo is triggered; capture result (SUCCESS/FAILURE) is verified; retries up to `max_capture_retries`. |
| 6 | **Flawless execution** | No undefined behaviour: nav failures trigger retry then recovery; timeouts trigger rotation or next vehicle; mission ends in DONE or ERROR with a clear report. |

---

## 2. State machine → requirements mapping

| State | Purpose | Exit conditions (success / failure) |
|-------|---------|-------------------------------------|
| **IDLE** | Wait for start (optional: wait for `/inspection_manager/start_mission`) | → INIT or SEARCH_VEHICLE |
| **INIT** | Optional sensor health check | → SEARCH_VEHICLE (healthy/timeout) |
| **SEARCH_VEHICLE** | Wait for first vehicle detection | → WAIT_VEHICLE_BOX (vehicle seen); → TURN_IN_PLACE_SEARCH (timeout); → DONE (no vehicle after full rotation) |
| **TURN_IN_PLACE_SEARCH** | Rotate to find vehicle | → SEARCH_VEHICLE (after rotation) |
| **WAIT_VEHICLE_BOX** | Wait for selected vehicle in current message | → APPROACH_VEHICLE (box found, goal sent); → TURN_IN_PLACE_VEHICLE / NEXT_VEHICLE (timeout) |
| **APPROACH_VEHICLE** | Navigate to vehicle | → WAIT_TIRE_BOX (nav success); → retry (nav fail, under budget); → NEXT_VEHICLE (nav fail, over budget) |
| **WAIT_TIRE_BOX** | Wait for next un-inspected tire | → INSPECT_TIRE (tire found); → TURN_IN_PLACE_TIRE / NEXT_VEHICLE (timeout) |
| **TURN_IN_PLACE_TIRE** | Rotate to find tires | → WAIT_TIRE_BOX |
| **INSPECT_TIRE** | Navigate to tire | → VERIFY_CAPTURE (nav success); → retry or WAIT_TIRE_BOX (nav fail) |
| **VERIFY_CAPTURE** | Wait for photo result | → WAIT_TIRE_BOX or NEXT_VEHICLE (success/fail after retries) |
| **NEXT_VEHICLE** | Mark vehicle done, select next | → WAIT_VEHICLE_BOX (more vehicles); → DONE (none left) |
| **DONE / ERROR** | Mission end | Report published; mission log finalized. |

---

## 3. Failure modes and mitigations

| Failure | Mitigation |
|---------|------------|
| No vehicle ever detected | Detection timeout → rotate (TURN_IN_PLACE_SEARCH); after `max_rotation_attempts` → DONE with report. |
| Wrong vehicle approached (multiple in view) | WAIT_VEHICLE_BOX selects box **closest to** `detected_vehicles[current_vehicle_idx]` position (not just highest probability). |
| Nav2 rejects goal | Log and retry (approach/tire); after `nav_retry_budget` → recover (NEXT_VEHICLE or next tire). |
| Nav2 returns FAILED/ABORTED | Same as above: check `result.status == 4` (SUCCEEDED); else retry then recover. |
| TF invalid / map mismatch | TF watchdog pauses mission; goal transform falls back to identity when slamware_map ≡ map. |
| Photo capture timeout / failure | VERIFY_CAPTURE timeout → retry up to `max_capture_retries`; then proceed to next tire and log failure. |
| Same state repeated indefinitely | `max_state_repeats` → transition to ERROR. |
| Mission runs too long | `hard_mission_timeout` → DONE with report. |

---

## 4. Pre-mission checklist (production)

Before starting the mission:

1. **Aurora / sensors**: Device online; Aurora topics active: `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/depth_image_raw`, `/slamware_ros_sdk_server_node/left_image_raw`, etc. (see `docs/AURORA_TOPIC_NODE_REFERENCE.md` and `docs/AURORA_MISSION_SETUP.md`).
2. **Frames**: `slamware_map` and `map` defined (e.g. static transform); TF tree valid.
3. **Detection**: `/darknet_ros_3d/bounding_boxes` publishing; segment_3d and depth pipeline running.
4. **Nav2**: NavigateToPose action server available.
5. **Inspection manager**: Launched with photo_capture_service; parameters loaded (e.g. from `PRODUCTION_CONFIG.yaml`).
6. **Optional**: Run `scripts/aurora_pre_mission_checklist.sh` and fix any failures.

Start the mission only after the checklist passes. Use `/inspection_manager/start_mission` (Bool) if `start_mission_on_ready` is false.

---

## 5. Success criteria per run

- **Detect**: All vehicles in view are in `detected_vehicles` (no duplicate within `vehicle_duplicate_tolerance`).
- **Log**: Each vehicle published on `vehicle_detected`; mission log (if enabled) contains vehicle and tire events.
- **Navigate**: Every approach and tire goal either succeeds (status SUCCEEDED) or exhausts retries and recovery is taken.
- **Photos**: Every tire stop triggers capture; each capture is verified (success or logged failure after retries).
- **Termination**: Mission ends in DONE (all vehicles processed) or ERROR (spin/timeout); report published on `mission_report`.

---

## 6. Mission start and logging

- **Start**: By default the mission starts on first tick (IDLE → SEARCH_VEHICLE or INIT). Set `start_mission_on_ready: false` and publish `True` on `/inspection_manager/start_mission` (Bool) to start only when ready (e.g. after pre-flight checklist).
- **Mission log**: Set `mission_log_path` to a file path (e.g. `~/mission_logs/mission.jsonl`) to append JSONL lines: `mission_start`, `vehicle_detected`, `tire_approach`, `photo_captured`, `mission_end`.
- **Launch**: Use `config_file:=/path/to/PRODUCTION_CONFIG.yaml` when launching inspection_manager so all production params are loaded.

## 7. Config and topics (frozen in PRODUCTION_CONFIG.yaml)

- **Frames**: `world_frame: slamware_map`, `base_frame: base_link`, `map_frame: map`
- **Detection**: `detection_topic: /darknet_ros_3d/bounding_boxes`
- **Photo**: `photo_capture_topic`, `capture_result_topic`, `capture_metadata_topic`, `vehicle_detected_topic`
- **Retries**: `nav_retry_budget`, `max_capture_retries`, `max_rotation_attempts`, `max_state_repeats`
- **Timeouts**: `detection_timeout`, `capture_verify_timeout_s`, `hard_mission_timeout`, `tf_watchdog_timeout`

See `PRODUCTION_CONFIG.yaml` and `docs/AURORA_TOPIC_NODE_REFERENCE.md` for Aurora topic names (all under `/slamware_ros_sdk_server_node/`).
