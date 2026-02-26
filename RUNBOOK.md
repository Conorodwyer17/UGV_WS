# Runbook — Autonomous Tire Inspection Robot (Aurora 2.11)

## Detection Pipeline

- **Vehicles:** Aurora built-in COCO80 semantic segmentation (car, truck, bus, motorcycle, bicycle) via `aurora_semantic_fusion` → `/aurora_semantic/vehicle_bounding_boxes`. If Aurora does not publish semantic, fusion falls back to YOLO vehicle boxes from `/darknet_ros_3d/bounding_boxes`.
- **Tires:** YOLO `best.pt` via `ultralytics_node` + `segmentation_processor` → `/darknet_ros_3d/bounding_boxes` (inspection_manager switches mode to `inspection` at the vehicle).

---

## Mission flow (end-to-end)

All 3D bounding boxes are in **`slamware_map`** (Aurora map frame). Inspection manager uses `world_frame: slamware_map` and looks up robot pose in the same frame. Navigation goals are computed in slamware_map, then transformed to **`map`** for Nav2 (identity transform via `map` → `slamware_map` in nav_aurora).

| Step | State / Topic | What happens |
|------|----------------|---------------|
| 1 | IDLE → SEARCH_VEHICLE | Mission starts; mode = navigation (vehicles from `/aurora_semantic/vehicle_bounding_boxes` or YOLO fallback). |
| 2 | Vehicle detected | Boxes saved; transition to WAIT_VEHICLE_BOX → approach goal dispatched (offset from vehicle center in slamware_map). |
| 3 | APPROACH_VEHICLE | Nav2 drives to standoff; on arrival → WAIT_TIRE_BOX; mode = inspection (tires from `/darknet_ros_3d/bounding_boxes`, class `car-tire`). |
| 4 | Tire detected | Goal dispatched to tire (tire_offset); on arrival → INSPECT_TIRE → photo capture. |
| 5 | Repeat | Next tire or NEXT_VEHICLE until DONE. |

**Topic alignment (PRODUCTION_CONFIG):** `detection_topic` = `/darknet_ros_3d/bounding_boxes`, `vehicle_boxes_topic` = `/aurora_semantic/vehicle_bounding_boxes`, `world_frame` = `slamware_map`. Segment_3d config uses `working_frame: slamware_map` so published boxes match. **Aurora topics:** All Aurora (slamware_ros_sdk_server_node) topics use the full prefix `/slamware_ros_sdk_server_node/` (e.g. `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/left_image_raw`). Do not subscribe to legacy short names like `/odom` or `/scan`; see `docs/AURORA_TOPIC_NODE_REFERENCE.md` for the full list. **Detection health:** inspection_manager logs `detection_stream_stale` or `vehicle_boxes_stream_stale` if those topics stop publishing for more than `detection_stale_s`/`vehicle_boxes_stale_s`.

**Localization (Aurora):** The robot always knows where it is and where objects are. Robot pose comes from TF `slamware_map` → `base_link` (Aurora). Bounding boxes (vehicle, tire) are in `slamware_map`. Goals are built in that frame and sent to Nav2 as `map` (identity when map ≡ slamware_map). Each photo is logged with map pose, tire position (FL/FR/RL/RR), and vehicle anchor.

**No “wait for next message” at vehicle:** When a vehicle is detected (or when moving to the next vehicle), the approach goal is dispatched on the first tick in WAIT_VEHICLE_BOX using the box already in hand—so the robot always approaches the vehicle (no spin loop), then goes to each tire and captures a photo.

**TF required before mission start:** The mission does not leave IDLE until TF (`slamware_map` → `base_link`) is valid (wait up to `tf_wait_timeout`, default 60 s). If TF never appears, the mission goes to ERROR (`tf_unavailable_at_start`). This avoids starting in SEARCH_VEHICLE and then immediately pausing due to TF watchdog. If TF fails during the mission, any in-flight Nav2 goal is cancelled so the robot does not keep spinning.

**TF stability at start:** Mission does not leave IDLE until TF has been valid for **5 s** in a row (`tf_stable_s` in PRODUCTION_CONFIG), so Aurora must be connected and streaming odom→base_link before the mission can start.

**Aurora TF and launch:** See `docs/AURORA_TF_AND_LAUNCH.md` for the full TF chain (slamware_map→odom→base_link), what can go wrong, and how to run **Aurora-only diagnostics**: start Aurora alone, then run `bash scripts/aurora_tf_diagnostic.sh` to verify TF and topics before full launch.

**Forensics:** If the robot spins or makes no progress, see `docs/MISSION_FORENSIC_REPORT_SPIN_2026-02-24.md` and check `logs/mission_latest.jsonl` for `tf_watchdog`, `tick_skipped_tf_invalid`, `approach_dispatch_attempt`, or `wait_vehicle_timeout`.

---

## Bulletproof mission guarantees

These ensure the mission does not spin in circles, knows where it is and where it’s going, and follows the plan to completion or fails safely.

| Guarantee | How it’s enforced |
|-----------|-------------------|
| **TF before start** | Mission does not leave IDLE until `slamware_map`→`base_link` is valid and stable for **5 s** (`tf_stable_s`). If TF never appears within `tf_wait_timeout` (60 s), transition to ERROR. |
| **Nav2 before start** | Mission does not leave IDLE until Nav2 `navigate_to_pose` action server is available (up to `nav2_wait_timeout`, 90 s). Otherwise ERROR. |
| **No spin at vehicle** | Approach is dispatched **once per entry** into WAIT_VEHICLE_BOX when we have a vehicle box (from SEARCH, NEXT_VEHICLE, or after TURN_IN_PLACE_VEHICLE). We only transition to APPROACH_VEHICLE when `_dispatch_box_goal` **succeeds** (TF + Nav2 OK). |
| **Goal rejected** | If Nav2 rejects the goal, we immediately return to WAIT_VEHICLE_BOX (or WAIT_TIRE_BOX for tires) so we can retry on next tick/callback instead of staying stuck in APPROACH_VEHICLE. |
| **TF lost mid-mission** | TF watchdog: if TF is invalid for > `tf_watchdog_timeout` (0.2 s), we **cancel the in-flight Nav2 goal** so the robot stops; we pause ticks until TF is valid again. After `tf_unavailable_abort_s` (60 s) of continuous TF failure, transition to ERROR. |
| **Nav stuck** | If we stay in APPROACH_VEHICLE or INSPECT_TIRE longer than `approach_timeout_s` (120 s), we cancel the goal and return to WAIT_VEHICLE_BOX or WAIT_TIRE_BOX to retry or rotate. |
| **Spin protection** | If the same “return” state (e.g. WAIT_VEHICLE_BOX after TURN_IN_PLACE_VEHICLE) is entered `max_state_repeats` (3) times in a row without progress, transition to ERROR. |
| **Hard timeout** | Mission aborts to ERROR after `hard_mission_timeout` (1800 s). |
| **Same DDS everywhere** | `full_bringup.launch.py` and `aurora_bringup.launch.py` set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; `startup.sh` also sets it. All nodes see the same TF and topics. |

**Result:** The robot either completes the mission (detect → approach car → tires → photos) or fails into ERROR with a clear cause (TF, Nav2, timeout, spin protection). It does not spin indefinitely or “forget” where it’s going.

---

## Pre-Flight Checklist

| Step | Command / Check | Expected |
|------|-----------------|----------|
| 1 | `ping -c 1 192.168.11.1` | Aurora reachable |
| 2 | `ls /dev/ttyTHS1` (or UART) | Port exists |
| 3 | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Set in launch terminal if not using startup.sh (full_bringup and aurora_bringup set it automatically) |
| 4 | `source ~/ugv_ws/install/setup.bash` | Workspace sourced |
| 5 | `colcon build --packages-select ugv_nav ugv_base_driver segmentation_3d inspection_manager` | Build succeeds |
| 6 | `best.pt` in `~/ugv_ws/` or `~/ugv_ws/src/Tyre_Inspection_Bot/` | Tire model exists |

Or run the script (after full stack or at least Aurora is running):
```bash
bash scripts/aurora_pre_mission_checklist.sh
```
**If TF fails but topics are present:** The checklist and the running stack must use the same RMW. Always use `scripts/mission_launch.sh` or `scripts/startup.sh` to start the stack (they set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`). `mission_launch.sh` now sources the workspace and sets RMW before running the checklist so both match. If you run the checklist in a **second** terminal, run `source install/setup.bash` and `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` there first.

**Aurora-only diagnostic (recommended before first full launch):** In one terminal start only Aurora; in another run the TF diagnostic. This confirms the robot pose (TF) and Aurora topics before adding Nav2/perception.
```bash
# Terminal 1
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
# Terminal 2 (after Aurora shows "Connected to the selected device")
bash scripts/aurora_tf_diagnostic.sh
```

---

## Full system on launch (power on → full mission)

One command runs the **entire** tire inspection pipeline from power-on:

1. **Aurora** connects and publishes map, odom, scan, depth, images, TF.
2. **Segment_3d** (8s): vehicle boxes from Aurora semantic or YOLO fallback → `/aurora_semantic/vehicle_bounding_boxes`; tires from YOLO + depth → `/darknet_ros_3d/bounding_boxes`.
3. **Nav2** (10s + 15s): costmaps, planner, controller; **lifecycle** (45s after nav start) brings Nav2 active; **depth_gate** forwards `cmd_vel_nav` → `/cmd_vel` when permitted.
4. **Inspection manager + photo capture** (120s): loads PRODUCTION_CONFIG (`start_mission_on_ready: true`, `dry_run: false`), waits for `navigate_to_pose` (up to 90s), then **auto-starts**: SEARCH_VEHICLE → detect cars → approach each car → WAIT_TIRE_BOX → detect tires → approach each tire → INSPECT_TIRE → trigger photo → VERIFY_CAPTURE → next tire or NEXT_VEHICLE until DONE.
5. **Motor driver** (from start): subscribes to `/cmd_vel`, sends to ESP32 for motion.

**Single command (no dry_run):**
```bash
bash scripts/mission_launch.sh
# or: bash scripts/startup.sh
# or: ros2 launch ugv_nav full_bringup.launch.py
```
Do **not** pass `dry_run:=true` for real inspection and photos. Ensure Aurora is on and `best.pt` is in place before launch.

---

## Launch

### Option A: Mission Launch (recommended)

Runs pre-flight checklist, then starts full stack:

```bash
bash scripts/mission_launch.sh
bash scripts/mission_launch.sh ip_address:=192.168.11.1
```

### Option B: Startup only

```bash
bash scripts/startup.sh
bash scripts/startup.sh use_motor_driver:=false dry_run:=true  # validate without motion
```

### Option C: Manual (separate terminals)

1. Aurora: `ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1`
2. Motor: `ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:=/dev/ttyTHS1`
3. Nav2: `ros2 launch ugv_nav nav_aurora.launch.py` (wait ~2 min)
4. Detection: `ros2 launch segmentation_3d segment_3d.launch.py` (YOLO + depth + fusion; fusion uses YOLO vehicle boxes if Aurora semantic not published)
5. Inspection: `ros2 launch inspection_manager inspection_manager.launch.py`

**Note:** Inspection manager starts 120s after launch so Nav2 lifecycle can complete; it still waits for Nav2 `navigate_to_pose` (up to 90s) then auto-starts the mission. PRODUCTION_CONFIG is auto-loaded from workspace.

### Vehicle detection only (see the car in RViz)

With Aurora already running and looking at a car:

1. **Terminal 1 (Aurora):** `ros2 launch ugv_nav aurora_bringup.launch.py`
2. **Terminal 2 (perception):** `ros2 launch segmentation_3d segment_3d.launch.py`
3. **Terminal 3 (markers):** `ros2 launch segmentation_3d vehicle_detection_visualize.launch.py fallback_vehicle_boxes_topic:=/darknet_ros_3d/bounding_boxes`
4. **RViz:** Fixed frame `slamware_map`, add by topic `/aurora_semantic/vehicle_markers` (MarkerArray).

**Validate pipeline:** `bash scripts/validate_vehicle_detection.sh` (run after step 2; checks topics and rates).

### Option D: Auto-start on boot (headless)

For unattended runs (e.g. parking lot, no WiFi):

1. **Install systemd service:**
   ```bash
   sudo cp scripts/ugv_mission.service /etc/systemd/system/
   # Edit User/Group and paths if not conor /home/conor
   sudo systemctl daemon-reload
   sudo systemctl enable ugv_mission
   ```

2. **On boot:** Mission launches automatically (SKIP_PREFLIGHT=1). Logs go to `~/ugv_ws/logs/`.

3. **Control:**
   ```bash
   sudo systemctl start ugv_mission   # start now
   sudo systemctl stop ugv_mission    # stop
   sudo systemctl status ugv_mission
   journalctl -u ugv_mission -f       # follow console output
   ```

4. **After mission (reconnect WiFi):** Copy `~/ugv_ws/logs/mission_latest.jsonl` and `mission_report_latest.json` to review what happened.

---

## Mission logging (offline review)

Logs are written to disk (no WiFi needed). **On each mission start**, any existing `mission_latest` / `mission_report_latest` are moved to `logs/archive/` with a timestamp so the current run is always the "latest" and past runs are preserved.

| File | Description |
|------|-------------|
| `~/ugv_ws/logs/mission_latest.jsonl` | Current run: JSONL event stream (mission_start, state_transition, vehicle_detected, nav_result, photo_captured, mission_end, etc.) |
| `~/ugv_ws/logs/mission_report_latest.json` | Current run: final mission summary (vehicles, tires, success, duration) |
| `~/ugv_ws/logs/archive/mission_YYYY-MM-DD_HH-MM-SS.jsonl` | Previous runs: timestamped event log |
| `~/ugv_ws/logs/archive/mission_report_YYYY-MM-DD_HH-MM-SS.json` | Previous runs: timestamped report |

Configured via PRODUCTION_CONFIG: `mission_log_path`, `mission_report_path`. After reconnecting post-mission, copy `logs/` (or just `logs/archive/`) to review all runs.

---

## Map: Build vs Load

The Aurora device holds the map. Use RoboStudio or device UI to:
- **Build:** SLAM → Clear Map
- **Load:** Load a pre-saved map before starting the stack

---

## Verification

**Correct ROS2 syntax:** `tf2_echo` does **not** support `-n 1`; use `ros2 run tf2_ros tf2_echo <source_frame> <target_frame>` (optional: `-r 1` for 1 Hz). For one message from a topic use `ros2 topic echo <topic> --once` (not `-n 1`).

| Check | Command |
|-------|---------|
| Aurora odom | `ros2 topic echo /slamware_ros_sdk_server_node/odom --once` |
| Semantic vehicles | `ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once` |
| 3D boxes (tires) | `ros2 topic echo /darknet_ros_3d/bounding_boxes --once` |
| Inspection state | `ros2 topic echo /inspection_state` or `--once` for one message |
| Mission state (one shot) | `ros2 topic echo /inspection/mission_state --once` |
| Mission report | `ros2 topic echo /inspection_manager/mission_report` |
| TF | `ros2 run tf2_ros tf2_echo slamware_map base_link` (or `map base_link`) |
| Nav2 ready | `ros2 action info /navigate_to_pose` — must show **Action servers: 1** |

**Mission readiness (TF + Nav2):** After full bringup, run `bash scripts/mission_readiness_check.sh`. It checks TF and that `/navigate_to_pose` has one action server; exit 0 only when both pass. Ensure launch log shows "Nav2 lifecycle bringup complete" before expecting the mission to start.

**Headless boot (no Wi‑Fi) logging:** `ugv_mission.service` appends systemd output to `~/ugv_ws/logs/mission_service.log` so logs persist across power‑off. Mission logs and reports are written to `~/ugv_ws/logs/mission_latest.jsonl` and `~/ugv_ws/logs/mission_report_latest.json` with per‑event fsync enabled (`mission_log_fsync: true`).

### Dry run (no motion)

Run `ros2 launch ugv_nav full_bringup.launch.py dry_run:=true`. Expected:

- **Launch:** Aurora, motor_driver, segment_3d (aurora_semantic_fusion, ultralytics, depth pipeline, segmentation_processor), Nav2, inspection_manager, photo_capture_service start in order. No "Old-style arguments" TF warnings (aurora_bringup uses new-style static_transform_publisher).
- **Vehicle detection:** inspection_manager logs "Received N vehicle box(es) from semantic topic", "Published vehicle_detected", "Saved vehicle position", "Found 1 vehicle(s). Starting inspection of vehicle 1", state **SEARCH_VEHICLE → WAIT_VEHICLE_BOX**.
- **Approach (dry):** "Vehicle detected (...); approaching.", "Computing navigation goal" (slamware_map frame), "Dry run: goal validated, not sending to Nav2", then state advances to **WAIT_TIRE_BOX** (same as full launch after nav arrival). Segmentation mode switches to **inspection** (Model 2 / tires).
- **Full launch (no dry_run):** The same approach goal is sent to Nav2; robot drives to vehicle, then waits for tires; on tire detection, navigates to tire and triggers photo capture. Mission logs rotate into `logs/archive/` on each start.

**Known benign messages during dry run:** `No transform available from camera_depth_optical_frame to slamware_map` (early in segmentation_processor) can appear once until TF is fully up; later callbacks succeed. `RTPS_READER_HISTORY Error` / `Change payload size` during Nav2 lifecycle are DDS quirks; Nav2 still activates. `Message Filter dropping message: frame 'laser'` at startup is timestamp/cache related and usually clears.

---

## Troubleshooting

| Issue | Check | Fix |
|-------|-------|-----|
| No vehicles detected | Aurora semantic topics | Aurora 2.11 firmware; `ros2 topic echo /slamware_ros_sdk_server_node/semantic_labels --once`; check `detection_stream_stale` or `vehicle_boxes_stream_stale` in `logs/mission_latest.jsonl` |
| No tires detected | best.pt, segmentation_mode | Place best.pt in workspace; mode should switch to "inspection" at vehicle |
| Nav2 lifecycle fails | DDS | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| TF lookup failed | Aurora running | Start Aurora first; verify: `ros2 run tf2_ros tf2_echo slamware_map base_link` |
| Mission spins / never leaves SEARCH_VEHICLE | TF or detection | Check `~/ugv_ws/logs/mission_latest.jsonl` for `tf_watchdog`; if present, TF chain is broken (Aurora not publishing odom→base_link). Fix Aurora connectivity, then re-run. |

---

## Stop Mission

Press `Ctrl+C` in the terminal running the launch.

---

## Post-Firmware Audit

After upgrading Aurora firmware to 2.11:

```bash
bash scripts/aurora_post_firmware_audit.sh
```
