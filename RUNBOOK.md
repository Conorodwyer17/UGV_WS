# Runbook — Autonomous Tyre Inspection Robot (Aurora 2.11)

## Canonical stack and topics

- **Building the workspace:** `cd ~/ugv_ws && colcon build --symlink-install && source install/setup.bash`. Place `best_fallback.pt` (or `best_fallback.engine`) in `src/Tyre_Inspection_Bot/` or workspace root. See [Clean build and cleanup](#clean-build-and-cleanup).
- **Running a mission:** `./scripts/start_mission.sh` (recommended: disk check, Jetson performance, optional verification, then launch). Or `./scripts/startup.sh`; mission auto-starts when TF and Nav2 are ready. See [Option A: Start mission](#option-a-start-mission-recommended).
- **Canonical launch:** `full_bringup.launch.py` (e.g. via `scripts/mission_launch.sh` or `scripts/startup.sh`). For real missions on the Jetson, use this launch — it starts the Aurora SDK, motor driver, Nav2, perception, and inspection manager. Do not use `inspection_full_mission.launch.py` alone (it does not start the Aurora SDK by default).
- **Mission node:** **inspection_manager_node**. Flow: IDLE → SEARCH_VEHICLE → WAIT_VEHICLE_BOX → (first goal = **nearest corner**, then tyres 2–4 by distance). See [MISSION_PIPELINE.md](docs/MISSION_PIPELINE.md) for tyre order and scenario behaviour.
- **First goal:** Always the **nearest** of the four tire corners (not “front left first” or clockwise). Flow: WAIT_VEHICLE_BOX → APPROACH_VEHICLE → WAIT_TIRE_BOX → INSPECT_TIRE / FACE_TIRE → VERIFY_CAPTURE (repeat tires, then NEXT_VEHICLE or DONE).
- **Vehicle boxes:** Primary `/aurora_semantic/vehicle_bounding_boxes`. YOLO vehicle node is **disabled by default** (`use_vehicle_yolo:=false`) to save GPU/CPU; use `use_vehicle_yolo:=true` for YOLO fallback (feasible on 16 GB Jetson as redundant detection source).
- **Tyre boxes:** `/darknet_ros_3d/tire_bounding_boxes` (YOLO) and merged `/tire_bounding_boxes_merged` when PCL fallback enabled.
- **Tyre class:** `wheel` (best_fallback.pt), per PRODUCTION_CONFIG `tire_label`.
- **Start mission:** With default `start_mission_on_ready: true`, the mission auto-starts when TF and Nav2 are ready. Otherwise publish once: `ros2 topic pub --once /inspection_manager/start_mission std_msgs/msg/Bool "{data: true}"`. Goals are in `map` frame; Aurora TF chain (map → slamware_map → odom → base_link) is required.
- **Field readiness:** Pre-field checklist: [docs/ACCEPTANCE_CRITERIA.md](docs/ACCEPTANCE_CRITERIA.md). Mission report includes per-tire `goal_source` (`planned` / `detection` / `detection_refined`) for observability.

---

## Detection Pipeline

- **Vehicles:** Aurora built-in COCO80 semantic segmentation via `aurora_semantic_fusion` → `/aurora_semantic/vehicle_bounding_boxes`. With default `use_vehicle_yolo:=false`, the YOLO vehicle node is not launched; vehicle boxes come from Aurora only. Set `use_vehicle_yolo:=true` for YOLO fallback if needed.
- **Tyres:** YOLO `best_fallback.pt`/`.engine` via `ultralytics_tire` + `segmentation_processor_tire` → `/darknet_ros_3d/tire_bounding_boxes`; merged with PCL fallback at `/tire_bounding_boxes_merged`. **16 GB Jetson:** GPU is default (`use_cpu_inference:=false`); 640×640 inference at 10 Hz; TensorRT when engine exists.
- **Disk/GPU cleanup:** Run `bash scripts/cleanup.sh` for a dry run; `bash scripts/cleanup.sh --execute` to remove build artifacts. See [docs/TIRE_DETECTION_TROUBLESHOOTING.md](docs/TIRE_DETECTION_TROUBLESHOOTING.md) for perception issues.

**Perception checklist (pre-mission):** (1) `interested_class_names` includes `wheel` (or configured tyre class) in ultralytics node; (2) segmentation_processor `interested_classes` includes `wheel` and SOR is enabled (config or launch); (3) tire_merger `yolo_stale_s` is set (default 2.0 s); (4) merged boxes use consistent frame_id (`slamware_map`); (5) ultralytics warm-up inference succeeds at startup; (6) TensorRT engine used when present (check logs for 'Using TensorRT engine for inspection').

---

## Mission flow (end-to-end)

All 3D bounding boxes are in **`slamware_map`** (Aurora map frame). Inspection manager uses `world_frame: slamware_map` and looks up robot pose in the same frame. Navigation goals are computed in slamware_map, then transformed to **`map`** for Nav2 (identity transform via `map` → `slamware_map` in nav_aurora).

| Step | State / Topic | What happens |
|------|----------------|---------------|
| 1 | IDLE → SEARCH_VEHICLE | Mission starts; mode = navigation (vehicles from `/aurora_semantic/vehicle_bounding_boxes` or YOLO fallback). |
| 2 | Vehicle detected | Boxes saved; transition to WAIT_VEHICLE_BOX → approach goal dispatched (offset from vehicle centre in slamware_map). |
| 3 | APPROACH_VEHICLE | Nav2 drives to standoff; on arrival → WAIT_TIRE_BOX; mode = inspection (tyres from `/darknet_ros_3d/tire_bounding_boxes`, class `wheel`). |
| 4 | Tyre detected | Goal dispatched to tyre (tire_offset); on arrival → INSPECT_TIRE → FACE_TIRE (optional) → WAIT_WHEEL_FOR_CAPTURE (if capture_require_wheel_detection) → photo when wheel in view (or after timeout if capture_on_wheel_timeout: true). VERIFY_CAPTURE waits for capture_result; then next tyre or NEXT_VEHICLE. |
| 5 | Repeat | Next tyre or NEXT_VEHICLE until DONE. |

**State flow (flat state machine):** IDLE → INIT → SEARCH_VEHICLE → WAIT_VEHICLE_BOX → APPROACH_VEHICLE → WAIT_TIRE_BOX → INSPECT_TIRE → (FACE_TIRE, WAIT_WHEEL_FOR_CAPTURE) → VERIFY_CAPTURE → (next tyre or NEXT_VEHICLE) → DONE / ERROR. Each state has entry/exit logging, timeouts, and error recovery. See mission_state_machine.py and inspection_manager_node.

**Mission report (observability):** Written to `mission_report_path` (e.g. `~/ugv_ws/logs/mission_report_latest.json`). Fields: `mission_duration_s`, `tires_captured`, `tire_capture_log` (per-tyre: success, image_path, distance_m, timestamp, `goal_source`), `tire_goal_sources`, `failures` (list of failure reasons when any tyre skipped or error), `error_states_encountered`, `mission_end_cause`, `all_tires_captured`, `success`. PRODUCTION_CONFIG sets `mission_report_path` and `mission_log_path`.

**Topic alignment (PRODUCTION_CONFIG):** `detection_topic` = `/tire_bounding_boxes_merged` (YOLO + PCL fallback for robustness), `vehicle_boxes_topic` = `/aurora_semantic/vehicle_bounding_boxes`, `world_frame` = `slamware_map`. Segment_3d config uses `working_frame: slamware_map` so published boxes match. **Aurora topics:** All Aurora (slamware_ros_sdk_server_node) topics use the full prefix `/slamware_ros_sdk_server_node/` (e.g. `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/left_image_raw`). Do not subscribe to legacy short names like `/odom` or `/scan`; use full Aurora prefix `/slamware_ros_sdk_server_node/`. **Detection health:** inspection_manager logs `detection_stream_stale` or `vehicle_boxes_stream_stale` if those topics stop publishing for more than `detection_stale_s`/`vehicle_boxes_stale_s`.

**Localisation (Aurora):** The robot always knows where it is and where objects are. Robot pose comes from TF `slamware_map` → `base_link` (Aurora). Bounding boxes (vehicle, tyre) are in `slamware_map`. Goals are built in that frame and sent to Nav2 as `map` (identity when map ≡ slamware_map). Each photo is logged with map pose, tyre position (FL/FR/RL/RR), and vehicle anchor.

**EKF production path (wheel odom fusion):** When ESP32 provides wheel odometry (T:1001 feedback), use `full_bringup.launch.py` with `publish_wheel_odom:=true use_ekf:=true`. robot_localization EKF fuses Aurora odom + IMU + wheel odom; Nav2 uses `/odometry/filtered`. Reduces short-term drift during slow approach. See `ugv_nav/param/ekf_aurora.yaml` and `nav_aurora_ekf.yaml`.

**No “wait for next message” at vehicle:** When a vehicle is detected (or when moving to the next vehicle), the approach goal is dispatched on the first tick in WAIT_VEHICLE_BOX using the box already in hand—so the robot always approaches the vehicle (no spin loop), then goes to each tire and captures a photo.

**TF required before mission start:** The mission does not leave IDLE until TF (`slamware_map` → `base_link`) is valid (wait up to `tf_wait_timeout`, default 60 s). If TF never appears, the mission goes to ERROR (`tf_unavailable_at_start`). This avoids starting in SEARCH_VEHICLE and then immediately pausing due to TF watchdog. If TF fails during the mission, any in-flight Nav2 goal is cancelled so the robot does not keep spinning.

**TF stability at start:** Mission does not leave IDLE until TF has been valid for **5 s** in a row (`tf_stable_s` in PRODUCTION_CONFIG), so Aurora must be connected and streaming odom→base_link before the mission can start.

**Forensics:** If the robot spins or makes no progress, check `logs/mission_latest.jsonl` for `tf_watchdog`, `tick_skipped_tf_invalid`, `approach_dispatch_attempt`, or `wait_vehicle_timeout`.

---

## Bulletproof mission guarantees

These ensure the mission does not spin in circles, knows where it is and where it’s going, and follows the plan to completion or fails safely.

| Guarantee | How it’s enforced |
|-----------|-------------------|
| **TF before start** | Mission does not leave IDLE until `slamware_map`→`base_link` is valid and stable for **5 s** (`tf_stable_s`). If TF never appears within `tf_wait_timeout` (60 s), transition to ERROR. |
| **Nav2 before start** | Mission does not leave IDLE until Nav2 `navigate_to_pose` action server is available (up to `nav2_wait_timeout`, 90 s). Otherwise ERROR. |
| **No spin at vehicle** | Approach is dispatched **once per entry** into WAIT_VEHICLE_BOX when we have a vehicle box (from SEARCH, NEXT_VEHICLE, or after TURN_IN_PLACE_VEHICLE). We only transition to APPROACH_VEHICLE when `_dispatch_box_goal` **succeeds** (TF + Nav2 OK). |
| **Goal rejected** | If Nav2 rejects the goal, we immediately return to WAIT_VEHICLE_BOX (or WAIT_TIRE_BOX for tyres) so we can retry on next tick/callback instead of staying stuck in APPROACH_VEHICLE. |
| **TF lost mid-mission** | TF watchdog: if TF is invalid for > `tf_watchdog_timeout` (0.2 s), we **cancel the in-flight Nav2 goal** so the robot stops; we pause ticks until TF is valid again. After `tf_unavailable_abort_s` (60 s) of continuous TF failure, transition to ERROR. |
| **Recovery and watchdog** | Recovery: Nav2 behaviour tree runs ClearCostmap → Wait → BackUp on failure. Inspection manager tracks `number_of_recoveries`; after `max_recoveries_before_skip` (and still far from goal), the current tyre can be skipped (recovery-aware skip). No process-level auto-restart of nodes (optional future work). See [TIRE_DETECTION_TROUBLESHOOTING.md](docs/TIRE_DETECTION_TROUBLESHOOTING.md). |
| **Nav stuck** | If we stay in APPROACH_VEHICLE or INSPECT_TIRE longer than `approach_timeout_s` (120 s), we cancel the goal and return to WAIT_VEHICLE_BOX or WAIT_TIRE_BOX to retry or rotate. |
| **Spin protection** | If the same “return” state (e.g. WAIT_VEHICLE_BOX after TURN_IN_PLACE_VEHICLE) is entered `max_state_repeats` (3) times in a row without progress, transition to ERROR. |
| **Hard timeout** | Mission aborts to ERROR after `hard_mission_timeout` (1800 s). |
| **Same DDS everywhere** | `full_bringup.launch.py` and `aurora_bringup.launch.py` set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; `startup.sh` also sets it. All nodes see the same TF and topics. |
| **One goal at a time** | `send_nav_goal` refuses to send a new goal if one is already in flight. Nav2 replans the path dynamically as the costmap updates; we do **not** send new goals to "update" the path. |
| **No goal flooding** | `nav_goal_min_interval_s` (1.0 s default) enforces a minimum delay between dispatches. Set to 0 to disable. Ensures the robot can respond before the next goal is sent. |
| **All four tyres** | The mission plans **all four** tyre positions (both sides of the vehicle) at commit time. It only transitions to DONE/NEXT_VEHICLE when `expected_tires_per_vehicle` (4) are captured. Nav2 uses the full map and costmap to path **around** the vehicle to the far side. The mission never 'finishes' with only two tyres unless it goes to ERROR (e.g. spin protection). See [MISSION_PIPELINE.md](docs/MISSION_PIPELINE.md). |

**Result:** The robot either completes the mission (detect → approach car → tires → photos) or fails into ERROR with a clear cause (TF, Nav2, timeout, spin protection). It does not spin indefinitely or “forget” where it’s going.

---

## Navigation (Nav2) and TF tree

**TF tree:** Full chain: map → slamware_map → odom → base_link → base_footprint, camera_left, camera_depth_optical_frame. Required for `verify_system.py` and costmap lookups.

**Nav2 parameters (nav_aurora.yaml):** Controller uses `xy_goal_tolerance: 0.15` m and `yaw_goal_tolerance: 0.25` (tighter yaw e.g. 0.1 can be tried for tire approach but may increase oscillations). `transform_tolerance` is 2.0 s in DWB for Aurora TF timing; a tighter profile (e.g. 1.0 s) is in `nav_aurora_tight_*.yaml` for testing. Recovery behavior tree order: **ClearCostmap** (local then global) → **Wait** → **BackUp** (see `behavior_trees/navigate_to_pose_no_spin.xml`).

**Costmap:** Local and global costmaps use `/slamware_ros_sdk_server_node/scan` and `/camera/depth/points`. Optionally, `/segmentation_processor/registered_pointcloud` can be added as an extra obstacle source if needed.

**Speed filter (vehicle_speed_filter_node):** In `ugv_nav/param/vehicle_speed_filter.yaml`: `vehicle_buffer_m` (default 1.5 m), `speed_percent_in_zone` (e.g. 50), `publish_rate_hz` (2.0). Filter mask is published for Nav2 SpeedFilter.

---

## Jetson Optimization and TensorRT

**16 GB Jetson Orin Nano:** GPU tyre detection is now the default. Expected performance: 640×640 resolution, 10 Hz inference, TensorRT when `best_fallback.engine` exists. OOM is rare on 16 GB; fallbacks below apply mainly to 8 GB or older hardware. CPU fallback: `use_cpu_inference:=true` for 8 GB Jetson, debugging, or if OOM persists.

**startup.sh** (and mission_launch.sh, including ugv_mission.service) runs `nvpmodel -m 0` and `jetson_clocks` for max performance (PATH_FORWARD, acc-qcar2-autonomy). For manual pre-mission tuning, run `sudo nvpmodel -m 0` and `sudo jetson_clocks`.

**TensorRT (faster YOLO inference):** Export best_fallback.pt to TensorRT engine on the Jetson:

```bash
cd ~/ugv_ws && bash scripts/export_tensorrt.sh
```

TensorRT is auto-enabled when `best_fallback.engine` exists at `~/ugv_ws/src/Tyre_Inspection_Bot/best_fallback.engine`. Default export: imgsz=640, workspace=8 GB for 16 GB Jetson. Override with `prefer_tensorrt_inspection:=false` if needed. If export fails, the node falls back to `.pt`. See `scripts/export_tensorrt.sh` for details.

**CUDA OOM on startup:** Rare on 16 GB Jetson. If `ultralytics_tire` crashes with `NvMapMemAllocInternalTagged: error 12` or `Cuda Runtime (out of memory)` during TensorRT engine load, use one of: (1) `prefer_tensorrt_inspection:=false` to use PyTorch .pt (slower but avoids OOM); (2) `model_load_delay_s:=10.0` to delay model load so other nodes settle first (use decimal to avoid type mismatch); (3) export a smaller engine with `IMGSZ=320 WORKSPACE=4 ./scripts/export_tensorrt.sh`. See [docs/TIRE_DETECTION_TROUBLESHOOTING.md](docs/TIRE_DETECTION_TROUBLESHOOTING.md).

**PyTorch and CUDA verification:** Confirm CUDA is available:

```bash
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'No GPU')"
```

Expected: `CUDA available: True` and device name (e.g. Orin). If CUDA is missing on Jetson, install the NVIDIA ARM64 wheel (see Ultralytics assets or FINAL_VERIFICATION_AND_HANDOVER.md).

**System verifier (pre-mission):** Run the Python verifier after sourcing the workspace (and optionally after the stack is up) to check TF tree, required topics, CUDA, and disk space:

```bash
source ~/ugv_ws/install/setup.bash
python3 ~/ugv_ws/scripts/verify_system.py
```

Use `--skip-ros` to only run CUDA and disk checks. This script is also invoked by `bash scripts/pre_mission_verify.sh` when ROS is sourced.

**Simulated detections (no vehicle):** To test state transitions without a real car, run in a separate terminal: `python3 scripts/verify_system.py --simulate --duration 300` — publishes fake vehicle and tire boxes at 2 Hz.

**Manual photo trigger (debugging):** `ros2 service call /photo_capture_service/capture_photo std_srvs/srv/Trigger`

---

## First Boot on New Jetson (16 GB)

Use this checklist when the new 16 GB Jetson arrives. See [SETUP.md](SETUP.md) for full installation steps.

| Step | Action | Expected |
|------|--------|----------|
| 1 | Connect cables | UART (`/dev/ttyTHS1`) to ESP32, Ethernet to Aurora |
| 2 | Verify device permissions | `ls -l /dev/ttyTHS1`; user in `dialout` group |
| 3 | Test Aurora connectivity | `ping -c 1 192.168.11.1` |
| 4 | Run minimal bringup | `ros2 launch ugv_nav full_bringup.launch.py` — check `slamware_ros_sdk_server_node` logs "Connected to the selected device" |
| 5 | Verify topics | `ros2 topic list` — expect slamware_*, aurora_semantic topics |
| 6 | Run simulated mission | `ros2 launch ugv_nav full_bringup.launch.py use_mock:=true sim_tyre_detections:=true` — state machine should cycle |
| 7 | Live vehicle test | Place robot with car in view; run `./scripts/start_mission.sh` |

**Before first live run:** Ensure `best_fallback.pt` is in `src/Tyre_Inspection_Bot/` and TensorRT engine is exported (`bash scripts/export_tensorrt.sh`). GPU inference is default on 16 GB.

---

## Pre-Flight Checklist

| Step | Command / Check | Expected |
|------|-----------------|----------|
| 1 | `ping -c 1 192.168.11.1` | Aurora reachable |
| 2 | `ls /dev/ttyTHS1` (or UART) | Port exists |
| 3 | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Set in launch terminal if not using startup.sh (full_bringup and aurora_bringup set it automatically) |
| 4 | `source ~/ugv_ws/install/setup.bash` | Workspace sourced |
| 5 | `colcon build --packages-select ugv_nav ugv_base_driver segmentation_3d inspection_manager` | Build succeeds |
| 6 | `best_fallback.pt` in `~/ugv_ws/` or `~/ugv_ws/src/Tyre_Inspection_Bot/` | Tire model exists |

**If TF fails but topics are present:** Always use `scripts/mission_launch.sh` or `scripts/startup.sh` to start the stack (they set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`). Ensure Aurora is reachable and TF `slamware_map` → `base_link` is valid before the mission starts.

**Pre-mission verification:** Run `bash scripts/pre_mission_verify.sh` to check Aurora reachability, best_fallback.pt, topics, TF, Nav2, depth points, and inspection_manager params.

**Hardware integration (first-time / before real mission):** Verify: (1) Aurora reachable (`ping -c 1 192.168.11.1`), (2) motor port `/dev/ttyTHS1` present, (3) `LD_LIBRARY_PATH` includes Aurora SDK (add to `~/.bashrc` or use `startup.sh`/`start_mission.sh`), (4) workspace built, (5) tyre model present. Ensure `slamware_ros_sdk_server_node` logs "Connected to the selected device" when the stack starts.

**Acceptance criteria:** See [ACCEPTANCE_CRITERIA.md](docs/ACCEPTANCE_CRITERIA.md) for real-robot validation.

---

## Clean build and cleanup

**When to run cleanup:** When disk space is tight, before a fresh build, or to remove stale build artifacts and optional model/conversion files. Do **not** run with `--execute` when free space is below 5 GB (the script will refuse unless `--force`).

**Cleanup script:** `scripts/cleanup.sh`

- **Dry run:** `bash scripts/cleanup.sh` — shows what would be removed.
- **Execute:** `bash scripts/cleanup.sh --execute` — removes workspace `build/`, `install/`, `log/`.
- **Optional flags:** `--force` (skip disk space check).

**Clean build after cleanup:**

1. Run cleanup: `bash scripts/cleanup.sh --execute` (ensure at least 5 GB free, or use `--force` with care).
2. Source nothing (or only system ROS); environment should not have workspace in `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH` before building.
3. Build: `cd ~/ugv_ws && colcon build --symlink-install`
4. Source: `source ~/ugv_ws/install/setup.bash`

After a clean build, `install/setup.bash` is generated and must be sourced before running any launch or node.

---

## Aurora and mission readiness

When the robot is placed with a car in view, the following must be true for a full autonomous tire inspection (detect → navigate to each tire → photograph).

**Map reset on startup:** By default, the stack clears the Aurora map 6 s after launch so **each mission uses a fresh map**. Aurora then builds from the live scene (LiDAR, depth, point cloud); costmap and vehicle detection reflect what is actually in front of the robot. No loaded/stale map to confuse the area of interest. To disable: `./scripts/startup.sh reset_map_on_startup:=false`.

**Aurora SDK shared library (LD_LIBRARY_PATH):** The `slamware_ros_sdk_server_node` needs the Aurora remote public library. For **interactive use** (terminal), add to your `~/.bashrc`:

```bash
# Aurora ROS2 SDK (use linux_aarch64 on Jetson, linux_x86_64 on x86_64)
export LD_LIBRARY_PATH=~/ugv_ws/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
```

Replace `linux_aarch64` with `linux_x86_64` on a desktop. **startup.sh** and **ugv_mission.service** already prepend this path when present, so launching via `scripts/startup.sh` or the systemd service does not require `~/.bashrc`.

**Required Aurora topics (slamware_ros_sdk_server_node):** `/slamware_ros_sdk_server_node/map`, `/slamware_ros_sdk_server_node/left_image_raw`, `/slamware_ros_sdk_server_node/depth_image_raw`, `/slamware_ros_sdk_server_node/semantic_labels`, `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/odom`.

**What Aurora publishes (slamware_ros_sdk_server_node):**
- **TF:** `odom` → `base_link` (Aurora localization). Static publishers add `map` → `slamware_map` (identity) and `slamware_map` → `odom`, so the full chain is `map` → `slamware_map` → `odom` → `base_link`.
- **Topics:** `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/map`, `/slamware_ros_sdk_server_node/depth_image_raw`, `/slamware_ros_sdk_server_node/semantic_labels`, `/slamware_ros_sdk_server_node/left_image_raw`, `/slamware_ros_sdk_server_node/point_cloud`.

**What the stack adds:**
- **aurora_semantic_fusion** → `/aurora_semantic/vehicle_bounding_boxes` (vehicle 3D boxes in `slamware_map`).
- **segment_3d** → `/darknet_ros_3d/vehicle_bounding_boxes`, `/darknet_ros_3d/tire_bounding_boxes`; **depth_to_registered_pointcloud** → `/camera/depth/points` (costmap obstacle source).
- **Nav2** → `navigate_to_pose` action; costmaps use scan + `/camera/depth/points` + Aurora point_cloud (dynamic obstacle avoidance).
- **inspection_manager** and **photo_capture_service** start **120 s** after launch (full_bringup). Until they appear, the mission cannot run. With `start_mission_on_ready: true`, the mission auto-starts when TF is stable and Nav2 is available; otherwise call `ros2 service call /inspection_manager/start_mission inspection_manager_interfaces/srv/StartMission "{object_id: '', mission_config_json: '{}'}"`.

**Quick readiness check (after 120 s):**
```bash
bash scripts/pre_mission_verify.sh
ros2 node list | grep -E "inspection_manager|photo_capture"
ros2 action info /navigate_to_pose   # Action servers: 1
```
If `inspection_manager` is not in the list, wait for the 120 s delay or confirm full_bringup is running.

**Live check that Aurora is publishing (run in a separate terminal):**
```bash
ros2 topic hz /slamware_ros_sdk_server_node/odom --window 3
ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once
```
If vehicle boxes are empty, the mission will still start once a car enters the camera FOV (Aurora semantic or YOLO). Ultralytics logs like "0: 480x640 (no detections)" mean no car/wheel in the current image; the pipeline uses Aurora semantic vehicle boxes when available.

---

## Full system on launch (power on → full mission)

One command runs the **entire** tire inspection pipeline from power-on:

1. **Aurora** connects and publishes map, odom, scan, depth, images, TF.
2. **Segment_3d** (8s): vehicle boxes from Aurora semantic or YOLO fallback → `/aurora_semantic/vehicle_bounding_boxes`; tires from YOLO + depth → `/darknet_ros_3d/tire_bounding_boxes`.
3. **Nav2** (10s + 15s): costmaps, planner, controller; **lifecycle** (45s after nav start) brings Nav2 active; **depth_gate** forwards `cmd_vel_nav` → `/cmd_vel` when permitted.
4. **Inspection manager + photo capture** (120s): loads PRODUCTION_CONFIG via `params_file` (`start_mission_on_ready: true`, `dry_run: false`), waits for `navigate_to_pose` (up to 90s), then **auto-starts**: SEARCH_VEHICLE → detect cars → approach each car → WAIT_TIRE_BOX → detect tires → approach each tire → INSPECT_TIRE → trigger photo → VERIFY_CAPTURE → next tire or NEXT_VEHICLE until DONE. **photo_capture_service** subscribes to `/inspection_manager/capture_photo` and publishes to `/inspection_manager/capture_result`; it must run for VERIFY_CAPTURE to succeed.
5. **Motor driver** (from start): subscribes to `/cmd_vel`, sends to ESP32 for motion.

**Single command (no dry_run):**
```bash
bash scripts/mission_launch.sh
# or: bash scripts/startup.sh
# or: ros2 launch ugv_nav full_bringup.launch.py
```
Do **not** pass `perception_only_mode:=true` for real inspection and photos. Ensure Aurora is on and `best_fallback.pt` is in place before launch.

---

## Launch

### Option A: Start mission (recommended)

Checks disk, sets Jetson max performance, optionally runs verification, then launches full stack:

```bash
./scripts/start_mission.sh
./scripts/start_mission.sh --no-verify    # skip verification
./scripts/start_mission.sh --verify-only  # run verification only and exit
```

See README quick start. Make executable if needed: `chmod +x scripts/start_mission.sh`.

### Option B: Mission launch (pre-flight + launch)

Runs pre-flight checklist, then starts full stack:

```bash
bash scripts/mission_launch.sh
bash scripts/mission_launch.sh ip_address:=192.168.11.1
```

### Option C: Startup only

```bash
bash scripts/startup.sh
bash scripts/startup.sh use_motor_driver:=false dry_run:=true  # validate without motion
```

For headless runs, set `STARTUP_DELAY=10` to wait for system readiness; redirect output: `bash scripts/startup.sh >> "$UGV_WS/logs/mission_startup.log" 2>&1`

### Option D: Manual (separate terminals)

1. Aurora: `ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1`
2. Motor: `ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:=/dev/ttyTHS1`
3. Nav2: `ros2 launch ugv_nav nav_aurora.launch.py` (wait ~2 min)
4. Detection: `ros2 launch segmentation_3d segment_3d.launch.py` (YOLO + depth + fusion; fusion uses YOLO vehicle boxes if Aurora semantic not published)
5. Inspection: `ros2 launch inspection_manager inspection_manager.launch.py`

**Note:** Inspection manager starts 120s after launch so Nav2 lifecycle can complete; it still waits for Nav2 `navigate_to_pose` (up to 90s) then auto-starts the mission. PRODUCTION_CONFIG is loaded via `params_file` from workspace (`UGV_WS/PRODUCTION_CONFIG.yaml`).

### Vehicle detection only (see the car in RViz)

With Aurora already running and looking at a car:

1. **Terminal 1 (Aurora):** `ros2 launch ugv_nav aurora_bringup.launch.py`
2. **Terminal 2 (perception):** `ros2 launch segmentation_3d segment_3d.launch.py`
3. **Terminal 3 (markers):** `ros2 launch segmentation_3d vehicle_detection_visualize.launch.py fallback_vehicle_boxes_topic:=/darknet_ros_3d/vehicle_bounding_boxes`
4. **RViz:** Fixed frame `slamware_map`, add by topic `/aurora_semantic/vehicle_markers` (MarkerArray).

**Validate pipeline:** `bash scripts/validate_vehicle_detection.sh` (run after step 2; checks topics and rates).

### Option E: Simulation (no Aurora hardware)

Offline testing with Aurora mock or rosbag replay:

```bash
# Mock: synthetic Aurora + full stack (no bag required)
ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true

# Rosbag replay (record first with ./sim/record_aurora_mission.sh)
ros2 launch sim vehicle_inspection_sim.launch.py use_bag:=true bag_path:=/path/to/bag
```

With `use_mock:=true`, the mock publishes odom, scan, map, depth, images, and synthetic vehicle(s) at (2, 0, 0.3) m. `simulated_detection_node` publishes `ObjectsSegment` from ground-truth vehicle geometry so centroid_servo and segment_3d run without YOLO. The aurora_mock_node uses MultiThreadedExecutor and odom at 100 Hz to avoid TF extrapolation errors (research_log §24).

**Multi-vehicle:** `vehicle_count:=2` publishes two vehicles; mission completes 8 tires across both. **Stress-test:** `use_stress_test:=true` enables noise, latency, jitter, dropout, and occasional tire misses (PCL fallback). See sim/README.md.

**Simulation launch variants:** `use_mock:=true`, `use_synthetic_vehicle:=true` (sensor realism), `use_stress_test:=true`, `vehicle_count:=N`. Record mock missions with `./sim/record_mock_mission.sh`; compare reports with `python3 scripts/compare_sim_to_real.py`.

---

### Simulation Testing (verify_system.py)

Full mission simulation **without a real vehicle**, using the real Aurora for SLAM and depth. Two modes:

**One-command launch (recommended):**
```bash
ros2 launch ugv_nav full_bringup.launch.py sim_no_move:=true sim_tyre_detections:=true
```
- `sim_no_move:=true` — uses `stub_motor_node` instead of motor driver; robot does not move.
- `sim_tyre_detections:=true` — launches `verify_system.py --simulate --publish-objects-segment` after 15 s, publishing to `/sim/vehicle_bounding_boxes` and `/sim/tire_bounding_boxes_merged`. The inspection manager is remapped to those topics so it sees only simulated detections (no competition from aurora_semantic_fusion / tire_merger).

**Manual (two terminals):**
```bash
# Terminal 1: Full stack (with or without sim_no_move)
./scripts/start_mission.sh
# Or: ros2 launch ugv_nav full_bringup.launch.py sim_no_move:=true

# Terminal 2: After stack is up and TF is ready
source install/setup.bash
python3 scripts/verify_system.py --simulate --duration 120 --publish-objects-segment
```

The simulation script publishes to `/aurora_semantic/vehicle_bounding_boxes` (1 vehicle at 3 m) and `/tire_bounding_boxes_merged` (4 tires) at 5 Hz. With `--publish-objects-segment`, also publishes `ObjectsSegment` to `/ultralytics_tire/segmentation/objects_segment` for centroid_servo (requires TF: `camera_left` ← `slamware_map`).

**Parameters:**
- `--duration N` — seconds to run (default 60)
- `--vehicle-count N` — number of vehicles (default 1)
- `--tires-per-vehicle N` — tires per vehicle (default 4)
- `--detection-rate R` — probability of publishing each cycle, 0.0–1.0 (default 1.0)
- `--tire-detection-rate R` — tire-only publish probability (default: same as `--detection-rate`)
- `--tire-drop-probability P` — probability to drop one tire per cycle for occlusion test (default 0)
- `--position-variance S` — std dev in m for tire position noise (default 0.05)
- `--confidence-mean M`, `--confidence-std S` — detection confidence
- `--publish-vehicle` / `--no-publish-vehicle` — publish synthetic vehicle boxes (default: publish)
- `--vehicle-offset-x X`, `--vehicle-offset-y Y` — vehicle position in slamware_map (default 3.0, 0.0)
- `--vehicle-confidence C` — vehicle confidence (default 0.9)
- `--topic-vehicle T`, `--topic-tire T` — override topics
- `--publish-objects-segment` — also publish for centroid_servo
- `--log-level LEVEL` — debug, info, warn, error (default: info)

**Test scenarios:**
- **Basic mission:** `--duration 300` (default params)
- **Missing detections:** `--detection-rate 0.8`
- **Occluded tyre:** `--tire-drop-probability 0.05`
- **Multiple vehicles:** `--vehicle-count 2`
- **Low confidence:** `--confidence-mean 0.7`
- **Real Aurora vehicles only:** `--no-publish-vehicle` (tires at fixed offset)

Monitor `/inspection_manager/runtime_diagnostics` for state transitions. At exit, the script prints a summary: tires captured, mission duration, states visited. See `docs/SIMULATION_RESULTS.md` for outcomes.

**Simulation mode (sim_tyre_detections:=true):**
- `ultralytics_tire` and `tire_merger` are disabled (verify_system provides detections).
- Nav2 uses `nav_aurora_sim.yaml`: rolling 20×20 m global costmap (width/height must be integers), lower controller/planner frequencies.
- Inspection manager uses `PRODUCTION_CONFIG_SIM.yaml`: `tire_confirmations_required: 1`.
- Tire positions from vehicle_modeler now use box dimensions when larger than defaults (fixes sim tire goals).

**Troubleshooting:** If no messages appear on sim topics, the script logs a heartbeat every 10 s. Run with `--log-level debug` for verbose output. The script uses `time.sleep()` (not ROS Rate) for reliable 5 Hz publishing; explicit RELIABLE QoS matches the inspection manager.

---

### Performance profiling

Run during a mission to capture CPU/GPU for post-mission analysis:

```bash
./scripts/profile_mission.sh                  # background
./scripts/profile_mission.sh --foreground     # Ctrl+C to stop
./scripts/profile_mission.sh --duration 300   # auto-stop after 300 s
./scripts/profile_mission.sh --per-process    # per-process CPU/MEM every 10 s
```

Logs: `logs/profile_<timestamp>.log`. After mission: `python3 scripts/analyze_profile_log.py logs/profile_*.log`. On Jetson, uses `tegrastats` when available.

---

### Validation checklist (pre-release)

- [ ] Aurora topics published (map, left_image_raw, vehicle boxes, scan, odom)
- [ ] TF tree complete; no extrapolation errors
- [ ] `ultralytics_tire` starts without errors (warm-up OK; TensorRT or PyTorch logged)
- [ ] Tire detections on `/tire_bounding_boxes_merged`
- [ ] Nav2 lifecycle completes; `/navigate_to_pose` action server available
- [ ] Robot navigates to vehicle; goal refinement works (log "Refining goal: planned → detection (X.XXm)")
- [ ] Centroid servo positions at ~30 cm standoff; photo capture with metadata
- [ ] Mission report shows 4 tires captured; `tire_goal_sources` / `goal_source` present
- [ ] CPU/GPU within limits (e.g. GPU < 80%, CPU < 70%; use `profile_mission.sh`)

---

### Option F: Auto-start on boot (headless)

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

**Log retention:** On each mission start, existing `mission_latest` / `mission_report_latest` are moved to `logs/archive/` with a timestamp. The inspection manager keeps the **last 10** archive files per type (mission_*, mission_report_*); older archives are compressed with gzip. To free space, delete or move `logs/archive/*.gz` manually.

Configured via PRODUCTION_CONFIG: `mission_log_path`, `mission_report_path`. After reconnecting post-mission, copy `logs/` (or just `logs/archive/`) to review all runs.

**Interpreting mission reports:** Open `mission_report_latest.json`. Key fields: `success` (true if all tires captured and no errors), `tires_captured` (should be 4), `tire_capture_log` (each entry has `goal_source`: planned / detection / detection_refined, plus image_path, distance_m, timestamp), `tire_goal_sources` (list of goal sources per tire), `failures` (empty if successful; otherwise lists reasons), `mission_duration_s`. See [TESTING_AND_VALIDATION.md](docs/TESTING_AND_VALIDATION.md) for validation procedure.

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
| 3D boxes (tires) | `ros2 topic echo /darknet_ros_3d/tire_bounding_boxes --once` |
| Inspection state | `ros2 topic echo /inspection_state` or `--once` for one message |
| Mission state (one shot) | `ros2 topic echo /inspection/mission_state --once` |
| Mission report | `ros2 topic echo /inspection_manager/mission_report` |
| Current nav goal | `ros2 topic echo /inspection_manager/current_goal` (PoseStamped in map; only when a nav goal is active) |
| TF | `ros2 run tf2_ros tf2_echo slamware_map base_link` (or `map base_link`) |
| Nav2 ready | `ros2 action info /navigate_to_pose` — must show **Action servers: 1** |

**Mission readiness (TF + Nav2):** Ensure launch log shows "Nav2 lifecycle bringup complete" before expecting the mission to start. The mission waits for TF and Nav2 before auto-starting.

**Headless boot (no Wi‑Fi) logging:** `ugv_mission.service` appends systemd output to `~/ugv_ws/logs/mission_service.log` so logs persist across power‑off. Mission logs and reports are written to `~/ugv_ws/logs/mission_latest.jsonl` and `~/ugv_ws/logs/mission_report_latest.json` with per‑event fsync enabled (`mission_log_fsync: true`).

### Dry run (no motion)

Run `ros2 launch ugv_nav full_bringup.launch.py dry_run:=true`. Expected:

- **Launch:** Aurora, motor_driver, segment_3d (aurora_semantic_fusion, ultralytics, depth pipeline, segmentation_processor), Nav2, inspection_manager, photo_capture_service start in order. No "Old-style arguments" TF warnings (aurora_bringup uses new-style static_transform_publisher).
- **Vehicle detection:** inspection_manager logs "Received N vehicle box(es) from semantic topic", "Published vehicle_detected", "Saved vehicle position", "Found 1 vehicle(s). Starting inspection of vehicle 1", state **SEARCH_VEHICLE → WAIT_VEHICLE_BOX**.
- **Approach (dry):** "Vehicle detected (...); approaching.", "Computing navigation goal" (slamware_map frame), "Dry run: goal validated, not sending to Nav2", then state advances to **WAIT_TIRE_BOX** (same as full launch after nav arrival). Segmentation mode switches to **inspection** (Model 2 / tires).
- **Full launch (no dry_run):** The same approach goal is sent to Nav2; robot drives to vehicle, then waits for tires; on tire detection, navigates to tire and triggers photo capture via **photo_capture_service** (topic-based: `/inspection_manager/capture_photo` → `/inspection_manager/capture_result`). photo_capture_service provides the topic flow (capture_photo → capture_result). Mission logs rotate into `logs/archive/` on each start.

**Known benign messages during dry run:** `No transform available from camera_depth_optical_frame to slamware_map` (early in segmentation_processor) can appear once until TF is fully up; later callbacks succeed. `RTPS_READER_HISTORY Error` / `Change payload size` during Nav2 lifecycle are DDS quirks; Nav2 still activates. `Message Filter dropping message: frame 'laser'` at startup is timestamp/cache related and usually clears.

### Simulation validation (sim vs real)

To ensure simulation matches field behavior:

1. **Record mock mission:** With `use_mock:=true` sim running, run `./sim/record_mock_mission.sh` in another terminal. Let the mission complete (or drive via teleop), then Ctrl+C. Bag saved to `sim/bags/mock_mission_YYYYMMDD_HHMMSS/`.

2. **Compare mission reports:** After a sim run and a real run, compare outcomes:
   ```bash
   python3 scripts/compare_sim_to_real.py --sim-report ~/ugv_ws/logs/mission_report_latest.json --real-report /path/to/real_report.json
   ```
   Reports success, tires captured, errors, and duration. Use `--json` for machine-readable output.

3. **Acceptance:** Sim should achieve same outcome as real (4 tires captured, no errors) when vehicle geometry and environment match. Differences indicate sim gaps (e.g. costmap, depth, detection) to address.

**Multi-vehicle testing:** `vehicle_count:=2` or `vehicle_count:=3` to test sequencing and return-later across vehicles. **Stress-test:** `use_stress_test:=true` to inject noise, latency, jitter, dropout, and tire misses; mission may still complete or fail in a controlled way.

---

## Troubleshooting

| Issue | Check | Fix |
|-------|-------|-----|
| No vehicles detected | Aurora semantic topics | Aurora 2.11 firmware; `ros2 topic echo /slamware_ros_sdk_server_node/semantic_labels --once`; check `detection_stream_stale` or `vehicle_boxes_stream_stale` in `logs/mission_latest.jsonl` |
| No tires detected | best_fallback.pt, tire_label | Place best_fallback.pt in Tyre_Inspection_Bot; ultralytics_tire uses inspection mode. Full class list: docs/BEST_FALLBACK_MODEL_CLASSES.md. Run `ros2 run segmentation_3d print_inspection_model_classes`; set `tire_label: wheel` in PRODUCTION_CONFIG (best_fallback.pt id=22). |
| Nav2 lifecycle fails | DDS | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| TF lookup failed | Aurora running | Start Aurora first; verify: `ros2 run tf2_ros tf2_echo slamware_map base_link` |
| Mission spins / never leaves SEARCH_VEHICLE | TF or detection | Check `~/ugv_ws/logs/mission_latest.jsonl` for `tf_watchdog`; if present, TF chain is broken (Aurora not publishing odom→base_link). Fix Aurora connectivity, then re-run. |
| VERIFY_CAPTURE times out / no capture_result | photo_capture_service | inspection_manager_node expects topic-based capture. Ensure **photo_capture_service** is running. Check `ros2 node list | grep photo_capture`. |
| Robot drives into car | Costmap, depth, TF | Costmap uses `/camera/depth/points` (Aurora depth → point cloud) and `/slamware_ros_sdk_server_node/scan`. If the car is not in the costmap: (1) Verify `/camera/depth/points` publishes: `ros2 topic hz /camera/depth/points`; (2) Verify TF `camera_depth_optical_frame` → `map`; (3) If needed, increase `inflation_radius` in nav_aurora.yaml (local_costmap 0.6, global 0.35). |

---

## Stop Mission

Press `Ctrl+C` in the terminal running the launch.

---

## Crontab Auto-Start (Optional)

To start the mission on boot (after Jetson/Wi-Fi is ready):

```bash
crontab -e
```

Add:

```
@reboot sleep 15 && STARTUP_DELAY=10 /home/jetson/ugv_ws/scripts/mission_launch.sh >> /home/jetson/ugv_ws/logs/mission_startup.log 2>&1
```

- **`sleep 15`** — Wait for system/Wi-Fi after boot (aligns with reference).
- **`STARTUP_DELAY=10`** — Optional extra delay (seconds) inside startup.sh for system readiness; omit for interactive use.
- **Log redirection** — `>> .../logs/mission_startup.log 2>&1` persists console output for headless runs.

Adjust paths for your user and workspace. Ensure `~/.bashrc` sources ROS and `install/setup.bash`; the startup script sources the workspace.

---

## Deep research prompt

For AI-assisted web/code research (e.g. ChatGPT, Claude), use **[docs/DEEP_RESEARCH_PROMPT.md](docs/DEEP_RESEARCH_PROMPT.md)**. It fully describes the project goal, all hardware and software, what has been tried, known issues, and instructs the AI to find existing sources, code, and fixes (Nav2, Aurora, costmap, TF, clustering, inspection missions). Copy the prompt (or sections) into the assistant and ask it to search and cite concrete references and actionable suggestions.
