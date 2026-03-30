# Demonstration Preparation

This note describes how to run **no-motion** and **real-motion** demonstrations for the autonomous tyre inspection stack on the Jetson (Ubuntu 22.04, ROS 2 Humble). Workspace root: `~/ugv_ws`. British English terminology is used in prose; ROS topic and parameter names follow the existing codebase (some use US spelling).

## Prerequisites

- Workspace built: `cd ~/ugv_ws && colcon build --symlink-install && source install/setup.bash`.
- Dedicated CPU tyre model: `tyre_detection_project/best.pt` with matching `tyre_detection_project/best.onnx` exported at the same square input size (default **480**):  
  `MODEL_PT=tyre_detection_project/best.pt IMGSZ=480 bash scripts/export_onnx.sh`
- Aurora reachable at the configured IP (default `192.168.11.1` unless changed).
- Motor driver UART available when not using stub motor (`/dev/ttyTHS1` by default).
- Same `ROS_DOMAIN_ID` on the robot and any laptop running RViz (commonly **0**).
- Cyclone DDS: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (set by `start_mission.sh` on the robot).

## No-Motion Demonstration (Planning and Pipeline)

**Purpose:** Validate Nav2 planning, semantic vehicle cues, tyre overlay, and mission state transitions **without** driving the base. `/cmd_vel` is consumed by the stub motor node.

**Steps**

1. Optional: start `tegrastats` logging in the background for later review:  
   `tegrastats --interval 500 --logfile ~/ugv_ws/benchmarks/tegrastats_stub.log &`
2. Source the workspace: `source ~/ugv_ws/install/setup.bash`
3. Launch:  
   `MISSION_PROFILE=stable_viz ./scripts/start_mission.sh --no-verify`  
   This sets `sim_no_move:=true`, CPU tyre inference, `use_tyre_3d_positions:=true`, `demo_mode:=true`, and related parameters (see `scripts/start_mission.sh`).
4. Wait until the inspection manager leaves **IDLE** (TF stable per `PRODUCTION_CONFIG`, Nav2 action server available).
5. Place a vehicle in the Aurora field of view as for a real run.
6. On a laptop (same LAN, same `ROS_DOMAIN_ID`):  
   `~/ugv_ws/scripts/monitor_mission.sh`  
   or open the RViz configuration referenced in `RUNBOOK.md`.

**Expected observations**

- Global path on `/plan`, local path on `/local_plan` when goals are active.
- Vehicle markers on `/aurora_semantic/vehicle_markers` (and related topics per runbook).
- Tyre overlay on `/ultralytics_tire/segmentation/image` and markers on `/tyre_markers` when the tyre pipeline is running.
- Mission log lines in `~/ugv_ws/logs/mission_latest.jsonl` showing state transitions.

**Optional parameters for batch navigation (if your stack includes the waypoint plugin server)**

If you need **FollowWaypoints** with perimeter bridging, pass launch arguments after the script flags, for example:  
`MISSION_PROFILE=stable_viz ./scripts/start_mission.sh --no-verify use_batch_waypoints:=true tyre_perimeter_bridge_enabled:=true`  
Ensure `inspection_waypoint_plugins` is built and Nav2 is configured with the corresponding waypoint task plugins.

## Real-Motion Demonstration

**Purpose:** Full autonomous approach and tyre inspection with the real motor driver.

**Steps**

1. Pre-flight: clear space, charged battery, e-stop accessible, robot inside the mapped area.
2. `source ~/ugv_ws/install/setup.bash`
3. Launch:  
   `./scripts/start_mission.sh`  
   Default profile is **`mission_dedicated_cpu`** (`sim_no_move:=false`, CPU ONNX tyre model, real motor). Adjust launch args as required (e.g. `inference_interval_s:=1.0` to reduce CPU load).
4. Optional verification after the stack is up:  
   `python3 ~/ugv_ws/scripts/check_mission_topics.py --timeout 60`
5. Allow the mission to start when TF and Nav2 are ready (`start_mission_on_ready` in `PRODUCTION_CONFIG.yaml`).

**Expected observations**

- Physical motion toward approach and tyre standoff poses.
- Photos under `~/ugv_ws/tire_inspection_photos` (or `save_directory` in `PRODUCTION_CONFIG.yaml`).
- JSON report `~/ugv_ws/logs/mission_report_latest.json` with per-tyre entries when captures succeed.

**Optional: batch waypoints and perimeter bridge**

`./scripts/start_mission.sh use_batch_waypoints:=true tyre_perimeter_bridge_enabled:=true`  
(Only if FollowWaypoints and waypoint plugins are correctly installed and enabled.)

## Troubleshooting

| Symptom | Checks |
|---------|--------|
| Robot does not move | Confirm `sim_no_move:=false`; motor driver node running; UART permissions (`dialout`); stub motor not selected. |
| No `/plan` in RViz | Nav2 lifecycle active; goal in map bounds; TF `map`–`base_link` stable. |
| No tyre detections | ONNX present and `IMGSZ` matches export; `interested_class_names` includes `wheel`; camera and depth topics active. |
| Photos not saved | `photo_capture_service` running; write permissions on `save_directory`; inspection manager reaches capture states (see mission log). |
| Planner errors | Match BT `planner_id` to `nav_aurora.yaml`; clear costmaps manually if stuck after recovery failures. |
| High load or latency | Reduce `inference_interval_s`, lower ONNX `IMGSZ` (re-export), reduce depth publish rate per `RUNBOOK.md`. |

## Related Documentation

- `RUNBOOK.md` — Full operational detail.
- `docs/PLANNING_TEST_STUB_MOTOR.md` — Stub motor planning test.
- `docs/TIRE_DETECTION_TROUBLESHOOTING.md` — Perception faults.
