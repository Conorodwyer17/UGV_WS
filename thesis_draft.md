# Autonomous Ground Vehicle for Tyre Inspection Using ROS 2, Visual Detection, and the SLAMTEC Aurora Platform

**Author:** Conor O’Dwyer  
**Project:** Final-year engineering project (repository: `UGV_WS`)  
**Date:** March 2026  

## Abstract

This project develops an autonomous differential-drive robot that approaches a parked vehicle, navigates to each wheel location, and captures inspection images for four tyres. The system integrates ROS 2 Humble on an NVIDIA Jetson Orin Nano (16 GB), with a SLAMTEC Aurora 6DOF sensor stack for localisation, mapping, and semantic vehicle cues, and a dedicated Ultralytics YOLO-based pipeline for wheel segmentation at close range. Navigation uses Nav2 with SmacPlanner2D, behaviour trees tuned for inspection without unnecessary in-place spins, and an inspection manager that coordinates state transitions, goal dispatch, and photo capture. The work addresses practical constraints on embedded memory and compute by supporting CPU ONNX inference, reduced depth publication rates, and optional modular bringups for demonstration. The thesis traces the evolution of the software from initial Aurora integration and stub motor tests through to tyre-centric geometry, 3D tyre projection, optional batch waypoint navigation, and field-oriented safeguards.

## 1. Introduction

Automated visual inspection of vehicle tyres can reduce manual workload in fleet depots and controlled environments. A mobile robot must reliably localise, detect a vehicle, plan feasible paths around obstacles, and align a camera with each tyre. This project implements such a system on commodity robot hardware with a modern ROS 2 stack (Humble), prioritising observability, reproducible launch profiles, and clear separation between perception, navigation, and mission logic.

## 2. System Overview

### 2.1 Hardware

| Component | Role |
|-----------|------|
| Waveshare UGV Rover | Differential-drive base |
| Jetson Orin Nano (16 GB) | Onboard compute |
| SLAMTEC Aurora | LiDAR, stereo depth, IMU, semantic segmentation, odometry |
| ESP32 (UART) | Motor driver interface; optional wheel odometry feedback |

The Aurora provides semantic segmentation (COCO-style classes including car, truck, bus) used for vehicle bounding boxes in the `slamware_map` frame. Tyre detection uses a separate trained model (`best_fallback.pt` or a dedicated `tyre_detection_project/best.pt`) with class `wheel`, aligned with production configuration.

### 2.2 Software Stack

- **ROS 2 Humble** with Cyclone DDS for consistent discovery across robot and laptop.
- **`slamware_ros_sdk_server_node`** (Aurora SDK) for mapping, TF, and sensor topics.
- **Segmentation and 3D pipeline** (`segmentation_3d`): Ultralytics YOLO for wheels, depth registration, optional PCL fusion, and `tyre_3d_projection_node` publishing `geometry_msgs/PoseArray` on `/tyre_3d_positions`.
- **Nav2** with parameters in `nav_aurora.yaml`, global costmaps fed by scan and depth-derived obstacles, and behaviour trees `navigate_to_pose_no_spin.xml` / `navigate_through_poses_no_spin.xml` aligned with planner plugin names.
- **Inspection manager** (`inspection_manager_node`): flat state machine from search through approach, tyre inspection, capture verification, and completion or error.
- **Photo capture service** (`photo_capture_service`): saves images under a configurable directory (default `~/ugv_ws/tire_inspection_photos`).

### 2.3 Coordinate Frames

Bounding boxes and tyre poses are expressed in `slamware_map`. Nav2 uses `map`; the stack assumes identity between `map` and `slamware_map` for goal dispatch. The robot pose is obtained from TF `slamware_map` → `base_link`.

## 3. Development History and Major Phases

### 3.1 Initial Integration and Stub Motor Tests

Early work focused on bringing the Aurora ROS 2 SDK online, verifying topics (`/slamware_ros_sdk_server_node/odom`, scan, images, depth), and integrating Nav2. A **stub motor** node discards `cmd_vel` while leaving the rest of the stack live, enabling **planning-only** validation (global and local plans in RViz) without physical motion. The launch profile `MISSION_PROFILE=stable_viz` uses `sim_no_move:=true`, CPU tyre inference, and `demo_mode` so that distance gates for photography can be relaxed for bench demonstrations. Documentation for this mode is in `docs/PLANNING_TEST_STUB_MOTOR.md`.

### 3.2 Detection Pipeline: Training, ONNX, and 3D Projection

Tyre detection uses Ultralytics YOLO (segmentation). Training artefacts and logs are retained **locally** under `tyre_training_results/` (not version-controlled owing to size; training was performed on cloud GPU). For deployment on the Jetson, models are exported to **ONNX** (`scripts/export_onnx.sh`) with an image size (`IMGSZ`) matching `wheel_imgsz` / `tyre_onnx_imgsz` (commonly 480 for CPU missions). The **3D projection** node fuses wheel masks with registered depth to publish tyre positions for direct navigation goals when `use_tyre_3d_positions` is true.

### 3.3 Navigation: Costmaps, Footprint, and SmacPlanner

Nav2 global planning uses **SmacPlanner2D** with tuned inflation and goal tolerance so goals near inflated vehicle cells remain reachable. Recovery behaviours clear costmaps and back up on failure. The inspection manager queries **global costmap** cost at the footprint before dispatching goals and can perform a short **reverse** manoeuvre if the robot starts in lethal cost (e.g. after a capture). Behaviour tree XML files **must** match the planner plugin identifier in YAML (`planner_id="SmacPlanner"` when Smac is active).

### 3.4 Tyre-Centric Geometry

When multiple tyre detections are available, the system can infer vehicle orientation and planned tyre order from geometry (`use_tyre_geometry`, tyre ordering utilities in the codebase). This reduces reliance on a single jittery vehicle bounding box for long-range orientation. Planned fallback positions (four corners) remain when detections are missing, using `estimate_tire_positions` with wheelbase and track from configuration.

### 3.5 Perimeter Waypoints and Batch Navigation

For operators who need a single multi-goal action, the Nav2 **FollowWaypoints** action is supported: `use_follow_waypoints` and `use_batch_waypoints` send a batch of poses (with optional **perimeter bridge** waypoints when `tyre_perimeter_bridge_enabled` is true) so the robot rounds the vehicle before standoff poses. This requires the **`inspection_waypoint_plugins`** package so waypoint-follow tasks can trigger capture at each stop. Default full bringup uses **sequential** `NavigateToPose` unless these flags are enabled.

### 3.6 Memory Constraints and CPU Inference

Jetson devices with limited unified memory can hit **CUDA out-of-memory** when loading TensorRT engines alongside other GPU consumers. The production profile **`mission_dedicated_cpu`** runs the dedicated tyre model on **CPU** via ONNX (`use_cpu_inference:=true`, `yolo_device:="cpu"`), reduces depth registration rate (e.g. 2 Hz), and disables optional heavy nodes as needed. A stress profile (`crash_fallback_seg`) exercises GPU TensorRT with the fallback segmentation model for comparison. The **modular** `ugv_bringup` demos split subsystems for thesis presentations on constrained hardware.

### 3.7 Demo Mode and No-Motion Validation

The combination of `stable_viz`, stub motor, and `demo_mode` supports **no-motion** demonstrations: Nav2 and planning remain active, synthetic navigation success can be tied to topics when configured, and photos can be captured without full motion to tyre standoffs. This separates **algorithmic** behaviour from **locomotion** and **powertrain** issues.

### 3.8 Real-Motion Tests

Real-motion runs use `mission_dedicated_cpu` (or GPU profiles on suitable hardware), real motor driver, and the same mission start gating: valid TF for `tf_stable_s`, Nav2 action server availability, and configured timeouts. Mission logs are written to `~/ugv_ws/logs/mission_latest.jsonl` with JSON mission reports summarising duration, captures per tyre, and failure reasons.

## 4. Lessons Learned: Failures and Resolutions

| Issue | Cause | Resolution |
|-------|--------|------------|
| Aurora vehicle box jitter | Semantic segmentation updates shift the box frame-to-frame | Confirmation counters, EMA smoothing, tyre geometry; tyre 3D goals reduce dependence on vehicle box alone |
| ONNX / dimension mismatch | Export `IMGSZ` differs from runtime `wheel_imgsz` | Re-export ONNX with `scripts/export_onnx.sh` matching `IMGSZ` |
| Planner not loading Smac | Plugin name mismatch | Align `nav_aurora.yaml` plugins with BT `planner_id` |
| No plan / goal rejected | Goal off map or in lethal cost | Costmap clearing, inflation tuning, `escape_lethal_start_if_needed`, post-capture backup |
| Progress stall near goal | Slip, control, or load (see archived logs) | Stub-motor test to isolate planning; reduce load (inference interval, resolution) |
| TensorRT OOM | Engine workspace too large on 8 GB class devices | Smaller `IMGSZ`, delay model load, or CPU ONNX path |

## 5. Results

### 5.1 No-Motion and Planning Validation

Planning tests with the stub motor (`MISSION_PROFILE=stable_viz`, `sim_no_move:=true`) confirm that Nav2 accepts goals and publishes `/plan` and `/local_plan` when a vehicle is in view and TF is stable. **Exact timings** depend on map reset delay, `inspection_delay_s`, and scene; for a representative bench run, the mission transitions from IDLE through search and approach states within **tens of seconds** once TF and Nav2 are ready. For a quantitative table for the final thesis, extract timestamps from `~/ugv_ws/logs/mission_latest.jsonl` for your specific runs.

### 5.2 Real-Motion Behaviour

Real-motion missions aim for **four verified captures** per vehicle (`expected_tires_per_vehicle: 4`). Success is defined by `mission_report_latest.json` (`success`, `tires_captured`, `tire_capture_log`). Field logs in the repository’s development history include cases where **progress** stalled near the goal with small `distance_m` (see `docs/PLANNING_TEST_STUB_MOTOR.md` analysis), motivating load reduction and TF stability work rather than a single change to goal mathematics.

### 5.3 Resource Observations

Local **tegrastats** captures (not committed to the repository; see `benchmarks/` for offline logs) are used to monitor RAM and CPU during bringup. The **CPU** tyre path trades inference rate for predictable memory use on the Jetson.

Figures for the thesis (architecture diagrams, RViz screenshots, sample images) may be placed under `docs/figures/` when prepared; they are not bundled in this repository snapshot.

## 6. Hardware Limitations

The Jetson shares memory between CPU and GPU. Running Aurora semantic fusion, depth registration, YOLO at high resolution, and TensorRT concurrently can exhaust memory. The **CPU ONNX** path for tyre detection, throttled depth, and optional **minimal_perception** bringups mitigate this at the cost of throughput. UART bandwidth and motor latency bound how aggressively the controller can recover from tracking error.

## 7. Safety and Operational Guarantees

The inspection manager implements: TF watchdog with goal cancellation on prolonged TF loss; maximum time in approach states; spin protection via repeated state detection; hard mission timeout; minimum interval between nav goals; and zero `cmd_vel` on certain Nav2 failures. These are documented in `RUNBOOK.md`.

## 8. Future Work

- **MPPI or other advanced controllers** for tighter tracking in cluttered tyre-adjacent areas.
- **GPU inference** on 16 GB or newer Jetson modules when memory allows, with quantised or smaller models.
- **Fleet integration** and scheduling across multiple robots.
- **Robust multi-vehicle tracking** when IDs churn in crowded scenes.
- **Persistent quality metrics** for tyre images (blur, exposure) before accepting a capture.

## 9. Conclusion

This project delivers an integrated ROS 2 system for autonomous tyre inspection with Aurora-based localisation, YOLO-based wheel detection, Nav2 navigation, and a mission layer designed for field behaviour. The development history reflects iterative hardening: costmap and planner alignment, tyre-centric geometry, optional batch waypoints, CPU inference for constrained hardware, and repeatable demo modes. The repository is structured for reproducible launches, clear logging, and modular demonstrations suitable for final assessment and further research.

## References and Repository Map

- `RUNBOOK.md` — Operational procedures and topic catalogue.
- `ARCHITECTURE.md` — Hardware and node graph.
- `docs/MISSION_PIPELINE.md` — Mission state flow.
- `MISSION.md` / `docs/MISSION_FLOW.md` — If present, detailed flow diagrams.
- `PRODUCTION_CONFIG.yaml` — Runtime parameters for inspection and capture.

---

*End of first draft. Review all numerical results against your own logged runs before submission.*
