# UGV Workspace – Autonomous Tyre Inspection Robot

An autonomous robot for inspecting commercial vehicle tyres using ROS 2, computer vision, and SLAM-based navigation. Runs on a Waveshare UGV Rover with SLAMTEC Aurora 6DOF.

## Overview

- **Vehicles:** Aurora COCO80 semantic (car, truck, bus)
- **Tyres:** YOLO `best_fallback.pt` (wheel class)
- **Navigation:** SLAMTEC Aurora 6DOF SLAM + Nav2
- **Mission:** State machine, 3D bounding boxes, inspection photo capture

## Quick start

```bash
cd ~/ugv_ws
bash scripts/start_mission.sh
```

Or `bash scripts/mission_launch.sh` (pre-flight + launch) or `bash scripts/startup.sh` (launch only). `start_mission.sh` checks disk space, sets Jetson max performance, optionally runs verification, then launches the full stack. Inspection manager starts ~120 s after launch.

**Detection:** Vehicle boxes come from **Aurora** only by default (YOLO vehicle node is off to save GPU/CPU). Tyre detection uses YOLO `best_fallback.pt` or TensorRT `best_fallback.engine` when present. To free disk after setup, run `./scripts/cleanup.sh` (dry run) or `./scripts/cleanup.sh --execute` to remove large temporary files and optional packages; see [docs/TIRE_DETECTION_TROUBLESHOOTING.md](docs/TIRE_DETECTION_TROUBLESHOOTING.md).

## Documentation

| Doc | Description |
|-----|-------------|
| [RUNBOOK.md](RUNBOOK.md) | Operations runbook, canonical stack, mission flow |
| [docs/MISSION_PIPELINE.md](docs/MISSION_PIPELINE.md) | Mission flow: approach vehicle → tyre inspection → photo |
| [docs/ACCEPTANCE_CRITERIA.md](docs/ACCEPTANCE_CRITERIA.md) | Acceptance criteria for live mission |
| [docs/TIRE_DETECTION_TROUBLESHOOTING.md](docs/TIRE_DETECTION_TROUBLESHOOTING.md) | Tyre detection pipeline, OOM workarounds, CPU inference |
| [docs/SIMULATION_RESULTS.md](docs/SIMULATION_RESULTS.md) | Simulation outcomes |
| [docs/REPOSITORY_CLEANUP_PLAN.md](docs/REPOSITORY_CLEANUP_PLAN.md) | Repository cleanup plan |
| [docs/100_PERCENT_RELIABILITY_PLAN.md](docs/100_PERCENT_RELIABILITY_PLAN.md) | Reliability plan |
| [SETUP.md](SETUP.md) | Fresh Jetson installation guide (JetPack 6.0, ROS 2, TensorRT) |

## Installation

1. Install ROS 2 (Humble or Jazzy), Nav2, TF2
2. Install [SLAMTEC Aurora ROS 2 SDK](https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/) into `src/`
3. `pip3 install -r requirements.txt`
4. `colcon build && source install/setup.bash`
5. Place `best_fallback.pt` or `best_fallback.engine` (tyre model) in `~/ugv_ws/` or `src/Tyre_Inspection_Bot/`. On Jetson, run `scripts/export_tensorrt.sh` to build the engine for fast inference.

See [src/Tyre_Inspection_Bot/README.md](src/Tyre_Inspection_Bot/README.md) for full install and manual launch steps.
