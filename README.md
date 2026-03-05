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

**Detection:** Vehicle boxes come from **Aurora** only by default (YOLO vehicle node is off to save GPU/CPU). Tire detection uses YOLO `best_fallback.pt` or TensorRT `best_fallback.engine` when present. To free disk after setup, run `./scripts/cleanup.sh` (dry run) or `./scripts/cleanup.sh --execute` to remove large temporary files and optional packages; see [docs/TIRE_DETECTION_CLEANUP_REPORT.md](docs/TIRE_DETECTION_CLEANUP_REPORT.md).

## Documentation

| Doc | Description |
|-----|-------------|
| [docs/PROJECT_OVERVIEW.md](docs/PROJECT_OVERVIEW.md) | **Vehicle as single source of truth:** bounding box + costmap = vehicle |
| [RUNBOOK.md](RUNBOOK.md) | Operations runbook |
| [docs/MISSION_PIPELINE.md](docs/MISSION_PIPELINE.md) | Mission flow: approach vehicle → tire inspection → photo |
| [docs/MISSION_TIRE_ORDER_AND_SCENARIO.md](docs/MISSION_TIRE_ORDER_AND_SCENARIO.md) | Tire order (nearest first), scenario behaviour, no guesswork |
| [docs/NAVIGATION_SAFETY.md](docs/NAVIGATION_SAFETY.md) | Nav2 safety, offsets, costmap, obstacle avoidance |
| [docs/MISSION_READINESS_CHECKLIST.md](docs/MISSION_READINESS_CHECKLIST.md) | Mission node, topics, commit points |
| [docs/VEHICLE_INSPECTION_FAILURE_MODES.md](docs/VEHICLE_INSPECTION_FAILURE_MODES.md) | Possible errors and failure modes |
| [docs/HOW_WE_KNOW_WHERE_TO_DRIVE.md](docs/HOW_WE_KNOW_WHERE_TO_DRIVE.md) | How the robot chooses where to drive (car, tire, face) |
| [docs/vehicle_detection_to_navigation_flow.md](docs/vehicle_detection_to_navigation_flow.md) | Detection → navigation flow |
| [docs/NO_MOVEMENT_DEBUG.md](docs/NO_MOVEMENT_DEBUG.md) | Debug when the robot does not move |
| [docs/VISION_PIPELINE.md](docs/VISION_PIPELINE.md) | Aurora to 3D bounding boxes (vision pipeline) |
| [docs/BEST_FALLBACK_MODEL_CLASSES.md](docs/BEST_FALLBACK_MODEL_CLASSES.md) | best_fallback.pt class names (wheel, etc.) |
| [docs/vision_navigation_dependencies.md](docs/vision_navigation_dependencies.md) | ROS 2 and Python dependencies |
| [docs/PRE_LAUNCH_CHECKLIST.md](docs/PRE_LAUNCH_CHECKLIST.md) | Pre-launch checks |
| [docs/aurora_topics_and_frames.md](docs/aurora_topics_and_frames.md) | Aurora topics and frames |
| [docs/TOPIC_VERIFICATION.md](docs/TOPIC_VERIFICATION.md) | Topic names and publish rates vs code |
| [docs/ACCEPTANCE_CRITERIA.md](docs/ACCEPTANCE_CRITERIA.md) | Acceptance criteria for live mission |
| [docs/TIRE_DETECTION_CLEANUP_REPORT.md](docs/TIRE_DETECTION_CLEANUP_REPORT.md) | Tire pipeline, cleanup script, verification |
| [docs/FINAL_VERIFICATION_AND_HANDOVER.md](docs/FINAL_VERIFICATION_AND_HANDOVER.md) | Final verification and production handover |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Common issues and solutions |
| [docs/TESTING_AND_VALIDATION.md](docs/TESTING_AND_VALIDATION.md) | Simulation and real-hardware validation procedure |
| [docs/RELEASE_NOTES_v1.0.0.md](docs/RELEASE_NOTES_v1.0.0.md) | Release notes template for v1.0.0 |
| [src/Tyre_Inspection_Bot/ARCHITECTURE.md](src/Tyre_Inspection_Bot/ARCHITECTURE.md) | Hardware/software layout |
| [src/Tyre_Inspection_Bot/DEPLOYMENT.md](src/Tyre_Inspection_Bot/DEPLOYMENT.md) | ROS 2 version, Ubuntu, build |

## Installation

1. Install ROS 2 (Humble or Jazzy), Nav2, TF2
2. Install [SLAMTEC Aurora ROS 2 SDK](https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/) into `src/`
3. `pip3 install -r requirements.txt`
4. `colcon build && source install/setup.bash`
5. Place `best_fallback.pt` or `best_fallback.engine` (tire model) in `~/ugv_ws/` or `src/Tyre_Inspection_Bot/`. On Jetson, run `scripts/export_tensorrt.sh` to build the engine for fast inference.

See [src/Tyre_Inspection_Bot/README.md](src/Tyre_Inspection_Bot/README.md) for full install and manual launch steps.
