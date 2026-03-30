# UGV Workspace – Autonomous Tyre Inspection Robot

An autonomous robot for inspecting commercial vehicle tyres using ROS 2, computer vision, and SLAM-based navigation. Runs on a Waveshare UGV Rover with SLAMTEC Aurora 6DOF.

## Overview

- **Vehicles:** Aurora COCO80 semantic (car, truck, bus)
- **Tyres:** YOLO `best_fallback.pt` (wheel class)
- **Navigation:** SLAMTEC Aurora 6DOF SLAM + Nav2
- **Mission:** State machine, 3D bounding boxes, inspection photo capture

## Quick start

**On robot (real hardware):**
```bash
cd ~/ugv_ws
bash scripts/start_mission.sh
```

Or `bash scripts/mission_launch.sh` (pre-flight + launch) or `bash scripts/startup.sh` (launch only). `start_mission.sh` checks disk space, sets Jetson max performance, optionally runs verification, then launches the full stack. Inspection manager starts ~120 s after launch.

**Simulation (no hardware):** To run with mock data (synthetic Aurora, no real sensor):
```bash
cd ~/ugv_ws
source install/setup.bash
ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true
```

Note: `full_bringup.launch.py` does not fully support mock mode; it is intended for real hardware. Use `vehicle_inspection_sim.launch.py use_mock:=true` for simulation.

**Detection:** Vehicle boxes come from **Aurora** only by default (YOLO vehicle node is off to save GPU/CPU). Tyre detection uses YOLO `best_fallback.pt` and, when you build it locally, TensorRT `best_fallback.engine`. To free disk after setup, run `./scripts/cleanup.sh` (dry run) or `./scripts/cleanup.sh --execute` to remove large temporary files and optional packages; see [docs/TIRE_DETECTION_TROUBLESHOOTING.md](docs/TIRE_DETECTION_TROUBLESHOOTING.md).

## Documentation

| Doc | Description |
|-----|-------------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | System architecture, node graph, TF tree, data flow |
| [RUNBOOK.md](RUNBOOK.md) | Operations runbook, canonical stack, mission flow |
| [docs/MISSION_PIPELINE.md](docs/MISSION_PIPELINE.md) | Mission flow: approach vehicle → tyre inspection → photo |
| [docs/MISSION_FLOW.md](docs/MISSION_FLOW.md) | Mermaid state-machine diagram of the inspection mission |
| [docs/ACCEPTANCE_CRITERIA.md](docs/ACCEPTANCE_CRITERIA.md) | Acceptance criteria for live mission |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Consolidated troubleshooting guide |
| [docs/TIRE_DETECTION_TROUBLESHOOTING.md](docs/TIRE_DETECTION_TROUBLESHOOTING.md) | Tyre detection pipeline, OOM workarounds, CPU inference |
| [docs/SIMULATION_RESULTS.md](docs/SIMULATION_RESULTS.md) | Simulation outcomes |
| [docs/vision_benchmark_results.md](docs/vision_benchmark_results.md) | Vision inference benchmark |
| [SETUP.md](SETUP.md) | Fresh Jetson installation guide (JetPack 6.0, ROS 2, TensorRT) |

## Installation

1. Clone the repository and `cd` into the workspace.
2. Install ROS 2 (Humble or Jazzy), Nav2, TF2.
3. Install [SLAMTEC Aurora ROS 2 SDK](https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/) into `src/`.
4. Install Python dependencies: `pip3 install -r requirements.txt` (see [SETUP.md](SETUP.md) for Jetson-specific PyTorch).
5. Build: `colcon build --symlink-install && source install/setup.bash`
6. Place `best_fallback.pt` (tyre weights) in `src/Tyre_Inspection_Bot/`. Generate **TensorRT** `.engine` and **ONNX** `.onnx` files locally; they are not tracked in Git. On Jetson, run `scripts/export_tensorrt.sh` for the engine and `scripts/export_onnx.sh` for CPU ONNX (see [RUNBOOK.md](RUNBOOK.md)). Dedicated mission weights in `tyre_detection_project/best.pt` follow the same pattern.

See [SETUP.md](SETUP.md) for detailed Jetson installation and [RUNBOOK.md](RUNBOOK.md) for launch steps.
