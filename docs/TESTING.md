# Testing Guide

## Unit Tests

The inspection_manager package includes unit tests:

```bash
cd ~/ugv_ws
source install/setup.bash
colcon test --packages-select inspection_manager
colcon test-result --all
```

Tests cover:
- `test_goal_generator.py` – goal computation from box
- `test_vehicle_modeler.py` – tyre position estimation
- `test_mission_state_machine.py` – state transitions
- `test_perception_handler.py` – vehicle/tire selection
- `test_photo_trigger.py` – distance threshold logic

## Simulation Mission Test

Full mission simulation (no hardware):

```bash
ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true
```

Let it run for a complete mission cycle. Verify:
- Vehicle detected within 30 s
- All 4 tyres visited in order
- Photos captured at each tyre
- No node crashes

## System Verification

```bash
bash scripts/pre_mission_verify.sh
python3 scripts/verify_system.py --skip-ros
```

With ROS running:
```bash
python3 scripts/verify_system.py
```

Checks: TF tree, required topics, CUDA, disk space.

## Simulated Detections (No Vehicle)

To test state machine without a real car:

```bash
# Terminal 1: Full stack
./scripts/start_mission.sh

# Terminal 2: After stack is up
python3 scripts/verify_system.py --simulate --duration 120 --publish-objects-segment
```
