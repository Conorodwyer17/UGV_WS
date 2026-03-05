# Simulation Results — Tyre Inspection Robot

This document summarises simulation testing of the autonomous tyre inspection robot using `verify_system.py --simulate` and the full stack. It records test scenarios, observed state transitions, parameter tuning, and readiness for field trials.

---

## 1. Test Setup

**One-command launch (no-move + synthetic detections):**
```bash
ros2 launch ugv_nav full_bringup.launch.py sim_no_move:=true sim_tyre_detections:=true
```

**Manual:** `./scripts/start_mission.sh` in terminal 1; `python3 scripts/verify_system.py --simulate --publish-objects-segment` in terminal 2.

**No-move mode:** `sim_no_move:=true` launches `stub_motor_node` instead of `motor_driver_node`. The stub subscribes to `/cmd_vel` and discards all commands; the robot does not physically move. Aurora still provides SLAM, odometry, and depth.

**Topics published by simulation (one-command launch with sim_tyre_detections:=true):**
- `/sim/vehicle_bounding_boxes` — BoundingBoxes3d (1 vehicle at 3 m in slamware_map)
- `/sim/tire_bounding_boxes_merged` — BoundingBoxes3d (4 tires at vehicle corners)
- `/ultralytics_tire/segmentation/objects_segment` — ObjectsSegment (optional, with `--publish-objects-segment` when TF available)

The inspection manager is remapped to these sim topics so it sees only simulated detections (no competition from aurora_semantic_fusion / tire_merger). Manual runs can still use the live topics via `--topic-vehicle` and `--topic-tire`.

**Frame:** All boxes in `slamware_map`. Vehicle: 4.5 m × 2 m × 1.5 m, centred at (3, 0, 0.75). Tires at ±1.5 m (front/rear), ±1 m (left/right).

---

## 2. Test Scenarios

| Scenario | Parameters | Purpose |
|----------|------------|---------|
| **Basic mission** | `--duration 300` | Full detection rate; verify state machine transitions. |
| **Missing detections** | `--detection-rate 0.8` | Simulate occasional missed cycles; test merger fallback. |
| **Occluded tyre** | `--tire-drop-probability 0.05` | Occasionally drop one tyre; test recovery. |
| **Noise** | `--position-variance 0.1` | Tire position jitter; test spatial filter and refinement. |
| **Centroid servo** | `--publish-objects-segment` | ObjectsSegment for visual servoing; requires TF. |
| **Multi-vehicle** | `--vehicle-count 2` | Two vehicles; test mission sequencing. |
| **Low confidence** | `--confidence-mean 0.7` | Test inspection manager with lower confidence. |
| **Real Aurora vehicles** | `--no-publish-vehicle` | Use real Aurora vehicle boxes; only inject tyre detections. |

---

## 3. Expected State Transitions

With the full stack running and simulation publishing:

1. **IDLE** → **SEARCH_VEHICLE** — Mission starts when TF and Nav2 are ready.
2. **SEARCH_VEHICLE** → **WAIT_VEHICLE_BOX** — Vehicle detected on vehicle topic (`/sim/vehicle_bounding_boxes` when sim_tyre_detections:=true).
3. **WAIT_VEHICLE_BOX** → **APPROACH_VEHICLE** — Approach goal dispatched.
4. **APPROACH_VEHICLE** → **WAIT_TIRE_BOX** — Nav2 arrived at goal.
5. **WAIT_TIRE_BOX** → **INSPECT_TIRE** — Tire detected on tire topic (`/sim/tire_bounding_boxes_merged` when sim_tyre_detections:=true).
6. **INSPECT_TIRE** → **FACE_TIRE** / **WAIT_WHEEL_FOR_CAPTURE** → **VERIFY_CAPTURE** — Centroid servo (if enabled) and photo capture.
7. **VERIFY_CAPTURE** → **WAIT_TIRE_BOX** (next tire) or **NEXT_VEHICLE** / **DONE** — After four tires.

Monitor: `ros2 topic echo /inspection_manager/runtime_diagnostics`.

---

## 4. Parameter Tuning Outcomes

| Parameter | Default | Recommended | Notes |
|-----------|---------|-------------|-------|
| `--detection-rate` | 1.0 | 0.95–1.0 | Lower values test recovery; 1.0 for baseline. |
| `--position-variance` | 0.05 | 0.05–0.1 | Higher values test spatial filter. |
| `--confidence-mean` | 0.85 | 0.8–0.9 | Must exceed `tire_min_prob` in inspection_manager. |

---

## 5. Issues Found and Resolutions

| Issue | Resolution |
|-------|------------|
| Missing Header import | Added `from std_msgs.msg import Header` (already present in current script). |
| Static detections only | Added configurable noise, detection rate, and position variance. |
| Centroid servo not exercised | Added `--publish-objects-segment` to publish ObjectsSegment when TF available. |
| 2 Hz vs 5 Hz | Increased publish rate to 5 Hz to match CPU inference node. |

---

## 6. Simulation Pipeline Perfection (2026-03)

**Tire goal generation:** `vehicle_modeler.estimate_tire_positions_from_box` now uses box dimensions when they exceed default wheelbase/track (e.g. sim vehicle 4.5×2 m). Tire positions align with box corners.

**Global costmap:** When `sim_tyre_detections:=true`, Nav2 uses `nav_aurora_sim.yaml` with rolling 20×20 m global costmap (no static layer; Aurora map may be small when robot stationary). Note: `width` and `height` must be integers (nav2_costmap_2d requirement).

**CPU load:** Controller 5 Hz, planner 10 Hz; `ultralytics_tire` and `tire_merger` disabled in sim.

**Tire detections:** verify_system tire offsets aligned with vehicle_modeler (2.13, ±0.88) from center. `tire_confirmations_required: 1` in PRODUCTION_CONFIG_SIM.yaml.

---

## 7. Readiness for Field Trials

After simulation testing:

- **State machine:** Transitions SEARCH_VEHICLE → WAIT_VEHICLE_BOX → APPROACH_VEHICLE → WAIT_TIRE_BOX → INSPECT_TIRE → VERIFY_CAPTURE verified.
- **Detection injection:** Vehicle and tire boxes accepted by inspection_manager.
- **Recovery:** Occasional missed detections and dropped tires handled by merger and PCL fallback.
- **Centroid servo:** ObjectsSegment projection works when TF (camera_left ← slamware_map) is available.

**Next step:** Run extended simulation (e.g. 1 hour) with `--detection-rate 0.95` to stress-test recovery, then proceed to field validation with a real vehicle.

---

## 8. Logs and Analysis

Simulation logs can be written to `logs/simulation/` for post-analysis. Run:

```bash
mkdir -p logs/simulation
python3 scripts/verify_system.py --simulate --duration 300 2>&1 | tee logs/simulation/run_$(date +%Y%m%d_%H%M%S).log
```

Monitor mission report: `logs/mission_report_latest.json`.
