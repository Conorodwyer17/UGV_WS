# Repository Cleanup Plan — Essential Files Only

**Date:** 2026-03-04
**Goal:** Reduce the repository to essential files for build, run, and maintenance. End with exactly three startup scripts plus essential utilities.

---

## 1. Analysis Summary

### 1.1 Repository Structure

| Directory | Purpose | Action |
|-----------|---------|--------|
| `src/` | Core packages (inspection_manager, segmentation_3d, ugv_nav, aurora_mock, sim, Tyre_Inspection_Bot) | **Keep all** source code |
| `scripts/` | 21 files: startup, diagnostics, profiling, verification | **Reduce** to 7 files |
| `docs/` | 45+ Markdown files | **Reduce** to 5–6 essential |
| `sim/` | Simulation launch, bags, test harness | **Keep** launch and bags; **remove** redundant docs |
| `patches/` | Jetson optimizations, diagrams | **Delete** (not in active use) |
| `thesis/` | Thesis draft | **Delete** (not part of deployment) |
| `ci/` | CI pipeline | **Keep** (run_tests.sh) |
| `logs/` | Mission logs (generated) | **Keep** directory; contents are runtime data |
| `PRODUCTION_CONFIG.yaml` | Mission parameters | **Keep** |

### 1.2 Model Files

| File | Location | Action |
|------|----------|--------|
| `best_fallback.pt` | `src/Tyre_Inspection_Bot/` | **Keep** (essential) |
| `best_fallback.engine` | `src/Tyre_Inspection_Bot/` | **Keep** (essential) |
| `best_fallback.onnx` | `src/Tyre_Inspection_Bot/` | **Delete** (regenerable) |
| `yolov8n.pt`, `yolov8n-cls.pt`, `yolov8m.pt`, `yolov8m-seg.pt` | `~/ugv_ws/` (root) | **Delete** (not used; segment_3d uses best_fallback) |

### 1.3 Scripts (Current → Target)

| Current Script | Purpose | Action |
|----------------|---------|--------|
| `start_mission.sh` | Full mission with verify | **Keep** (one of three) |
| `startup.sh` | Full bringup (no verify) | **Delete** (redundant with start_mission) |
| `mission_launch.sh` | Wrapper for startup.sh | **Delete** (redundant) |
| `cleanup.sh` | Build artifact cleanup | **Keep** (essential utility) |
| `export_tensorrt.sh` | TensorRT engine export | **Keep** (essential utility) |
| `pre_mission_verify.sh` | Pre-flight checks | **Keep** (essential utility) |
| `verify_system.py` | System verification | **Keep** (essential utility) |
| `ugv_mission.service` | systemd unit | **Keep** (deployment) |
| `profile_mission.sh` | Performance profiling | **Delete** |
| `analyze_profile_log.py` | Profile analysis | **Delete** |
| `analyze_aurora_bag.py` | Bag analysis | **Delete** |
| `compare_sim_to_real.py` | Sim vs real comparison | **Delete** |
| `diagnose_mission_stopped.sh` | Diagnostic | **Delete** |
| `diagnose_mission_not_starting.sh` | Diagnostic | **Delete** |
| `e2e_system_test.sh` | E2E test | **Delete** |
| `follow_mission.sh` | Mission follow | **Delete** |
| `hardware_integration_check.sh` | Hardware check | **Delete** |
| `inspection_no_movement_diagnose.sh` | Diagnostic | **Delete** |
| `jetson_opt.sh` | Jetson tuning | **Delete** |
| `launch_rviz_vm.sh` | RViz from VM | **Delete** |
| `mission_status.sh` | Status check | **Delete** |
| `full_system_check_live_test.md` | Test doc | **Delete** |
| `cmd_vel_watchdog_node.py` | Watchdog node | **Evaluate** — if part of a package, keep in package; if standalone script, delete |

---

## 2. Essential Files to Keep

### 2.1 Source Code (all under `src/`)

- All packages: `inspection_manager`, `segmentation_3d`, `ugv_nav`, `aurora_mock`, `sim`, `Tyre_Inspection_Bot`, `aurora_ros2_sdk_linux`
- All `.py`, `.cpp`, `.h`, `.yaml`, `.launch.py` in these packages
- `best_fallback.pt`, `best_fallback.engine` in `src/Tyre_Inspection_Bot/`

### 2.2 Scripts (final set)

| Script | Purpose |
|--------|---------|
| `scripts/start_mission.sh` | Full mission with hardware (Aurora, Nav2, motor, inspection) |
| `scripts/start_simulation.sh` | **NEW** — Simulation with aurora_mock (no hardware) |
| `scripts/start_vision_only.sh` | **NEW** — Perception only (Aurora + segment_3d, no Nav2/motor) |
| `scripts/cleanup.sh` | Clean build artifacts |
| `scripts/export_tensorrt.sh` | Rebuild TensorRT engine |
| `scripts/pre_mission_verify.sh` | Pre-flight verification |
| `scripts/verify_system.py` | System verification |
| `scripts/ugv_mission.service` | systemd unit for headless |

### 2.3 Documentation (keep)

| File | Purpose |
|------|---------|
| `README.md` | Project overview |
| `RUNBOOK.md` | Operational runbook |
| `docs/TIRE_DETECTION_TROUBLESHOOTING.md` | Perception debugging |
| `docs/100_PERCENT_RELIABILITY_PLAN.md` | Reliability roadmap |
| `docs/ACCEPTANCE_CRITERIA.md` | Acceptance criteria |

### 2.4 Config and CI

- `PRODUCTION_CONFIG.yaml`
- `ci/ci_pipeline.yml`, `ci/run_tests.sh`

---

## 3. Files to Delete

### 3.1 Scripts to Delete

```
scripts/analyze_aurora_bag.py
scripts/analyze_profile_log.py
scripts/compare_sim_to_real.py
scripts/diagnose_mission_stopped.sh
scripts/diagnose_mission_not_starting.sh
scripts/e2e_system_test.sh
scripts/follow_mission.sh
scripts/full_system_check_live_test.md
scripts/hardware_integration_check.sh
scripts/inspection_no_movement_diagnose.sh
scripts/jetson_opt.sh
scripts/launch_rviz_vm.sh
scripts/mission_launch.sh
scripts/mission_status.sh
scripts/profile_mission.sh
scripts/startup.sh
```

**Note:** `cmd_vel_watchdog_node.py` — standalone script in scripts/; not in any package. Delete.

### 3.2 Model Files to Delete

```
/home/conor/ugv_ws/src/Tyre_Inspection_Bot/best_fallback.onnx
/home/conor/ugv_ws/yolov8n.pt
/home/conor/ugv_ws/yolov8n-cls.pt
/home/conor/ugv_ws/yolov8m.pt
/home/conor/ugv_ws/yolov8m-seg.pt
```

### 3.3 Documentation to Delete

**Root-level .md (move or delete):**
```
integration_filter_report.md
mission_determinism_audit.md
autonomous_tyre_inspection_master_plan.md
system_reaudit_report.md
failure_injection_defense_report.md
stress_test_results.md
nav2_alignment_report.md
concurrency_review.md
observability_upgrade.md
perception_stability_report.md
best_practices_matrix.md
integration_plan.md
research_report.md
FINAL_README.md
```

**docs/ to delete:**
```
docs/PATH_FORWARD_NAVIGATION_AND_PERCEPTION.md
docs/DEEP_RESEARCH_PROMPT.md
docs/PERFECTION_AUDIT.md
docs/TROUBLESHOOTING.md
docs/TESTING_AND_VALIDATION.md
docs/RELEASE_NOTES_v1.0.0.md
docs/FINAL_VERIFICATION_AND_HANDOVER.md
docs/TIRE_DETECTION_CLEANUP_REPORT.md
docs/COSTMAP_AND_PATH.md
docs/KNOWN_ISSUES.md
docs/PERFORMANCE_TUNING.md
docs/CORRECTIVE_ACTION_PLAN_FIRST_MISSION.md
docs/FIELD_READINESS_PLAN.md
docs/E2E_SYSTEM_TEST_REPORT.md
docs/FINAL_READINESS_AND_LAUNCH.md
docs/PRODUCTION_READY_SUMMARY.md
docs/CONTINUOUS_IMPROVEMENT_PLAN.md
docs/REFLECTIVE_RIM_HANDLING.md
docs/INDUSTRIAL_REFINEMENT_REPORT.md
docs/aurora_topics_and_frames.md
docs/AURORA_VEHICLE_DETECTION.md
docs/BEST_FALLBACK_MODEL_CLASSES.md
docs/MISSION_FOLLOW_2026-03-02_17-06.md
docs/STARTUP_AND_PLAN_VERIFICATION.md
docs/MISSION_ALL_FOUR_TIRES_AND_FAR_SIDE.md
docs/MISSION_REVIEW_TWO_PHOTOS_2026-03-02.md
docs/MISSION_REVIEW_2026-03-02.md
docs/MISSION_TIRE_ORDER_AND_SCENARIO.md
docs/NAVIGATION_SAFETY.md
docs/NO_MOVEMENT_DEBUG.md
docs/PRE_LAUNCH_CHECKLIST.md
docs/MISSION_READINESS_CHECKLIST.md
docs/PROJECT_OVERVIEW.md
docs/TOPIC_VERIFICATION.md
docs/VEHICLE_INSPECTION_FAILURE_MODES.md
docs/vehicle_detection_to_navigation_flow.md
docs/vision_navigation_dependencies.md
docs/VISION_PIPELINE.md
docs/HOW_WE_KNOW_WHERE_TO_DRIVE.md
docs/TIRE_INSPECTION_APPROACH_SPEC.md
docs/TF_TREE.md
```

**Other docs to delete:**
```
research_log.md
src/Tyre_Inspection_Bot/ARCHITECTURE.md
src/Tyre_Inspection_Bot/DEPLOYMENT.md
src/Tyre_Inspection_Bot/README.md
sim/README.md
sim/bags/README.md
patches/README.md
patches/perf_cluster/README.md
thesis/thesis_first_draft.md
thesis/README.md
```

### 3.4 Directories to Delete

```
patches/          # Jetson opt, diagrams — not in active use
# thesis/          # KEPT per user request
```

### 3.5 Other Files to Delete

```
get-docker.sh     # Root-level; not part of tire inspection
```

---

## 4. Startup Scripts

### 4.1 `scripts/start_mission.sh` (existing, keep)

- Already does: Jetson max perf, disk check, pre_mission_verify, full_bringup
- No changes needed

### 4.2 `scripts/start_simulation.sh` (create new)

**Purpose:** Launch simulation with aurora_mock (no hardware).

**Content:**
```bash
#!/bin/bash
# Start tire inspection simulation (aurora_mock, no hardware).
# Usage: ./scripts/start_simulation.sh [use_bag:=false] [use_mock:=true]

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
INSTALL_SETUP="$UGV_WS/install/setup.bash"

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "ERROR: install/setup.bash not found. Build first: cd $UGV_WS && colcon build --symlink-install"
  exit 1
fi

source "$INSTALL_SETUP"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "=== Tire Inspection Simulation (aurora_mock) ==="
exec ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true "$@"
```

### 4.3 `scripts/start_vision_only.sh` (create new)

**Purpose:** Launch only perception (Aurora + segment_3d). No Nav2, no motor. Dry run to test detection.

**Requires:** New launch file `vision_only.launch.py` in ugv_nav that launches:
- aurora_bringup
- segment_3d (delayed)

**Content of `start_vision_only.sh`:**
```bash
#!/bin/bash
# Start perception pipeline only (Aurora + segment_3d). No Nav2, no motor.
# Usage: ./scripts/start_vision_only.sh [ip_address:=192.168.11.1]

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
INSTALL_SETUP="$UGV_WS/install/setup.bash"

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "ERROR: install/setup.bash not found. Build first: cd $UGV_WS && colcon build --symlink-install"
  exit 1
fi

source "$INSTALL_SETUP"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Aurora SDK library path
AURORA_LIB="$UGV_WS/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib"
case "$(uname -m)" in
  aarch64) AURORA_LIB="$AURORA_LIB/linux_aarch64" ;;
  x86_64)  AURORA_LIB="$AURORA_LIB/linux_x86_64"  ;;
  *)       AURORA_LIB="" ;;
esac
if [[ -n "$AURORA_LIB" ]] && [[ -d "$AURORA_LIB" ]]; then
  export LD_LIBRARY_PATH="$AURORA_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

echo "=== Vision Only (Aurora + segment_3d) ==="
exec ros2 launch ugv_nav vision_only.launch.py "$@"
```

---

## 5. Launch File Modifications

### 5.1 Create `ugv_nav/launch/vision_only.launch.py`

**Purpose:** Launch Aurora + segment_3d only. No Nav2, no motor, no inspection manager.

**Content:** Include aurora_bringup and segment_3d with appropriate delays. Reuse existing launch descriptions from full_bringup.

---

## 6. Cleanup Script

See `scripts/cleanup_repo.sh` below. It will:
- Support `--dry-run` (default: list only, no delete)
- Support `--execute` (actually delete)
- Create backup as `backup_pre_cleanup_YYYYMMDD_HHMMSS.tar.gz` when `--execute`
- Require confirmation before delete when `--execute`

---

## 7. Summary

| Category | Before | After |
|----------|--------|-------|
| Scripts | 21 | 8 |
| Docs (root + docs/) | 60+ | 5 |
| Model files | 6 | 2 |
| Directories removed | — | patches/, thesis/ |
