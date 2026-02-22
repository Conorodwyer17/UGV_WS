# Cursor AI Agent Changelog

## 2026-02-22 — Master directive execution start

### Timestamp
- Start: 2026-02-22 (UTC)

### Phase A — Inventory & Baseline (in progress)

#### Actions performed
1. Created `logs/cursor_runs/initial/` and `logs/phaseA/` directories
2. Ran immediate first command sequence:
   - Git status: workspace root `/home/conor/ugv_ws` is **not a git repository**
   - Ping 192.168.11.1: **SUCCESS** (3/3 packets, ~0.4 ms RTT)
   - Jetson enP8p1s0: 192.168.11.131/24 (same subnet as Aurora)
3. Aurora demo build: Already built; `depthcam_view` exists at `aurora_remote_sdk_demo/build/depthcam_view`
4. Aurora depthcam_view demo: **BLOCKED** — Device connects but reports "Depth camera is not supported by this device"
   - Auto-discovery works: found tcp/192.168.11.1:7447
   - Classic Aurora (stereo) does not have SDK 2.0 hardware depth camera; ROS stack uses software SGBM stereo depth
5. Colcon build: Running...

#### Files modified
- `docs/CHANGELOG_CURSOR.md` (created)

### Phase C - Calibration Rebuild (2026-02-22)

#### Actions
- Audited equidistant_calibration.yaml, stereo_calibration_verified.yaml
- Created calibration_audit.py, calibration_validator.py
- Added stereo_calibration_verified.yaml to segmentation_3d/config
- Updated aurora_stereo_calibration.py to output rms_error
- aurora_sdk_bridge launch prefers verified calibration path

#### Result
- Depth consistency: PASS (0% error)
- RMS: Pending live checkerboard (40-60 pairs)

### Phase D - TF Determinism (2026-02-22)

#### Actions
- segmentation_processor: tf_max_age_ms (250), reject stale frames
- tf_latency_trace.py: 60s histogram script
- config: tf_max_age_ms: 250
