# Cursor AI Agent Changelog

## 2026-02-22 — Master directive execution

### Phases A-I (summary)

- **Phase A+B**: Inventory, Aurora connectivity, repo audit, feature map
- **Phase C**: Calibration audit, validator, stereo_calibration_verified.yaml
- **Phase D**: TF determinism (tf_max_age_ms), tf_latency_trace
- **Phase E**: Stereo pipeline (max_stereo_sync_delta_ms, stereo_sgbm_params_verified)
- **Phase F**: Point cloud validation, costmap verified
- **Phase G**: depth_gate hardening (stale_timeout 300ms, fail-safe default)
- **Phase H**: Closed-loop test procedure documented (requires live car)

### Modified files (outside Tyre_Inspection_Bot sub-repo)

- scripts/calibration_audit.py, calibration_validator.py
- scripts/tf_latency_trace.py, collect_lr_timestamps.py, pointcloud_validate.py
- scripts/aurora_stereo_calibration.py (rms_error)
- src/aurora_sdk_bridge/ (launch, config, scripts)
- src/Tyre_Inspection_Bot/.../segment_3d/ (config stereo_calibration_verified)
- ugv_nav/scripts/depth_gate_node.py (in Tyre_Inspection_Bot): nav_permitted_default=False, stale_timeout_s=0.3
