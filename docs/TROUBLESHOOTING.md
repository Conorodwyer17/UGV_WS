# Troubleshooting Guide

Consolidated troubleshooting for the UGV tyre inspection robot.

## Quick Reference

| Area | Document |
|------|----------|
| Tyre detection (YOLO, TensorRT, OOM) | [TIRE_DETECTION_TROUBLESHOOTING.md](TIRE_DETECTION_TROUBLESHOOTING.md) |
| Vision pipeline (camera, intrinsics) | [VISION_TROUBLESHOOTING.md](VISION_TROUBLESHOOTING.md) |
| Navigation (Nav2, costmap) | [NAVIGATION_TUNING.md](NAVIGATION_TUNING.md) |
| Field recovery | [FIELD_RECOVERY.md](FIELD_RECOVERY.md) |

## Common Issues

**Mission does not start:**
- TF `slamware_map` → `base_link` must be valid. Check Aurora connection.
- Nav2 `navigate_to_pose` action server must be available. Wait for lifecycle (45–120 s after launch).

**No vehicle detection:**
- Aurora semantic + depth must be publishing. Check `/aurora_semantic/vehicle_bounding_boxes`.
- With `use_mock:=true` sim, synthetic vehicles are published.

**No tyre detection:**
- YOLO model must load. Check logs for `ultralytics_tire`.
- See [TIRE_DETECTION_TROUBLESHOOTING.md](TIRE_DETECTION_TROUBLESHOOTING.md).

**Robot spins or does not move:**
- Check `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in all terminals.
- Use `scripts/start_mission.sh` or `scripts/startup.sh` to set it.

**Build fails:**
- Ensure ROS 2 Humble (or Jazzy) and dependencies installed.
- Run `rosdep install --from-paths src --ignore-src -r -y`.
