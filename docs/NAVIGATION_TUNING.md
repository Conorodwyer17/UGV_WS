# Navigation Tuning Guide

This guide describes how to tune Nav2 parameters for the Aurora-based UGV tyre inspection robot.

## Parameter Files

| File | Use case |
|------|----------|
| `nav_aurora.yaml` | Default: standard tolerances |
| `nav_aurora_tight_full.yaml` | Tighter xy (0.1 m), lower max_vel (0.18 m/s) |
| `nav_aurora_ekf.yaml` | EKF fusion with wheel odom |
| `nav_aurora_sim.yaml` | Simulation: rolling costmap |

## Key Parameters

### Controller (controller_server)

- **xy_goal_tolerance**: Distance from goal centre to consider arrived. Default 0.15 m; tighter (0.1 m) for tyre approach but may increase oscillations.
- **yaw_goal_tolerance**: Angular tolerance. Default 0.25 rad (~14°).
- **max_vel_x**: Maximum linear velocity. Reduce for safer approach.
- **transform_tolerance**: TF lookup tolerance. Default 2.0 s for Aurora TF timing.

### Costmap

- **robot_radius**: Robot footprint radius. Must match actual robot dimensions.
- **inflation_radius**: Obstacle inflation. Larger = safer clearance, narrower passages.
- **observation_sources**: Includes `/slamware_ros_sdk_server_node/scan` and `/camera/depth/points`.

### Common Adjustments

**Robot oscillates at tyre:**
- Increase `xy_goal_tolerance` to 0.2 m
- Reduce `max_vel_x` when approaching

**Robot does not reach goal:**
- Check `transform_tolerance` (TF latency)
- Verify costmap is not blocking (inspect costmap in RViz)

**Recovery behaviour:**
- Behaviour tree: ClearCostmap → Wait → BackUp
- See `behavior_trees/navigate_to_pose_no_spin.xml`

## Tuning Procedure

1. Run simulation: `ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true`
2. Launch RViz: `ros2 launch ugv_nav ugv_visualization.launch.py`
3. Observe costmap and path during mission
4. Adjust parameters in `ugv_nav/param/` and rebuild
5. Re-test in simulation before field deployment
