# Nav2 Parameters for Aurora-Based UGV

These YAML files configure Nav2 for the Aurora SLAM system. Aurora provides localisation and map; Nav2 handles path planning and control.

## Parameter Files

| File | Use case |
|------|----------|
| `nav_aurora.yaml` | Default: standard goal tolerances (xy 0.15 m, yaw 0.25 rad) |
| `nav_aurora_ekf.yaml` | EKF fusion: uses `/odometry/filtered` when wheel odom is available |
| `nav_aurora_tight_full.yaml` | Tighter tolerances: xy 0.1 m, max_vel 0.18 m/s |
| `nav_aurora_tight_ekf.yaml` | Tight + EKF |
| `nav_aurora_sim.yaml` | Simulation: rolling costmap, lower frequencies |

## Key Parameters

- **bt_navigator:** `default_nav_to_pose_bt_xml`, `default_nav_through_poses_bt_xml` — behaviour tree XML paths (substituted at launch with package-relative paths).
- **controller_server:** `xy_goal_tolerance`, `yaw_goal_tolerance` — arrival thresholds. Tighter values improve tyre approach but may increase oscillations.
- **local_costmap / global_costmap:** Use `/slamware_ros_sdk_server_node/scan` and `/camera/depth/points` for obstacles.
- **ekf_aurora.yaml:** robot_localization EKF when fusing Aurora odom + wheel odom.

## Selection Logic

The launch file (`nav_aurora.launch.py`) selects the param file based on:
- `sim_tyre_detections:=true` → `nav_aurora_sim.yaml`
- `use_tight_goal_tolerance:=true` and `use_ekf:=true` → `nav_aurora_tight_ekf.yaml`
- `use_tight_goal_tolerance:=true` → `nav_aurora_tight_full.yaml`
- `use_ekf:=true` → `nav_aurora_ekf.yaml`
- Default → `nav_aurora.yaml`
