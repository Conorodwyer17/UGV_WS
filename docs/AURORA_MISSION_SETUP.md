# Slamtec Aurora — Tire Inspection Mission Setup

Use **Aurora native (factory-calibrated)** outputs only. No checkerboard or fisheye calibration.

## What the Aurora publishes (use these)

| Use case | Topic | Notes |
|----------|--------|------|
| **Where the robot is** | `/slamware_ros_sdk_server_node/odom` | Odometry |
| **Where the robot is (map)** | `/slamware_ros_sdk_server_node/robot_pose` | Pose in map |
| **Where it's going / map** | `/slamware_ros_sdk_server_node/map` | Occupancy grid for Nav2 |
| **Obstacles (LiDAR)** | `/slamware_ros_sdk_server_node/scan` | For costmaps |
| **Depth (distance)** | `/slamware_ros_sdk_server_node/depth_image_raw` | Factory-calibrated depth image |
| **Point cloud (3D)** | `/slamware_ros_sdk_server_node/point_cloud` | Frame: slamware_map |
| **RGB for detection** | `/slamware_ros_sdk_server_node/left_image_raw` | YOLO/segmentation |
| **Stereo keypoints** | `/slamware_ros_sdk_server_node/stereo_keypoints` | Optional debug |
| **Semantic** | `/slamware_ros_sdk_server_node/semantic_segmentation` | If supported |

## Pipeline (no bridge by default)

1. **Aurora** publishes raw depth, point cloud, scan, odom, map, images.
2. **depth_to_registered_pointcloud** (in segment_3d launch) subscribes to `/slamware_ros_sdk_server_node/depth_image_raw` + `/camera/depth/camera_info`, publishes:
   - `/segmentation_processor/registered_pointcloud` (for 3D boxes)
   - `/camera/depth/points` (for Nav2 costmap)
3. **segment_3d** uses `/slamware_ros_sdk_server_node/left_image_raw` + registered_pointcloud → 3D bounding boxes.
4. **Nav2** uses scan, odom, map, and `/camera/depth/points` for planning.

## Launch: one command (recommended)

Start the full stack (Aurora, Nav2, perception, inspection manager) in one go:

```bash
./scripts/startup.sh
```

With options (e.g. different Aurora IP or production config):

```bash
./scripts/startup.sh ip_address:=192.168.11.1
./scripts/startup.sh config_file:=/path/to/ugv_ws/PRODUCTION_CONFIG.yaml
```

The script sources the workspace, sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and runs `ros2 launch ugv_nav full_bringup.launch.py`. Nodes start in order with delays so dependencies are up first (Aurora → segment_3d → Nav2 → inspection_manager). **Inspection manager is delayed to 135s** so Nav2’s NavigateToPose action server is active before the mission tries to rotate or drive. Allow ~2 minutes after launch before the mission can navigate.

**If Aurora logs "Depth camera not supported":** the device is not publishing depth or point cloud; segmentation will report "No PointCloud2 message available" and 3D vehicle/tire detection will not work until the unit provides depth (firmware/model support).

**Velocity flow:** Nav2 → cmd_vel_nav → depth_gate (gated by /stereo/navigation_permitted) → cmd_vel → motor_driver → ESP32. When use_bridge=false, `navigation_permitted_publisher` publishes True so depth_gate forwards cmd_vel. Motor driver is launched by full_bringup (use_motor_driver:=false to skip if no hardware).

## Launch order (manual, four terminals)

If you prefer to start components separately:

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1) Aurora (native depth; no bridge)
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1

# 2) In another terminal: Nav2
ros2 launch ugv_nav nav_aurora.launch.py

# 3) In another terminal: perception (YOLO + 3D boxes + depth pipeline)
ros2 launch segmentation_3d segment_3d.launch.py

# 4) In another terminal: mission
ros2 launch inspection_manager inspection_manager.launch.py
```

## Pre-mission check

```bash
bash scripts/aurora_pre_mission_checklist.sh
```

Verifies: device ping, Aurora topics (`/slamware_ros_sdk_server_node/depth_image_raw`, `/slamware_ros_sdk_server_node/point_cloud`, `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/odom`, etc.), depth pipeline topics, TF slamware_map → base_link.

**Aurora-only TF diagnostic (run with only Aurora launch):** Ensures TF and Aurora topics are good before full stack. `aurora_bringup.launch.py` sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` so the launch and the diagnostic script see the same topics/TF. See `docs/AURORA_TF_AND_LAUNCH.md` and run `bash scripts/aurora_tf_diagnostic.sh` in a second terminal after starting Aurora.

## Reference

- **All Aurora topics:** `docs/AURORA_TOPICS_REFERENCE.md`
- **Phase 4 (nav, perception):** `docs/AURORA_PHASE4_NAV_PERCEPTION.md`
