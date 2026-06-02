# TF Frame Reference

Complete TF tree for the UGV tyre inspection robot (Aurora + Waveshare UGV Rover).

---

## Full TF Tree

```
map (Nav2 global planning frame)
 └── slamware_map (Aurora SLAM map frame; identity with map)
      └── odom (Aurora wheel/visual odometry frame)
           └── base_link (robot chassis centre)
                ├── base_footprint         (floor projection; z = −0.08 m below base_link)
                ├── laser                  (Aurora 2D LiDAR; z = 0.0315 m above base_link)
                ├── camera_depth_optical_frame  (depth projection; same as camera_left but separate name)
                ├── camera_left            (Aurora left stereo camera)
                │    ├── camera_right      (Aurora right stereo camera; x = 0.06 m right of camera_left)
                │    └── imu_link          (Aurora IMU; x = 0.03 m, yaw = −90°)
                ├── base_imu_link          (alias IMU frame; identity with base_link — only for URDF viz)
                ├── camera_holder_link     (mechanical camera mount)
                │    └── aurora_link       (Aurora sensor body — visualization only)
                └── pt_base_link / pt_link1 / pt_link2 / pt_camera_link  (pan-tilt mechanism — unused, visualization only)
```

---

## Publisher Map

| Transform | Publisher | Transport | Rate | Notes |
|-----------|-----------|-----------|------|-------|
| `map → slamware_map` | `world_frame_tf_publisher.py` | `/tf` | 10 Hz | Identity; current timestamp prevents costmap "old transform" errors |
| `slamware_map → odom` | `world_frame_tf_publisher.py` | `/tf` | 10 Hz | Identity; current timestamp |
| `base_link → base_footprint` | `world_frame_tf_publisher.py` | `/tf` | 10 Hz | z = −0.08 m; current timestamp avoids ~20s delay from RSP stamp=0 |
| `base_link → laser` | `world_frame_tf_publisher.py` | `/tf` | 10 Hz | Fallback for costmap until Aurora publishes slamware_map→laser |
| `base_link → camera_depth_optical_frame` | `world_frame_tf_publisher.py` | `/tf` | 10 Hz | Optical frame for depth costmap layer; matches URDF camera_left offset |
| `odom → base_link` | `slamware_ros_sdk_server_node` | `/tf` | ~20 Hz | Aurora primary localization |
| `slamware_map → laser` | `slamware_ros_sdk_server_node` | `/tf` | scan rate | Aurora provides when scan data is live |
| `base_link → camera_left` | `robot_state_publisher` | `/tf_static` | one-shot | From `ugv_rover.urdf`; stamp=0 (static) |
| `camera_left → camera_right` | `robot_state_publisher` | `/tf_static` | one-shot | From `ugv_rover.urdf` |
| `camera_left → imu_link` | `robot_state_publisher` | `/tf_static` | one-shot | From `ugv_rover.urdf` |

---

## Known Conflicts and Resolutions

### `base_link → laser` (dual publisher)

`world_frame_tf_publisher` and `slamware_ros_sdk_server_node` both publish transforms reaching the `laser` frame:

- `world_frame_tf_publisher`: `base_link → laser` at 10 Hz (current timestamp)
- `slamware_ros_sdk_server_node`: `slamware_map → laser` at scan rate

These are NOT conflicting — `laser` has different parents in each. TF2 would report a conflict only if both publishers claim the SAME parent→child pair. Here `base_link→laser` and `slamware_map→laser` give laser two parents, which IS a TF2 conflict.

**Impact**: Nav2 costmap will use the most recently published transform. In practice, once Aurora starts publishing `slamware_map→laser`, it dominates and the fallback `base_link→laser` becomes stale. This can cause brief inconsistency during startup.

**Recommendation**: Remove `base_link→laser` from `world_frame_tf_publisher.py` once Aurora is confirmed to always publish `slamware_map→laser` before the costmap starts processing scans. The comment in the code says this is a "fallback" — investigate whether it's still needed with Aurora 2.11 firmware.

### `camera_left` vs `camera_depth_optical_frame` (duplicate position)

Both frames are at offset (0.0418, 0.03, 0) with optical rotation from `base_link`. They represent the same physical position with different frame names:

- `camera_left`: published by `robot_state_publisher` via URDF, used for visualization
- `camera_depth_optical_frame`: published by `world_frame_tf_publisher`, used by the depth costmap layer and depth projection nodes

The URDF comment explains: `camera_depth_optical_frame` cannot be in the URDF because RSP would publish it to `/tf_static` with stamp=0, causing ~67s "transform too old" errors in Nav2 costmap layers.

**This is intentional and safe**, but the offsets (0.0418, 0.03, 0) must match the actual Aurora mounting position or the depth costmap will be wrong.

---

## Critical Hardcoded Values (Must Match Physical Robot)

| Value | Location | Description | Verify Against |
|-------|----------|-------------|----------------|
| `camera_depth_optical_frame` at (0.0418, 0.03, 0.0) | `world_frame_tf_publisher.py` line 89 | Aurora depth sensor position | Aurora SDK calibration / physical measurement |
| `laser` at (0.0, 0.0, 0.0315) | `world_frame_tf_publisher.py` line 76 | Aurora LiDAR height | Aurora SDK documentation (3.15 cm above base) |
| `camera_left` at xyz=(0.0418, 0.03, 0) | `ugv_rover.urdf` joint `camera_left_joint` | Stereo camera offset from base_link | Must match `world_frame_tf_publisher.py` values |

**If Aurora is not mounted at exactly (0.0418, 0.03, 0) from base_link**, update BOTH `world_frame_tf_publisher.py` AND `ugv_rover.urdf` to match the actual mounting position.

---

## Consumer Map

| Frame required | Consumer | Purpose |
|---------------|----------|---------|
| `slamware_map → base_link` | `inspection_manager_node.py` | Robot pose for goal generation, TF watchdog |
| `map → base_link` | Nav2 | Robot localization for planning |
| `odom → base_link` | Nav2 controller | Velocity feedback |
| `camera_depth_optical_frame → map` | Nav2 local costmap | Transform registered point cloud obstacles |
| `laser → map` | Nav2 global costmap | Transform LiDAR scan obstacles |
| `camera_depth_optical_frame → slamware_map` | `tyre_3d_projection_node.py` | Project depth pixels to 3D world coordinates |
| `slamware_map → base_link` | `aurora_semantic_fusion_node.py` | Project semantic detections to world frame |

---

## Startup TF Sequence

The correct startup order for TF availability:

1. `world_frame_tf_publisher.py` starts (provides `map→slamware_map`, `slamware_map→odom`)
2. `robot_state_publisher` starts (provides static sensor frames)
3. `slamware_ros_sdk_server_node` starts, connects to Aurora
4. Aurora SDK begins publishing `odom→base_link` (robot pose available)
5. Aurora SDK begins publishing `slamware_map→laser` (scan transform available)
6. Nav2 costmap begins accepting transforms (`transform_tolerance: 2.0` in yaml gives 2s window)

The `inspection_manager` waits for `slamware_map→base_link` to be stable for `tf_stable_s` (5s) before starting the mission.

---

## Verification Commands

```bash
# Full TF tree
ros2 run tf2_ros tf2_monitor

# Check robot localization
ros2 run tf2_ros tf2_echo slamware_map base_link

# Check depth costmap transforms
ros2 run tf2_ros tf2_echo camera_depth_optical_frame map

# Check laser scan transforms
ros2 run tf2_ros tf2_echo laser map

# Frame delay audit (look for >0.5s delays on costmap frames)
ros2 run tf2_tools view_frames
```
