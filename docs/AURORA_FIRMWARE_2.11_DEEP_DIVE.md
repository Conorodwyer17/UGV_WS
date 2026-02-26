# Aurora Firmware 2.11

**Purpose:** Exhaustive reference of what the Aurora publishes, what firmware 2.11 (or 2.1.1) changes, and how to audit your device after upgrade.

---

## 1. Firmware & SDK Versions

| Item | Version | Source |
|------|---------|--------|
| **Aurora firmware** | 2.1.1-rtm (latest) | SLAMTEC Support page |
| **Aurora SDK** | 2.1.0-rtm / 2.1.1-rtm | developer.slamtec.com |
| **ROS2 SDK** | 1.5 | SLAMTEC download |
| **Your upgrade** | 1.2 → 2.11 | User report |

Firmware 2.11 is likely **2.1.1** (Slamtec versions often use `Major.Minor.Patch`; 2.11 may be shown as 2.1.1 in some UIs). The support page lists "Aurora Firmware Latest Version: 2.1.1-rtm".

---

## 2. What the SDK Node Publishes (Full Inventory)

The `slamware_ros_sdk_server_node` uses these **workers** (from `slamware_ros_sdk_server.cpp`):

| Worker | Topic(s) | Condition | Description |
|--------|----------|-----------|-------------|
| ServerOdometryWorker | `/slamware_ros_sdk_server_node/odom` | Always | Wheel/odom pose |
| ServerRobotPoseWorker | `/slamware_ros_sdk_server_node/robot_pose` | Always | Robot pose in map |
| ServerExploreMapUpdateWorker | (internal) | map_update_period > 0 | Map update |
| ServerExploreMapPublishWorker | `/slamware_ros_sdk_server_node/map` | map_pub_period > 0 | Occupancy grid |
| ServerLaserScanWorker | `/slamware_ros_sdk_server_node/scan` | scan_pub_period > 0 | LiDAR scan |
| ServerImuRawDataWorker | `/slamware_ros_sdk_server_node/imu_raw_data` | Always | IMU data |
| RosConnectWorker | `/slamware_ros_sdk_server_node/state` | Always | Connection state |
| ServerSystemStatusWorker | `/slamware_ros_sdk_server_node/system_status` | Always | System status |
| ServerStereoImageWorker | `/slamware_ros_sdk_server_node/left_image_raw`, `/slamware_ros_sdk_server_node/right_image_raw`, `/slamware_ros_sdk_server_node/stereo_keypoints` | raw_image_on=true | Stereo images |
| **ServerStereoCameraInfoWorker** | `/camera/left/camera_info`, `/camera/right/camera_info` | **stereo_camera_info_enable=true** | Factory camera calibration |
| **ServerEnhancedImagingWorker** | `/slamware_ros_sdk_server_node/depth_image_raw`, `/slamware_ros_sdk_server_node/depth_image_colorized`, `/slamware_ros_sdk_server_node/semantic_segmentation` | **Device capability** | Depth & semantic |
| ServerPointCloudWorker | `/slamware_ros_sdk_server_node/point_cloud` | Always | SLAM map points (slamware_map frame) |

---

## 3. Device Capabilities (Firmware-Controlled)

The SDK checks **device basic info** (`slamtec_aurora_sdk_device_basic_info_t`) to decide what to publish:

| Capability | SDK check | Bit flag | Topics enabled when true |
|------------|-----------|----------|--------------------------|
| **Depth camera** | `isDepthCameraSupported()` | `SENSING_FEATURE_BIT_STEREO_DENSE_DISPARITY` | `depth_image_raw`, `depth_image_colorized` |
| **Semantic segmentation** | `isSemanticSegmentationReady()` | `SENSING_FEATURE_BIT_SEMANTIC_SEGMENTATION` | `semantic_segmentation` |
| **Camera calibration** | `getCameraCalibration()` | N/A (API call) | `/camera/left/camera_info`, `/camera/right/camera_info` |

These flags are reported by the **device firmware**. Firmware 2.11 may enable depth and/or semantic on devices that previously did not support them.

---

## 4. Full Topic List (Possible Outputs)

| Topic | Type | Frame | When published |
|-------|------|-------|----------------|
| `/slamware_ros_sdk_server_node/scan` | LaserScan | laser | Always |
| `/slamware_ros_sdk_server_node/odom` | Odometry | odom | Always |
| `/slamware_ros_sdk_server_node/robot_pose` | PoseStamped | slamware_map | Always |
| `/slamware_ros_sdk_server_node/map` | OccupancyGrid | map | map_pub_period > 0 |
| `/slamware_ros_sdk_server_node/map_metadata` | MapMetaData | — | With map |
| `/slamware_ros_sdk_server_node/imu_raw_data` | Imu | imu_link | Always |
| `/slamware_ros_sdk_server_node/state` | String | — | Always |
| `/slamware_ros_sdk_server_node/system_status` | slamware_ros_sdk/SystemStatus | — | Always |
| `/slamware_ros_sdk_server_node/left_image_raw` | Image | — | raw_image_on=true |
| `/slamware_ros_sdk_server_node/right_image_raw` | Image | — | raw_image_on=true |
| `/slamware_ros_sdk_server_node/stereo_keypoints` | Image | — | raw_image_on=true |
| `/slamware_ros_sdk_server_node/depth_image_raw` | Image | camera_depth_optical_frame | **isDepthCameraSupported()** |
| `/slamware_ros_sdk_server_node/depth_image_colorized` | Image | camera_depth_optical_frame | **isDepthCameraSupported()** |
| `/slamware_ros_sdk_server_node/semantic_segmentation` | Image | camera_depth_optical_frame | **isSemanticSegmentationReady()** |
| `/slamware_ros_sdk_server_node/point_cloud` | PointCloud2 | slamware_map | Always |
| `/camera/left/camera_info` | CameraInfo | camera_left | **stereo_camera_info_enable + getCameraCalibration ok** |
| `/camera/right/camera_info` | CameraInfo | camera_right | Same |

---

## 5. Services

| Service | Description |
|---------|-------------|
| `/slamware_ros_sdk_server_node/sync_map` | Sync map |
| `/slamware_ros_sdk_server_node/clear_map` | Clear map |
| `/slamware_ros_sdk_server_node/set_map_update` | Set map update mode |
| `/slamware_ros_sdk_server_node/set_map_localization` | Set localization |
| `/slamware_ros_sdk_server_node/relocalization/cancel` | Cancel relocalization |
| `/slamware_ros_sdk_server_node/relocalization` | Relocalization request |
| `/slamware_ros_sdk_server_node/sync_get_stcm` | Sync get |
| `/slamware_ros_sdk_server_node/sync_set_stcm` | Sync set |

---

## 6. What Firmware 2.11 May Change

Firmware 2.11 (2.1.1) is a major jump from 1.2. Likely changes:

1. **Depth camera** — Device may now report `SENSING_FEATURE_BIT_STEREO_DENSE_DISPARITY` → `depth_image_raw` published.
2. **Semantic segmentation** — May report `SENSING_FEATURE_BIT_SEMANTIC_SEGMENTATION` → `semantic_segmentation` published.
3. **Camera calibration** — `getCameraCalibration()` may succeed (previously -7 on Classic Aurora) → `/camera/left/camera_info`, `/camera/right/camera_info`.
4. **SLAM / mapping** — Algorithm or behavior may change.
5. **VSLAM features** — `sensing_feature_bitmaps` may include more bits (loop closure, localization, etc.).

---

## 7. Running the Post-Firmware Audit

```bash
# Terminal 1: Start Aurora bringup with raw images (needed for stereo/vision)
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1 raw_image_on:=true use_bridge:=false

# Terminal 2: Run audit (after ~30s for connection)
source /home/conor/ugv_ws/install/setup.bash
bash scripts/aurora_post_firmware_audit.sh
```

Optional: enable stereo camera info to test `getCameraCalibration`:

```bash
# In aurora_bringup.launch.py params, add:
'stereo_camera_info_enable': True,
```

Report is written to `logs/aurora_integration/aurora_firmware_audit_YYYYMMDD_HHMMSS.txt`.

---

## 8. SDK Node Startup Logs (What to Look For)

When the slamware_ros_sdk_server_node starts, it logs:

- `"Depth camera supported"` → `/slamware_ros_sdk_server_node/depth_image_raw` will publish
- `"Depth camera not supported"` → `/slamware_ros_sdk_server_node/depth_image_raw` will NOT publish
- `"Semantic segmentation supported"` → `/slamware_ros_sdk_server_node/semantic_segmentation` will publish
- `"Semantic segmentation not supported"` → `/slamware_ros_sdk_server_node/semantic_segmentation` will NOT publish
- `"Label ID: X, Name: ..."` → Semantic labels (if semantic supported)
- `"Failed to get camera calibration: error -7"` → stereo_camera_info not available

After firmware 2.11, the first two may have flipped from "not supported" to "supported".

---

## 9. Point Cloud: Map vs Depth

| Topic | Frame | Source | Use |
|-------|-------|--------|-----|
| `/slamware_ros_sdk_server_node/point_cloud` | slamware_map | SLAM map points (sparse) | Mapping, visualization |
| `/camera/depth/points` | camera_depth_optical_frame | depth_image_raw → unproject | Pixel↔3D, segmentation |

The slamware `point_cloud` is **not** pixel-aligned with the camera. For tire detection you need camera-frame dense depth (from `depth_image_raw` or bridge).

---

## 10. Online Resources

- [SLAMTEC Support](https://www.slamtec.com/en/Support) — Firmware & SDK downloads
- [Aurora SDK 2.1.0](https://developer.slamtec.com/docs/slamware/aurora-sdk/2.1.0-rtm/) — API reference
- [Aurora ROS2 SDK](https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/) — ROS2 integration
- [GitHub aurora_remote_sdk_demo](https://github.com/Slamtec/aurora_remote_sdk_demo) — Demo apps

---

## 11. Next Steps After Audit

1. **If `/slamware_ros_sdk_server_node/depth_image_raw` is published** — Use Aurora native depth (`use_bridge:=false`). Configure `depth_to_registered_pointcloud` with `depth_topic:=/slamware_ros_sdk_server_node/depth_image_raw`.
2. **If `/slamware_ros_sdk_server_node/semantic_segmentation` is published** — Check label IDs/names; consider using Aurora semantic instead of or alongside YOLO for car/tire detection.
3. **If camera_info is published** — Enable `stereo_camera_info_enable` in launch; no need for aurora_camera_info_node placeholder.
4. **If depth still not published** — Stay on bridge path with calibration; firmware did not enable depth for your hardware variant.
