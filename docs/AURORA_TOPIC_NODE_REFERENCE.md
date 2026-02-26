# Aurora Topic & Node Reference

**Node name:** `slamware_ros_sdk_server_node` (package: `slamware_ros_sdk`).  
**Launch first:** `ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1` (or `aurora_testing.launch.py` for tests).  
**Then** run other nodes in separate terminals; use **exactly** these topic names when subscribing.

---

## Publishers (Aurora — slamware_ros_sdk_server_node)

All topics use the **`/slamware_ros_sdk_server_node/`** prefix. Do not subscribe to legacy or shortened names (e.g. `/odom`, `/scan`); use the full path below.

| Topic | Type | QoS |
|-------|------|-----|
| `/slamware_ros_sdk_server_node/left_image_raw` | sensor_msgs/Image | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/right_image_raw` | sensor_msgs/Image | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/scan` | sensor_msgs/LaserScan | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/odom` | nav_msgs/Odometry | RELIABLE |
| `/slamware_ros_sdk_server_node/map` | nav_msgs/OccupancyGrid | RELIABLE, TRANSIENT_LOCAL |
| `/slamware_ros_sdk_server_node/map_metadata` | nav_msgs/MapMetaData | |
| `/slamware_ros_sdk_server_node/point_cloud` | sensor_msgs/PointCloud2 | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/robot_pose` | geometry_msgs/PoseStamped | |
| `/slamware_ros_sdk_server_node/depth_image_raw` | sensor_msgs/Image (32FC1) | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/depth_image_colorized` | sensor_msgs/Image (BGR8) | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/semantic_segmentation` | sensor_msgs/Image | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/semantic_labels` | sensor_msgs/Image | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/imu_raw_data` | sensor_msgs/Imu | |
| `/slamware_ros_sdk_server_node/state` | std_msgs/String | |
| `/slamware_ros_sdk_server_node/system_status` | slamware_ros_sdk/SystemStatus | |
| `/slamware_ros_sdk_server_node/relocalization_status` | slamware_ros_sdk/RelocalizationStatus | |
| `/slamware_ros_sdk_server_node/stereo_keypoints` | sensor_msgs/Image | |

### aurora_sdk_bridge

| Topic | Type | QoS |
|-------|------|-----|
| `/camera/depth/image` | sensor_msgs/Image | BEST_EFFORT |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | RELIABLE |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | BEST_EFFORT |
| `/stereo/navigation_permitted` | std_msgs/Bool | |

---

## Subscribers (Aurora stack)

### aurora_sdk_bridge

| Subscribes to | Notes |
|---------------|-------|
| `/slamware_ros_sdk_server_node/left_image_raw` | Param: left_image_topic |
| `/slamware_ros_sdk_server_node/right_image_raw` | Param: right_image_topic |

---

## Downstream subscribers (our nodes)

| Node | Subscribes to |
|------|---------------|
| **ultralytics_node** | `/slamware_ros_sdk_server_node/left_image_raw` (param: camera_rgb_topic) |
| **depth_to_registered_pointcloud_node** | `/slamware_ros_sdk_server_node/depth_image_raw`, `/camera/depth/camera_info` (Aurora native); or `/camera/depth/image`, `/camera/depth/camera_info` (bridge) |
| **segmentation_processor_node** | ObjectsSegment, `/segmentation_processor/registered_pointcloud` |
| **Nav2** (nav_aurora.yaml) | `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/odom`, `/camera/depth/points`, `/map` (remapped from `/slamware_ros_sdk_server_node/map`) |
| **inspection_manager** | `/darknet_ros_3d/bounding_boxes`, `/aurora_semantic/vehicle_bounding_boxes` (PRODUCTION_CONFIG) |
| **photo_capture_service** | `/slamware_ros_sdk_server_node/left_image_raw` (param: camera_topic), `/inspection_manager/capture_photo` |
| **aurora_health_monitor** | `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/scan`, `/map`, `/slamware_ros_sdk_server_node/left_image_raw`, `/slamware_ros_sdk_server_node/right_image_raw` (params) |
| **aurora_semantic_fusion** | `/slamware_ros_sdk_server_node/semantic_labels`, `/slamware_ros_sdk_server_node/depth_image_raw` |

---

## Pre-flight check

Before running test scripts, verify Aurora is publishing:

```bash
ros2 topic list | grep -E "left_image|scan|map|depth"
```

Expected (Aurora native, `slamware_ros_sdk_server_node` running):
- `/slamware_ros_sdk_server_node/left_image_raw`
- `/slamware_ros_sdk_server_node/right_image_raw`
- `/slamware_ros_sdk_server_node/scan`
- `/slamware_ros_sdk_server_node/odom`
- `/slamware_ros_sdk_server_node/map`
- `/slamware_ros_sdk_server_node/robot_pose`
- `/slamware_ros_sdk_server_node/point_cloud`
- `/slamware_ros_sdk_server_node/depth_image_raw` (if device supports)
- `/slamware_ros_sdk_server_node/semantic_segmentation` (if device supports)

After segment_3d: `/camera/depth/camera_info`, `/camera/depth/points`, `/segmentation_processor/registered_pointcloud`.  
If you only see `/parameter_events` and `/rosout`, the Aurora stack is **not** running. Launch it first.
