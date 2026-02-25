# AORA Topic Map (Expected vs Observed)

## Expected topics (from `PRODUCTION_CONFIG.yaml`)
- Pose / TF:
  - `world_frame: slamware_map`
  - `base_frame: base_link`
  - `map_frame: map`
- Detection:
  - `vehicle_boxes_topic: /aurora_semantic/vehicle_bounding_boxes`
  - `detection_topic: /darknet_ros_3d/bounding_boxes`
- Sensors:
  - `depth_image_topic: /slamware_ros_sdk_server_node/depth_image_raw`
  - `semantic_segmentation_topic: /slamware_ros_sdk_server_node/semantic_segmentation`
  - `point_cloud_topic_stereo: /slamware_ros_sdk_server_node/point_cloud`
- Motion:
  - `cmd_vel_topic: /cmd_vel`
  - `cmd_vel_nav_topic: /cmd_vel_nav`

## Observed from `journalctl -u ugv_mission` (2026-02-25)
- Semantic fusion configured:
  - `aurora_semantic_fusion: semantic=/slamware_ros_sdk_server_node/semantic_labels + depth=/slamware_ros_sdk_server_node/depth_image_raw -> /aurora_semantic/vehicle_bounding_boxes`
- Semantic/depth missing during mission window:
  - `aurora_semantic_fusion: No semantic or depth received after 5s...`
- Depth gate:
  - `depth_gate: cmd_vel_nav -> cmd_vel gated by /stereo/navigation_permitted`
- Motor driver subscribes to output cmd_vel:
  - `motor_driver_node: Subscribed to /cmd_vel`
- 3D box processor reports no boxes:
  - `segmentation_processor_node: No 3D bounding boxes computed from N segmented objects`
- TF errors:
  - `Timed out waiting for transform from base_link to odom ... unconnected trees`
  - `Could not get current pose: slamware_map and base_link ... not part of the same tree`

## Gaps
- No saved output of `ros2 node list`, `ros2 topic list`, or `ros2 topic info` from the target run.
- No rosbag to reconstruct `/cmd_vel` or `/tf`.

## Next capture (required)
Run on the active system:
```
ros2 node list
ros2 topic list
ros2 topic info /aurora_semantic/vehicle_bounding_boxes
ros2 topic info /cmd_vel
ros2 topic info /cmd_vel_nav
ros2 topic echo /tf --once
ros2 topic echo /cmd_vel --once
```
# AORA Integration Topic Map (2026-02-25)

## From logs (journald / launch.log)
### Publishers / key nodes observed
- `slamware_ros_sdk_server_node` (AORA SDK bridge)
  - Connect attempts to `tcp/192.168.11.1:7447`
  - On connect success: `Semantic segmentation supported and subscription enabled`
  - Label set: `coco80` (includes `car`, `truck`)
- `aurora_semantic_fusion_node`
  - Input topics: `/slamware_ros_sdk_server_node/semantic_labels`, `/slamware_ros_sdk_server_node/depth_image_raw`
  - Output: `/aurora_semantic/vehicle_bounding_boxes`
- `ultralytics_node`
  - RGB input: `/slamware_ros_sdk_server_node/left_image_raw`
  - Output: segmented objects (but only non-vehicle classes seen)
- Nav2 stack: `controller_server`, `planner_server`, `bt_navigator`, `behavior_server`
- `depth_gate` gating cmd_vel by `/stereo/navigation_permitted`

## Needed live captures (not executed here)
Run on the target system during the failing scenario:
- `ros2 node list`
- `ros2 topic list`
- `ros2 topic info /aurora_semantic/vehicle_bounding_boxes`
- `ros2 topic info /darknet_ros_3d/bounding_boxes`
- `ros2 topic info /cmd_vel`
- `ros2 topic echo /cmd_vel --once --noarr`
- `ros2 topic echo /tf --once --noarr`
- `ros2 param list /inspection_manager`

## Frame IDs to confirm
- `map`, `slamware_map`, `odom`, `base_link`, `camera_left`, `camera_depth_optical_frame`

