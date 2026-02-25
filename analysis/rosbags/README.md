# Rosbags (2026-02-25)

No rosbag artifacts were available in the workspace for the 2026-02-25 run.

## Expected paths (if captured)
- `/home/conor/ugv_ws/rosbags/`
- `/home/conor/ugv_ws/logs/rosbags/`

## Required captures for next run
```
ros2 bag record /tf /tf_static /cmd_vel /cmd_vel_nav \
/aurora_semantic/vehicle_bounding_boxes \
/slamware_ros_sdk_server_node/semantic_labels \
/slamware_ros_sdk_server_node/depth_image_raw \
/slamware_ros_sdk_server_node/left_image_raw \
/inspection_manager/mission_state \
/inspection_manager/vehicle_detected \
/inspection_manager/capture_photo \
/inspection_manager/capture_metadata \
/inspection_manager/capture_result
```
