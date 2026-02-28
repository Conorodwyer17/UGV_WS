# Aurora Connectivity and Capture Metadata

- capture_date_utc: `2026-02-28`
- aurora_ip: `192.168.11.1`
- connectivity: `connected` via `slamware_ros_sdk_server_node`
- semantic_support: `enabled` (`coco80` labels reported by SDK node)

## Topic Discovery Snapshot

- topic list file: `research/data/aurora_samples/aurora_topics_20260228_004903.txt`
- required topics observed:
  - `/slamware_ros_sdk_server_node/point_cloud`
  - `/slamware_ros_sdk_server_node/left_image_raw`
  - `/slamware_ros_sdk_server_node/right_image_raw`
  - `/slamware_ros_sdk_server_node/depth_image_raw`
  - `/slamware_ros_sdk_server_node/imu_raw_data`
  - `/slamware_ros_sdk_server_node/odom`
  - `/tf`

## Rosbag Capture

- first attempt:
  - command: sqlite3 default recorder on selected topics
  - result: failed with `rosbag2_storage_plugins::SqliteException` (`constraint failed`)
- workaround:
  - command: sqlite3 recorder with `--no-discovery` on explicit topic list
  - bag dir: `research/data/aurora_samples/aurora_sample_20260228_004935`
  - bag info: `research/data/aurora_samples/aurora_sample_20260228_004935_info.txt`
  - manifest + hashes: `research/data/aurora_samples/aurora_sample_20260228_004935_manifest.json`
- production workaround capture set (successful):
  - recorded as per-topic bags to avoid sqlite multi-topic constraint crash
  - capture set manifest: `research/data/aurora_samples/aurora_capture_set_20260228_010125_manifest.json`
  - included modalities:
    - `aurora_pointcloud_20260228_010125`
    - `aurora_left_20260228_010125`
    - `aurora_right_20260228_010125`
    - `aurora_imu_20260228_010125`
    - `aurora_odom_20260228_010125`

## Notes

- MCAP storage plugin is not available in this ROS install (`ros2 bag record --storage mcap` unsupported).
- Captured bag should be used as reproducible offline input for detector/tracker/object-manager bringup.
