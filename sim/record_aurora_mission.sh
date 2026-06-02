#!/usr/bin/env bash
# Record Aurora topics for offline simulation replay.
# Run with full stack (Aurora + segment_3d + optional inspection) before starting mission.
# Output: sim/bags/aurora_mission_YYYYMMDD_HHMMSS/

set -e
BAGS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/bags" && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="${BAGS_DIR}/aurora_mission_${TIMESTAMP}"

# Aurora SDK topics (slamware_ros_sdk_server_node)
AURORA_TOPICS=(
  /slamware_ros_sdk_server_node/odom
  /slamware_ros_sdk_server_node/scan
  /slamware_ros_sdk_server_node/map
  /slamware_ros_sdk_server_node/depth_image_raw
  /slamware_ros_sdk_server_node/left_image_raw
  /slamware_ros_sdk_server_node/point_cloud
  /slamware_ros_sdk_server_node/semantic_labels
)

# TF (required for Nav2 and segment_3d)
TF_TOPICS=(/tf /tf_static)

# Segmentation outputs (optional; if segment_3d is running during record)
# Include these to replay without running YOLO/segment_3d
SEGMENT_TOPICS=(
  /ultralytics_tire/segmentation/objects_segment
  /segmentation_processor/registered_pointcloud
  /darknet_ros_3d/tire_bounding_boxes
  /darknet_ros_3d/vehicle_bounding_boxes
  /aurora_semantic/vehicle_bounding_boxes
)

# Depth pipeline (if depth_to_registered_pointcloud is running)
DEPTH_TOPICS=(
  /camera/depth/points
)

ALL_TOPICS=("${AURORA_TOPICS[@]}" "${TF_TOPICS[@]}" "${SEGMENT_TOPICS[@]}" "${DEPTH_TOPICS[@]}")

echo "Recording to ${OUTPUT_DIR}"
echo "Topics: ${ALL_TOPICS[*]}"
echo ""
echo "Ensure Aurora and (optionally) segment_3d are running before starting."
echo "Press Ctrl+C to stop recording."
echo ""

mkdir -p "${BAGS_DIR}"
ros2 bag record -o "${OUTPUT_DIR}" "${ALL_TOPICS[@]}"

echo ""
echo "Recording stopped. Bag saved to: ${OUTPUT_DIR}"
echo "Replay with:"
echo "  ros2 launch sim vehicle_inspection_sim.launch.py use_bag:=true bag_path:=${OUTPUT_DIR}"
