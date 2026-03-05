#!/usr/bin/env bash
# Record mock simulation topics for validation and comparison to real bags.
# Run with: ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true
# Then run this script in another terminal. Let the mission run (or drive via teleop), Ctrl+C to stop.
# Output: sim/bags/mock_mission_YYYYMMDD_HHMMSS/

set -e
BAGS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/bags" && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="${BAGS_DIR}/mock_mission_${TIMESTAMP}"

# Aurora mock topics (same structure as real Aurora for comparison)
AURORA_TOPICS=(
  /slamware_ros_sdk_server_node/odom
  /slamware_ros_sdk_server_node/scan
  /slamware_ros_sdk_server_node/map
  /slamware_ros_sdk_server_node/depth_image_raw
  /slamware_ros_sdk_server_node/left_image_raw
  /slamware_ros_sdk_server_node/point_cloud
  /slamware_ros_sdk_server_node/semantic_labels
)

# TF
TF_TOPICS=(/tf /tf_static)

# Detection and mission (for validation)
DETECTION_TOPICS=(
  /aurora_semantic/vehicle_bounding_boxes
  /ultralytics_tire/segmentation/objects_segment
  /segmentation_processor/registered_pointcloud
  /darknet_ros_3d/tire_bounding_boxes
  /tire_bounding_boxes_merged
  /inspection_manager/mission_report
  /inspection_manager/state
)

# Clock (use_sim_time)
CLOCK_TOPICS=(/clock)

ALL_TOPICS=("${AURORA_TOPICS[@]}" "${TF_TOPICS[@]}" "${DETECTION_TOPICS[@]}" "${CLOCK_TOPICS[@]}")

echo "Recording mock mission to ${OUTPUT_DIR}"
echo "Topics: ${ALL_TOPICS[*]}"
echo ""
echo "Ensure sim is running: ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true"
echo "Let the mission run or drive via teleop. Press Ctrl+C to stop."
echo ""

mkdir -p "${BAGS_DIR}"
ros2 bag record -o "${OUTPUT_DIR}" "${ALL_TOPICS[@]}"

echo ""
echo "Recording stopped. Bag saved to: ${OUTPUT_DIR}"
echo "Compare to real: python3 scripts/compare_sim_to_real.py --sim-report ~/ugv_ws/logs/mission_report_latest.json --real-report /path/to/real_report.json"
