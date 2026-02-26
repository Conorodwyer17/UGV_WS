#!/bin/bash
# Run only vehicle detection (Aurora semantic fusion) + bounding box visualization.
# Prerequisite: Aurora must be running (slamware_ros_sdk_server_node publishing
#   /slamware_ros_sdk_server_node/semantic_labels and .../depth_image_raw). If you use full_bringup, stop it and
#   run only aurora_bringup, then this script.
#
# Usage:
#   ./scripts/run_vehicle_detection_visualize.sh
#
# Then in RViz: Add -> By topic -> /aurora_semantic/vehicle_markers (MarkerArray).
# Fixed frame: slamware_map.
#
# To also see detection in the terminal:
#   ros2 topic echo /aurora_semantic/vehicle_bounding_boxes

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
source "$UGV_WS/install/setup.bash" 2>/dev/null || true

if ! ros2 pkg prefix segmentation_3d &>/dev/null; then
  echo "Run from workspace: source install/setup.bash (and build: colcon build --packages-select segmentation_3d --symlink-install)"
  exit 1
fi

echo "=== Vehicle detection + marker visualization ==="
echo "Ensure Aurora is running (/slamware_ros_sdk_server_node/semantic_labels + .../depth_image_raw)."
echo "Launching aurora_semantic_fusion + vehicle_boxes_marker..."
echo ""

exec ros2 launch segmentation_3d vehicle_detection_visualize.launch.py
