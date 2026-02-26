#!/usr/bin/env bash
# Validate vehicle detection pipeline with Aurora running.
# Prereq: Aurora bringup running (ros2 launch ugv_nav aurora_bringup.launch.py).
# Run in another terminal: ros2 launch segmentation_3d segment_3d.launch.py
# Then run this script: ./scripts/validate_vehicle_detection.sh

set -e
cd "$(dirname "$0")/.."
source install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "=== Vehicle detection validation (Aurora + segment_3d) ==="
echo ""

# Get topic list once to avoid BrokenPipeError when piping ros2 to grep
topic_list=$(ros2 topic list 2>/dev/null) || true

echo "--- Required topics (should have at least one publisher when stack is up) ---"
for topic in \
  /slamware_ros_sdk_server_node/depth_image_raw \
  /slamware_ros_sdk_server_node/left_image_raw \
  /aurora_semantic/vehicle_bounding_boxes \
  /darknet_ros_3d/bounding_boxes \
  /camera/depth/camera_info \
  /segmentation_processor/registered_pointcloud \
  /ultralytics/segmentation/objects_segment \
  /slamware_ros_sdk_server_node/semantic_labels \
  /tf \
  /slamware_ros_sdk_server_node/robot_pose \
  /slamware_ros_sdk_server_node/map; do
  if echo "$topic_list" | grep -q "^${topic}$"; then
    echo "  OK  $topic"
  else
    echo "  --  $topic (not found)"
  fi
done
echo ""

echo "--- Topic rates (5s sample; run with segment_3d + Aurora) ---"
for topic in /slamware_ros_sdk_server_node/depth_image_raw /slamware_ros_sdk_server_node/left_image_raw; do
  if echo "$topic_list" | grep -q "^${topic}$"; then
    echo -n "  $topic: "
    timeout 5 ros2 topic hz "$topic" 2>&1 | tail -1 || echo "no data"
  fi
done
if echo "$topic_list" | grep -q "^/aurora_semantic/vehicle_bounding_boxes$"; then
  echo -n "  /aurora_semantic/vehicle_bounding_boxes: "
  timeout 5 ros2 topic hz /aurora_semantic/vehicle_bounding_boxes 2>&1 | tail -1 || echo "no data"
fi
if echo "$topic_list" | grep -q "^/darknet_ros_3d/bounding_boxes$"; then
  echo -n "  /darknet_ros_3d/bounding_boxes: "
  timeout 5 ros2 topic hz /darknet_ros_3d/bounding_boxes 2>&1 | tail -1 || echo "no data"
fi
echo ""

echo "--- One message from vehicle boxes (frame_id should be slamware_map) ---"
if echo "$topic_list" | grep -q "^/aurora_semantic/vehicle_bounding_boxes$"; then
  timeout 8 ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once 2>/dev/null || echo "  (no message within 8s)"
else
  echo "  Topic not found; start segment_3d first."
fi
echo ""

echo "=== Done. For RViz markers run: ros2 launch segmentation_3d vehicle_detection_visualize.launch.py fallback_vehicle_boxes_topic:=/darknet_ros_3d/bounding_boxes ==="
