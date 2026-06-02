#!/bin/bash
# Diagnostic: verify point cloud and costmap inputs for Nav2 obstacle layer.
# Run with: ./scripts/check_costmap_inputs.sh (after start_no_move.sh is running)

set -e
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

echo "=== Costmap input diagnostics ==="
echo ""

echo "1. Point cloud topic /segmentation_processor/registered_pointcloud:"
if grep -q "/segmentation_processor/registered_pointcloud" < <(ros2 topic list 2>/dev/null); then
  echo "   Topic exists."
  HZ=$(timeout 5 ros2 topic hz /segmentation_processor/registered_pointcloud 2>/dev/null | grep "average rate" || echo "   (no messages in 5s)")
  echo "   $HZ"
else
  echo "   NOT FOUND - depth_to_registered_pointcloud may not be running."
fi
echo ""

echo "2. Point cloud frame_id and sample (one message):"
MSG=$(timeout 3 ros2 topic echo /segmentation_processor/registered_pointcloud --once 2>/dev/null || true)
if [ -n "$MSG" ]; then
  echo "$MSG" | head -20
  echo "   ..."
  # Count valid points (rough: look for non-zero data)
  if echo "$MSG" | grep -q "data:"; then
    echo "   (Message received)"
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    PC_VALID=$(timeout 6 python3 "$SCRIPT_DIR/check_pointcloud_valid.py" 2>/dev/null || true)
    if [ -n "$PC_VALID" ]; then
      echo "   $PC_VALID"
    fi
  fi
else
  echo "   No message received in 3s."
fi
echo ""

echo "3. TF chain odom -> camera_depth_optical_frame:"
TF_OUT=$(timeout 3 ros2 run tf2_ros tf2_echo odom camera_depth_optical_frame 2>&1 | head -12 || true)
if echo "$TF_OUT" | grep -q "Translation:"; then
  echo "   TF OK."
  echo "$TF_OUT" | head -6 | sed 's/^/   /'
else
  echo "   TF lookup failed - check world_frame_tf_publisher and slamware."
  [ -n "$TF_OUT" ] && echo "$TF_OUT" | head -5 | sed 's/^/   /'
fi
echo ""

echo "4. Costmap (check for non-zero cells):"
COSTMAP_TOPIC=$(grep -E "local_costmap/costmap|controller_server.*costmap" < <(ros2 topic list 2>/dev/null) | head -1 || true)
if [ -z "$COSTMAP_TOPIC" ]; then
  echo "   Costmap topic not found (Nav2 controller_server may not be active yet)."
else
  COSTMAP=$(timeout 4 ros2 topic echo "$COSTMAP_TOPIC" --once 2>/dev/null || true)
  if [ -n "$COSTMAP" ]; then
    NONZERO=$(echo "$COSTMAP" | grep -E "^- [1-9]" | wc -l)
    echo "   Non-zero cells in sample: $NONZERO"
    if [ "$NONZERO" -gt 0 ]; then
      echo "   Costmap has obstacles."
    else
      echo "   Costmap all zeros - point cloud may not be reaching obstacle layer."
      echo "   Ensure: (a) depth camera sees obstacles, (b) robot is indoors with walls/furniture."
    fi
  else
    echo "   No costmap message (topic exists but no data in 4s)."
  fi
fi
echo ""

echo "5. Controller server logs (TF errors):"
echo "   Run: ros2 topic echo /rosout | grep -i 'TF Exception\\|transform\\|obstacle'"
echo ""
echo "6. For obstacle layer debug logs, relaunch with: nav2_log_level:=debug"
echo ""

echo "=== Done ==="
