#!/usr/bin/env bash
# Aurora-only TF and topic diagnostic.
# Run AFTER: ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
# (aurora_bringup sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp so launch and this script see the same topics/TF.)
#
# In this terminal: source install/setup.bash (RMW defaults to rmw_cyclonedds_cpp here to match launch).
# This script: waits for TF slamware_map->base_link, prints TF tree, lists nodes/topics,
# and echoes the transform a few times so you can confirm the robot pose is updating.
# Exit: 0 = TF and key topics OK, 1 = TF or required topic missing.

set -e
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
UGV_WS="${UGV_WS:-$HOME/ugv_ws}"
if [ -f "$UGV_WS/install/setup.bash" ]; then
  source "$UGV_WS/install/setup.bash"
fi

echo "=== Aurora TF & topic diagnostic ==="
echo "RMW: $RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo ""
echo "  (Run aurora_bringup in another terminal and leave it running; use same RMW there.)"
echo ""

# 0) Can we see any nodes? If not, RMW/domain mismatch with launch.
NODES=$(ros2 node list 2>/dev/null || true)
if ! echo "$NODES" | grep -q "slamware_ros_sdk_server_node"; then
  echo "0) WARNING: slamware_ros_sdk_server_node not visible in 'ros2 node list'."
  if [ -z "$NODES" ] || [ "$NODES" = "" ]; then
    echo "    No ROS2 nodes visible. In the launch terminal set: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo "    and export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}, then restart aurora_bringup."
  else
    echo "    Visible nodes: $NODES"
  fi
  echo ""
fi

# 1) Wait for slamware_map -> base_link (static slamware_map->odom on /tf_static; odom->base_link on /tf from SDK)
# tf2_echo needs a few seconds to receive /tf_static (TRANSIENT_LOCAL) before "slamware_map" exists; use 5s per attempt.
echo "1) Waiting for TF slamware_map -> base_link (up to 6 attempts x 5s = 30s) ..."
TF_OK=0
for i in $(seq 1 6); do
  if timeout 5 ros2 run tf2_ros tf2_echo slamware_map base_link 2>/dev/null | grep -m1 "At time" | head -1 | grep -q "At time"; then
    echo "    TF OK on attempt $i"
    TF_OK=1
    break
  fi
  echo "  attempt $i: no transform yet (tf_static may need a moment)."
done
echo ""
if [ $TF_OK -eq 0 ]; then
  echo "    FAIL: No transform slamware_map -> base_link after 30s."
  echo "    Ensure Aurora is on and slamware_ros_sdk_server_node is running and connected."
  echo "    If nodes are not visible above, launch and this script use different RMW or ROS_DOMAIN_ID."
  exit 1
fi

# 2) TF echo (first sample)
echo "2) TF slamware_map -> base_link (first sample):"
timeout 2 ros2 run tf2_ros tf2_echo slamware_map base_link 2>/dev/null | head -6 || true
echo ""

# 3) Nodes
echo "3) Nodes (expect slamware_ros_sdk_server_node, static_transform_publishers, navigation_permitted_publisher):"
ros2 node list 2>/dev/null || true
echo ""

# 4) Key Aurora topics (publishers)
echo "4) Key Aurora topics (must be present for mission):"
for topic in \
  /slamware_ros_sdk_server_node/odom \
  /slamware_ros_sdk_server_node/robot_pose \
  /slamware_ros_sdk_server_node/scan \
  /slamware_ros_sdk_server_node/left_image_raw \
  /slamware_ros_sdk_server_node/depth_image_raw \
  /slamware_ros_sdk_server_node/point_cloud \
  /slamware_ros_sdk_server_node/semantic_segmentation; do
  if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
    echo "  OK   $topic"
  else
    echo "  MISS $topic"
  fi
done
echo ""

# 5) Who publishes odom / who uses it
echo "5) Topic info /slamware_ros_sdk_server_node/odom (frame_id=odom, child_frame_id=base_link):"
ros2 topic info /slamware_ros_sdk_server_node/odom 2>/dev/null || true
echo ""

# 6) Echo transform 3 times (1s apart) to see if pose is updating (optional)
echo "6) TF slamware_map -> base_link (3 samples, 1s apart):"
for i in 1 2 3; do
  echo "  Sample $i:"
  timeout 2 ros2 run tf2_ros tf2_echo slamware_map base_link 2>/dev/null | head -6 || true
  sleep 1
done
echo ""
echo "=== Diagnostic complete ==="
echo "If TF and topics are OK, full launch (full_bringup) can proceed. Mission will not start until TF is stable 3–5s in IDLE."
