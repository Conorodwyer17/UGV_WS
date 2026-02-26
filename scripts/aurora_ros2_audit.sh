#!/usr/bin/env bash
# Full ROS2 audit while Aurora (aurora_bringup) is running.
# Run in a SECOND terminal; leave aurora_bringup running in the first.
#
# Use SAME RMW in BOTH terminals, or this terminal will see no nodes/topics/TF:
#   Terminal 1 (launch): export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 launch ugv_nav aurora_bringup.launch.py ...
#   Terminal 2 (this):  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && bash this script
#
# This script lists every node, every topic (with pub/sub counts), /tf and /tf_static,
# and tries TF lookup slamware_map -> base_link.

set -e
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
UGV_WS="${UGV_WS:-$HOME/ugv_ws}"
if [ -f "$UGV_WS/install/setup.bash" ]; then
  source "$UGV_WS/install/setup.bash"
fi

echo "=============================================="
echo "Aurora ROS2 audit (Aurora must be running)"
echo "=============================================="
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo ""

echo "--- 1) Nodes ---"
ros2 node list 2>/dev/null || true
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
echo "(total: $NODE_COUNT nodes)"
echo ""

if [ "$NODE_COUNT" -eq 0 ]; then
  echo ">>> No nodes visible. This terminal cannot see the launch."
  echo "    In the terminal where you run aurora_bringup, run FIRST:"
  echo "      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  echo "    Then start the launch. Keep same RMW and ROS_DOMAIN_ID in both terminals."
  echo ""
fi

echo "--- 2) All topics (publishers / subscribers) ---"
ros2 topic list -t 2>/dev/null || true
echo ""

echo "--- 3) /tf and /tf_static ---"
for t in /tf /tf_static; do
  if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
    echo "Topic: $t"
    ros2 topic info "$t" -v 2>/dev/null || true
    echo ""
  else
    echo "Topic $t: not present"
    echo ""
  fi
done

echo "--- 4) Who publishes /tf? ---"
ros2 topic info /tf -v 2>/dev/null | grep -E "Publisher|Node" || true
echo ""

echo "--- 5) TF lookup slamware_map -> base_link (5s timeout) ---"
if timeout 5 ros2 run tf2_ros tf2_echo slamware_map base_link 2>&1 | head -20; then
  true
else
  echo "(tf2_echo timed out or failed)"
fi
echo ""

echo "--- 6) One message from /tf (if any) ---"
timeout 2 ros2 topic echo /tf --once 2>/dev/null | head -30 || echo "(no message in 2s)"
echo ""

echo "--- 7) Key Aurora topics: do they exist? ---"
for topic in \
  /slamware_ros_sdk_server_node/odom \
  /slamware_ros_sdk_server_node/robot_pose \
  /slamware_ros_sdk_server_node/scan \
  /slamware_ros_sdk_server_node/left_image_raw \
  /slamware_ros_sdk_server_node/map; do
  if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
    echo "  OK   $topic"
  else
    echo "  MISS $topic"
  fi
done
echo ""
echo "=============================================="
echo "If node count is 0 or TF lookup failed: set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp in the LAUNCH terminal and restart aurora_bringup."
echo "=============================================="
