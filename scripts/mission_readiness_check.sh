#!/bin/bash
# Mission readiness: TF and Nav2 action server. Run after full bringup.
# Usage: source install/setup.bash && bash scripts/mission_readiness_check.sh

FAIL=0
HAS_PERMIT=0
echo "=== Mission readiness (TF + Nav2) ==="
echo "RMW: ${RMW_IMPLEMENTATION:-<not set>}"
echo ""

# TF: slamware_map or map -> base_link (tf2_echo has no -n)
for frame in slamware_map map; do
  echo -n "TF ${frame} -> base_link ... "
  out=$(timeout 5 ros2 run tf2_ros tf2_echo "$frame" base_link 2>&1) || true
  if echo "$out" | grep -m1 "At time" | grep -q "At time"; then
    echo "OK"
    break
  fi
  if [ "$frame" = "map" ]; then
    echo "FAIL"
    FAIL=1
  fi
done

# Nav2: need Action servers: 1+
echo -n "Nav2 /navigate_to_pose ... "
info=$(ros2 action info /navigate_to_pose 2>&1) || true
if echo "$info" | grep -qE "Action servers: [1-9]"; then
  echo "OK (server up)"
else
  echo "FAIL (Action servers: 0 or not found)"
  FAIL=1
fi

# Detection topics (vehicle + tire)
echo "Detection topics ..."
topic_list=$(ros2 topic list 2>/dev/null) || true
missing=0
for topic in /aurora_semantic/vehicle_bounding_boxes /darknet_ros_3d/bounding_boxes; do
  if echo "$topic_list" | grep -q "^${topic}$"; then
    echo "  OK  $topic"
  else
    echo "  MISSING  $topic"
    missing=1
  fi
done
if [ $missing -eq 1 ]; then
  FAIL=1
fi

# Navigation permitted (depth gate)
echo -n "Nav permitted (/stereo/navigation_permitted) ... "
if ros2 topic list 2>/dev/null | grep -q "^/stereo/navigation_permitted$"; then
  HAS_PERMIT=1
  out=$(timeout 5 ros2 topic echo /stereo/navigation_permitted --once 2>&1) || true
  if echo "$out" | grep -q "data: true"; then
    echo "OK (true)"
  else
    echo "WARN (not true)"
  fi
else
  echo "MISSING"
  FAIL=1
fi

if [ $FAIL -eq 0 ]; then
  if [ $HAS_PERMIT -eq 1 ]; then
    echo "Ready. TF/Nav2 OK; check nav_permitted above."
  else
    echo "Ready. TF is valid and Nav2 action server is available."
  fi
  exit 0
else
  echo "Not ready. Wait for Nav2 lifecycle or fix TF before starting mission."
  exit 1
fi
