#!/usr/bin/env bash
# Non-interactive mission snapshot for tyre 3D / goal / planning debug.
# Uses `timeout` so tf2_echo and missing topics cannot hang the script.

# Do not use nounset: workspace setup.bash may reference unset vars (e.g. COLCON_TRACE).
set -o pipefail

WS="${UGV_WS:-/home/conor/ugv_ws}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
DIAG_DIR="${WS}/diagnostics"
mkdir -p "${DIAG_DIR}"
DIAG_FILE="${DIAG_DIR}/diagnostic_${TIMESTAMP}.txt"
TYRE_TMP="$(mktemp)"

# Seconds: topic echo --once (no publisher = can hang without timeout)
TO_TOPIC="${COLLECT_DIAG_TOPIC_TIMEOUT:-15}"
# tf2_echo prints repeatedly; capture a few samples then stop
TO_TF="${COLLECT_DIAG_TF_TIMEOUT:-4}"

cleanup() { rm -f "${TYRE_TMP}"; }
trap cleanup EXIT

if [[ -f "${WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS}/install/setup.bash"
else
  echo "WARNING: ${WS}/install/setup.bash not found; sourcing /opt/ros/humble only" >&2
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

exec > >(tee "${DIAG_FILE}")
exec 2>&1

echo "collect_mission_diagnostics.sh — saved to: ${DIAG_FILE}"
echo ""

echo "=== System time ==="
date
echo ""

echo "=== ROS time (if /clock exists) ==="
timeout 3 ros2 topic echo /clock --once 2>&1 || echo "(no /clock or timeout)"
echo ""

echo "=== inspection_manager runtime_diagnostics ==="
timeout "${TO_TOPIC}" ros2 topic echo /inspection_manager/runtime_diagnostics --once --full-length 2>&1 || echo "(timeout or no message)"
echo ""

echo "=== inspection_manager current_goal ==="
timeout "${TO_TOPIC}" ros2 topic echo /inspection_manager/current_goal --once --no-arr 2>&1 || echo "(timeout or no message)"
echo ""

echo "=== /tyre_3d_positions (full) ==="
timeout "${TO_TOPIC}" ros2 topic echo /tyre_3d_positions --once 2>&1 | tee "${TYRE_TMP}" || echo "(timeout or no message)"
echo ""

echo "=== /aurora_semantic/vehicle_bounding_boxes ==="
timeout "${TO_TOPIC}" ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once --no-arr 2>&1 || echo "(timeout or no message)"
echo ""

echo "=== /plan ==="
timeout "${TO_TOPIC}" ros2 topic echo /plan --once --no-arr 2>&1 || echo "(timeout or no message)"
echo ""

echo "=== /local_plan ==="
timeout "${TO_TOPIC}" ros2 topic echo /local_plan --once --no-arr 2>&1 || echo "(timeout or no message)"
echo ""

echo "=== TF slamware_map -> base_link (${TO_TF}s sample) ==="
timeout "${TO_TF}" ros2 run tf2_ros tf2_echo slamware_map base_link 2>&1 || true
echo ""

echo "=== TF camera_depth_optical_frame -> slamware_map (${TO_TF}s sample) ==="
timeout "${TO_TF}" ros2 run tf2_ros tf2_echo camera_depth_optical_frame slamware_map 2>&1 || true
echo ""

echo "=== Robot pose: map -> base_link (${TO_TF}s sample) ==="
timeout "${TO_TF}" ros2 run tf2_ros tf2_echo map base_link 2>&1 || true
echo ""

echo "=== ROS nodes (inspection / nav) ==="
timeout 8 ros2 node list 2>&1 | grep -E 'inspection|nav2|controller' || timeout 8 ros2 node list 2>&1 | head -40
echo ""

echo "=== inspection_manager parameters (dump; tyre 3D / goal) ==="
IM_NODE=""
for n in /inspection_manager inspection_manager; do
  if timeout 8 ros2 param list "${n}" &>/dev/null; then
    IM_NODE="${n}"
    break
  fi
done
if [[ -n "${IM_NODE}" ]]; then
  timeout 25 ros2 param dump "${IM_NODE}" 2>&1 \
    | grep -E 'use_tyre_3d|prefer_tyre|tyre_3d_|tire_offset|world_frame|map_frame|dry_run' \
    || timeout 25 ros2 param dump "${IM_NODE}" 2>&1 | head -120
else
  echo "(could not find inspection_manager node; check: ros2 node list)"
fi
echo ""

echo "=== mission_latest.jsonl (last 50 lines) ==="
if [[ -f "${WS}/logs/mission_latest.jsonl" ]]; then
  tail -50 "${WS}/logs/mission_latest.jsonl"
else
  echo "(file missing: ${WS}/logs/mission_latest.jsonl)"
fi
echo ""

echo "=== Optional: GetCosts on global costmap at first tyre (x,y) from snapshot ==="
PY="${WS}/scripts/costmap_value_at_xy.py"
if grep -qE '^\s+x:\s*[-0-9.]+' "${TYRE_TMP}" 2>/dev/null; then
  X="$(grep -E '^\s+x:\s*' "${TYRE_TMP}" | head -1 | awk '{print $2}')"
  Y="$(grep -E '^\s+y:\s*' "${TYRE_TMP}" | head -1 | awk '{print $2}')"
  if [[ -n "${X:-}" && -n "${Y:-}" ]]; then
    echo "Using first tyre pose from snapshot: x=${X} y=${Y}"
    if [[ -f "${PY}" ]]; then
      timeout 15 python3 "${PY}" "${X}" "${Y}" --service /global_costmap/get_cost_global_costmap 2>&1 \
        || echo "(python GetCosts failed; trying ros2 service call)"
    fi
    if timeout 12 ros2 service type /global_costmap/get_cost_global_costmap &>/dev/null; then
      timeout 15 ros2 service call /global_costmap/get_cost_global_costmap nav2_msgs/srv/GetCosts "{use_footprint: false, poses: [{header: {frame_id: map, stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}" 2>&1 \
        || echo "(ros2 service call GetCosts failed)"
    else
      echo "(service /global_costmap/get_cost_global_costmap not available)"
    fi
  else
    echo "Could not parse x,y from tyre snapshot; skip GetCosts."
  fi
else
  echo "No tyre position lines in snapshot; skip GetCosts."
fi
echo ""

echo "=== End of diagnostic ${TIMESTAMP} ==="
