#!/usr/bin/env bash
# Remote RViz for full mission monitoring (run on laptop / second PC).
# Requires: ROS 2 Humble, same LAN as Jetson, same ROS_DOMAIN_ID, Cyclone DDS.
#
# Usage:
#   export ROS_DOMAIN_ID=0   # must match the robot
#   ~/ugv_ws/scripts/monitor_mission.sh
#
# If the workspace is not built on this machine, copy
#   src/.../ugv_bringup/config/full_mission_monitoring.rviz
# and run: rviz2 -d /path/to/full_mission_monitoring.rviz

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "ERROR: source your ROS 2 install (e.g. /opt/ros/humble/setup.bash)" >&2
  exit 1
fi

if [[ -f "$UGV_WS/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$UGV_WS/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

RVIZ_CONFIG=""
if [[ -f "$UGV_WS/install/ugv_bringup/share/ugv_bringup/config/full_mission_monitoring.rviz" ]]; then
  RVIZ_CONFIG="$UGV_WS/install/ugv_bringup/share/ugv_bringup/config/full_mission_monitoring.rviz"
elif command -v ros2 &>/dev/null && ros2 pkg prefix ugv_bringup &>/dev/null; then
  RVIZ_CONFIG="$(ros2 pkg prefix ugv_bringup)/share/ugv_bringup/config/full_mission_monitoring.rviz"
fi

if [[ -z "$RVIZ_CONFIG" || ! -f "$RVIZ_CONFIG" ]]; then
  echo "ERROR: full_mission_monitoring.rviz not found. On the dev machine:" >&2
  echo "  cd $UGV_WS && colcon build --packages-select ugv_bringup" >&2
  echo "Or open the rviz file from the repo:" >&2
  echo "  rviz2 -d $UGV_WS/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_bringup/config/full_mission_monitoring.rviz" >&2
  exit 1
fi

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RViz config: $RVIZ_CONFIG"
exec rviz2 -d "$RVIZ_CONFIG"
