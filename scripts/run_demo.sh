#!/usr/bin/env bash
# One-click helper for thesis no-motion demo (optional). Review commands before use.
#
# Typical workflow (recommended: three terminals):
#   1) This script or manual: start mission + RViz + auto_advance
#   2) Or run start_mission.sh in foreground in terminal 1 only.
#
set -e
UGV_WS="${UGV_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=/dev/null
source "$UGV_WS/install/setup.bash"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

RVIZ_CFG="$UGV_WS/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/rviz/tyre_inspection_demo.rviz"

echo "=== Tyre inspection no-motion demo (stable_viz) ==="
echo "Workspace: $UGV_WS"
echo ""

echo "--- Terminal A (mission) ---"
echo "MISSION_PROFILE=stable_viz \"$UGV_WS/scripts/start_mission.sh\" --no-verify \\"
echo "  tyre_perimeter_bridge_enabled:=true use_tyre_geometry:=true"
echo ""

echo "--- Terminal B (RViz, after ~30 s) ---"
echo "rviz2 -d \"$RVIZ_CFG\""
echo ""

echo "--- Terminal C (auto-advance, after mission is up) ---"
echo "python3 \"$UGV_WS/scripts/auto_advance.py\""
echo ""

read -r -p "Launch mission in background now? [y/N] " ans
if [[ "${ans,,}" == "y" ]]; then
  MISSION_PROFILE=stable_viz "$UGV_WS/scripts/start_mission.sh" --no-verify \
    tyre_perimeter_bridge_enabled:=true use_tyre_geometry:=true &
  echo "Mission PID $! — waiting 10 s before optional RViz..."
  sleep 10
  if [[ -f "$RVIZ_CFG" ]]; then
    read -r -p "Start RViz? [y/N] " rv
    if [[ "${rv,,}" == "y" ]]; then
      rviz2 -d "$RVIZ_CFG" &
    fi
  else
    echo "RViz config not found: $RVIZ_CFG"
  fi
  read -r -p "Start auto_advance.py in foreground? [y/N] " aa
  if [[ "${aa,,}" == "y" ]]; then
    exec python3 "$UGV_WS/scripts/auto_advance.py"
  fi
else
  echo "Copy the commands above into separate terminals when ready."
fi
