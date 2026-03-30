#!/usr/bin/env bash
# No-motion thesis demo helper: source workspace and print one-liners (no gnome-terminal dependency).
set -e
UGV_WS="${UGV_WS:-$HOME/ugv_ws}"
# shellcheck source=/dev/null
source "$UGV_WS/install/setup.bash"

echo "=== Tyre inspection thesis demo (stable_viz / stub motor) ==="
echo "Workspace: $UGV_WS"
echo ""
echo "1) Full stack + mission (existing script):"
echo "   MISSION_PROFILE=stable_viz $UGV_WS/scripts/start_mission.sh --no-verify"
echo ""
echo "2) RViz (tyre_inspection_demo.rviz):"
echo "   rviz2 -d $UGV_WS/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/rviz/tyre_inspection_demo.rviz"
echo ""
echo "3) Optional: synthetic /tyre_3d_positions (if projection node off):"
echo "   ros2 run inspection_manager demo_cycle_tyre_poses"
echo "   Synthetic Nav2 arrival (pairs with demo_mode + demo_simulate_nav_success_topic in stable_viz):"
echo "   ros2 run inspection_manager demo_cycle_tyre_poses --ros-args -p simulate_navigation:=true"
echo ""
echo "4) Last mission JSONL summary:"
echo "   python3 $UGV_WS/scripts/generate_mission_report.py"
echo ""
echo "5) Captured photo display topic (after photo_capture publishes):"
echo "   ros2 run rqt_image_view rqt_image_view /captured_photo_display"
echo ""
