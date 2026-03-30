#!/bin/bash
# Launch full stack with NO physical movement (stub_motor discards cmd_vel).
# Use for TF diagnostics (view_frames, tf2_monitor, costmap checks) without moving the robot.
#
# Usage:
#   ./scripts/start_no_move.sh
#   ./scripts/start_no_move.sh --no-verify   # skip pre-mission verification
#
# Then in another terminal run diagnostics:
#   ros2 run tf2_tools view_frames -o /tmp/tf_frames
#   ros2 topic hz /tf
#   ros2 topic hz /tf_static
#   ros2 run tf2_ros tf2_monitor
#   ros2 topic echo /local_costmap/costmap --once | head -50

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/start_mission.sh" --no-verify sim_no_move:=true "$@"
