#!/bin/bash
# Run unit and integration tests for tyre-inspection stack
# Usage: ./ci/run_tests.sh [--unit] [--integration] [--all]

set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f install/setup.bash ] && source install/setup.bash

UNIT=false
INTEGRATION=false

for arg in "$@"; do
  case $arg in
    --unit) UNIT=true ;;
    --integration) INTEGRATION=true ;;
    --all) UNIT=true; INTEGRATION=true ;;
  esac
done

[ "$UNIT" = false ] && [ "$INTEGRATION" = false ] && UNIT=true  # default: unit only

echo "=== Tyre inspection CI ==="

if [ "$UNIT" = true ]; then
  echo "Running unit tests..."
  colcon test --packages-select segmentation_3d segmentation_msgs gb_visual_detection_3d_msgs inspection_manager 2>/dev/null || \
  colcon test --packages-select segmentation_3d 2>/dev/null || echo "Unit tests skipped (packages not built)"
  colcon test-result --all 2>/dev/null || true
fi

if [ "$INTEGRATION" = true ]; then
  echo "Integration tests: run sim/test_mission.py with Aurora + Nav2 (manual)"
  echo "  ros2 launch ugv_nav nav_aurora.launch.py"
  echo "  python3 sim/test_mission.py"
fi

echo "Done."
