#!/bin/bash
# Start tire inspection simulation (aurora_mock). No hardware required.
#
# Usage: ./scripts/start_simulation.sh [use_mock:=true] [use_bag:=false]

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
INSTALL_SETUP="$UGV_WS/install/setup.bash"

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "ERROR: install/setup.bash not found. Build first:"
  echo "  cd $UGV_WS && colcon build --symlink-install"
  exit 1
fi

source "$INSTALL_SETUP"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "=== Tire Inspection Simulation (aurora_mock) ==="
exec ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true "$@"
