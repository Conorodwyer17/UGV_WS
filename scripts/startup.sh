#!/bin/bash
# Start full tire-inspection stack: Aurora 2.11 native depth + semantic vehicles + YOLO tires.
# Vehicles: Aurora COCO80 semantic (car, truck, bus). Tires: YOLO best.pt.
#
# Usage:
#   ./scripts/startup.sh
#   ./scripts/startup.sh ip_address:=192.168.11.1
#   ./scripts/startup.sh use_motor_driver:=false dry_run:=true
#
# Args: ip_address, use_bridge (default false), use_motor_driver, dry_run, config_file

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

# Ensure logs dir exists for mission JSON/JSONL (offline review after WiFi reconnect)
mkdir -p "$UGV_WS/logs"

echo "=== Tire Inspection Stack (Aurora 2.11) ==="
echo "Workspace: $UGV_WS"
echo "RMW:       $RMW_IMPLEMENTATION"
echo "Launch:    ugv_nav full_bringup.launch.py $*"
echo ""
echo "Inspection manager starts 120s after launch; waits for Nav2 then runs full mission. PRODUCTION_CONFIG auto-loaded."
echo ""

exec ros2 launch ugv_nav full_bringup.launch.py "$@"
