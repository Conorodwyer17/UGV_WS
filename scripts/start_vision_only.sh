#!/bin/bash
# Start perception pipeline only (Aurora + segment_3d). No Nav2, no motor.
# Dry run to test detection. Check /tire_bounding_boxes_merged, /ultralytics_tire/segmentation/objects_segment.
#
# Usage: ./scripts/start_vision_only.sh [ip_address:=192.168.11.1]

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

# Aurora SDK library path
AURORA_LIB="$UGV_WS/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib"
case "$(uname -m)" in
  aarch64) AURORA_LIB="$AURORA_LIB/linux_aarch64" ;;
  x86_64)  AURORA_LIB="$AURORA_LIB/linux_x86_64"  ;;
  *)       AURORA_LIB="" ;;
esac
if [[ -n "$AURORA_LIB" ]] && [[ -d "$AURORA_LIB" ]]; then
  export LD_LIBRARY_PATH="$AURORA_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

echo "=== Vision Only (Aurora + segment_3d) ==="
echo "No Nav2, no motor. Use ros2 topic echo /tire_bounding_boxes_merged to see detections."
echo ""
exec ros2 launch ugv_nav vision_only.launch.py "$@"
