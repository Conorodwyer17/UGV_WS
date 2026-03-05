#!/bin/bash
# Start the tire inspection mission stack (Jetson max performance, disk check, optional verify, launch).
#
# Usage:
#   ./scripts/start_mission.sh
#   ./scripts/start_mission.sh --no-verify    # skip verify_system.py and pre_mission_verify.sh
#   ./scripts/start_mission.sh --verify-only  # run verification only, then exit
#
# Prerequisite: workspace built (install/setup.bash exists).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
INSTALL_SETUP="$UGV_WS/install/setup.bash"
RUN_VERIFY=true
VERIFY_ONLY=false

for arg in "$@"; do
  case "$arg" in
    --no-verify)   RUN_VERIFY=false ;;
    --verify-only) VERIFY_ONLY=true ;;
    -h|--help)
      echo "Usage: $0 [--no-verify] [--verify-only]"
      echo "  --no-verify    Do not run verify_system.py or pre_mission_verify.sh before launch"
      echo "  --verify-only Run verification only and exit"
      exit 0
      ;;
  esac
done

echo "=== Starting Tire Inspection Robot ==="
echo "Workspace: $UGV_WS"
echo ""

# Jetson max performance
if command -v nvpmodel &>/dev/null; then
  sudo nvpmodel -m 0 2>/dev/null || true
fi
if command -v jetson_clocks &>/dev/null; then
  sudo jetson_clocks 2>/dev/null || true
fi

# Disk space check (warn if >90% used or <1GB free)
if command -v df &>/dev/null; then
  USED_PCT=$(df -h "$UGV_WS" 2>/dev/null | awk 'NR==2 {print $5}' | tr -d '%')
  FREE_KB=$(df -k "$UGV_WS" 2>/dev/null | awk 'NR==2 {print $4}')
  FREE_GB=$((FREE_KB / 1024 / 1024))
  if [[ -n "$USED_PCT" ]] && [[ "$USED_PCT" -gt 90 ]]; then
    echo "WARNING: Disk usage >90%. Consider cleaning (e.g. scripts/cleanup.sh --execute)."
  fi
  if [[ -n "$FREE_GB" ]] && [[ "$FREE_GB" -lt 1 ]]; then
    echo "WARNING: Less than 1 GB free. Mission logs may fail."
  fi
fi

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "ERROR: install/setup.bash not found. Build first:"
  echo "  cd $UGV_WS && colcon build --symlink-install"
  exit 1
fi

source "$INSTALL_SETUP"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Aurora SDK library path (same as startup.sh)
AURORA_LIB="$UGV_WS/src/aurora_ros2_sdk_linux/src/aurora_remote_public/lib"
case "$(uname -m)" in
  aarch64) AURORA_LIB="$AURORA_LIB/linux_aarch64" ;;
  x86_64)  AURORA_LIB="$AURORA_LIB/linux_x86_64"  ;;
  *)       AURORA_LIB="" ;;
esac
if [[ -n "$AURORA_LIB" ]] && [[ -d "$AURORA_LIB" ]]; then
  export LD_LIBRARY_PATH="$AURORA_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

mkdir -p "$UGV_WS/logs"

if $VERIFY_ONLY; then
  echo "Running verification only..."
  bash "$UGV_WS/scripts/pre_mission_verify.sh" || true
  python3 "$UGV_WS/scripts/verify_system.py" --skip-ros || true
  echo "Verification done."
  exit 0
fi

if $RUN_VERIFY; then
  echo "Running pre-mission verification..."
  if ! bash "$UGV_WS/scripts/pre_mission_verify.sh"; then
    echo "Pre-mission verify had failures; continuing anyway. Fix issues before field run."
  fi
  echo ""
fi

echo "Launching full bringup..."
exec ros2 launch ugv_nav full_bringup.launch.py "$@"
