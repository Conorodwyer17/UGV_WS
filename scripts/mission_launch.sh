#!/bin/bash
# Launch autonomous tire inspection mission.
# Runs pre-flight checklist, then starts full stack.
# For headless/autostart: SKIP_PREFLIGHT=1 bash scripts/mission_launch.sh
#
# Usage:
#   ./scripts/mission_launch.sh
#   ./scripts/mission_launch.sh ip_address:=192.168.11.1
#   ./scripts/mission_launch.sh dry_run:=true   # validate without motion

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
INSTALL_SETUP="${UGV_WS}/install/setup.bash"

# Use same env as startup.sh so checklist and launch see the same nodes/TF (critical for TF check).
if [[ -f "$INSTALL_SETUP" ]]; then
  source "$INSTALL_SETUP"
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Optional: run pre-flight checklist (skip for dry_run or SKIP_PREFLIGHT=1)
if [[ "${SKIP_PREFLIGHT}" != "1" ]] && [[ "$*" != *"dry_run:=true"* ]]; then
  echo "=== Pre-flight checklist ==="
  if bash "$SCRIPT_DIR/aurora_pre_mission_checklist.sh"; then
    echo "Pre-flight OK."
  else
    echo "Pre-flight had failures. Continue? [y/N]"
    read -r r
    [[ "$r" =~ ^[yY] ]] || exit 1
  fi
  echo "=== Mission readiness (TF + Nav2) ==="
  if bash "$SCRIPT_DIR/mission_readiness_check.sh"; then
    echo "Mission readiness OK."
  else
    echo "Mission readiness failed. Continue? [y/N]"
    read -r r
    [[ "$r" =~ ^[yY] ]] || exit 1
  fi
  echo ""
fi

# startup.sh will source again and set RMW again (idempotent)
exec bash "$SCRIPT_DIR/startup.sh" "$@"
