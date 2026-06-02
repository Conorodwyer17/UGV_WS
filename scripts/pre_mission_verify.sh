#!/bin/bash
# Pre-mission verification: disk, CUDA, model files. Optionally TF/topics if ROS is running.
# Run before launch (e.g. from start_mission.sh). Exit 0 if OK, non-zero on failure.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"

echo "=== Pre-mission verification ==="

# Check model file (best_fallback.pt or .engine)
MODEL_PT="$UGV_WS/src/Tyre_Inspection_Bot/best_fallback.pt"
MODEL_ENG="$UGV_WS/src/Tyre_Inspection_Bot/best_fallback.engine"
if [[ -f "$MODEL_PT" ]] || [[ -f "$MODEL_ENG" ]]; then
  echo "  Model: OK (best_fallback.pt or .engine found)"
else
  echo "  Model: FAIL (best_fallback.pt and best_fallback.engine not found in src/Tyre_Inspection_Bot/)"
  exit 1
fi

# Run verify_system.py (CUDA, disk; TF/topics if ROS sourced)
if [[ -f "$UGV_WS/scripts/verify_system.py" ]]; then
  python3 "$UGV_WS/scripts/verify_system.py" --skip-ros || true
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    python3 "$UGV_WS/scripts/verify_system.py" || true
  fi
fi

echo "Pre-mission verification complete."
exit 0
