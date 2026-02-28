#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u

export PYTHONPATH=".:${PYTHONPATH:-}"

echo "[test] simulated mission replay using latest Aurora capture manifest"
python3 research/tests/evaluate_mission.py
echo "[test] done"

