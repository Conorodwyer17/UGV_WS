#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u

export PYTHONPATH=".:${PYTHONPATH:-}"

python3 -m pytest -q inspection_manager/tests
bash research/tests/test_mission_replay_from_rosbag.sh

echo "All local tests completed."

