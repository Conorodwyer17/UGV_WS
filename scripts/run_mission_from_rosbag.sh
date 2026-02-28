#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/conor/ugv_ws/install/setup.bash
set -u

WORKSPACE_ROOT="/home/conor/ugv_ws"
LOG_DIR="${WORKSPACE_ROOT}/research/logs"
mkdir -p "${LOG_DIR}" "${WORKSPACE_ROOT}/research/logs/missions" "${WORKSPACE_ROOT}/research/logs/metrics"

python3 - <<'PY'
import glob
m=sorted(glob.glob('/home/conor/ugv_ws/research/data/aurora_samples/aurora_capture_set_*_manifest.json'))
print(m[-1] if m else '')
PY

pushd /tmp >/dev/null
ros2 run inspection_manager photo_capture_api > "${LOG_DIR}/photo_capture_api_integration.log" 2>&1 &
PID_PHOTO=$!
sleep 1
ros2 run inspection_manager manager_node > "${LOG_DIR}/manager_node_integration.log" 2>&1 &
PID_MANAGER=$!
sleep 2

ros2 service call /inspection_manager/start_mission inspection_manager_interfaces/srv/StartMission "{object_id: 'object_1', mission_config_json: '{\"max_retries\":5,\"allow_partial_success\":false}'}" > "${LOG_DIR}/start_mission_integration.txt" 2>&1 || true
sleep 8

python3 "${WORKSPACE_ROOT}/research/tests/evaluate_mission.py" > "${LOG_DIR}/integration_results.txt" 2>&1 || true

kill ${PID_MANAGER} ${PID_PHOTO} || true
popd >/dev/null

echo "integration run complete"
