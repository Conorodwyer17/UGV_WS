#!/usr/bin/env bash
# Export YOLO .pt models to TensorRT .engine for Jetson.
# Usage: ./scripts/export_tensorrt.sh /path/to/model.pt [imgsz]

set -euo pipefail

MODEL_PATH="${1:-}"
IMGSZ="${2:-640}"

if [[ -z "${MODEL_PATH}" ]]; then
  echo "Usage: $0 /path/to/model.pt [imgsz]"
  exit 1
fi

if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "Model not found: ${MODEL_PATH}"
  exit 1
fi

echo "Exporting TensorRT engine for ${MODEL_PATH} (imgsz=${IMGSZ})"
python3 - "${MODEL_PATH}" "${IMGSZ}" <<'PY'
import sys
from ultralytics import YOLO

model_path = sys.argv[1]
imgsz = int(sys.argv[2])
model = YOLO(model_path)
model.export(format="engine", imgsz=imgsz, half=True)
PY

ENGINE_PATH="${MODEL_PATH%.pt}.engine"
if [[ -f "${ENGINE_PATH}" ]]; then
  echo "Engine created: ${ENGINE_PATH}"
else
  echo "Engine not found after export. Check logs."
fi
