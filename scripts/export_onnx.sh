#!/bin/bash
# Export a YOLO .pt model to ONNX for ultralytics_node_cpu (CPU inference).
#
# Default: best_fallback.pt → best_fallback.onnx (IMGSZ default 224).
# Tyre model (matches default CPU mission tyre path, wheel_imgsz 480):
#   cd ~/ugv_ws && MODEL_PT=tyre_detection_project/best.pt IMGSZ=480 bash scripts/export_onnx.sh
#
# The exported ONNX input size must match launch `wheel_imgsz` (and ultralytics predict imgsz).

set -e
UGV_WS="${UGV_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BOT_DIR="${UGV_WS}/src/Tyre_Inspection_Bot"

if [[ -n "${MODEL_PT:-}" ]]; then
  # Allow relative path from ugv_ws
  if [[ "${MODEL_PT}" != /* ]]; then
    MODEL_PT="${UGV_WS}/${MODEL_PT}"
  fi
else
  MODEL_PT="${BOT_DIR}/best_fallback.pt"
  if [[ ! -f "$MODEL_PT" ]]; then
    MODEL_PT="${UGV_WS}/best_fallback.pt"
  fi
fi

if [[ ! -f "$MODEL_PT" ]]; then
  echo "ERROR: Model not found: ${MODEL_PT}"
  echo "Set MODEL_PT to your .pt file, e.g. MODEL_PT=\${UGV_WS}/tyre_detection_project/best.pt"
  exit 1
fi

OUTPUT_ONNX="${OUTPUT_ONNX:-${MODEL_PT%.pt}.onnx}"

if [[ -z "${IMGSZ:-}" ]]; then
  if [[ "$MODEL_PT" == *"tyre_detection"* ]]; then
    IMGSZ=480
  else
    IMGSZ=224
  fi
fi

echo "Exporting $MODEL_PT to ONNX (imgsz=$IMGSZ)..."
echo "Output: $OUTPUT_ONNX"
echo ""

export MODEL_PT_FOR_EXPORT="$MODEL_PT"
export IMGSZ_FOR_EXPORT="$IMGSZ"
python3 - <<'PY'
import os
from ultralytics import YOLO
p = os.environ["MODEL_PT_FOR_EXPORT"]
sz = int(os.environ["IMGSZ_FOR_EXPORT"])
YOLO(p).export(format="onnx", imgsz=sz, opset=12, simplify=True)
PY

GENERATED="${MODEL_PT%.pt}.onnx"
if [[ -f "$GENERATED" ]]; then
  if [[ "$GENERATED" != "$OUTPUT_ONNX" ]]; then
    mv -f "$GENERATED" "$OUTPUT_ONNX"
  fi
  echo "Done. Match launch wheel_imgsz to IMGSZ=${IMGSZ} (mission_dedicated_cpu / stable_viz use 480 for tyre ONNX)."
else
  echo "ERROR: Export may have failed; $GENERATED not found"
  exit 1
fi
