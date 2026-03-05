#!/bin/bash
# Export best_fallback.pt to ONNX for CPU inference (no GPU, no OOM).
# Run: cd ~/ugv_ws && bash scripts/export_onnx.sh
# The ultralytics_node_cpu uses best_fallback.onnx when use_cpu_inference:=true.

set -e
UGV_WS="${UGV_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BOT_DIR="${UGV_WS}/src/Tyre_Inspection_Bot"
MODEL_PT="${BOT_DIR}/best_fallback.pt"
OUTPUT_ONNX="${BOT_DIR}/best_fallback.onnx"

# Also check workspace root
if [[ ! -f "$MODEL_PT" ]]; then
  MODEL_PT="${UGV_WS}/best_fallback.pt"
fi
if [[ ! -f "$MODEL_PT" ]]; then
  echo "ERROR: best_fallback.pt not found at ${BOT_DIR}/ or ${UGV_WS}/"
  exit 1
fi

# ONNX export works on CPU (no CUDA needed)
IMGSZ="${IMGSZ:-224}"
echo "Exporting $MODEL_PT to ONNX (imgsz=$IMGSZ)..."
echo "Output: $OUTPUT_ONNX"
echo ""

python3 -c "
from ultralytics import YOLO
model = YOLO('$MODEL_PT')
model.export(format='onnx', imgsz=$IMGSZ, opset=12, simplify=True)
"

# Ultralytics exports to same dir as input with .onnx extension
GENERATED="${MODEL_PT%.pt}.onnx"
if [[ -f "$GENERATED" ]]; then
  if [[ "$GENERATED" != "$OUTPUT_ONNX" ]]; then
    mv -f "$GENERATED" "$OUTPUT_ONNX"
  fi
  echo "Done. Use use_cpu_inference:=true in launch to use ONNX on CPU."
  echo "  ./scripts/start_mission.sh use_cpu_inference:=true"
else
  echo "ERROR: Export may have failed; $GENERATED not found"
  exit 1
fi
