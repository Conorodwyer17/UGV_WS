#!/bin/bash
# Export best_fallback.pt to TensorRT engine for Jetson Orin (PATH_FORWARD §3.6).
# Run on the Jetson: cd ~/ugv_ws && bash scripts/export_tensorrt.sh
# The ultralytics_node will use best_fallback.engine when prefer_tensorrt_inspection=true.

set -e
UGV_WS="${UGV_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
MODEL_PT="${UGV_WS}/src/Tyre_Inspection_Bot/best_fallback.pt"
OUTPUT_ENGINE="${UGV_WS}/src/Tyre_Inspection_Bot/best_fallback.engine"

if [[ ! -f "$MODEL_PT" ]]; then
  echo "ERROR: $MODEL_PT not found"
  exit 1
fi

# TensorRT export requires NVIDIA GPU and PyTorch with CUDA (e.g. on the Jetson).
if ! python3 -c "import torch; exit(0 if torch.cuda.is_available() else 1)" 2>/dev/null; then
  echo "ERROR: CUDA not available (torch.cuda.is_available() is False)."
  echo "TensorRT export must be run on a machine with an NVIDIA GPU and PyTorch built with CUDA."
  echo "  - On your desktop: you have PyTorch CPU-only (torch-2.10.0+cpu). Run this script on the Jetson instead."
  echo "  - On the Jetson: ensure PyTorch with CUDA is installed (e.g. from NVIDIA or pip install torch with CUDA)."
  echo "  - Copy $MODEL_PT to the Jetson, run: cd ~/ugv_ws && ./scripts/export_tensorrt.sh"
  echo "  - Then copy the generated .engine file back if you build the rest of the workspace on the desktop."
  exit 1
fi

echo "Exporting $MODEL_PT to TensorRT engine..."
echo "Output: $OUTPUT_ENGINE"
echo ""

# Optional: IMGSZ=320 WORKSPACE=1 for OOM mitigation on Jetson (smaller engine, less memory at load)
IMGSZ="${IMGSZ:-640}"
WORKSPACE="${WORKSPACE:-4}"
echo "Using imgsz=$IMGSZ workspace=${WORKSPACE}GB (override with env: IMGSZ=320 WORKSPACE=1 for OOM)"
echo ""

# Requires: pip install ultralytics
yolo export model="$MODEL_PT" format=engine half=True simplify=True imgsz=$IMGSZ workspace=$WORKSPACE

# Ultralytics exports to same dir as input with .engine extension
GENERATED="${MODEL_PT%.pt}.engine"
if [[ -f "$GENERATED" ]]; then
  mv -f "$GENERATED" "$OUTPUT_ENGINE"
  echo "Done. Set prefer_tensorrt_inspection: true in segment_3d launch to use the engine."
  echo ""
  echo "If you see 'invalid class index' (28,37,39,45,47) at runtime, the engine may have"
  echo "been built from a different model. Re-run this script on the Jetson with the"
  echo "correct best_fallback.pt (23 classes, wheel at id=22). See docs/TIRE_DETECTION_TROUBLESHOOTING.md"
else
  echo "ERROR: Export may have failed; $GENERATED not found"
  exit 1
fi
