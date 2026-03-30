#!/bin/bash
# Start the tire inspection mission stack (Jetson max performance, disk check, optional verify, launch).
#
# Default launch: `ugv_nav/full_bringup.launch.py` with **`MISSION_PROFILE`** (see below).
#
# **Field mission (default):** `mission_dedicated_cpu` — **dedicated tyre model** `tyre_detection_project/best.pt`
# on **CPU** (`use_cpu_inference:=true`, **`ultralytics_node_cpu`** loads **`best.onnx`** next to `best.pt`),
# **`wheel_imgsz:=480`**, **real motor**, semantic + Nav2 throttled as below. **Export ONNX first** if missing:
#   cd ~/ugv_ws && MODEL_PT=tyre_detection_project/best.pt IMGSZ=480 bash scripts/export_onnx.sh
#
# **Other profiles** (no extra launch args):
#   stable_viz — same CPU tyre path as above but **stub motor** (`sim_no_move:=true`) for bench demos.
#   crash_fallback_seg — GPU TensorRT **`best_fallback.engine`**, **`wheel_imgsz:=640`**, real motor (stress / OOM recovery).
#   Lighter stack (memory-constrained): `ros2 launch ugv_bringup minimal_tyre_inspection.launch.py`
# Run **`tegrastats`** while launching, e.g.:
#   tegrastats --interval 500 --logfile ~/ugv_ws/benchmarks/tegrastats_fallback_crash.log &
#
# **Parameter types:** ROS 2 launch treats `key:=value` types strictly. Use **2.0** not **2**
# for float parameters, and **`yolo_device:=0`** (launch file default is string `"0"`); if your
# stack ever coerces `0` to integer, use explicit quoting: `yolo_device:='"0"'` (see full_bringup).
#
# Usage:
#   ./scripts/start_mission.sh
#   MISSION_PROFILE=stable_viz ./scripts/start_mission.sh   # CPU tyre + stub motor
#   MISSION_PROFILE=crash_fallback_seg ./scripts/start_mission.sh   # GPU fallback stress
#   ./scripts/start_mission.sh --no-verify    # skip verify_system.py and pre_mission_verify.sh
#   ./scripts/start_mission.sh --verify-only  # run verification only, then exit
#   ./scripts/start_mission.sh sim_no_move:=true   # override: use stub motor instead of defaults
#
# Prerequisite: workspace built (install/setup.bash exists).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
INSTALL_SETUP="$UGV_WS/install/setup.bash"
RUN_VERIFY=true
VERIFY_ONLY=false

LAUNCH_ARGS=()
for arg in "$@"; do
  # Strip trailing ~ from accidental paste (e.g. --no-verify~); otherwise the flag does not match
  # and is forwarded to ros2 launch, and MISSION_PROFILE defaults are skipped.
  while [[ "$arg" == *~ ]]; do
    arg="${arg%\~}"
  done
  case "$arg" in
    --no-verify)   RUN_VERIFY=false ;;
    --verify-only) VERIFY_ONLY=true ;;
    -h|--help)
      echo "Usage: $0 [--no-verify] [--verify-only] [ros2 launch args for full_bringup...]"
      echo "  --no-verify    Skip verify_system.py and pre_mission_verify.sh before launch"
      echo "  --verify-only  Run verification only and exit"
      echo "  With no extra args, uses MISSION_PROFILE (default: mission_dedicated_cpu). See script header."
      exit 0
      ;;
    *) LAUNCH_ARGS+=("$arg") ;;
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

# Defaults for full bringup. Override by passing ros2 launch arguments after the script flags.
# Note: full_bringup uses sim_no_move: false = real motor_driver; true = stub_motor (no motion).
MISSION_PROFILE="${MISSION_PROFILE:-mission_dedicated_cpu}"
FB_DIR="${UGV_WS}/src/Tyre_Inspection_Bot"
FB_ENGINE="${FB_DIR}/best_fallback.engine"
FB_PT="${FB_DIR}/best_fallback.pt"
TYRE_PT="${UGV_WS}/tyre_detection_project/best.pt"
TYRE_ONNX="${UGV_WS}/tyre_detection_project/best.onnx"

if [[ ${#LAUNCH_ARGS[@]} -eq 0 ]]; then
  # Each arg quoted so the shell does not split paths; use explicit string "0" for GPU and 2.0 for float Hz.
  case "$MISSION_PROFILE" in
    mission_dedicated_cpu)
      LAUNCH_ARGS=(
        'sim_no_move:=false'
        'use_motor_driver:=true'
        'use_tyre_3d_positions:=true'
        'require_nav_permitted:=false'
        'require_detection_topic_at_startup:=false'
        'minimal_perception:=false'
        'enable_semantic_segmentation:=true'
        'pcl_fallback_enabled:=false'
        'centroid_servo_enabled:=false'
        'costmap_resolution:=0.10'
        'depth_registered_publish_hz:=2.0'
        'yolo_device:="cpu"'
        'prefer_tensorrt_inspection:=false'
        'use_cpu_inference:=true'
        'inference_interval_s:=0.5'
        'wheel_imgsz:=480'
        "wheel_inspection_model:=${TYRE_PT}"
        'enable_cmd_vel_mux:=false'
        'enable_depth_gate:=false'
        'enable_vehicle_speed_filter:=false'
      )
      echo "Using MISSION_PROFILE=mission_dedicated_cpu (dedicated tyre ONNX/CPU, real motor)."
      ;;
    crash_fallback_seg)
      if [[ -f "$FB_ENGINE" ]]; then
        WHEEL_MODEL="$FB_ENGINE"
        PREF_TRT='prefer_tensorrt_inspection:=true'
      else
        echo "WARNING: ${FB_ENGINE} not found — using ${FB_PT} with TensorRT disabled. Export with:"
        echo "  cd ${FB_DIR} && python3 -c \"from ultralytics import YOLO; YOLO('best_fallback.pt').export(format='engine', imgsz=640, half=True)\""
        WHEEL_MODEL="$FB_PT"
        PREF_TRT='prefer_tensorrt_inspection:=false'
      fi
      LAUNCH_ARGS=(
        'sim_no_move:=false'
        'use_motor_driver:=true'
        'use_tyre_3d_positions:=true'
        'require_nav_permitted:=false'
        'require_detection_topic_at_startup:=false'
        'minimal_perception:=false'
        'enable_semantic_segmentation:=true'
        'pcl_fallback_enabled:=false'
        'centroid_servo_enabled:=false'
        'costmap_resolution:=0.10'
        'depth_registered_publish_hz:=2.0'
        'yolo_device:="0"'
        "${PREF_TRT}"
        'use_cpu_inference:=false'
        # Must match TensorRT export (best_fallback exported at 640×640).
        'wheel_imgsz:=640'
        "wheel_inspection_model:=${WHEEL_MODEL}"
        'enable_cmd_vel_mux:=false'
        'enable_depth_gate:=false'
        'enable_vehicle_speed_filter:=false'
      )
      echo "Using MISSION_PROFILE=crash_fallback_seg (fallback seg TensorRT stress, real motor)."
      ;;
    stable_viz)
      LAUNCH_ARGS=(
        'sim_no_move:=true'
        'use_motor_driver:=true'
        'use_tyre_3d_positions:=true'
        'require_nav_permitted:=false'
        'require_detection_topic_at_startup:=false'
        'minimal_perception:=false'
        'enable_semantic_segmentation:=true'
        'pcl_fallback_enabled:=false'
        'centroid_servo_enabled:=false'
        'costmap_resolution:=0.10'
        'depth_registered_publish_hz:=2.0'
        'yolo_device:="cpu"'
        'prefer_tensorrt_inspection:=false'
        'use_cpu_inference:=true'
        'inference_interval_s:=0.5'
        'wheel_imgsz:=480'
        "wheel_inspection_model:=${TYRE_PT}"
        'enable_cmd_vel_mux:=false'
        'enable_depth_gate:=false'
        'enable_vehicle_speed_filter:=false'
        'demo_mode:=true'
        'demo_simulate_nav_success_topic:=/navigation_success'
      )
      echo "Using MISSION_PROFILE=stable_viz (CPU tyre + stub motor, demo_mode bypasses photo distance gates). For a lighter bringup: ros2 launch ugv_bringup minimal_tyre_inspection.launch.py"
      ;;
    *)
      echo "ERROR: Unknown MISSION_PROFILE=${MISSION_PROFILE} (use mission_dedicated_cpu, stable_viz, or crash_fallback_seg)." >&2
      exit 1
      ;;
  esac
else
  echo "Using user-supplied launch arguments (${#LAUNCH_ARGS[@]} args)."
fi

if [[ "$MISSION_PROFILE" == mission_dedicated_cpu ]] || [[ "$MISSION_PROFILE" == stable_viz ]]; then
  if [[ ! -f "$TYRE_ONNX" ]]; then
    echo ""
    echo "WARNING: CPU tyre inference expects ONNX at ${TYRE_ONNX}"
    echo "  Export (matches wheel_imgsz=480):"
    echo "    cd ${UGV_WS} && MODEL_PT=tyre_detection_project/best.pt IMGSZ=480 bash scripts/export_onnx.sh"
    echo "  Without this, segment_3d may fall back to best_fallback.onnx (wrong classes for dedicated tyre mission)."
    echo ""
  fi
fi

echo "Launching: ros2 launch ugv_nav full_bringup.launch.py ${LAUNCH_ARGS[*]}"
echo ""
exec ros2 launch ugv_nav full_bringup.launch.py "${LAUNCH_ARGS[@]}"
