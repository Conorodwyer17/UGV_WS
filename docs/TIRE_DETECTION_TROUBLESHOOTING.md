# Tyre Detection Troubleshooting

Troubleshooting guide for the `ultralytics_tire` node and tyre detection pipeline on the autonomous tyre inspection robot.

**16 GB Jetson Orin Nano:** GPU is default (`use_cpu_inference:=false`); 640×640 at 10 Hz. OOM workarounds below are for 8 GB or older hardware.

---

## 1. Invalid Class Indices (28, 37, 39, 45, 47)

### Symptom

```
[ultralytics_tire]: Skipping detection with invalid class index 37 (valid range 0-22); TensorRT may return out-of-range indices
```

The model expects class indices 0–22 (custom vehicle-parts dataset; wheel = 22). Indices 28, 37, 39, 45, 47 correspond to **COCO80** classes (e.g. 37=skateboard, 45=spoon).

**Note:** The ultralytics_node now filters out-of-range class indices (`class_index < 0 or class_index >= num_classes`) and skips those detections. The warning is logged once per invalid index per session. Valid wheel detections (class 22) are still published.

### Root Cause

The TensorRT engine (`best_fallback.engine`) was likely built from a model with 80 COCO classes, or from a different `.pt` file than the current `best_fallback.pt`. The engine’s output layer produces COCO indices, while the node uses `best_fallback.pt` metadata (23 classes).

### Fixes

**Option A: Use PyTorch model instead of TensorRT (recommended short-term)**

```bash
./scripts/start_mission.sh prefer_tensorrt_inspection:=false
```

Or directly:

```bash
ros2 launch ugv_nav full_bringup.launch.py prefer_tensorrt_inspection:=false
```

Inference will be slower (~50–100 ms vs ~5 ms) but class indices will be correct.

**Option B: Re-export the TensorRT engine**

Ensure the engine is built from the correct 23-class `best_fallback.pt`:

```bash
cd ~/ugv_ws && ./scripts/export_tensorrt.sh
```

Verify the `.pt` file has 23 classes:

```bash
python3 -c "
from ultralytics import YOLO
m = YOLO('src/Tyre_Inspection_Bot/best_fallback.pt')
print('Classes:', len(m.names), list(m.names.values())[:5], '...')
"
```

Expected: 23 classes, including `wheel`. If you see 80 classes, the `.pt` file is wrong.

---

## 2. NMS Time Limit Exceeded

### Symptom

```
WARNING ⚠️ NMS time limit 2.050s exceeded
```

Inference or NMS is taking too long, often due to many candidate detections. Default is 10 Hz on 16 GB Jetson; if the model cannot keep up, increase `inference_interval_s` (e.g. 0.15 s).

### Fixes

**A. Reduce max detections** (launch args)

```bash
ros2 launch ugv_nav full_bringup.launch.py wheel_max_det:=50
```

**B. Reduce input size** (faster inference, slight accuracy trade-off)

```bash
ros2 launch ugv_nav full_bringup.launch.py wheel_imgsz:=480
```

**C. Raise confidence threshold** (fewer detections before NMS)

```bash
ros2 launch ugv_nav full_bringup.launch.py wheel_confidence:=0.6
```

**D. Combine for best effect**

```bash
ros2 launch ugv_nav full_bringup.launch.py wheel_max_det:=100 wheel_imgsz:=480 wheel_confidence:=0.55
```

---

## 3. CUDA Out of Memory (OOM) on Startup

### Symptom

```
NvMapMemAllocInternalTagged: 1075072515 error 12
Cuda Runtime (out of memory)
```

The `ultralytics_tire` node crashes immediately when loading the TensorRT engine. **16 GB Jetson:** OOM is rare; default is 1.0 s model load delay. **8 GB Jetson:** May have ~4.4 GB free; memory fragmentation or peak allocation during engine deserialization can cause failure.

### Root Cause

TensorRT engine loading at t=8s competes with Aurora SDK, depth pipeline, and other nodes. All start together; GPU memory may be fragmented or insufficient at load time.

### Fixes

**Option A: Use PyTorch instead of TensorRT (immediate workaround)**

```bash
./scripts/start_mission.sh prefer_tensorrt_inspection:=false
```

Inference is slower (~50–100 ms vs ~5 ms) but avoids OOM.

**Option B: Delay model load**

Let other nodes settle before loading the model. Default is 1.0 s on 16 GB Jetson (5.0 s on 8 GB); increase if OOM persists. Use decimal (e.g. `10.0`) to avoid parameter type mismatch:

```bash
./scripts/start_mission.sh model_load_delay_s:=10.0
```

**Option C: Export a smaller TensorRT engine**

Reduces peak memory during load (for 8 GB Jetson):

```bash
IMGSZ=320 WORKSPACE=4 ./scripts/export_tensorrt.sh
```

Then run normally. Slight accuracy trade-off for smaller input size. Default on 16 GB: imgsz=640, workspace=8 GB.

**Option D: Combine B + C**

```bash
IMGSZ=320 WORKSPACE=4 ./scripts/export_tensorrt.sh
./scripts/start_mission.sh model_load_delay_s:=10.0
```

---

## 4. Nav2 Bringup and Mission Start Verification

Pre-mission verification runs *before* nodes start, so TF and topics will not exist yet. That is expected. Use these checks **after** bringup.

### Post-bringup checklist

1. **TF chain** (wait ~30 s after launch):

   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

   Should print transforms without errors.

2. **Key topics**:

   ```bash
   ros2 topic list | grep -E "vehicle_bounding_boxes|tire_bounding_boxes|left_image_raw"
   ```

   Expected: `/aurora_semantic/vehicle_bounding_boxes`, `/tire_bounding_boxes_merged`, `/slamware_ros_sdk_server_node/left_image_raw`.

3. **inspection_manager state**:

   ```bash
   ros2 topic echo /inspection_manager/state --once
   ```

   Expected: `IDLE` initially, then `SEARCH_VEHICLE` when mission starts.

4. **Nav2 lifecycle**:

   ```bash
   ros2 lifecycle get /controller_server
   ros2 lifecycle get /bt_navigator
   ```

   Expected: `active [3]` for both.

5. **Full system verification** (after bringup):

   ```bash
   python3 ~/ugv_ws/scripts/verify_system.py
   ```

### Mission start flow

1. `start_mission.sh` → `full_bringup.launch.py`
2. Aurora + motor driver + static TF (~0–5 s)
3. segment_3d (perception) at 8 s
4. Nav2 at 15 s (or 5 s with external TF)
5. inspection_manager at 120 s (or 8 s standalone)
6. inspection_manager: `IDLE` → `SEARCH_VEHICLE` when it receives vehicle boxes

If the mission does not start, check:

- `/aurora_semantic/vehicle_bounding_boxes` is publishing
- TF `map` → `slamware_map` → `odom` → `base_link` is valid
- No errors in `ultralytics_tire` (e.g. invalid class indices, crashes)

---

## 5. Quick Reference: Launch Overrides

| Issue | Override |
|-------|----------|
| Invalid class indices | `prefer_tensorrt_inspection:=false` |
| CUDA OOM on startup (8 GB) | `prefer_tensorrt_inspection:=false` or `model_load_delay_s:=10.0` |
| NMS timeout | `wheel_max_det:=50` or `wheel_imgsz:=480` or `inference_interval_s:=0.15` |
| Too many false positives | `wheel_confidence:=0.6` |
| CPU fallback (8 GB) | `use_cpu_inference:=true` |

Example combined override:

```bash
./scripts/start_mission.sh prefer_tensorrt_inspection:=false wheel_max_det:=100 wheel_imgsz:=480
```
