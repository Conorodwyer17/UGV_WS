# Vision Pipeline Troubleshooting

For tyre detection issues, see [TIRE_DETECTION_TROUBLESHOOTING.md](TIRE_DETECTION_TROUBLESHOOTING.md). This document covers broader vision pipeline topics.

## Camera and Intrinsics

**Aurora depth:** The Aurora provides calibrated depth and intrinsics. The `aurora_depth_camera_info_node` publishes `/camera/depth/camera_info` from `aurora_depth_intrinsics.yaml`.

**Verification:**
```bash
ros2 topic echo /camera/depth/camera_info --once
```

## YOLO Model

- **Model files:** `best_fallback.pt` (PyTorch), `best_fallback.engine` (TensorRT), `best_fallback.onnx` (CPU fallback)
- **Class:** `wheel` (id=22) for tyre inspection
- **CPU fallback:** `use_cpu_inference:=true` when GPU unavailable or OOM

## TensorRT Export

```bash
cd ~/ugv_ws && bash scripts/export_tensorrt.sh
```

- Input size: 640×640 (default)
- Workspace: 8 GB for 16 GB Jetson
- If export fails, use `prefer_tensorrt_inspection:=false` to fall back to PyTorch

## Benchmarking

Run on the Jetson to measure inference performance:

```bash
cd ~/ugv_ws && python3 scripts/benchmark_vision.py
```

Results are written to `docs/vision_benchmark_results.md`. Target: < 10 ms avg for 10 Hz.

## Common Issues

| Symptom | Solution |
|---------|----------|
| Invalid class indices (28, 37, 39, 45, 47) | Node now forces `task='segment'` and auto-falls back to .pt; or use `prefer_tensorrt_inspection:=false` |
| NMS time limit exceeded | Default `wheel_max_det` is 50; reduce further (e.g. 30) if needed |
| OOM on startup | `model_load_delay_s:=10.0` or `use_cpu_inference:=true` |
| No detections | Check `interested_class_names` includes `wheel`; verify camera topic publishing |
