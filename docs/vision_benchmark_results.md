# Vision pipeline benchmark results

**Machine-generated runs** are written to [`vision_benchmark_latest.md`](vision_benchmark_latest.md) when you run `scripts/benchmark_vision.py` (single model or `--compare`). This file can hold **notes and archived tables** without being overwritten.

| Command | Output file |
|--------|-------------|
| `python3 scripts/benchmark_vision.py` | `docs/vision_benchmark_latest.md` — timing + GPU peak/allocated |
| `python3 scripts/benchmark_vision.py --compare-auto --iterations 200 --input_size 480` | `best_fallback` vs `tyre_detection_project/best.*` |
| Explicit paths | `--model …/best_fallback.pt --compare …/tyre_detection_project/best.pt` |

GPU numbers in the latest run are **YOLO-only** (CUDA for that process). Full-stack **system RAM** needs `tegrastats` during `full_bringup`.

---

## Archived snapshot — Jetson Orin Nano 16 GB (single model)

**Date:** 2026-03-05  
**Hardware:** Jetson Orin (nvgpu), CUDA 12.6, Driver 540.4.0  
**Model:** `best_fallback.engine` (TensorRT)  
**Input size:** 640×640  
**Device:** cuda:0

### Inference time (ms)

| Metric | Value |
|--------|-------|
| Min | 29.50 |
| Avg | 33.31 |
| Max | 39.75 |
| P50 | 33.24 |
| P99 | 39.62 |

### GPU memory

- **Allocated (after run):** 9.7 MiB (approximate; older benchmark path)

### Observations

- First inference slower due to TensorRT engine load (~2 s).
- Steady-state ~33 ms avg yields ~30 Hz; sufficient for 10 Hz inspection.
- Target < 10 ms would require smaller input (e.g. 416) or a lighter model.

### Notes

- Export TensorRT on-device: `scripts/export_tensorrt.sh`.
- For faster inference, try `--input_size 416` or `wheel_imgsz:=416` in launch.
