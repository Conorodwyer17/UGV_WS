#!/usr/bin/env python3
"""
Benchmark YOLO inference for the tyre inspection pipeline.
Runs N iterations and records min/avg/max inference time and GPU memory.
Run on the Jetson: cd ~/ugv_ws && python3 scripts/benchmark_vision.py
Output: docs/vision_benchmark_latest.md (does not overwrite vision_benchmark_results.md)
"""
import argparse
import os
import sys
import time

UGV_WS = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
os.chdir(UGV_WS)
sys.path.insert(0, os.path.join(UGV_WS, "src", "Tyre_Inspection_Bot", "src", "amr_hardware", "src"))

# Shim torchvision before ultralytics (for Jetson torch without torchvision ABI)
try:
    import torch
    if not hasattr(torch.ops, "torchvision"):
        import types
        _tv_ops = types.SimpleNamespace()
        _tv = types.ModuleType("torchvision")
        _tv.ops = _tv_ops
        sys.modules["torchvision"] = _tv
        sys.modules["torchvision.ops"] = _tv_ops
except Exception:
    pass

def find_model():
    candidates = [
        os.path.join(UGV_WS, "src", "Tyre_Inspection_Bot", "best_fallback.engine"),
        os.path.join(UGV_WS, "src", "Tyre_Inspection_Bot", "best_fallback.pt"),
        os.path.join(UGV_WS, "best_fallback.engine"),
        os.path.join(UGV_WS, "best_fallback.pt"),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None


def find_tyre_only_model():
    """Dedicated single-class tyre weights (tyre_detection_project), if present."""
    candidates = [
        os.path.join(UGV_WS, "tyre_detection_project", "best.engine"),
        os.path.join(UGV_WS, "tyre_detection_project", "best.pt"),
        os.path.join(UGV_WS, "src", "Tyre_Inspection_Bot", "tyre_detection_project", "best.engine"),
        os.path.join(UGV_WS, "src", "Tyre_Inspection_Bot", "tyre_detection_project", "best.pt"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return None


def get_gpu_memory_mb():
    try:
        import torch
        if torch.cuda.is_available():
            return torch.cuda.memory_allocated() / (1024 * 1024)
    except Exception:
        pass
    return 0.0


def run_benchmark(model_path, imgsz, n_iter, device, YOLO, np):
    """Run warmup + timed loop; return timing stats and GPU memory (CUDA only)."""
    import torch

    model = YOLO(model_path)
    warmup = min(50, n_iter // 20)
    dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)

    if device.startswith("cuda") and torch.cuda.is_available():
        torch.cuda.reset_peak_memory_stats()
        torch.cuda.empty_cache()

    for _ in range(warmup):
        model.predict(dummy, imgsz=imgsz, device=device, verbose=False)

    times = []
    for _ in range(n_iter):
        t0 = time.perf_counter()
        model.predict(dummy, imgsz=imgsz, device=device, verbose=False)
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000)

    times_ms = sorted(times)
    min_ms = times_ms[0]
    max_ms = times_ms[-1]
    avg_ms = sum(times) / len(times)
    p50 = times_ms[len(times) // 2]
    p99 = times_ms[int(len(times) * 0.99)] if len(times) > 100 else max_ms

    gpu_alloc_mb = 0.0
    gpu_peak_mb = 0.0
    if device.startswith("cuda") and torch.cuda.is_available():
        gpu_alloc_mb = torch.cuda.memory_allocated() / (1024 * 1024)
        gpu_peak_mb = torch.cuda.max_memory_allocated() / (1024 * 1024)

    del model
    if device.startswith("cuda") and torch.cuda.is_available():
        torch.cuda.empty_cache()

    return {
        "min_ms": min_ms,
        "max_ms": max_ms,
        "avg_ms": avg_ms,
        "p50_ms": p50,
        "p99_ms": p99,
        "gpu_allocated_mb": gpu_alloc_mb,
        "gpu_peak_mb": gpu_peak_mb,
    }

def main():
    parser = argparse.ArgumentParser(description="Benchmark YOLO inference for tyre inspection")
    parser.add_argument("--model", default="", help="Primary model path (default: auto-detect best_fallback.pt/.engine)")
    parser.add_argument(
        "--compare",
        default="",
        metavar="PATH",
        help="Second model for side-by-side timing + GPU memory (e.g. tyre_detection_project/best.pt). "
        "Use --compare-auto to pick tyre_detection_project/best.pt|.engine if present.",
    )
    parser.add_argument(
        "--compare-auto",
        action="store_true",
        help="Compare primary model against dedicated single-class tyre weights (tyre_detection_project/best.*).",
    )
    parser.add_argument("--iterations", type=int, default=1000, help="Number of inference iterations")
    parser.add_argument("--input_size", type=int, default=640, help="Input image size (width=height)")
    args = parser.parse_args()

    model_path = args.model if args.model and os.path.isfile(args.model) else find_model()
    if not model_path:
        print("No best_fallback.pt or .engine found. Run export_tensorrt.sh on the Jetson first.")
        sys.exit(1)

    compare_path = ""
    if args.compare_auto:
        compare_path = find_tyre_only_model() or ""
        if not compare_path:
            print("--compare-auto: no tyre_detection_project/best.pt or best.engine found.")
            sys.exit(1)
    elif args.compare and os.path.isfile(args.compare):
        compare_path = args.compare

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics not installed. pip install ultralytics")
        sys.exit(1)

    device = "cuda:0" if __import__("torch").cuda.is_available() else "cpu"
    imgsz = args.input_size
    n_iter = args.iterations

    import numpy as np

    if compare_path:
        print(f"Compare mode: {model_path}\n          vs {compare_path}\n")
        stats_a = run_benchmark(model_path, imgsz, n_iter, device, YOLO, np)
        stats_b = run_benchmark(compare_path, imgsz, n_iter, device, YOLO, np)

        label_a = "Multi-class fallback (best_fallback)" if "fallback" in os.path.basename(model_path).lower() else os.path.basename(model_path)
        label_b = "Single-class tyre (tyre_detection_project)" if "tyre_detection" in compare_path.replace("\\", "/").lower() else os.path.basename(compare_path)

        out = f"""# Vision Pipeline Benchmark Results

Produced by `scripts/benchmark_vision.py --compare` on the target hardware.

## Setup

- **Input size:** {imgsz}×{imgsz}
- **Device:** {device}
- **Iterations per model:** {n_iter}

## Model A — `{model_path}`

| Metric | Value |
|--------|-------|
| Avg inference (ms) | {stats_a["avg_ms"]:.2f} |
| P99 (ms) | {stats_a["p99_ms"]:.2f} |
| GPU peak (MiB) | {stats_a["gpu_peak_mb"]:.1f} |
| GPU allocated after run (MiB) | {stats_a["gpu_allocated_mb"]:.1f} |

## Model B — `{compare_path}`

| Metric | Value |
|--------|-------|
| Avg inference (ms) | {stats_b["avg_ms"]:.2f} |
| P99 (ms) | {stats_b["p99_ms"]:.2f} |
| GPU peak (MiB) | {stats_b["gpu_peak_mb"]:.1f} |
| GPU allocated after run (MiB) | {stats_b["gpu_allocated_mb"]:.1f} |

## Delta (B − A)

| Metric | Delta |
|--------|-------|
| Avg inference (ms) | {stats_b["avg_ms"] - stats_a["avg_ms"]:+.2f} |
| GPU peak (MiB) | {stats_b["gpu_peak_mb"] - stats_a["gpu_peak_mb"]:+.1f} |

## Notes

- **GPU memory** is for this process only (YOLO + CUDA context), not full ROS + Aurora + Nav2. Use `tegrastats` for **system RAM** under full bringup.
- Multi-class `best_fallback` stresses validation with many car-part classes; the dedicated **single-class tyre** model is the intended production path to reduce work per frame and avoid worst-case TensorRT configs.
- Same `imgsz` for both runs; align with your launch `wheel_imgsz` for fair comparison.

"""
        print(out)
        out_path = os.path.join(UGV_WS, "docs", "vision_benchmark_latest.md")
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w") as f:
            f.write(out)
        print(f"Results written to {out_path}")
        return

    print(f"Loading model: {model_path}")
    stats = run_benchmark(model_path, imgsz, n_iter, device, YOLO, np)
    min_ms = stats["min_ms"]
    max_ms = stats["max_ms"]
    avg_ms = stats["avg_ms"]
    p50 = stats["p50_ms"]
    p99 = stats["p99_ms"]
    gpu_mb = stats["gpu_allocated_mb"]
    gpu_peak = stats["gpu_peak_mb"]

    out = f"""# Vision Pipeline Benchmark Results

Produced by `scripts/benchmark_vision.py` on the target hardware.

## Model

- **Path:** `{model_path}`
- **Input size:** {imgsz}x{imgsz}
- **Device:** {device}

## Inference Time (ms)

| Metric | Value |
|--------|-------|
| Min | {min_ms:.2f} |
| Avg | {avg_ms:.2f} |
| Max | {max_ms:.2f} |
| P50 | {p50:.2f} |
| P99 | {p99:.2f} |

## GPU Memory (CUDA)

- **Peak during benchmark:** {gpu_peak:.1f} MiB
- **Allocated after run:** {gpu_mb:.1f} MiB

## Notes

- Target: < 10 ms avg for real-time inference at 10 Hz.
- TensorRT engine typically 2–3x faster than PyTorch (.pt) on Jetson.
- Run `scripts/export_tensorrt.sh` on the Jetson to generate the engine.
- Compare with your single-class tyre weights: `python3 scripts/benchmark_vision.py --compare-auto`
"""

    out_path = os.path.join(UGV_WS, "docs", "vision_benchmark_latest.md")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(out)
    print(f"Results written to {out_path}")
    print(out)

if __name__ == "__main__":
    main()
