#!/usr/bin/env python3
"""
Print class names (and IDs) from the inspection YOLO model (e.g. best_fallback.pt).
Use this to confirm the exact tire/car class names for config (tire_label, interested_classes).

Example:
  ros2 run segmentation_3d print_inspection_model_classes
  ros2 run segmentation_3d print_inspection_model_classes --ros-args -p inspection_model:=/path/to/best_fallback.pt
"""

import os
import sys

try:
    from ultralytics import YOLO
except ImportError:
    print("ultralytics not installed. pip install ultralytics", file=sys.stderr)
    sys.exit(1)


def main():
    model_path = os.path.expanduser(
        os.environ.get("INSPECTION_MODEL", "~/ugv_ws/src/Tyre_Inspection_Bot/best_fallback.pt")
    )
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        model_path = sys.argv[1]
    if not os.path.exists(model_path):
        print(f"Model not found: {model_path}", file=sys.stderr)
        sys.exit(1)
    model = YOLO(model_path)
    names = getattr(model, "names", None)
    if names is None:
        print("No 'names' on model.", file=sys.stderr)
        sys.exit(1)
    # names is dict id -> name
    print(f"Model: {model_path}")
    print("Class names (exact as in model):")
    for idx, name in sorted(names.items(), key=lambda x: x[0]):
        marker = " <- tire_label" if name == "wheel" else ""
        print(f"  id={idx}  name={repr(name)}{marker}")
    print("\nFor tire inspection: tire_label=wheel (id=22). Full reference: docs/BEST_FALLBACK_MODEL_CLASSES.md")


if __name__ == "__main__":
    main()
