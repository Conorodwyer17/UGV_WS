# Concurrency and Executor Strategy Review

Current executor model, node threading, risk of race conditions.

---

## 1. Current Executor Model

| Node | Executor | Entry Point |
|------|----------|-------------|
| inspection_manager | Default (rclpy.spin) | Single-threaded |
| segmentation_processor | rclcpp::spin(node) | Single-threaded |
| photo_capture_service | rclpy.spin | Single-threaded |

Launch: Nodes typically in separate processes. Each process has its own executor. No explicit MultiThreadedExecutor in Python.

---

## 2. Node Threading Model

| Component | Callbacks | Thread |
|-----------|-----------|--------|
| inspection_manager | _tick, _on_box_result, vehicle/tire boxes | Same (single-threaded) |
| segmentation_processor | segmentationCallback, pointCloud_sub | Same |

All callbacks run on the same thread. No concurrent access to shared state within a node.

---

## 3. Risk of Race Conditions

| Scenario | Risk | Mitigation |
|----------|------|------------|
| Nav2 result + _tick | Low | Same executor; sequential |
| Action done callback + _tick | Low | Same executor |
| segmentationCallback blocking | Medium | Separate node; no shared state with inspection_manager |

Verdict: No refactor required. Single-threaded model sufficient.
