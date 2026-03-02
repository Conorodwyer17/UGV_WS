# Perception Stability Report

Audit of segmentation_3d: memory, voxel logic, clustering, outlier filtering, depth fusion, bounding box stabilization.

---

## 1. Memory Allocation Patterns

| Component | Pattern | Risk |
|-----------|---------|------|
| point_processor | PCL clouds per object; VoxelGrid output | Bounded by object count |
| segmentation_processor | latest_pointcloud_ shared_ptr; local_pointcloud copy | No unbounded growth |
| calculate_boxes | cloud_pcl per callback | Released after callback |
| EuclideanCluster | std::vector of clusters | Bounded by point count |

**Verdict:** No unbounded allocation in hot loops. Cluster count bounded by input points.

---

## 2. Voxel Leaf Default Logic

| Parameter | Default | Behavior |
|-----------|---------|----------|
| voxel_leaf_size | 0.0 | Adaptive (point_processor uses 0.02–0.05 when > 0) |
| point_processor | 0.0 = skip VoxelGrid | Reduces points when set |

**Verdict:** Configurable; 0.02–0.05 recommended for vehicles. No change needed.

---

## 3. Cluster Size Thresholds

| Threshold | Value | Purpose |
|-----------|-------|---------|
| min_valid_points | 5 | Vehicle/object |
| min_valid_points_wheel | 3 | Tire (sparse depth) |
| wheel_cluster_tolerance | 0.18 m | Euclidean clustering |

**Verdict:** Relaxed for wheels; appropriate. No adaptive threshold by range yet.

---

## 4. Outlier Filtering

| Method | Implementation |
|--------|----------------|
| Passthrough | z in [-5, 2] (point_processor) |
| max_detection_threshold | Not present in segment_3d |
| Median depth | aurora_semantic_fusion |

**Verdict:** Passthrough sufficient. Median in fusion; no additional outlier filter in segmentation_processor.

---

## 5. Depth Fusion Method

| Component | Method |
|-----------|--------|
| aurora_semantic_fusion | Median depth over mask |
| segmentation_processor | Pixel→3D via depth; cluster centroid |

**Verdict:** Robust; median over mask in fusion. Segmentation uses cluster centroid.

---

## 6. Bounding Box Stabilization (Temporal Smoothing)

| Check | Status |
|-------|--------|
| Sliding-window pose smoothing | **Absent** |
| Low-pass filter on centroid | **Absent** |
| Temporal smoothing | **Not implemented** |

**Verdict:** Centroid jitter possible. Optional enhancement: sliding-window or exponential moving average on tire pose. Low priority; goal_generator uses far-side placement and MIN_SAFE_OFFSET which absorb some jitter.

---

## 7. Recommended Enhancements

| Enhancement | Priority | Effort |
|-------------|----------|--------|
| Sliding-window pose smoothing | Low | Medium |
| Adaptive cluster threshold by range | Low | Low |
| Low-pass filter on final pose | Low | Low |
| pointcloud_max_age_s | **Done** | — |

---

## 8. Benchmarking Script (To Add)

```bash
# Measure detection latency (segmentation_processor)
ros2 topic echo /segmentation_processor/bounding_boxes_3d --field header.stamp

# 95th percentile: compare stamp delta from /ultralytics/segmentation/objects_segment
# Cluster count consistency: log cluster count per callback
```

**Metrics to capture:**
- Detection latency (segment stamp → bbox publish)
- 95th percentile latency
- Cluster count per frame (consistency)

---

## 9. Summary

- Memory: bounded; no hot-loop allocation risk.
- Voxel: configurable; defaults reasonable.
- Clustering: thresholds appropriate.
- Smoothing: absent; optional for future.
- pointcloud_max_age_s: added for depth dropout defense.
