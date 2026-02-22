# Phase H — Closed-Loop Vehicle Approach Test Procedure

## Prerequisites

- Aurora + slamware + aurora_sdk_bridge running
- Nav2 + depth_gate + inspection_manager
- Real car or large box in environment

## Procedure

1. **Place vehicle** in Aurora view (indoor/outdoor with good lighting).

2. **Launch stack:**
   ```bash
   ros2 launch ugv_nav aurora_bringup.launch.py   # or aurora_testing + nav + inspection
   ros2 launch ugv_nav nav_aurora.launch.py
   ros2 launch inspection_manager inspection_manager.launch.py
   ros2 launch segmentation_3d segment_3d.launch.py
   ```

3. **Detect vehicle** — confirm `/darknet_ros_3d/bounding_boxes` or ultralytics outputs 2D boxes.

4. **Perception → navigation:**
   - Tire pixels → 3D via point cloud + TF to slamware_map
   - Compute approach pose (0.5–1.0 m standoff)
   - Send Nav2 goal via nav2_simple_commander or inspection_manager

5. **Stopping & capture:**
   - On arrival: verify depth confirms standoff
   - Halt and trigger photo_capture_service
   - Confirm `/inspection_manager/capture_result` SUCCESS with file path

6. **Repeat** for front-left, front-right, rear-left, rear-right.

## Acceptance

- 3 consecutive full approach + capture cycles without manual intervention.
