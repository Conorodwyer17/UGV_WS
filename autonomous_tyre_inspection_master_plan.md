# Autonomous Tyre Inspection — Master Plan

**Canonical blueprint** for a deterministic, failure-resistant, autonomous tyre inspection robot. All architectural decisions are justified by patterns extracted from research repos. No experimental shortcuts; emphasis on stability, debuggability, reproducibility, deterministic execution, and safety-first navigation.

---

## 1. Final Node Graph

### 1.1 Nodes

| Node | Package | Role |
|------|---------|------|
| slamware_ros_sdk_server_node | Aurora SDK | SLAM, map, odom, scan, point_cloud, depth, semantic, TF |
| ultralytics_node | segmentation_3d | YOLO segmentation → ObjectsSegment |
| segmentation_processor | segmentation_3d | ObjectsSegment + PointCloud2 → BoundingBoxes3d (tires) |
| aurora_semantic_fusion | segmentation_3d | semantic + depth → BoundingBoxes3d (vehicles) |
| inspection_manager | inspection_manager | Mission state machine, goals, photo capture |
| controller_server | nav2_controller | Nav2 controller |
| planner_server | nav2_planner | Nav2 planner |
| bt_navigator | nav2_bt_navigator | Behavior tree |
| waypoint_follower | nav2_waypoint_follower | FollowWaypoints |
| local_costmap | nav2_costmap_2d | Rolling window costmap |
| global_costmap | nav2_costmap_2d | Static + obstacle costmap |
| velocity_smoother | nav2_velocity_smoother | cmd_vel smoothing |
| ugv_base_driver | ugv_base_driver | cmd_vel → ESP32 UART |
| photo_capture_service | inspection_manager | Image capture with metadata |

### 1.2 Topics (Key)

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| /slamware_ros_sdk_server_node/scan | LaserScan | Aurora | costmap |
| /slamware_ros_sdk_server_node/point_cloud | PointCloud2 | Aurora | costmap |
| /slamware_ros_sdk_server_node/odom | Odometry | Aurora | Nav2 |
| /camera/depth/points | PointCloud2 | depth_to_registered | costmap |
| /darknet_ros_3d/vehicle_bounding_boxes | BoundingBoxes3d | aurora_semantic / segment_3d | inspection_manager |
| /darknet_ros_3d/tire_bounding_boxes | BoundingBoxes3d | segmentation_processor | inspection_manager |
| /cmd_vel | Twist | velocity_smoother | ugv_base_driver |
| /follow_waypoints | FollowWaypoints (action) | — | waypoint_follower |

### 1.3 Frames

| Frame | Source |
|-------|--------|
| map | Aurora (or identity from slamware_map) |
| slamware_map | Aurora |
| odom | Aurora |
| base_link / base_footprint | Aurora / robot_state_publisher |
| camera_depth_optical_frame | Aurora |
| laser | Aurora |

### 1.4 Data Flow

```
Aurora (SLAM, odom, map, scan, point_cloud, depth, semantic)
  ├── Nav2 (costmap, planner, controller) → cmd_vel → ugv_base_driver → ESP32
  ├── ultralytics_node → ObjectsSegment
  ├── segmentation_processor (ObjectsSegment + PointCloud2) → tire BoundingBoxes3d
  └── aurora_semantic_fusion (semantic + depth) → vehicle BoundingBoxes3d

inspection_manager (vehicle + tire boxes)
  → goals (PoseStamped)
  → follow_waypoints (4 poses) or navigate_to_pose (sequential)
  → photo_capture_service (on arrival)
```

### 1.5 Failure Flow

```
Nav2 FAILED → stop_on_failure: false → skip to next waypoint
approach_timeout → cancel → retry or skip tire
max_state_repeats → ERROR
TF invalid → reject goal
detection stale → reject goal
```

---

## 2. Ideal Nav2 Configuration

### 2.1 Costmap Layering

- **Local:** `global_frame: odom`, `rolling_window: true`, `origin_x`, `origin_y` = width/2, height/2
- **Plugins:** obstacle_layer, inflation_layer (no voxel on local; Nav2 #4299)
- **Observation sources:** scan, point_cloud, aurora_point_cloud
- **obstacle_max_range:** scan 2.5 m, point_cloud 2.5 m, aurora 4.0 m

### 2.2 transform_tolerance Philosophy

- **0.5–1.0 s** on costmaps, controller, behavior_server, recoveries_server
- Aurora TF timing; avoids extrapolation into future
- Consistent across all Nav2 components

### 2.3 Controller Frequency

- **10 Hz** when CPU contended (YOLO, segmentation)
- **20 Hz** if headroom; balance with bt_loop_duration

### 2.4 Obstacle Inflation

- **Global:** 0.35 m (allows gaps; IEEE obstacle-inflation-free)
- **Local:** 0.6 m (safety near vehicle)
- Do not reduce below robot half-width

### 2.5 Voxel vs Obstacle Layer

- **Local:** obstacle_layer only (no voxel; avoids sensor origin out of bounds)
- **Global:** obstacle_layer + static_layer

### 2.6 Map Frame with SLAMTEC Aurora

- Aurora publishes `/map` or `slamware_map`; no map_server
- Static identity `map` → `slamware_map` if needed
- Nav2 `global_frame: map`

### 2.7 Deterministic Goal Tolerances

- **xy_goal_tolerance:** 0.25 m (coarse tyre staging)
- **yaw_goal_tolerance:** 0.25 m (reduce oscillation)
- **stateful:** True

---

## 3. Ideal Perception Pipeline

### 3.1 Voxel Leaf Sizes

- **0.02–0.05 m** for vehicle-sized objects (Autoware, l2i)
- Adaptive: max_extent/80, clamped; min 0.05 for large clouds
- Passthrough: z > 2 m, range > 10 m before voxel

### 3.2 Clustering Strategy

- VoxelGrid → plane segmentation (skip for wheels) → EuclideanClusterExtraction
- cluster_tolerance: 0.1 (vehicle), relaxed for wheels
- min_cluster_size: 50 (vehicle), 1 (wheel fallback)

### 3.3 2D→3D Fusion Methodology

- **Mask-based:** use all pixels in mask (not single centroid)
- **Median depth** over valid mask pixels (aurora_semantic_fusion)
- **Centroid averaging** (sum/count) as alternative (l2i)

### 3.4 Median Depth Fallback Logic

- Valid depth: median of mask pixels in [min_depth, max_depth]
- Fallback: 25th percentile if median invalid
- Final fallback: configured fallback_depth_m (e.g. 4.0)

### 3.5 Outlier Rejection

- maximum_detection_threshold on z (not x)
- Reject NaN, degenerate boxes
- min_valid_points, min_valid_points_wheel

### 3.6 Goal Pose Smoothing

- Committed plan locks one vehicle/tire set
- No goal flooding; nav_goal_min_interval_s

### 3.7 Vehicle Orientation Inference

- _infer_orientation from robot position
- vehicle_modeler uses robot direction for front/rear

### 3.8 Tyre Pose Inference Algorithm

- Planned tire order from vehicle geometry (wheelbase, track)
- 2nd→3rd→4th nearest; strict_planned_tire_order

---

## 4. Mission Execution Strategy

### 4.1 Behavior Tree vs State Machine

- **Waypoint follower:** "dumb robot" — one unit of work (4 tyres); sufficient for fleet-style
- **Custom BT:** "smart robot" — more complex; nav2_bt_waypoint_follower future
- **Current:** inspection_manager state machine + Nav2; proven hybrid

### 4.2 Retry Logic Rules

- approach_timeout_s → cancel → retry (dispatch_retry_count)
- max_dispatch_retries_before_recover → rotate → retry
- capture_verify_timeout_s, max_capture_retries

### 4.3 Skip Rules

- stop_on_failure: false → skip unreachable tyre
- missed_waypoints in result
- tire_search_timeout_s → skip tire

### 4.4 Vehicle Re-detection Strategy

- TURN_IN_PLACE_SEARCH, TURN_IN_PLACE_VEHICLE, TURN_IN_PLACE_TIRE
- PATROL_SEARCH when no vehicle
- detection_timeout → rotate or next

### 4.5 Timeout Strategy

- Every WAIT state has timeout
- approach_timeout_s, face_tire_timeout_s, capture_wheel_wait_timeout_s

### 4.6 Pose Confidence Gating

- detection_stamp_max_age_s
- require_goal_transform
- _is_finite_box

### 4.7 Deterministic 4-Tyre Ordering

- First: nearest (approach_nearest_corner)
- Then: 2nd→3rd→4th nearest (strict_planned_tire_order)

### 4.8 Unreachable Tyre

- stop_on_failure: false → continue to next
- Log in missed_waypoints
- Mission completes with partial success

---

## 5. Navigation Determinism Strategy

### 5.1 Prevent Oscillation

- xy_goal_tolerance, yaw_goal_tolerance
- RotationShimController: rotate in place before path

### 5.2 Avoid Partial Successes

- goal_checker stateful
- Verify result before processAtWaypoint

### 5.3 Consistent Stopping Radius

- MIN_SAFE_OFFSET_M (vehicle), MIN_SAFE_OFFSET_TIRE_M
- Far-side goal placement

### 5.4 Alignment Before Photo

- capture_require_wheel_detection
- FACE_TIRE state before capture

### 5.5 Prevent Goal Inside Obstacle

- Far-side placement (vehicle_center → tire_center)
- _is_finite_box

### 5.6 Prevent Frame Jumps

- TF validation
- Consistent frame IDs

---

## 6. Hardening the System Against Real-World Failures

### 6.1 Dropped TF Frames

- **Pattern:** transform_tolerance 0.5–1.0 s (Nav2, PATH_FORWARD)
- **Action:** Costmap/controller accept slightly stale TF
- **Fallback:** transformer.py lookup_transform_at_stamp falls back to latest on future-timestamp; check_tf_validity fallback_to_latest
- **Detection:** TF watchdog; log when frames missing

### 6.2 Delayed Depth Images

- **Pattern:** message_filters sync (l2i); Aurora fused
- **Action:** Use latest point cloud in segmentation_processor
- **Fallback:** aurora_semantic_fusion fallback_depth when no valid depth
- **Detection:** pointcloud_max_age_s (reject stale cloud); depth_stale_s, semantic_stale_s

### 6.3 Moving Vehicles

- **Pattern:** Committed plan; costmap updates (VEHICLE_INSPECTION_FAILURE_MODES)
- **Action:** Costmap sees obstacle; box is stale; trust costmap for obstacles
- **Limitation:** No explicit handling; mission assumes static

### 6.4 Partial Occlusion

- **Pattern:** Median depth over mask (aurora_semantic_fusion)
- **Action:** Valid pixels in mask contribute; median robust
- **Fallback:** fallback_depth when all invalid

### 6.5 Costmap Inflation Spikes

- **Pattern:** Small inflation (0.35 global, 0.6 local)
- **Action:** Avoid over-inflation near vehicle
- **Detection:** Goal rejected; retry with larger offset

### 6.6 Localization Drift

- **Pattern:** Aurora SLAM; visual + LiDAR fusion
- **Action:** No AMCL; Aurora is single source
- **Detection:** Monitor odom rate; map continuity

### 6.7 Mission Logic Deadlocks

- **Pattern:** max_state_repeats; spin protection (mission_state_machine)
- **Action:** Transition to ERROR after N repeats
- **Detection:** Log state transitions; identify cycle

### 6.8 Auto-Recovery Logic

- **Pattern:** Nav2 recovery (backup, wait); behavior tree
- **Action:** Recovery plugins in BT
- **Pattern:** approach_timeout → cancel → retry/skip
- **Action:** inspection_manager handles timeout

---

## 7. Production vs Research Demarcation

| Aspect | Research/Demo | Production |
|--------|---------------|------------|
| Depth | Single centroid | Median over mask |
| Clustering | Full cloud | VoxelGrid first |
| Failure | Abort | Skip on failure |
| Goal | Any | Validated, far-side |
| TF | Best effort | Validated, tolerance |
| Mission | Single run | Deterministic order |

---

## 8. References (Extracted From)

- Nav2 waypoint_follower, costmap_2d, rotation_shim, collision_monitor
- Nav2 #4299, ROS Answers 416578, 334607
- PATH_FORWARD_NAVIGATION_AND_PERCEPTION.md
- VEHICLE_INSPECTION_FAILURE_MODES.md
- l2i_fusion_detection, gb_visual_detection_3d
- acc-qcar2-autonomy, unitree-go2-slam-nav2
- Autoware Euclidean Clustering, IEEE 10606581
