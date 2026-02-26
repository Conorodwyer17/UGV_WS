# System Architecture — Autonomous Tire Inspection Robot

## Overview

Production-grade autonomous tire inspection system using Slamtec Aurora firmware 2.11 (SLAM + LiDAR + native depth + semantic segmentation + IMU), Waveshare Rover base (ESP32), and NVIDIA Jetson.

---

## 1. Node Graph (Aurora 2.11 Native)

```
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│                    AURORA (slamware_ros_sdk_server_node) — Firmware 2.11                           │
│  Publishes (all /slamware_ros_sdk_server_node/<name>): scan, odom, map, left_image_raw,            │
│            right_image_raw, depth_image_raw (224×416), semantic_segmentation, semantic_labels       │
│  TF: slamware_map→odom→base_link, camera_left, camera_right, laser, imu_link                      │
└────────────────────┬──────────────────────────────────────────────────────────────────────────────┘
                     │
     ┌───────────────┼───────────────┬──────────────────────┬─────────────────────────────┐
     │               │               │                      │                             │
     ▼               ▼               ▼                      ▼                             ▼
┌─────────┐   ┌──────────────┐ ┌─────────────────┐  ┌──────────────────┐  ┌──────────────────────────┐
│  Nav2   │   │ depth_gate   │ │ aurora_depth_   │  │ aurora_semantic_ │  │ ultralytics_node         │
│ Stack   │   │ cmd_vel_nav  │ │ camera_info     │  │ fusion           │  │ (YOLO: vehicles + tires) │
│         │   │ → cmd_vel    │ │ CameraInfo      │  │ semantic+depth   │  │ /ultralytics/objects_    │
│ goals   │   │ (gated by    │ │ 416×224         │  │ → vehicle boxes  │  │ segment                  │
│         │   │ nav_permitted)│ └────────┬────────┘  └────────┬─────────┘  └──────────┬───────────────┘
└────┬────┘   └──────┬───────┘          │                    │                       │
     │               │                  ▼                    │                       │
     │               │         ┌──────────────────────┐      │                       │
     │               │         │ depth_to_registered_ │      │                       │
     │               │         │ pointcloud           │      │                       │
     │               │         │ depth + camera_info  │      │                       │
     │               │         │ → registered_pointcloud     │                       │
     │               │         └──────────┬───────────┘      │                       │
     │               │                    └──────────────────┼───────────────────────┘
     │               │                                       ▼
     │               │                           ┌─────────────────────┐
     │               │                           │ segmentation_       │
     │               │                           │ processor_node      │
     │               │                           │ YOLO + pointcloud   │
     │               │                           │ → BoundingBoxes3d   │
     │               │                           │   (tires)           │
     │               │                           └──────────┬──────────┘
     │               │                                      │
     │               ▼                                      │
     │        ┌──────────────┐                              │
     │        │ ugv_base_    │     ┌────────────────────────┴──────────────────────┐
     │        │ driver       │     │ inspection_manager                             │
     │        │ cmd_vel →    │     │ Vehicles: /aurora_semantic/vehicle_bounding_boxes
     │        │ ESP32 UART   │     │ Tires: /darknet_ros_3d/bounding_boxes          │
     │        └──────────────┘     │ State machine, Nav2 goals, capture_photo       │
     │                             └────────────────────────┬──────────────────────┘
     │                                                      │
     │                                                      ▼
     │                                             ┌─────────────────────┐
     └────────────────────────────────────────────►│ photo_capture_      │
                                                   │ service             │
                                                   │ Saves from left cam │
                                                   └─────────────────────┘
```

**Legacy (use_bridge=true, firmware 1.2):** aurora_sdk_bridge produces stereo depth; navigation_permitted from bridge. Default is use_bridge=false (Aurora 2.11 native).

---

## 2. Topic Map (Aurora 2.11 Native)

| Topic | Message Type | Publisher | Subscriber |
|-------|--------------|-----------|------------|
| `/slamware_ros_sdk_server_node/scan` | LaserScan | Aurora SDK | Nav2 costmaps |
| `/slamware_ros_sdk_server_node/odom` | Odometry | Aurora SDK | Nav2 |
| `/slamware_ros_sdk_server_node/map` | OccupancyGrid | Aurora SDK | Nav2 (remapped as /map) |
| `/slamware_ros_sdk_server_node/depth_image_raw` | Image (224×416, 32FC1) | Aurora SDK | aurora_semantic_fusion, depth_to_registered_pointcloud |
| `/slamware_ros_sdk_server_node/semantic_labels` | Image (480×640, mono8) | Aurora SDK | aurora_semantic_fusion |
| `/slamware_ros_sdk_server_node/left_image_raw` | Image | Aurora SDK | ultralytics_node, photo_capture_service |
| `/camera/depth/camera_info` | CameraInfo (416×224) | aurora_depth_camera_info | depth_to_registered_pointcloud |
| `/camera/depth/points` | PointCloud2 | depth_to_registered_pointcloud | Nav2 costmaps |
| `/aurora_semantic/vehicle_bounding_boxes` | BoundingBoxes3d | aurora_semantic_fusion | inspection_manager |
| `/segmentation_processor/registered_pointcloud` | PointCloud2 | depth_to_registered_pointcloud | segmentation_processor |
| `/ultralytics/segmentation/objects_segment` | ObjectsSegment | ultralytics_node | segmentation_processor |
| `/darknet_ros_3d/bounding_boxes` | BoundingBoxes3d | segmentation_processor | inspection_manager |
| `cmd_vel_nav` | Twist | Nav2 | depth_gate |
| `/cmd_vel` | Twist | depth_gate | ugv_base_driver |
| `/segmentation_mode` | String | inspection_manager | ultralytics_node |
| `/inspection_manager/capture_photo` | Bool | inspection_manager | photo_capture_service |
| `navigate_to_pose` | Action | inspection_manager | Nav2 |

---

## 3. TF Tree

```
slamware_map (Aurora global frame)
    └── odom (identity from aurora_bringup static_transform)
            └── base_link (from Aurora robot_pose)
                    └── base_footprint (identity from nav_aurora)
                    └── camera_left
                            └── camera_right
                            └── imu_link
                            └── camera_depth_optical_frame (identity; depth frame)
                    └── laser

map (Nav2 global_frame)
    └── slamware_map (identity from nav_aurora)
```

---

## 4. Depth & Semantic Pipeline (Aurora 2.11)

| Component | Resolution | Purpose |
|-----------|------------|---------|
| `/slamware_ros_sdk_server_node/depth_image_raw` | 224×416 | Native depth from device |
| semantic_labels | 480×640 | COCO80 class IDs (mono8) |
| aurora_depth_camera_info | 416×224 | Intrinsics for depth |
| aurora_semantic_fusion | — | Resize semantic→depth, fuse centroid+depth→3D vehicle boxes |

**Vehicle labels (COCO80):** bicycle(2), car(3), motorcycle(4), bus(6), truck(8).

---

## 5. Detection Split

| Source | Classes | Topic | Used For |
|--------|---------|-------|----------|
| aurora_semantic_fusion | car, truck, bus, motorcycle, bicycle | /aurora_semantic/vehicle_bounding_boxes | Vehicle detection (SEARCH_VEHICLE, APPROACH_VEHICLE) |
| segmentation_processor | car-tire | /darknet_ros_3d/bounding_boxes | Tire detection (WAIT_TIRE_BOX, INSPECT_TIRE) |

---

## 6. Node Execution Order

| Order | Component | Depends On |
|-------|-----------|------------|
| 1 | Aurora SDK (aurora_bringup) | Network to Aurora @ 192.168.11.1 |
| 2 | depth_gate, ugv_base_driver | Nav2, navigation_permitted_publisher |
| 3 | Nav2 (nav_aurora) | Aurora (map, odom, scan), TF |
| 4 | segment_3d (aurora_depth_camera_info, aurora_semantic_fusion, depth pipeline, ultralytics, segmentation_processor) | Aurora (depth, semantic, images) |
| 5 | inspection_manager | Nav2, vehicle_boxes_topic, detection_topic |
| 6 | photo_capture_service | Aurora camera, inspection_manager |

---

## 7. Configuration

- **PRODUCTION_CONFIG.yaml** — Loaded by inspection_manager when in workspace. Sets vehicle_boxes_topic for semantic vehicles, use_bridge=false.
- **aurora_depth_intrinsics.yaml** — Depth 416×224 intrinsics; used by aurora_depth_camera_info and aurora_semantic_fusion.
- **best.pt** — Tire detection model; place at ~/ugv_ws/best.pt or src/Tyre_Inspection_Bot/best.pt.
