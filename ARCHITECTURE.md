# System Architecture – Autonomous Tyre Inspection Robot

This document describes the overall architecture of the UGV tyre inspection system: hardware, ROS 2 node graph, TF tree, and data flow.

## 1. Hardware Overview

| Component | Description |
|-----------|-------------|
| **Robot base** | Waveshare UGV Rover (differential drive) |
| **Onboard computer** | NVIDIA Jetson Orin Nano (16 GB) |
| **Primary sensor** | SLAMTEC Aurora 6DOF – integrated LiDAR, stereo depth, IMU, semantic segmentation |
| **Motor interface** | ESP32 via UART (`/dev/ttyTHS1`) for cmd_vel and wheel odometry feedback |

The Aurora provides 6DOF localisation, 2D LiDAR scan, depth images, and built-in COCO80 semantic segmentation (vehicles: car, truck, bus). Tyre detection uses a separate YOLO model (`best_fallback.pt` / TensorRT engine).

## 2. ROS 2 Node Graph (Key Nodes)

```
                    ┌─────────────────────────────────┐
                    │  slamware_ros_sdk_server_node   │
                    │  (Aurora SDK: odom, scan, map,  │
                    │   depth, semantic, images)      │
                    └───────────────┬─────────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
┌───────────────┐    ┌─────────────────────────┐    ┌─────────────────┐
│ aurora_       │    │ aurora_semantic_fusion   │    │ depth_to_       │
│ sdk_bridge    │    │ (vehicle boxes from      │    │ registered_     │
│ (optional)    │    │  semantic + depth)       │    │ pointcloud      │
└───────────────┘    └─────────────┬─────────────┘    └────────┬────────┘
                                  │                           │
                                  │    ┌──────────────────────┘
                                  │    │
                                  ▼    ▼
                    ┌─────────────────────────────────┐
                    │  ultralytics_tire (YOLO)         │
                    │  segmentation_processor_tire     │
                    │  tire_merger → /tire_bounding_   │
                    │  boxes_merged                    │
                    └───────────────┬─────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────┐
                    │  inspection_manager_node         │
                    │  (state machine: SEARCH →        │
                    │   APPROACH → INSPECT → CAPTURE)  │
                    └───────────────┬─────────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
        ▼                           ▼                           ▼
┌───────────────┐    ┌─────────────────────────┐    ┌─────────────────┐
│ Nav2          │    │ photo_capture_service    │    │ motor_driver    │
│ (controller,  │    │ (capture tyre photos)   │    │ (cmd_vel →     │
│ planner, bt)  │    └─────────────────────────┘    │  ESP32)         │
└───────────────┘                                   └─────────────────┘
```

**Simulation mode:** Replace `slamware_ros_sdk_server_node` with `aurora_mock_node` (synthetic odom, scan, map, images). Use `ros2 launch sim vehicle_inspection_sim.launch.py use_mock:=true`.

## 3. TF Tree

```
map (Nav2 global frame)
 └── slamware_map (Aurora map frame; identity with map)
      └── odom (Aurora odom)
           └── base_link (robot base)
                ├── base_footprint
                ├── camera_left
                │    ├── camera_right
                │    ├── camera_depth_optical_frame
                │    └── imu_link
                └── laser (LiDAR frame)
```

All 3D bounding boxes (vehicles, tyres) are published in `slamware_map`. Navigation goals are computed in `slamware_map` and sent to Nav2 as `map` (identity transform).

## 4. Data Flow

1. **Vehicle detection:** Aurora semantic segmentation + depth → `aurora_semantic_fusion_node` → `/aurora_semantic/vehicle_bounding_boxes`
2. **Tyre detection:** Aurora left image → `ultralytics_tire` (YOLO) → `/darknet_ros_3d/tire_bounding_boxes`; merged with PCL fallback → `/tire_bounding_boxes_merged`
3. **Mission logic:** `inspection_manager_node` subscribes to vehicle and tyre boxes, dispatches Nav2 goals (approach vehicle, then each tyre), triggers photo capture via `/photo_capture_service/capture_photo`
4. **Navigation:** Nav2 receives goals in `map` frame; costmap uses `/slamware_ros_sdk_server_node/scan` and `/camera/depth/points`; `cmd_vel` flows through `cmd_vel_mux` (nav vs centroid_servo) and `depth_gate` (navigation_permitted) to `motor_driver`

## 5. Key Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/aurora_semantic/vehicle_bounding_boxes` | BoundingBoxes3d | aurora_semantic_fusion | inspection_manager |
| `/tire_bounding_boxes_merged` | BoundingBoxes3d | tire_merger | inspection_manager |
| `/cmd_vel` | Twist | depth_gate | motor_driver |
| `/slamware_ros_sdk_server_node/odom` | Odometry | Aurora SDK | Nav2, EKF |
| `/slamware_ros_sdk_server_node/scan` | LaserScan | Aurora SDK | Nav2 costmap |
| `/slamware_ros_sdk_server_node/map` | OccupancyGrid | Aurora SDK | Nav2 |

## 6. Configuration

- **PRODUCTION_CONFIG.yaml** – Mission parameters (timeouts, distances, paths)
- **ugv_nav/param/** – Nav2 parameters (controller, costmap, recovery)
- **segmentation_3d/config/** – YOLO and segmentation parameters

See [RUNBOOK.md](RUNBOOK.md) for operational procedures and [SETUP.md](SETUP.md) for installation.
