# Mission Pipeline: Approach Vehicle → Tire Inspection → Photo

**Canonical flow for inspection_manager_node.** When you run the mission, this is exactly what happens.

---

## Overview

1. **Phase 1 — Approach vehicle:** Detect car at distance (vehicle bounding box), navigate to approach pose (offset in front of vehicle).
2. **Phase 2 — Tire inspection:** When close, switch to tire inspection (best_fallback.pt / wheel class). Derive tire positions from vehicle box (FL, FR, RL, RR). Navigate to each tire, face it, take photo when wheel detected.

**Key invariant:** Do *not* look for tires at distance—they are too small. Approach the vehicle first, then use wheel detection or planned fallback.

---

## Phase 1: Approach Vehicle

| State | Action |
|-------|--------|
| **SEARCH_VEHICLE** | Search for car (yolov8m-seg or Aurora semantic). Patrol or rotate if none found. |
| **WAIT_VEHICLE_BOX** | Wait for vehicle box; when confirmed, dispatch approach goal. |
| **APPROACH_VEHICLE** / **INSPECT_TIRE** | Navigate to goal. First goal = **nearest corner** (tire); goal = nearest point on box + offset (no drive through vehicle). On success → WAIT_TIRE_BOX or next tire. |

### Car Detection (at Distance)

| Step | Component | Topic | Notes |
|------|-----------|-------|-------|
| 1a | **ultralytics_vehicle** | yolov8m-seg.pt (COCO: car, truck, bus) | fixed_mode: navigation |
| 1b | **segmentation_processor_vehicle** | `/darknet_ros_3d/vehicle_bounding_boxes` | 3D boxes from YOLO + depth |
| 1c | **aurora_semantic_fusion** | `/aurora_semantic/vehicle_bounding_boxes` | Aurora semantic (primary when configured) |
| 1d | **inspection_manager** | Subscribes to vehicle_boxes_topic or vehicle_detection_topic | vehicle_boxes_topic = Aurora primary |

**Result:** Vehicle box in slamware_map. With `approach_nearest_corner: true`, first goal = **nearest tire corner** (distance-sorted). Otherwise goal = nearest point on box + approach_offset (never through vehicle). See [MISSION_TIRE_ORDER_AND_SCENARIO.md](MISSION_TIRE_ORDER_AND_SCENARIO.md).

---

## Phase 2: Tire Inspection (When Close)

| State | Action |
|-------|--------|
| **WAIT_TIRE_BOX** | Switch to inspection mode (best_fallback.pt / wheel). Wait for tire box on `/darknet_ros_3d/tire_bounding_boxes` OR use planned fallback (4 tires from vehicle box: FL, FR, RL, RR). |
| **INSPECT_TIRE** | Navigate to tire (detected or planned). On success → FACE_TIRE. |
| **FACE_TIRE** | Rotate in place to face tire before capture. On success → VERIFY_CAPTURE. |
| **VERIFY_CAPTURE** | Publish to `/inspection_manager/capture_photo`; wait for result on `/inspection_manager/capture_result`. When wheel detected → capture. On success → WAIT_TIRE_BOX (next tire). |

### Tire Detection (Close Range)

| Step | Component | Topic | Notes |
|------|-----------|-------|-------|
| 2a | **ultralytics_tire** | best_fallback.pt (wheel class) | fixed_mode: inspection |
| 2b | **segmentation_processor_tire** | `/darknet_ros_3d/tire_bounding_boxes` | 3D boxes from YOLO + depth |
| 2c | **inspection_manager** | Subscribes | detection_topic (tire_bounding_boxes) |
| 2d | **Planned fallback** | When no wheels detected within timeout | 4 tires at vehicle geometry via `estimate_tire_positions(vehicle_center, robot_pos)` (FL, FR, RL, RR) |

**Result:** Tire position from wheel detection or planned fallback. `_dispatch_box_goal(tire_box, offset=tire_offset)` computes tire approach pose. Photo when wheel detected at VERIFY_CAPTURE.

---

## Segmentation Mode

| State | segmentation_mode | Model |
|-------|-------------------|-------|
| SEARCH_VEHICLE, WAIT_VEHICLE_BOX, PATROL_SEARCH | `navigation` | yolov8m-seg (vehicles) |
| WAIT_TIRE_BOX | `inspection` | best_fallback.pt (wheels) |

Published to `/segmentation_mode`. segment_3d dual-stream runs both vehicle and tire models; tire detection is always on via `/darknet_ros_3d/tire_bounding_boxes`.

---

## Topic Wiring

| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/darknet_ros_3d/vehicle_bounding_boxes` | segmentation_processor_vehicle | inspection_manager |
| `/darknet_ros_3d/tire_bounding_boxes` | segmentation_processor_tire | inspection_manager |
| `/aurora_semantic/vehicle_bounding_boxes` | aurora_semantic_fusion | inspection_manager (primary when vehicle_boxes_topic set) |
| `/segmentation_mode` | inspection_manager | (optional; segment_3d dual-stream always runs both models) |
| `/navigate_to_pose` | inspection_manager (action client) | Nav2 bt_navigator |
| `/inspection_manager/capture_photo` | inspection_manager | photo_capture_service |
| `/inspection_manager/capture_result` | photo_capture_service | inspection_manager |
| `/slamware_ros_sdk_server_node/left_image_raw` | Aurora | photo_capture_service, ultralytics |
| `/inspection_manager/current_goal` | inspection_manager | (optional; monitors) |

**Current goal:** To see where the robot is heading, subscribe to `inspection_manager/current_goal` (PoseStamped in map frame). Published only when a nav goal is active (approach vehicle or tire); when the goal is cleared or the mission is idle, no new message is sent (use topic timeout or inspection_state to infer no active goal).

---

## Parameter Mapping (Launch → inspection_manager_node)

| Launch arg | inspection_manager_node param |
|------------|------------------------------|
| tire_detection_topic | detection_topic |
| vehicle_detection_topic | vehicle_detection_topic |
| vehicle_fallback_topic | vehicle_boxes_topic |

**tire_label:** Must match best_fallback.pt output exactly. best_fallback.pt uses **wheel** (id=22). Set `tire_label: wheel` in PRODUCTION_CONFIG. Full class list: [docs/BEST_FALLBACK_MODEL_CLASSES.md](BEST_FALLBACK_MODEL_CLASSES.md). To verify: `ros2 run segmentation_3d print_inspection_model_classes`.

---

## Pre-Mission Checklist

1. **Stack running** — Aurora, segment_3d (dual-stream: vehicle + tire), Nav2, inspection_manager (inspection_manager_node), **photo_capture_service** (required for VERIFY_CAPTURE; subscribes to capture_photo, publishes capture_result), visual_servo (optional).
2. **TF** — map → base_link (Aurora odom + map).
3. **Vehicle visible** — At least one vehicle on `/aurora_semantic/vehicle_bounding_boxes` or `/darknet_ros_3d/vehicle_bounding_boxes`.
4. **Camera** — `/slamware_ros_sdk_server_node/left_image_raw` publishing.
5. **Nav2** — `/navigate_to_pose` action server available.
6. **Offsets** — `approach_offset` 0.7 m, `tire_offset` 0.5 m (see [NAVIGATION_SAFETY.md](NAVIGATION_SAFETY.md)).

---

## Start Mission

**inspection_manager_node** (canonical stack) subscribes to the **topic** `/inspection_manager/start_mission` (std_msgs/Bool). By default (`start_mission_on_ready: true`), the mission auto-starts when TF is ready and Nav2 is available. If `start_mission_on_ready: false`:

```bash
ros2 topic pub --once /inspection_manager/start_mission std_msgs/msg/Bool "{data: true}"
```

**Note:** Mission start is topic-based only: publish `true` to `/inspection_manager/start_mission` (std_msgs/Bool) when `start_mission_on_ready: false`. The default full_bringup stack uses `inspection_manager_node` and auto-starts when TF and Nav2 are ready.

---

## What Happens Every Time

1. **Car detection (distance)** — Aurora semantic or YOLO → vehicle bounding box in slamware_map.
2. **Approach vehicle** — Goal = vehicle_center − offset × dir(robot→vehicle). Nav2 drives there.
3. **Switch to tire inspection** — When close (WAIT_TIRE_BOX), use best_fallback.pt for wheel detection.
4. **Tire positions** — From wheel detection or planned fallback (4 tires: FL, FR, RL, RR from vehicle box).
5. **Navigate to tire** — For each tire: approach pose (tire − offset × dir), Nav2 drives.
6. **Face tire** — Rotate to face tire before capture.
7. **Take photo** — Publish to `/inspection_manager/capture_photo`; **photo_capture_service** saves PNG and publishes SUCCESS/FAILURE to `/inspection_manager/capture_result`. VERIFY_CAPTURE fails if photo_capture_service is not running.
8. **Next tire** — Return to WAIT_TIRE_BOX until all 4 tires done.
9. **Complete** — NEXT_VEHICLE or DONE.
