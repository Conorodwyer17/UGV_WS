# Integration Blueprint (Jetson + Aurora + ESP32)

## Executive Summary

This blueprint captures a reproducible P0 integration baseline validated on the local Jetson repository with live Aurora connectivity and live ESP32 serial telemetry. Current runtime evidence confirms:

- Aurora ROS SDK connection and topic availability (pointcloud/stereo/depth/IMU/odom/semantic).
- ESP32 telemetry over `/dev/ttyTHS1` at `115200` with encoder/IMU fields.
- A minimal object-manager demo creating persistent `object_<id>` records and publishing `object_1` TF evidence.

The full end-to-end autonomous tire inspection mission remains in progress, but the core runtime plumbing and evidence trail are now in place under `research/`.

## Reproduction Steps

1. Source environment.

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

2. Launch Aurora bringup.

```bash
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1 use_bridge:=false
```

3. Verify Aurora topic graph.

```bash
ros2 topic list
ros2 topic info -v /slamware_ros_sdk_server_node/point_cloud
```

4. Run detection pipeline check.

```bash
ros2 run segmentation_3d detection_pipeline_check
```

5. Probe ESP32 serial telemetry.

```bash
python3 - <<'PY'
import glob, serial, time
port='/dev/ttyTHS1'
ser=serial.Serial(port,115200,timeout=0.2)
t0=time.time()
while time.time()-t0<5:
    line=ser.readline().decode('utf-8','replace').strip()
    if line:
        print(line)
ser.close()
PY
```

6. Run minimal ESP32 ROS bridge.

```bash
python3 nodes/esp32_bridge/esp32_serial_bridge_node.py --ros-args -p serial_port:=/dev/ttyTHS1 -p baud_rate:=115200
```

7. Run minimal object manager demo and publish synthetic test detection.

```bash
python3 nodes/object_manager/demo_object_manager_node.py
ros2 topic pub --once --qos-reliability best_effort /darknet_ros_3d/vehicle_bounding_boxes gb_visual_detection_3d_msgs/msg/BoundingBoxes3d "{header: {frame_id: 'map'}, bounding_boxes: [{object_name: 'car', probability: 0.95, xmin: 1.0, ymin: 1.0, xmax: 2.0, ymax: 2.0, zmin: 0.0, zmax: 1.0}]}"
```

8. Verify object TF exists.

```bash
timeout 4 ros2 topic echo /tf | rg object_1
```

## Artifact Index

- Environment inventory: `research/snapshots/conor-desktop_2026-02-28T00-47-07+00-00.txt`
- Refresh inventory: `research/snapshots/conor-desktop_2026-02-28T01-15-46+00-00.txt`
- Code map: `docs/code_map.md`
- Commands/failures/progress logs: `research/logs/`
- Aurora manifests and bag infos: `research/data/aurora_samples/`
- ESP32 telemetry captures: `research/data/esp32_odometry/`
- Object manager evidence: `research/demos/`
- External resources index: `docs/resources_dump.csv`, `docs/resources_dump.json`

## Minimal Detector -> Tracker -> Object Demo

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash

python3 ~/ugv_ws/nodes/detector/demo_detector_node.py
python3 ~/ugv_ws/nodes/tracker/demo_tracker_node.py
python3 ~/ugv_ws/nodes/object_manager/demo_object_manager_node.py --ros-args -p input_topic:=/demo/tracks_3d

timeout 6 ros2 topic echo /tf > ~/ugv_ws/research/demos/tf_stream_object_demo_20260228_0119.txt
```

Evidence file containing object frames:

- `research/demos/object_tf_evidence_20260228_0119.txt`

## Known Gaps (Immediate Next P0)

1. Replace demo object manager with package-integrated service API (`object_list`, `publish_anchor`).
2. Integrate planner adapter for object-relative tire goals in Nav2 BT.
3. Implement final approach PBVS/IBVS loop and objective photo quality gates.
4. Add deterministic rosbag recording workaround (or rosbag2 storage plugin update) for multi-topic Aurora capture in one bag.
5. Expand resources crawl to full target set (repos/packages/papers) with licensing compatibility tags.
