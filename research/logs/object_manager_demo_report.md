# Minimal Detector -> Object Manager Demo Report

## Runtime Inputs

- detection health log: `research/logs/detection_pipeline_check_20260228_005303.log`
- live vehicle box topic: `/darknet_ros_3d/vehicle_bounding_boxes`

## Demo Node

- node file: `nodes/object_manager/demo_object_manager_node.py`
- behavior:
  - subscribes to 3D vehicle boxes
  - assigns persistent IDs by nearest-neighbor association
  - publishes `tf` frames: `object_<id>`
  - writes object registry snapshot to `research/demos/object_manager_registry_latest.json`

## Evidence

- registry snapshot with persistent object record:
  - `research/demos/object_manager_registry_latest.json`
- TF stream evidence containing `object_1`:
  - `research/demos/tf_stream_object_demo.txt`
- extracted frame matches:
  - `child_frame_id: object_1` detected in TF stream output

## Caveat

- This run validates minimal object ID persistence and TF publication.
- Full fused detector+tracker+planner integration remains a follow-on step.
