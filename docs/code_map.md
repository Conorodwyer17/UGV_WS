# Local Code Map (Jetson UGV Workspace)

## Scope

This map captures the current local ROS 2 workspace structure, package entrypoints, launch topology, interfaces, and test coverage status for the autonomous tire inspection stack.

## Workspace Packages (`colcon list`)

- `aurora_interface` (`ament_cmake`)
- `aurora_sdk_bridge` (`ament_cmake`)
- `gb_visual_detection_3d_msgs` (`ament_cmake`)
- `inspection_manager` (`ament_python`)
- `segmentation_3d` (`ament_cmake`)
- `segmentation_msgs` (`ament_cmake`)
- `slamware_ros_sdk` (`ament_cmake`)
- `ugv_base_driver` (`ament_python`)
- `ugv_description` (`ament_cmake`)
- `ugv_nav` (`ament_cmake`)
- `ugv_vision` (`ament_python`)

## Main Runtime Topology

- `ugv_nav/launch/full_bringup.launch.py`
  - includes Aurora bringup (`aurora_bringup.launch.py`)
  - includes perception stack (`segment_3d.launch.py`)
  - includes Nav2 bringup (`nav_aurora.launch.py`)
  - includes mission (`inspection_manager.launch.py`)
  - optionally launches `ugv_base_driver` motor process
- `aurora_interface/launch/aurora_full.launch.py`
  - includes `aurora_sdk_bridge.launch.py`
  - launches `aurora_health_monitor`

## Node Entrypoints (Python `console_scripts`)

- `inspection_manager`
  - `inspection_manager_node`
  - `photo_capture_service`
- `ugv_base_driver`
  - `motor_driver_node`
- `ugv_vision`
  - `color_track`
  - `kcf_track`
  - `gesture_ctrl`
  - `apriltag_ctrl`
  - `apriltag_track_0`
  - `apriltag_track_1`
  - `apriltag_track_2`

## Perception Stack

- `segmentation_3d/launch/segment_3d.launch.py` composes:
  - `ultralytics_vehicle` and `ultralytics_tire` detector nodes
  - `aurora_semantic_fusion_node` (Aurora semantic labels -> vehicle 3D boxes)
  - `aurora_depth_camera_info_node`
  - `depth_to_registered_pointcloud_node`
  - two `segmentation_processor_node` instances:
    - vehicle boxes -> `/darknet_ros_3d/vehicle_bounding_boxes`
    - tire boxes -> `/darknet_ros_3d/tire_bounding_boxes`

## Navigation and TF Conventions Found

- Aurora bringup (`aurora_bringup.launch.py`) configures:
  - `map_frame=slamware_map`
  - `odom_frame=odom`
  - `robot_frame=base_link`
  - sensor frames: `laser`, `imu_link`, `camera_left`, `camera_right`
- Nav2 launch (`nav_aurora.launch.py`) adds:
  - `map -> slamware_map` static transform
  - `base_link -> base_footprint` static transform
  - `depth_gate` for command gating (`cmd_vel_nav` -> `cmd_vel`)
- inspection mission logic and object/tire policy code exists in:
  - `inspection_manager/inspection_manager/inspection_manager_node.py`
  - `inspection_manager/inspection_manager/goal_generator.py`
  - `inspection_manager/inspection_manager/mission_state_machine.py`
  - `inspection_manager/inspection_manager/mission_policy.py`
  - `inspection_manager/inspection_manager/perception_handler.py`

## Interfaces (Custom Messages/Services)

- `segmentation_msgs/msg`
  - `ObjectSegment.msg`
  - `ObjectsSegment.msg`
- `gb_visual_detection_3d_msgs/msg`
  - `BoundingBox3d.msg`
  - `BoundingBoxes3d.msg`
- `slamware_ros_sdk/msg`
  - status + map/relocalization related messages
- `slamware_ros_sdk/srv`
  - `RelocalizationRequest.srv`
  - `SyncGetStcm.srv`
  - `SyncSetStcm.srv`

## Existing Validation/Diagnostics Utilities

- `segmentation_3d/scripts/perception_validation_node.py`
- `segmentation_3d/scripts/detection_pipeline_check.py`
- `aurora_interface/scripts/aurora_health_monitor.py`
- prior TF snapshot artifacts present at repository root:
  - `frames_2026-02-26_21.09.34.gv`
  - `frames_2026-02-26_21.09.34.pdf`

## Current Test Coverage Snapshot

- `colcon test --packages-select inspection_manager ugv_base_driver ugv_vision segmentation_3d`
  - all selected packages completed without failures
  - current outputs indicate `Ran 0 tests` for selected Python packages in this run
- `colcon test-result --all --verbose`
  - found existing `ugv_nav` lint/test results with 0 failures
- test artifacts saved under:
  - `research/test_results/colcon_test_output.txt`
  - `research/test_results/colcon_test_result.txt`

## Gaps Observed During Inventory

- `ros2 --version` unsupported in this environment (CLI expects subcommands).
- `jetson_release` utility missing.
- no integrated CI workflow file identified at top-level during this initial pass.
- large mission/perception code exists, but explicit unified object-manager service API and tire frame persistence contract still need formalization for P0 acceptance.
