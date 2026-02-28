# Port Plan: Deterministic Inspection Manager into `src/tire_inspection_bot`

## Path and Canonical Runtime Decision

- Requested canonical runtime root: `src/tire_inspection_bot`.
- Current working production stack is under: `src/Tyre_Inspection_Bot/src/amr_hardware/src/...`.
- Porting strategy:
  1. Introduce new canonical package path `src/tire_inspection_bot` with integrated deterministic manager modules.
  2. Keep compatibility with existing launch/data topics from `Tyre_Inspection_Bot` packages.
  3. Preserve old mission node during migration until build+integration tests pass.

## Module Mapping (Root Prototype -> Target)

- `inspection_manager/manager_node.py` -> `src/tire_inspection_bot/inspection_manager/manager_node.py`
- `inspection_manager/state_persistence.py` -> `src/tire_inspection_bot/inspection_manager/state_persistence.py`
- `inspection_manager/tire_enumerator.py` -> `src/tire_inspection_bot/inspection_manager/tire_enumerator.py`
- `inspection_manager/approach_planner.py` -> `src/tire_inspection_bot/inspection_manager/approach_planner.py`
- `inspection_manager/alignment.py` -> `src/tire_inspection_bot/inspection_manager/alignment.py`
- `inspection_manager/photo_verifier.py` -> `src/tire_inspection_bot/inspection_manager/photo_verifier.py`
- launch skeletons -> `src/tire_inspection_bot/launch/inspection_mission_launch.py`
- API contracts -> `src/tire_inspection_bot/api_msgs/{srv,action,msg}`

## Build System and Dependency Adjustments

## `package.xml`

Add/confirm dependencies:

- buildtool:
  - `ament_cmake`
  - `ament_cmake_python`
  - `rosidl_default_generators`
- runtime/build:
  - `rclpy`
  - `rclcpp`
  - `nav2_msgs`
  - `geometry_msgs`
  - `sensor_msgs`
  - `std_msgs`
  - `std_srvs`
  - `builtin_interfaces`
  - `tf2_ros`
  - `tf2_geometry_msgs`
  - `cv_bridge`
  - `image_transport`
  - `action_msgs`
  - `rosidl_default_runtime`
  - `python3-opencv` (system dep)

## `CMakeLists.txt`

- add `find_package(...)` for all interface dependencies.
- add `rosidl_generate_interfaces(...)` for:
  - `srv/StartMission.srv`
  - `srv/PhotoCapture.srv`
  - `msg/MissionStatus.msg`
  - `action/ApproachTire.action`
- install python package + launch files.

## Required Contracts to Implement/Replace

- service: `/inspection_manager/start_mission`
  - req: `object_id`, `mission_config_json`
  - res: `mission_id`, `started_bool`, `message`
- topic: `/inspection_manager/mission_status`
  - fields: `mission_id`, `state`, `current_tire_id`, `percent_complete`, `last_event_ts`
- action: `/tire_inspector/approach_tire`
  - goal/feedback/result per required contract
- service: `/photo_capture/capture`
  - req: `mission_id`, `object_id`, `tire_id`
  - res: `file_path`, `metadata`, `ok`

## Ordered Migration Checklist

1. Create `src/tire_inspection_bot` package skeleton (ament_cmake + python + rosidl).
2. Add API interface files and compile generated types.
3. Port persistence and deterministic FSM manager.
4. Wire enumerator/planner/alignment/photo_verifier modules.
5. Integrate Nav2 `NavigateToPose` client for approach state.
6. Add TF conventions and object/tire frame usage.
7. Add launch file with live Aurora topic defaults.
8. Add/port tests and mission replay script.
9. Build with `colcon build --packages-select tire_inspection_bot`.
10. Run replay integration and then live validation.

## Risks and Mitigations

- risk: duplicate package naming confusion (`Tyre_Inspection_Bot` vs `tire_inspection_bot`).
  - mitigation: isolate new package path and keep explicit topic contracts.
- risk: interface generation failures.
  - mitigation: start with minimal svc/msg/action definitions and incremental build.
- risk: live mission regressions.
  - mitigation: keep old manager runnable until new manager passes replay + dry-run checks.
