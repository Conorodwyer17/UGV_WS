# Code Map: `src/tire_inspection_bot` (canonical resolved to `src/Tyre_Inspection_Bot`)

## Path Resolution

- Requested path `src/tire_inspection_bot` does not exist in this repository.
- Canonical existing path is `src/Tyre_Inspection_Bot`.
- All inventory and mapping below uses the canonical path.

## Inventory Command Outcomes

- `ls -R src/tire_inspection_bot` -> not found
- `ros2 pkg prefix tire_inspection_bot` -> package not found
- `colcon list | ...` shows no `tire_inspection_bot` package name currently; active packages are nested under `src/Tyre_Inspection_Bot/src/amr_hardware/src/...`

## Package/Module Structure (Relevant to Mission)

- root docs and assets:
  - `src/Tyre_Inspection_Bot/README.md`
  - `src/Tyre_Inspection_Bot/ARCHITECTURE.md`
  - `src/Tyre_Inspection_Bot/DEPLOYMENT.md`
- mission stack packages:
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager`
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d`
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav`
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_base_driver`

## Current Inspection Runtime Wiring

- main manager:
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/inspection_manager_node.py`
  - contains a large in-node FSM loop, nav dispatch, capture handling, diagnostics, and recovery logic.
- mission state helper:
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/mission_state_machine.py`
- capture node:
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/photo_capture_service.py`
- launch:
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/launch/inspection_manager.launch.py`

## ROS Interfaces Observed (Current)

- publishers/subscribers/action client in manager:
  - state topic: `inspection_state`
  - mission report: `/inspection_manager/mission_report`
  - diagnostics: `/inspection_manager/runtime_diagnostics`
  - capture trigger: `/inspection_manager/capture_photo`
  - capture metadata: `/inspection_manager/capture_metadata`
  - capture result: `/inspection_manager/capture_result`
  - action client: `navigate_to_pose`
- capture service side:
  - subscribes to image + capture trigger + metadata
  - publishes capture result

## High-Risk Integration Observations

- Mission logic is monolithic and tightly coupled inside one node file.
- Current package namespace/package name differs from requested canonical `tire_inspection_bot`.
- Existing runtime expects multiple packages under `amr_hardware/src`; porting must preserve these dependencies while introducing deterministic refactored manager modules.

## Immediate Porting Implication

- Refactor should be integrated into existing `inspection_manager` package path under:
  - `src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/`
- If strict package name `tire_inspection_bot` is required for tooling, add a new wrapper package in `src/tire_inspection_bot` that launches/depends on the existing mission packages.
