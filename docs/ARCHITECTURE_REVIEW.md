# Architecture Review

Critical evaluation of the UGV tyre inspection system architecture for maintainability, robustness, and extensibility.

## Strengths

**Separation of concerns:** Mission logic (inspection_manager), perception (segment_3d), and motion (ugv_nav) are cleanly separated. Each package has a well-defined responsibility.

**State machine design:** Flat state machine with explicit transitions, timeouts, and error recovery. States are documented in MISSION_PIPELINE and RUNBOOK. No implicit or hidden transitions.

**Dual perception:** Aurora semantic (vehicles) + YOLO (tyres) with PCL fallback. Redundancy improves robustness when one stream fails.

**TF and frame consistency:** All 3D boxes in slamware_map; goals computed in world frame, transformed to map for Nav2. Clear frame chain documented in ARCHITECTURE.md.

**Observability:** Mission report JSON, per-tire goal_source, mission log (jsonl), and ROS logging provide sufficient forensics for field debugging.

**Configurability:** PRODUCTION_CONFIG, launch args, and Nav2 YAML allow tuning without code changes.

## Weaknesses

**Single mission manager:** inspection_manager_node is the only component that orchestrates the mission. If it crashes, the system stops. Mitigation: systemd restart; consider watchdog or health-monitoring node in future.

**Tight coupling to Aurora:** TF, odom, scan, and semantic labels all come from Aurora. No fallback localisation if Aurora disconnects. Acceptable for current hardware; document as assumption.

**Photo capture dependency:** Mission blocks on photo_capture_service. If the service never responds, VERIFY_CAPTURE times out. Timeout and retry logic exist; ensure photo_capture_service is robust.

**No formal interface contracts:** Topic and service contracts are implicit (message types, QoS). Consider adding .msg/.srv documentation or interface packages for large teams.

## Potential Improvements

1. **Health monitor node:** Optional node that subscribes to key topics (TF, odom, detection) and publishes a `/system_health` or similar. Enables dashboards and alerting.

2. **Footprint refinement:** If the UGV has an elongated base, define a polygon footprint in costmap params for better navigation in tight spaces.

3. **Recovery behaviour tuning:** Nav2 recovery (ClearCostmap, BackUp) is standard. Consider adding a "re-acquire vehicle" behaviour if the robot loses the vehicle box during approach (e.g. after long recovery).

4. **Simulation parity:** Mock simulation works; consider adding a Gazebo or similar physics sim for more realistic testing of costmap and collision avoidance.

## Performance Bottlenecks

- **Vision:** TensorRT on Jetson targets < 10 ms inference. Desktop benchmark showed ~36 ms; Jetson-specific benchmark required for production metrics.
- **Nav2:** Controller and planner run at configured rates. No observed bottlenecks in simulation.
- **TF:** Aurora publishes odom; static transforms for map/slamware_map. Latency acceptable for 10 Hz control.

## Maintainability and Extensibility

- **Adding a new sensor:** Would require a new fusion node or topic remapping. segment_3d already supports multiple detection sources (YOLO, PCL, Aurora semantic).
- **Adding a new vehicle type:** vehicle_labels and tire_label are parameters. No code change needed for new COCO classes if Aurora supports them.
- **Changing mission flow:** State machine is in mission_state_machine.py; well-structured for adding states (e.g. re-inspect, skip tyre).

## Conclusion

The architecture is sound for the current scope. Key risks (single mission manager, Aurora dependency) are documented and have mitigations (restart, pre-mission checks). Recommended next steps: expand unit tests, run Jetson benchmark, and optionally add a health monitor for production deployments.
