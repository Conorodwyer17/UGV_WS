# Poster Content (Suggested Layout)

Suggested structure for an A0 or A1 thesis poster. Add your official department and university names on the printed poster to match faculty cover requirements.

---

## Title block

**Title:** Autonomous Ground Vehicle for Visual Tyre Inspection with ROS 2 and SLAMTEC Aurora  

**Author:** Conor O’Dwyer  
**Project type:** Final-year engineering project  

---

## Abstract (short)

Autonomous mobile robot that detects a parked vehicle, plans paths with Nav2, navigates to four tyre locations, and captures inspection images. Integrates ROS 2 Humble, Aurora 6DOF sensing, Ultralytics YOLO wheel segmentation, optional 3D tyre projection, and a mission manager with field-oriented safeguards. CPU ONNX inference and modular bringups support memory-constrained Jetson deployment.

---

## System diagram (centre or left column)

**Description:** Block diagram: Aurora SDK node (odom, scan, depth, semantic) feeds vehicle boxes and sensor data; `segmentation_3d` produces tyre boxes and `/tyre_3d_positions`; Nav2 consumes costmaps from scan and depth; `inspection_manager_node` sends goals to Nav2 and triggers `photo_capture_service`; `motor_driver` executes `cmd_vel`. TF: `map` / `slamware_map` to `base_link`.  

**Figure file:** Prepare a single PDF or high-resolution PNG in `docs/figures/` (e.g. `system_overview.pdf`) and place the export alongside this document when available.

---

## Key hardware

- Jetson Orin Nano (16 GB), differential-drive UGV base  
- SLAMTEC Aurora: LiDAR, stereo depth, IMU, semantic segmentation  
- ESP32 UART motor interface; optional wheel odometry for EKF fusion  

## Key software

- ROS 2 Humble, Cyclone DDS  
- Nav2 (SmacPlanner2D, tuned costmaps and BTs without unnecessary spins)  
- Ultralytics YOLO (wheel class), ONNX export for CPU inference  
- `tyre_3d_projection_node`, inspection manager state machine, photo capture service  

---

## Challenges and solutions (bullet points)

- **Vehicle box jitter:** Smoothing, confirmations, tyre 3D goals and geometry-based ordering.  
- **Planner / BT mismatch:** Align YAML plugin names with behaviour tree `planner_id`.  
- **Memory pressure on Jetson:** CPU ONNX tyre path, throttled depth, optional minimal perception.  
- **Goals in inflated cost:** Pre-nav lethal escape, post-capture backup, inflation and tolerance tuning.  
- **Demonstrating without driving:** Stub motor profile (`stable_viz`) and `demo_mode`.  

---

## Results (brief)

- No-motion runs: planning and mission states observable in RViz; logs under `~/ugv_ws/logs/`.  
- Real-motion runs: four-tyre mission design with JSON mission report; exact timings and success rates should be taken from your own `mission_report_latest.json` and `mission_latest.jsonl` for the poster.  
- Screenshot placeholders: RViz showing `/plan` and tyre overlay; sample tyre image from `tire_inspection_photos/` (not committed if excluded by `.gitignore`; capture at demo time).

---

## Future work

- Advanced controllers (e.g. MPPI) for tight manoeuvres.  
- GPU inference where memory allows; model quantisation.  
- Fleet scheduling and multi-robot coordination.  
- Image quality gates before accepting a capture.

---

## Conclusion

Integrated ROS 2 stack combining Aurora localisation, learning-based wheel detection, Nav2 navigation, and a mission layer suitable for depot-style inspection. The repository emphasises reproducible launches, logging, and demonstration modes for assessment.

---

## Acknowledgements

Include supervisor, laboratory support, and any equipment or funding acknowledgements required by your faculty.
