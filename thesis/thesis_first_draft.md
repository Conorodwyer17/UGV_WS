# Autonomous Tyre Inspection Robot: From Drive-Over to Mobile Deployment

**First Draft**  
**September 2024 – March 2025**

*Revision note (March 2025): This draft has been revised for clarity, structure, and completeness. Changes include: expanded Abstract with hardware upgrade and SLAMTEC collaboration; numbered requirements with traceability in Chapter 3; acronym definitions and List of Acronyms in Appendix A; standard references (ROS 2, Nav2, YOLO, TensorRT); figure placeholders; and consistent cross-references throughout. The narrative flow and factual content are unchanged.*

---

## Abstract

This thesis documents the design, development, and current state of an autonomous mobile robot for tyre inspection of commercial vehicles. The work was carried out in collaboration with Tyrecheck, a Tyre Pressure Monitoring System (TPMS) sensor company, at their branch in Prague, Czech Republic, where the author completed a fourth-year placement as a support software engineer. Tyrecheck’s existing system relies on a drive-over setup: vehicles pass over a fixed speed bump or similar structure in car parks or truck yards, and cameras at that point capture images of the tyres for condition assessment. The aim of this project is to make inspection dynamic by placing the cameras on a mobile robot that drives to stationary vehicles, detects them, navigates to each of the four tyres, and photographs each one.

The thesis describes the problem and requirements, the choice of Robot Operating System 2 (ROS 2) and hardware, and the exploration of several robot and sensor configurations over the first months. It covers the training of detection models for vehicles, people, and wheel and vehicle components, and the integration of the SLAMTEC Aurora for localisation, mapping, and perception. Development followed a simulation-first approach: a mock Aurora node, synthetic vehicle publisher, and simulated detection node allowed testing of the perception pipeline and mission logic without a real vehicle. Significant setbacks are recorded: a simultaneous hardware failure of the first Jetson and the first Aurora, thought to be due to a power surge, which led to the loss of roughly three weeks of uncommitted work; a temporary migration to a Raspberry Pi 5 with a Coral accelerator; and the diagnosis and replacement of the Aurora unit. Further challenges emerged during real-world testing: GPU inference on the Jetson Orin proved difficult due to PyTorch–cuDNN incompatibilities (JetPack 6.0 ships cuDNN 9 whilst NVIDIA’s PyTorch wheels require cuDNN 8), and CUDA out-of-memory (OOM) errors when running You Only Look Once (YOLO) alongside Nav2 and other nodes. Profiling showed that the 8 GB unified memory was insufficient for simultaneous SLAM, depth processing, costmap generation, and YOLO inference. After consultation with supervisors and Tyrecheck engineers, the decision was made to purchase a 16 GB NVIDIA Jetson Orin Nano Developer Kit to provide adequate headroom for production-level operation. The author also investigated whether a custom YOLO model could run on the Aurora’s onboard AI processor; the Aurora SDK exposes only two fixed models (COCO and Cityscapes) with no API for custom model upload, and the camera is proprietary. SLAMTEC technical support was contacted and has initiated internal research on custom model deployment; this collaboration is ongoing. A CPU-based inference fallback using ONNX Runtime was implemented, allowing the system to run reliably at 224×224 resolution and 5 Hz without GPU memory exhaustion. Over many months the author has tested and refined the system repeatedly—different inspection-manager designs, different approaches to the vehicle (e.g. nearest face vs nearest tyre), refinements to navigate-to-pose behaviour and map updating, and the decision to build a fresh map on each mission start. Testing has included live runs with real cars, dry runs, single and multiple vehicles, and various detection and navigation configurations; the repository holds a long history of commits and documentation that reflects this iterative process.

The current system uses the Aurora for vehicle detection (via its built-in semantic segmentation), depth, LiDAR, and point cloud; a custom YOLO-based wheel model (best_fallback.pt) for tyre detection at each stop, with a CPU inference path as the default and an optional TensorRT path when PyTorch with CUDA is available; and a mission state machine that navigates around the detected vehicle to four tyre positions in a deterministic order. The thesis concludes that the approach is viable and that the project serves as a proof of concept: the hardware in use is testing equipment provided by Tyrecheck to validate that autonomous tyre inspection is feasible before production investment. The 16 GB Jetson is on order; the simulation-validated software is ready for integration once it arrives. Once feasibility is proven, the solution can be extended for deployment across the company’s bus depots, truck yards, trailer yards, and retail sites.

**Keywords:** autonomous robot, tyre inspection, ROS 2, SLAM, SLAMTEC Aurora, computer vision, YOLO, Nav2, drive-over system, mobile inspection, Tyrecheck, TPMS, proof of concept.

---

## Table of Contents

1. Introduction  
2. Background and Related Work (including ROS 2 integration and plug-and-play)  
3. Problem Statement and Requirements  
4. Methodology and Design (Aurora operation and specifications, TF/IMU/mapping, detection pipeline, committed plan, hardware extensibility, simulation environment)  
5. Implementation and Development (hardware failures, GPU/CUDA issues, Aurora investigation, CPU fallback)  
6. Current State and Testing (months of testing and iteration, failure modes, forensics, path forward, figures)  
7. Conclusions and Future Work (proof of concept, path to production)  
References  
Appendices (Glossary, Repository Structure)  

---

## 1. Introduction

### 1.1 Context and Motivation

In September 2024, the author approached Tyrecheck, the company at which they had completed a fourth-year placement—a support software engineer role at Tyrecheck’s Prague branch, focused on tyre checking and TPMS-related systems—to seek thesis ideas that would be of direct use to the company. The company presented the concept of an autonomous tyre inspection robot: a system capable of autonomously detecting vehicles, navigating to each tyre of a stationary vehicle, and photographing each tyre. The motivation is to move from a static, drive-over inspection system to a dynamic one in which the inspection system moves to the vehicle rather than the vehicle moving over a fixed installation.

The existing drive-over system is deployed in car parks and truck yards. Vehicles (cars, trucks, buses) drive over a defined point—a speed bump or similar structure—where cameras are installed. When the vehicle passes over this point, the system captures images of the tyres. From these images, the company derives information about tyre condition: damage, sidewall issues, flatness, and other defects. This allows pre-emptive replacement of tyres before further damage to the vehicle or tyre occurs. The limitation of this approach is that it requires the vehicle to be driven over a fixed location. In depots or yards where vehicles are parked, moving each vehicle to a drive-over point is impractical or costly. The company wished to explore making the cameras mobile so that the inspection could be brought to the vehicle—for example, in depots where vehicles are parked and need to be inspected without being moved.

### 1.2 Thesis Objective and Scope

The thesis objective is to design, implement, and evaluate an autonomous mobile robot that:

1. Detects vehicles (cars, trucks, buses) in its environment.  
2. Navigates safely around the detected vehicle.  
3. Stops at each of the four tyre positions and captures a photograph of each tyre.  
4. Does so in a manner that could eventually support a production or pilot deployment.

The scope covers the choice of hardware (chassis, onboard computer, sensors), the software architecture (ROS 2, perception, navigation, mission control), the training and integration of detection models for vehicles and tyres, and the handling of real-world setbacks such as hardware failures and platform migration. This thesis is explicitly a proof of concept: it does not claim that the system is yet fully production-ready. A good deal of time has gone into perfecting autonomous navigation and robustness, and that work continues. This document is a first draft and reflects the state of the project as of March 2025.

### 1.3 Document Structure

Chapter 2 summarises background and related work on tyre inspection, autonomous mobile robots, and the technologies used, and explains how ROS 2 is used in the project and why it supports plug-and-play integration of sensors. Chapter 3 states the problem and requirements. Chapter 4 describes the methodology and design choices, including the exploration of several robot platforms, the decision to adopt the SLAMTEC Aurora, how the Aurora works (mapping, TF, IMU), the detection pipeline and committed plan, and the simulation environment. Chapter 5 details the implementation and development, including the hardware failures, the migration to and from the Raspberry Pi 5, GPU and CUDA issues, the Aurora custom-model investigation, and the CPU inference fallback. Chapter 6 describes the current state of the system, testing, and the path forward (including the hardware upgrade and pending SLAMTEC collaboration). Chapter 7 concludes with a summary, the proof-of-concept nature of the project and its path to production, and future work.

---

## 2. Background and Related Work

### 2.1 Tyre Inspection in Industry

Tyre condition directly affects safety, fuel efficiency, and maintenance costs. Visual inspection of tread, sidewall, and pressure is a standard practice in fleet management and retail contexts. Automated systems range from drive-over pits with fixed cameras to handheld or vehicle-mounted imaging. The drive-over approach is well established as a baseline: the vehicle is driven over a known geometry, and cameras trigger at the correct moment to capture each tyre. The constraint is that the vehicle must be driven to that location. In depots or yards where vehicles are parked, moving each vehicle to a drive-over point may be impractical or costly; a mobile system that travels to the vehicle can reduce the need to move the vehicle and can support more flexible inspection workflows.

### 2.2 Autonomous Mobile Robots and ROS

Autonomous mobile robots typically rely on a combination of localisation (where the robot is), mapping (representation of the environment), perception (detection of objects), and motion planning and control. The Robot Operating System (ROS) and its successor ROS 2 provide middleware and tooling for integrating sensors, algorithms, and actuators. ROS 2 was chosen for this project because it allows sensors and nodes to be added, removed, or updated with relative ease—important for a system that must support different cameras, LiDAR, and Simultaneous Localisation and Mapping (SLAM) devices and that may evolve towards production. For readers unfamiliar with robotics: ROS 2 uses a publish–subscribe model where nodes exchange messages on topics; spatial relationships are published on a transform (TF) tree; and actions (such as navigation goals) are long-running operations with feedback.

Navigation in ROS 2 is commonly implemented with the Nav2 stack, which uses a global costmap (often from a static or SLAM-built map) and a local costmap (updated from live sensors such as LiDAR and depth) to plan paths and avoid obstacles. The SLAMTEC Aurora used in this project provides six-degree-of-freedom (6DOF) SLAM, odometry, LiDAR-style scan, depth, and optional semantic segmentation in a single device, which simplifies the sensor suite and keeps perception and localisation in one reference frame.

### 2.3 ROS 2 in This Project: Integration and Plug-and-Play

ROS 2 has been used to structure the entire project so that components can be swapped or upgraded without rewriting the core mission logic. This design allows sensor swapping and future upgrades, as described in Section 4.6. Sensors publish data on topics (scan, odom, point_cloud, images); nodes subscribe or publish without tight coupling, and the inspection manager subscribes to vehicle and tyre bounding-box topics and to TF without depending on which node produced those boxes. All spatial relationships—map to odom, odom to base_link, base_link to camera and laser frames—are published on the TF tree, so every component that needs “where is the robot in the map?” or “where is this detection in the map?” uses the same frame. Goals are computed in the map frame and sent to Nav2 there; the mission logic does not implement custom coordinate handling. Nav2 is driven via the navigate_to_pose action and photo capture via a service or topic; the state machine sends goals and waits for results without blocking other nodes. Launch arguments and parameters (approach_offset, tire_offset, vehicle_boxes_topic, and so on) allow the same codebase to run with different sensors or configs, including Aurora-only vs Aurora plus YOLO fallback.

The project deliberately removed the original sensor stack (OAK camera, separate 2D LiDAR) and replaced it with the SLAMTEC Aurora. Because the architecture is topic- and frame-based, the change meant running the Aurora ROS 2 SDK node so that it publishes the same logical data (map, odom, scan, point_cloud, images, TF), pointing Nav2 and the costmap at the Aurora’s topics and frames, and pointing the detection pipeline at the Aurora’s images and depth. No change was needed to the inspection manager’s state machine or goal logic—only to which topics and frames it uses. That is the intended benefit of ROS 2 here: the system can build maps and navigate autonomously with the Aurora as a drop-in replacement for the previous sensors, and a future production sensor suite could be integrated in the same way by publishing compatible topics and TF.

### 2.4 Computer Vision for Vehicle and Tyre Detection

Object detection and segmentation are used to find vehicles and tyres in images. YOLO (You Only Look Once) and similar architectures are widely used for real-time detection. For this project, two detection roles were defined: (1) detection of vehicles (and optionally people) for navigation and mission logic, and (2) detection of wheels or tyres when the robot is close to a tyre position, to confirm that the wheel is in view before triggering a photograph. The former can be satisfied by the Aurora’s built-in semantic segmentation (COCO80 classes: car, truck, bus, etc.) or by a YOLO-based vehicle detector; the latter requires a custom or fine-tuned model that reliably detects wheels. The project uses a custom YOLO segmentation model (best_fallback.pt) that includes a “wheel” class plus other vehicle components (bumpers, doors, mirrors, lights) for potential future use. Three-dimensional bounding boxes are obtained by projecting 2D detections into 3D using depth and camera intrinsics, so that the mission planner can reason about tyre positions in the map frame. The role of depth is critical: without it, 2D detections cannot be localised in the world; with depth and intrinsics, each pixel can be back-projected to a 3D point and transformed into the map frame for navigation.

---

## 3. Problem Statement and Requirements

### 3.1 From Drive-Over to Mobile

The company’s requirement is to make the inspection cameras “dynamic and robust” so that they can drive to the vehicles instead of requiring vehicles to drive over a fixed point. Placing the cameras on a robot is the chosen approach. The robot must:

- Operate in environments such as car parks and truck yards.  
- Detect vehicles (cars, trucks, buses) reliably.  
- Navigate to the vehicle and then to each of the four tyre positions without colliding with the vehicle or other obstacles.  
- Capture a photograph of each tyre with sufficient quality for the existing analysis pipeline (damage, sidewall, flatness, etc.).

Because the outcome is intended to support a production or pilot product, the system must be robust, repeatable, and maintainable. Sensors and software should be replaceable or upgradable where possible.

### 3.2 Requirements Summary

The requirements are organised as follows. Traceability to design decisions is indicated where relevant.

**Functional requirements:**

1. **Vehicle detection:** The system must detect vehicles (cars, trucks, buses) in the environment (Section 4.5, Aurora semantic segmentation or YOLO fallback).  
2. **Tyre position computation:** Four tyre positions (front-left, front-right, rear-left, rear-right) must be computed from the vehicle pose (Section 4.5, committed plan).  
3. **Navigation goals:** Goals must be generated with safe standoff from the vehicle and tyres (Section 4.5, offset parameters).  
4. **Mission execution:** The system must approach the vehicle and visit each tyre in a defined order (Section 4.8, state machine).  
5. **Photo capture:** A photograph must be captured at each tyre with metadata (pose, tyre label) (Section 4.8, photo_capture_service).

**Non-functional requirements:**

6. **Safe navigation:** The robot must not collide with the vehicle or other obstacles (Section 4.4, costmap, inflation).  
7. **Deterministic tyre order:** Behaviour must be predictable; tyre visit order must be well-defined (Section 4.5, strict_planned_tire_order).  
8. **Recovery from transient failures:** The system must recover from loss of TF, navigation timeouts, or similar without unbounded spinning or undefined behaviour (Section 6.1, timeouts, spin protection).  
9. **Detection–costmap alignment:** The “area of interest” (the vehicle) must be aligned between detection and the costmap, so that the robot navigates around the same physical object that was detected (Section 4.4, observation sources).

---

## 4. Methodology and Design

### 4.1 Choice of ROS 2

Given the need for a production-oriented, updatable system, ROS 2 was selected as the primary framework. It allows sensors to be added or removed, supports different computers (e.g. Raspberry Pi, Jetson), and provides standard interfaces for navigation (Nav2), TF, and topics. The same codebase is intended to run on either a Raspberry Pi 5 (with an optional Coral USB accelerator) or an NVIDIA Jetson Orin, with only minor configuration differences.

### 4.2 Hardware Exploration (September–December 2024)

Before settling on the current platform, several options were tested.

**Galaxy XE.** An initial test used a Galaxy XE platform with detection sensors and a camera, controlled via a phone application. This provided a baseline for “detect and move” but lacked sufficient processing power on the motor controller and was not suitable for running ROS 2 and vision pipelines onboard.

**Waveshare bases.** Several Waveshare UGV bases were tried, including tracked and wheeled variants. These offered a more capable mechanical platform and compatibility with an onboard computer.

**OAK camera and 2D LiDAR.** One configuration used an OAK stereo camera and a Waveshare Jetson base. The OAK provided stereo vision, but the 2D LiDAR alone was not sufficient for robust obstacle avoidance and mapping in the intended environments. The stereo baseline and integration were also not solid enough for the required 3D bounding box pipeline.

These experiments clarified that a more capable perception and localisation stack was needed: in particular, a device that could provide both a reliable map/odometry and rich perception (depth, semantic or detection) in a single reference frame. The company was briefed on this progress and agreed that the goal of a full autonomous tyre inspection robot was viable and reachable with better equipment. The company then provided a SLAMTEC Aurora unit for the next phase of development.

### 4.3 SLAMTEC Aurora as Central Sensor

The SLAMTEC Aurora (referred to in project documentation as “Aurora” or “AORA”) is an integrated positioning and mapping sensor that fuses LiDAR, binocular vision, and IMU with hardware time sync [1, 2]. SLAMTEC quote a weight of 505 g and dimensions 108 × 97 × 88 mm; the device offers 6DOF localisation with no external dependencies at startup, 2D map resolution selectable at 2, 5 or 10 cm, and a maximum mapping area in excess of 1 000 000 m² [1, 3]. The LiDAR range is given as up to 40 m; the binocular fisheye camera has 180° FOV, 6 cm baseline, HDR, and typically 15 Hz (10/30 Hz configurable). Relocalisation, map save/load, and map continuation are supported [3]. Connection to the host is over Ethernet or Wi-Fi; in access-point mode the unit is usually at 192.168.11.1. The Aurora ROS 2 SDK runs on the host and connects to that IP; it publishes map, odometry, scan, point cloud, left/right/depth images, semantic labels (COCO80 class IDs), and the TF tree (map, odom, base_link, laser, camera and IMU frames) [4, 5]. By using the Aurora as the single source of map, odometry, scan, depth, and point cloud, the need for separate LiDAR and depth cameras is reduced and all perception shares the same coordinate frame (slamware_map). The costmap used for navigation is fed from the Aurora’s scan, depth-derived point cloud, and Aurora point cloud, so that the occupied region in the costmap corresponds to the same physical vehicle that is detected semantically or by YOLO—an alignment that is essential for safe navigation around the vehicle.

### 4.4 How the Aurora Works: Mapping, TF, and Autonomous Navigation

The Aurora fuses LiDAR, vision, and IMU with hardware time sync to produce a consistent world model. Understanding how it exposes this to ROS 2 is central to how the project builds maps and navigates.

**TF (transform) tree:** The Aurora SDK publishes the spatial relationships between frames. The map frame (global) is tied to the Aurora’s internal map; the node publishes (or the launch adds) identity transforms so that `map`, `slamware_map`, and `odom` are aligned as required by Nav2. The robot pose is given by the chain `map` → `slamware_map` → `odom` → `base_link`. Camera and laser frames are linked to `base_link`. Thus every node that needs “robot pose in map” or “detection in map” uses the same TF tree; the inspection manager looks up the current robot pose in `slamware_map` (or `map`) when computing goals, and all bounding boxes are published in that frame so that goals and obstacles share one coordinate system.

**Odometry and localisation:** The Aurora publishes odometry on a dedicated topic (e.g. `/slamware_ros_sdk_server_node/odom`). No separate particle filter or AMCL is used; the Aurora provides 6DOF pose in the map. The host uses this odometry for the controller and for goal checks; TF is the single source of truth for “where is the robot” when building goals.

**Map building:** The Aurora maintains an occupancy grid (published as `/slamware_ros_sdk_server_node/map`) and supports map reset, save, and load. In this project, the map is reset shortly after launch by default so that each mission starts with a fresh map built from the current scene. The robot thus builds the map from live LiDAR, depth, and point cloud as it operates; the costmap layers (local and global) are updated from the same sensor streams so that obstacles (including the vehicle) are represented consistently.

**Observation sources for the costmap:** Nav2’s costmap is updated from: (1) the Aurora’s scan topic (LiDAR); (2) a depth-derived point cloud (e.g. from the Aurora’s depth image, converted to points and transformed into the map frame); and (3) the Aurora’s own point cloud topic (map-frame points from the SLAM pipeline). So the “occupied” cells in the costmap come directly from what the Aurora sees—the same device that provides localisation and mapping. The inflation layer then expands those obstacles by a configured radius (e.g. 0.6 m locally) so that planned paths keep a safe clearance from the vehicle and other obstacles. In this way, the Aurora is used to full effect: one device supplies map, odom, TF, scan, depth, and point cloud, and the system builds maps and navigates autonomously from that single sensor suite.

### 4.5 Detection Pipeline, Saving Locations, and Safe Dynamic Navigation

Vehicle detection is produced either by the Aurora’s built-in semantic segmentation (COCO80 classes: car, truck, bus) or by a YOLO-based detector; the chosen source publishes 3D vehicle bounding boxes in the slamware_map frame. Tyre detection uses a separate YOLO model (best_fallback.pt, class “wheel”) and publishes 3D tyre boxes in the same frame. The 3D boxes are built by combining 2D detections with depth and camera intrinsics and transforming points into the map frame. The inspection manager subscribes to the vehicle and tyre topics and uses the boxes to decide where to drive.

**Committed plan.** As soon as a vehicle is chosen for inspection, the manager “commits” that vehicle: it stores the vehicle’s 3D bounding box and computes and stores the four tyre positions (front-left, front-right, rear-left, rear-right) from the box geometry and an inferred orientation (“front” is taken as the end of the vehicle closer to the robot). This committed state is not updated by later detections for that vehicle; all subsequent goals—approach and each tyre—are derived from this single committed plan. So the system effectively saves the vehicle and tyre locations at commit time and then navigates to those saved locations in a fixed order (nearest tyre first, then second, third, fourth nearest). That avoids the robot chasing moving or flickering detections and keeps behaviour deterministic.

Goals are never placed inside the vehicle box; they are placed at a standoff (0.5 m for tyres, 0.7 m for the vehicle approach) in front of the object. The goal generator computes the nearest point on the box to the robot and then places the goal at that point plus offset along the direction from robot to object, so the robot always drives to a safe pose in front of the car or tyre. An early bug in `estimate_tire_positions_from_box` used a fixed wheelbase (2.7 m) regardless of the actual vehicle box size; for a 1 m box, the front tyres were placed far in front of the bumper. This was fixed by clamping tyre positions to the vehicle's front and rear faces so that goals lie on the actual vehicle geometry. Before sending any goal, the manager validates that the goal is finite, in the correct frame, and not inside the committed vehicle box; if validation fails, the goal is rejected and the next target is tried. The costmap is updated continuously from the Aurora’s scan and point cloud, so the vehicle and other obstacles remain marked as occupied. Nav2 plans paths around those occupied regions and respects the inflation radius. In short: detection and commit give the target locations; the costmap and Nav2 ensure the robot moves to those targets only along safe, obstacle-free paths.

Observability and robustness measures include per-tire goal source logging (`planned`, `detection`, `detection_refined`) in mission reports; a web dashboard (`inspection_dashboard`) for real-time monitoring; spatial filters that require detections within 0.5 m of the expected tyre and a vehicle proximity filter to ignore distant false positives; and EMA smoothing (`refinement_ema_alpha: 0.3`) to reduce flicker in goal refinement.

### 4.6 Hardware Integration and Extensibility for Production

The hardware is used so that a single sensor device (the Aurora) provides mapping, localisation, and obstacle input: map and odom for “where we are,” TF for “where everything is in one frame,” and scan plus point clouds for “what is in the way.” The chassis (Waveshare UGV), ESP32 motor controller, and host (Jetson or Pi) are integrated via ROS 2 topics and actions: Nav2 sends velocity commands on `/cmd_vel`, the base driver forwards them to the ESP32, and the inspection manager sends goals and receives results without depending on the exact sensor hardware. The physical setup (rover with onboard computer and Aurora) will be shown in Figure 1. This design is intended to be built upon for further product development. When the system is taken towards a production-ready product, the same architecture can be retained: replace or upgrade the sensor unit (e.g. a different or next-generation Aurora or another SLAM device) by ensuring it publishes compatible topics and TF; adjust costmap and frame parameters; and keep the inspection manager and mission logic unchanged. The proof-of-concept nature of the current work is discussed in Chapter 7.

### 4.7 Model Training (Before First Jetson Failure)

Prior to the hardware failure described in Chapter 5, the author trained detection models for the project. YOLO-based models were used to detect vehicles, people, and vehicle components. The pipeline was designed to produce 3D bounding boxes from 2D segmentation masks and depth: 2D segments (e.g. from YOLO or from the Aurora semantic segmentation) are combined with depth and camera intrinsics and transformed into the map frame, yielding 3D boxes for vehicles and, later, for tyres. A dedicated wheel model was trained to detect wheels and related vehicle parts (bumpers, doors, mirrors, lights, etc.). This model is stored as best_fallback.pt and is used at inspection time to confirm that a wheel is in view before triggering a photograph. The model includes a “wheel” class (class id 22) and other classes (e.g. back_bumper, front_door, hood) that could support future extensions (e.g. pose validation or damage inspection). All of this training was completed before the first Jetson failed; the models and pipeline design were later restored and integrated once replacement hardware was available.

### 4.8 Software Architecture (Current Design)

The current architecture is documented in the repository (ARCHITECTURE.md, RUNBOOK.md, and related docs). The ugv_nav package handles Aurora bringup, Nav2 (with costmaps fed by Aurora scan, depth point cloud, and Aurora point cloud), and map reset on each mission start so that the map reflects the current scene. The ugv_base_driver subscribes to /cmd_vel and forwards velocity commands to the Waveshare UGV’s ESP32 over UART. The segment_3d pipeline combines Aurora semantic segmentation (and optionally YOLO vehicle boxes) with depth to produce vehicle 3D bounding boxes, and runs YOLO (best_fallback.pt) for wheel detection to produce tyre 3D boxes; all boxes are published in slamware_map. The inspection_manager implements the mission state machine (IDLE, SEARCH_VEHICLE, WAIT_VEHICLE_BOX, APPROACH_VEHICLE, WAIT_TIRE_BOX, INSPECT_TIRE, FACE_TIRE, VERIFY_CAPTURE, and so on). It commits the first detected vehicle, computes four tyre positions from the vehicle box using a geometric model (wheelbase, track), and sends navigation goals in a deterministic order: first the nearest tyre to the robot, then the second, third, and fourth. Goals are placed at a configurable offset (e.g. 0.5 m) in front of each tyre so that the robot stops facing the tyre, and no goal is ever placed inside the vehicle bounding box. The costmap is used by Nav2 for obstacle avoidance; the inspection manager does not override it. The photo_capture_service is triggered by the inspection manager at each tyre and records images and metadata (pose, tyre label, timestamp). Vehicle detection is configured to use the Aurora’s built-in semantic segmentation (COCO80: car, bus, truck) as the primary source, with an optional YOLO fallback; tyre detection at each stop uses best_fallback.pt. Each mission starts with a fresh map (Aurora map reset 6 seconds after launch by default) so that the costmap and detection correspond to the current scene.

### 4.9 Simulation Environment

To test the perception pipeline and mission logic without a real vehicle, a simulation environment was developed. A mock Aurora node (`aurora_mock`) publishes synthetic odometry, scans, depth, and semantic labels. A `synthetic_vehicle_publisher` places a virtual car in the simulation. A `simulated_detection_node` publishes ground-truth tyre detections, allowing the mission state machine to be exercised end-to-end. Configurable noise, latency, and dropout can be applied to test robustness. A multi-vehicle simulation was added to test mission sequencing. The `verify_system.py` script, when run with `--simulate`, publishes fake vehicle and tyre detections to exercise the state machine without a real vehicle or Aurora. Simulation was used to validate the state machine, tyre goal generation, and Nav2 integration before live testing (Section 6.7).

---

## 5. Implementation and Development

### 5.1 First Hardware Failure: Jetson and Aurora

Upon receiving the SLAMTEC Aurora, the author connected it to the Jetson via Ethernet. Shortly thereafter, a hardware failure occurred: the Jetson exhibited a constant red power indicator and became unusable. The Aurora also failed, and the author believes that a voltage or current surge affected both devices simultaneously, although the root cause has not been conclusively identified.

The Jetson was examined in detail. A multimeter was used to check every relevant pin (including J14); no short circuit was found on any pin. A general voltage drop across the Jetson was observed, consistent with a rail power fault. The Jetson did not recover. Because the author had not pushed code to the project repository for approximately three weeks, all development from that period—including the latest integration work and any uncommitted changes—was lost. This resulted in a setback of roughly three weeks and underscored the importance of regular version control and backup.

### 5.2 Aurora Diagnostics and Replacement

The first Aurora unit was disassembled for diagnostics. The author gained access to the device’s internal interface (e.g. via the shell and a connected monitor, keyboard, and mouse) and confirmed that the fault was hardware-related and that the unit needed to be replaced. The supplier was contacted and, via remote access from the author’s laptop to the Aurora, confirmed the hardware fault and arranged a replacement. Obtaining the replacement took considerable time and delayed the project by several weeks.

### 5.3 Migration to Raspberry Pi 5

While waiting for the replacement Jetson and Aurora, the project was migrated to a Raspberry Pi 5. A buck converter was used to regulate the voltage from the Waveshare UGV base (which had previously powered the Jetson) so that the Pi could be powered from the same source. The wiring was adapted accordingly. A Coral AI accelerator was added to the Pi to speed up inference. Ubuntu 24.04 (64-bit, desktop) was installed on the Pi and configured for ROS 2 Humble. The codebase was integrated and run on this platform.

However, the Raspberry Pi 5 was not deemed a viable long-term platform for this thesis. Compatibility with the desired version of ROS 2 and with the YOLO and inference stack was more straightforward on the Jetson. The Jetson was also considered more robust and better suited to the production-oriented goals of the project. The author therefore requested that the company purchase a new Jetson, and the project was moved back to the Jetson once the replacement unit and the replacement Aurora were available.

### 5.4 Return to Jetson and Current Stack

With the new Jetson and the replacement Aurora in place, the project was restored and development continued. The detection models (vehicle and wheel) and the pipeline design were reinstated. The current stack runs on the Jetson with the Aurora connected via Ethernet. The Waveshare UGV Rover (six-wheel, 4WD, ESP32-controlled) carries the Jetson and the Aurora; the ugv_base_driver forwards /cmd_vel to the ESP32. The full bringup (Aurora, segment_3d, Nav2, inspection manager, photo capture) is launched via a single script (e.g. scripts/startup.sh or mission_launch.sh), with the inspection manager and photo service starting after a delay (e.g. 120 s) so that Nav2 and TF are ready. The mission can auto-start when TF and Nav2 are available, or it can be started manually via a service or topic.

### 5.5 First Outdoor Mission and Diagnosis

During the first outdoor mission with a real vehicle, the robot drove to the middle of the vehicle's side (the door) instead of a tyre and took a photograph there, logging it as a tyre. Two causes were identified: (1) the goal generation error described in Section 4.5, which placed the first goal incorrectly—fixed by clamping tyre positions to the vehicle faces; and (2) a false tyre detection—the YOLO model detected a "wheel" on the vehicle side (confidence 0.79), which triggered the centroid servo and photo capture. After the false capture, the robot attempted to go to a second tyre but got stuck due to TF extrapolation errors and control-loop misses; the mission entered ERROR. Diagnosis showed that the YOLO node was running on CPU (2–3 s per frame), starving Nav2 and causing TF delays. Immediate fixes were applied: the YOLO confidence threshold for tyre detections was raised from 0.65 to 0.8; `yolo_stale_s` was lowered from 2.0 to 0.5 so the merger falls back to PCL sooner if YOLO is uncertain; a spatial filter was added in the inspection manager to reject detections far from the expected tyre; and vehicle YOLO was disabled by default (`use_vehicle_yolo:=false`) because the Aurora provides vehicle boxes, freeing GPU and CPU.

### 5.6 GPU Acceleration and TensorRT

YOLO on CPU made real-time navigation impossible; GPU inference was required. The Jetson Orin runs JetPack 6.0 (L4T R36.4.7) with CUDA 12 and cuDNN 9. The PyTorch wheels provided by NVIDIA for JetPack 6.0 were compiled against cuDNN 8, causing `libcudnn.so.8` missing errors. Symlinking cuDNN 9 to cuDNN 8 failed because the library internals differed. As a workaround, the YOLO model was exported to TensorRT via `trtexec` (ONNX to engine); the resulting engine gave approximately 5 ms inference, and the `ultralytics_node` was modified to load it automatically when present. However, the Ultralytics library still imports PyTorch and checks for CUDA; with CPU-only PyTorch, this caused an `AssertionError: Torch not compiled with CUDA enabled`. The solution was to install cuDNN 8 alongside cuDNN 9 (manually adding the NVIDIA cuDNN 8 repository) and reinstall the PyTorch wheel. Disk space on the Jetson’s 64 GB eMMC became critical during development—the root filesystem reached 100% full after many builds and logs. A `scripts/cleanup.sh` script was created to remove large directories (`build/`, `log/`, `install/`) and manage space; see Appendix B.

### 5.7 CUDA Out-of-Memory and Inference Optimisation

During extensive live-vehicle testing, the YOLO-based tyre detection node frequently crashed with CUDA out-of-memory (OOM) errors. A CUDA OOM occurs when the GPU’s unified memory (on the Jetson, shared between CPU and GPU) is exhausted by the combined demands of all processes. Real-time inference requires the model weights, activations, and intermediate tensors to reside in GPU memory; when the Aurora SDK (depth processing, semantic segmentation), Nav2 costmap generation, and YOLO inference compete for the same 8 GB, allocation fails and the process terminates. A representative error log is shown below:

```
[ultralytics_node-13] RuntimeError: CUDA error: out of memory
[ultralytics_node-13] CUDA kernel errors might be asynchronously reported on the next operation.
[ultralytics_node-13] TORCH_CUDA_ARCH_LIST="8.7" ...
```

Profiling of GPU memory usage confirmed that the Jetson Orin’s 8 GB unified memory was fully utilised by the Aurora SDK (depth, semantic labels, point cloud), costmap maintenance, and YOLO inference. Even after optimising input size (224×224), inference frequency (5 Hz), and model load delay (10 s), the system remained unstable during multi-tyre missions. Crashes typically occurred when the robot approached a tyre and the detection pipeline was under load.

Given the production requirements of the Tyrecheck project, the decision was made—in consultation with project supervisors and Tyrecheck engineers—to purchase a **16 GB NVIDIA Jetson Orin Nano Developer Kit**. The additional memory provides sufficient headroom to run all necessary processes (SLAM, depth processing, costmap generation, and YOLO inference) concurrently without OOM. The upgraded platform will serve as the target for final field trials and production validation.

Multiple mitigations were tried before this decision: lowering input size (`imgsz` from 640 to 480, 320, and 224); reducing maximum detections (`max_det` from 100 to 50); disabling FP16 (counterproductive); model load delay (5 s, then 10 s) to allow other GPU-using nodes to settle; and lower inference frequency (10 Hz to 5 Hz). TensorRT export itself sometimes failed with `NvMapMemAllocInternalTagged` errors; an export at 224×224 with `workspace=128` succeeded but the resulting engine produced invalid class indices (37, 44, 45), suggesting COCO80 class mappings in the ONNX or model file. A parameter type mismatch caused launch crashes: `prefer_tensorrt_inspection` was declared as a string but the launch system passed a boolean; this was fixed using an OpaqueFunction in `segment_3d.launch.py`. NMS (non-maximum suppression) timeouts (`WARNING  NMS time limit 2.050s exceeded`) indicated that post-processing on CPU was blocking. The node now discards detections with class IDs outside the valid range (0–22).

### 5.8 Aurora Custom-Model Investigation and SLAMTEC Collaboration

To offload tyre detection from the Jetson and eliminate OOM errors, the author investigated whether a custom YOLO model could run on the Aurora’s onboard AI processor. The `py_aurora_remote` SDK exposes only two pre-installed semantic segmentation models (COCO and Cityscapes); no API for uploading or loading custom models was found. Port scanning revealed port 5555 (ADB) open on the Aurora; root shell access was obtained. The Aurora runs Ubuntu 20.04 on a Rockchip ARM64 CPU with 7.7 GB RAM, Python 3.8, and pip; PyTorch and Ultralytics were successfully installed. However, the Aurora’s camera is proprietary and only accessible through its SDK. Attempts to access it via OpenCV on `/dev/video*` after stopping Aurora services failed with "select() timeout"—the camera is not a standard V4L2 device. The built-in COCO and Cityscapes models do not include a wheel class.

The conclusion is that running a custom YOLO model on the Aurora is not feasible with the current SDK. Despite this limitation, SLAMTEC technical support was contacted (early 2025) to enquire whether custom model deployment could be enabled in future firmware or via developer channels. The author provided application details (tyre inspection use case, custom wheel-detection model) and the team has initiated internal research on the matter. This collaboration is ongoing; any findings from SLAMTEC regarding custom model support will be incorporated into subsequent project phases. If SLAMTEC enables custom model deployment in future, offloading inference to the Aurora would reduce host load and power consumption. The investigation demonstrates thorough exploration of all options—including on-device AI offloading—and engagement with the sensor manufacturer’s technical team.

### 5.9 CPU Inference Fallback

With the Aurora path blocked and GPU inference unstable, a CPU-based inference node was implemented using ONNX Runtime with `CPUExecutionProvider`. It runs at 224×224 resolution and 5 Hz, consuming negligible GPU memory. The node is integrated via a `use_cpu_inference` parameter (default `true`). The system now starts without OOM; GPU memory remains stable (typically under 1 GB for tyre detection, around 4 GB total). The trade-off is a small amount of CPU overhead for guaranteed stability, enabling the robot to complete missions without crashes.

### 5.10 Additional Robustness Improvements

Further improvements include: distance-based visual servoing, where the centroid servo gains are scaled by the remaining distance to the tyre to prevent overshoot during final approach; photo capture metadata, with filenames including tyre number and vehicle ID and a manifest CSV with a `tire_number` column; and documentation updates in `TIRE_DETECTION_TROUBLESHOOTING.md` and the RUNBOOK for OOM workarounds, CPU inference usage, and parameter explanations.

---

## 6. Current State and Testing

### 6.1 What Has Been Completed

The following has been implemented and documented. Vehicle detection uses the Aurora’s built-in semantic segmentation (COCO80: car, truck, bus) to produce 3D vehicle bounding boxes in slamware_map, with an optional YOLO fallback if the Aurora does not publish semantic data. Tyre detection uses the YOLO model best_fallback.pt (class “wheel”) when the robot is near a tyre position to confirm the wheel is in view before capture. The mission logic is implemented in a state machine that commits one vehicle, computes four tyre positions from the vehicle box (orientation inferred from robot position), and visits tyres in order: nearest first, then second, third, and fourth nearest. Navigation is handled by Nav2 with costmaps driven by Aurora scan, depth point cloud, and Aurora point cloud; inflation and footprint clearing avoid collision with the vehicle, and goals are rejected if they lie inside the vehicle box. By default the Aurora map is reset 6 seconds after launch so that each mission runs on a fresh map and the costmap reflects the current scene. Safety and robustness measures include TF and Nav2 availability checks before mission start, a TF watchdog, timeouts to avoid unbounded spinning, spin protection (max state repeats), and approach and tyre timeouts with recovery to wait states. The repository documents the architecture, mission tyre order, approach specification for arbitrary robot placement, navigation safety, Aurora vehicle detection, runbook, and production configuration [6, 7, 8]. A TensorRT engine exists and can be used when PyTorch with CUDA is available; the CPU inference path is validated and the robot launches reliably with CPU-based tyre detection. The mission state machine has been validated in simulation via `verify_system.py --simulate`.

The hardware upgrade is on order: a 16 GB Jetson Orin Nano Developer Kit will replace the current 8 GB unit to eliminate CUDA OOM during production runs. SLAMTEC technical support has been contacted regarding custom model deployment; this collaboration is ongoing (Section 5.8).

### 6.2 What Remains in Progress

The system is not yet fully autonomous in all real-world conditions. Work in progress includes perfecting autonomous navigation to all four tyres from any reasonable starting pose (front, back, side, or diagonal) without getting stuck or spinning; validating behaviour with different vehicles, lighting, and ground conditions; and running full missions repeatedly to document success and failure modes. The design is in place and the codebase reflects a single, well-defined behaviour; the remaining work is to harden execution and validate it in the target environment. For the GPU path, the PyTorch wheel reinstall remains the final technical blocker where applicable. The model file (`best_fallback.pt`) may retain COCO class mappings; verification is needed for a clean TensorRT export if invalid class indices persist.

### 6.3 Testing Undertaken

Testing has included unit tests for components (goal generator, vehicle model, perception helpers), verification of the mission state machine and tyre order logic, and runs with the full stack on the Jetson and Aurora. Failures observed during development led to concrete fixes: for example, dispatching the first goal only from the timer (tick) and not from the vehicle callback when using “nearest corner” mode, so that the tyre list is always sorted by distance before the first goal is sent; and enforcing a strict tyre order (second, third, fourth nearest) so that behaviour is deterministic. Documentation such as MISSION_TIRE_ORDER_AND_SCENARIO.md and TIRE_INSPECTION_APPROACH_SPEC.md was written so that the intended behaviour is unambiguous and traceable to the code.

### 6.4 Months of Testing and Iteration

The thesis does not only describe a final design; it reflects a long period of testing and refinement. Over many months the author has run the system again and again—with live cars, in dry runs, with one vehicle and with multiple vehicles, and with different detection and navigation settings. The repository history and the project documentation are the record of that work.

**Inspection manager and approach logic.** Multiple designs for the inspection manager were tried. Early versions dispatched the first goal from the vehicle-detection callback, which could send the robot towards the nearest *face* of the vehicle (e.g. the bumper) instead of the nearest *tyre*; that was corrected so that the first goal is always the nearest tyre, and dispatch happens from the main tick after sorting tyres by distance. The choice between “approach nearest face” and “approach nearest corner (tyre)” was parameterised (approach_nearest_corner); the order of the remaining tyres was made strict (strict_planned_tire_order) so that the robot always visits the 2nd, 3rd, and 4th nearest tyre in that order instead of letting live detection reorder them. These changes are documented in MISSION_TIRE_ORDER_AND_SCENARIO.md and TIRE_INSPECTION_APPROACH_SPEC.md and are enforced in the code so that behaviour is deterministic for any robot placement around the vehicle [7, 8].

**Navigation and map behaviour.** The way the robot navigates to each goal was refined repeatedly. The interaction with Nav2’s navigate_to_pose action was tuned (e.g. goal-in-flight guards, nav_goal_min_interval_s) to avoid flooding the planner. The decision to build a new map on each mission start—resetting the Aurora map a few seconds after launch—came from testing: using a fresh map each time avoids confusion from stale or incorrectly loaded maps and keeps the costmap and detection aligned with the current scene [8, 9]. Map reset, TF stability before mission start, and costmap observation sources (scan, depth point cloud, Aurora point cloud) are documented in the runbook and in NAVIGATION_SAFETY.md [6, 9].

**Failure modes and forensics.** Dozens of failure modes have been identified and documented (VEHICLE_INSPECTION_FAILURE_MODES.md): bounding box issues (jitter, wrong orientation, Aurora 1×1 m fixed box), costmap issues (lag, wrong frame, inflation), navigation issues (goal inside vehicle, path blocked, oscillation), detection and committed-plan issues, and mission state machine issues [10]. For each, mitigations or known limitations are noted. When the robot spins or makes no progress, forensics rely on mission logs (e.g. mission_latest.jsonl) and reports (mission_report_latest.json), with explicit checks for tf_watchdog, tick_skipped_tf_invalid, approach_dispatch_attempt, wait_vehicle_timeout, spin_detected, progress_stall, and goal_offset_validation_failed [6, 11]. Acceptance criteria for a basic acceptance run—vehicle detected within 30 s, approach goal dispatched, no drive into car, four tyres visited in distance order, four photos captured, no prolonged spin—are defined in ACCEPTANCE_CRITERIA.md and validated with pre_mission_verify.sh and the same log/report paths [11].

**Volume of testing.** The author has pushed to the project repository a large number of commits across many files: changes to the inspection manager (state machine, dispatch logic, tyre order), to navigation parameters and costmap configuration, to the detection pipeline and vehicle/tyre topics, and to launch files and runbooks. Dry runs, live runs with a single car, and runs with multiple cars have been performed repeatedly. Different approaches to the car (nearest face vs nearest tyre, strict vs detection-driven tyre order) were tried and compared. The result is a codebase and a set of documents that capture not only the current behaviour but the iterations that led to it. This thesis is the written account of that process: months of testing and refinement, not a single snapshot.

### 6.5 Figures and Physical Setup (To Be Inserted)

The author will include photographs of the physical system to support the thesis. Planned figures include:

- **[Insert Figure 1: Robot in operation.]** The autonomous rover (Waveshare UGV with onboard computer and Aurora).
- **[Insert Figure 2: Aurora disassembled for diagnostics.]** The SLAMTEC Aurora unit (hardware failure and replacement, Section 5.2).
- **[Insert Figure 3: Orviz unit.]** The Orviz unit (or equivalent sensor/interface unit) as used in the setup.
- **[Insert Figure 4: Tyre detection configuration.]** The tyre detection unit or camera/sensor configuration used for wheel detection and photo capture.

Captions and placement will be finalised when the images are inserted. Figures should be referenced in the text where relevant (e.g. “as shown in Figure 1”).

### 6.6 Lessons Learned

**What went right.** The simulation-first approach caught many bugs early (goal generation, TF timing). Incremental testing—single tyre, then four tyres, then real vehicle—allowed issues to be isolated. Observability (goal source logging, mission reports) was invaluable for debugging. The PCL fallback ensures robustness when YOLO fails. TensorRT, when it worked, provided fast inference. The cleanup script helped manage disk space. Installing cuDNN 8 alongside cuDNN 9 satisfied PyTorch CUDA requirements.

**What went wrong.** The goal generation assumption (fixed wheelbase for all vehicles) was wrong; clamping to box faces fixed it. False tyre detections (YOLO seeing wheels on the vehicle side) were addressed by raising confidence and adding spatial filtering. The PyTorch–cuDNN incompatibility (JetPack 6.0 ships cuDNN 9, PyTorch wheels require cuDNN 8) caused weeks of effort. Disk space on the Jetson's 64 GB eMMC filled up repeatedly. A Docker detour (NVIDIA L4T PyTorch container) encountered signature errors and space issues. The Aurora is not user-programmable for custom AI; the camera is proprietary. NMS post-processing proved a bottleneck.

**Key decisions.** Vehicle YOLO was disabled by default to save GPU/CPU and avoid confusion with Aurora vehicle boxes. TensorRT (or CPU) is used for tyre detection when PyTorch CUDA is unavailable. The original `.pt` file is kept for future re-exports. Spatial and confidence filters reduce false detections.

### 6.7 Path Forward

With the 16 GB Jetson Orin Nano on order and the software pipeline validated in simulation (Section 4.9) and testing (Chapter 6), the next step is to conduct a full mission on the upgraded hardware with a real vehicle. The simulation environment—including `verify_system.py --simulate`, the stub motor for no-move testing, and the remapped inspection manager—has confirmed that the mission state machine, tyre goal generation, and Nav2 integration behave correctly. Once the new Jetson is installed, the same codebase will be deployed; the additional memory is expected to eliminate CUDA OOM and allow stable GPU inference alongside the full stack.

Pending SLAMTEC’s response on custom model deployment, the option to offload inference to the Aurora remains a future optimisation. For the immediate path to production validation, the project will proceed with the upgraded Jetson and the existing CPU or GPU inference paths as appropriate.

---

## 7. Conclusions and Future Work

### 7.1 Summary

This thesis has described the development of an autonomous tyre inspection robot from a problem posed by Tyrecheck: to move from a drive-over inspection system to one in which a mobile robot carries the cameras to the vehicle. The project adopted ROS 2 and explored several hardware configurations before settling on the Waveshare UGV Rover, the NVIDIA Jetson, and the SLAMTEC Aurora. Detection models for vehicles and wheels were trained and integrated; the Aurora’s semantic segmentation and depth are used for vehicle detection and for the costmap, and a custom YOLO model (best_fallback.pt) is used for wheel detection at each tyre stop. A mission state machine commits the detected vehicle, computes four tyre positions, and navigates to them in a deterministic order—nearest first, then by distance. Significant setbacks were encountered: the simultaneous failure of the first Jetson and first Aurora, the loss of uncommitted work, and a temporary migration to a Raspberry Pi 5. Those were partially mitigated, and the project now runs on replacement hardware. Further challenges—CUDA out-of-memory errors on the 8 GB Jetson and PyTorch–cuDNN incompatibilities—led to a CPU inference fallback and, after consultation with supervisors and Tyrecheck engineers, the decision to upgrade to a 16 GB Jetson Orin Nano. The author investigated offloading inference to the Aurora’s onboard AI; the current SDK does not support custom models, but SLAMTEC technical support was contacted and has initiated internal research on the matter. A long period of testing and refinement—different inspection-manager designs, different approach strategies, refinements to navigation and map behaviour, and repeated runs with live cars and dry runs—is reflected in the repository and in the project documentation. The system is in a phase of perfecting autonomous navigation and real-world robustness; the simulation-validated software is ready for integration once the upgraded hardware arrives.

### 7.2 Proof of Concept and Path to Production

This project is explicitly a proof of concept and a first draft towards both a thesis and a viable product. The aim is not to deliver the final production system but to prove that autonomous tyre inspection by a mobile robot is feasible and can be realised at a production level. The hardware in use—the Waveshare UGV, the Jetson, and the SLAMTEC Aurora—is testing hardware provided by Tyrecheck so that the author could validate the approach before the company invests in production-grade hardware and deployment. The thesis demonstrates that the full pipeline (detect vehicle, build map, navigate safely around it, visit each tyre, capture photographs) can be implemented and integrated with ROS 2; that sensors can be swapped, as when the earlier OAK and 2D LiDAR were replaced with the Aurora, without rewriting the mission logic; and that the system uses the Aurora to build maps and navigate autonomously using TF, odometry, and live costmap updates. The decision to upgrade to a 16 GB Jetson Orin Nano—taken in consultation with supervisors and Tyrecheck engineers—addresses the CUDA OOM limitation and will support production-level validation. The ongoing collaboration with SLAMTEC regarding custom model deployment demonstrates thorough exploration of all options and engagement with the sensor manufacturer. Once feasibility is proven, Tyrecheck can take the next step: investing in production hardware and rolling out the solution across their businesses. The company operates in bus depots, truck yards, trailer yards, and retail contexts worldwide; a successful proof of concept shows that an autonomous tyre inspection robot can be deployed in those environments, with appropriate hardening and compliance, so that the same inspection quality currently achieved at drive-over points can be offered where vehicles are stationary.

### 7.3 Future Work

Immediate next steps are: (1) to achieve reliable end-to-end autonomous runs (detect, navigate to all four tyres, capture four photos) across a range of starting positions and vehicle types on the upgraded 16 GB Jetson; (2) to tune timeouts, recovery behaviour, and costmap parameters based on field tests; and (3) to document failure modes and acceptance criteria for a pilot or production deployment. The simulation-validated software is ready for integration once the hardware arrives. For the tyre detection pipeline: diagnose `best_fallback.pt` for residual COCO mappings and re-export TensorRT if the model is clean; profile inference to separate GPU compute time from NMS, and consider TensorRT's EfficientNMS plugin if post-processing is the bottleneck; use CPU inference at very low resolution as a fallback if the GPU path remains unstable; and use the PCL fallback as temporary tyre detection if YOLO optimisation fails. Field validation with a real vehicle should fine-tune visual servoing and ensure robust operation in varied lighting. Longer-term directions could include: multi-vehicle missions; integration with Tyrecheck’s existing image analysis pipeline; optional use of the additional classes in best_fallback.pt (e.g. bumpers, doors) for pose validation or damage inspection; memory leak monitoring, watchdog mechanisms, and auto-reboot on node crash; and, on Tyrecheck’s side, selection of production hardware and rollout across bus depots, truck yards, trailer yards, and retail sites. If SLAMTEC enables custom model deployment on the Aurora, offloading inference to the sensor would be a further optimisation to reduce host load and power consumption.

---

## References

[1] SLAMTEC, “Aurora – Integrated positioning and mapping sensor,” SLAMTEC. [Online]. Available: https://www.slamtec.com/en/Aurora  
[2] SLAMTEC, “Aurora ROS2 SDK,” SLAMWARE ROS SDK documentation. [Online]. Available: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/  
[3] SLAMTEC, “Aurora – Spec,” SLAMTEC. [Online]. Available: https://www.slamtec.com/en/Aurora/Spec  
[4] Project documentation: docs/aurora_topics_and_frames.md — Aurora topics and TF tree (slamware_ros_sdk_server_node).  
[5] Project documentation: src/Tyre_Inspection_Bot/ARCHITECTURE.md — Aurora specs and connection (weight, size, map resolution, LiDAR, camera, IMU fusion).  
[6] Project documentation: RUNBOOK.md — Canonical stack, mission flow, bulletproof guarantees, forensics (logs/mission_latest.jsonl), pre-flight checklist.  
[7] Project documentation: docs/MISSION_TIRE_ORDER_AND_SCENARIO.md — Mission flow, tyre order, first goal = nearest tyre, strict_planned_tire_order.  
[8] Project documentation: docs/TIRE_INSPECTION_APPROACH_SPEC.md — Behaviour for any robot placement angle, commit, distance order.  
[9] Project documentation: docs/NAVIGATION_SAFETY.md — Fresh map every mission, detection and costmap alignment, observation sources, inflation, offsets.  
[10] Project documentation: docs/VEHICLE_INSPECTION_FAILURE_MODES.md — Bounding box, costmap, navigation, detection, committed plan, and mission state machine failure modes and mitigations.  
[11] Project documentation: docs/ACCEPTANCE_CRITERIA.md — Acceptance run criteria, validation commands, mission log and report paths.  

[12] Open Robotics, “ROS 2 Documentation,” ROS 2 Humble. [Online]. Available: https://docs.ros.org/en/humble/  
[13] S. Macenski et al., “Nav2: A Navigational Framework for Mobile Robots,” *IEEE Robotics and Automation Letters*, vol. 7, no. 4, pp. 10006–10013, 2022.  
[14] J. Redmon et al., “You Only Look Once: Unified, Real-Time Object Detection,” in *Proc. IEEE CVPR*, 2016, pp. 779–788.  
[15] Ultralytics, “YOLOv8 Documentation,” Ultralytics. [Online]. Available: https://docs.ultralytics.com/  
[16] NVIDIA, “TensorRT Documentation,” NVIDIA Developer. [Online]. Available: https://developer.nvidia.com/tensorrt  
[17] Waveshare, “UGV Rover User Manual,” Waveshare Wiki. [Online]. Available: https://www.waveshare.com/wiki/UGV_Rover  

Further references may be added in a later draft as needed.

---

## Appendices

**Appendix A: Glossary and List of Acronyms**

**Acronyms (defined at first use in text; listed here for reference):**

- **6DOF:** Six degrees of freedom.  
- **COCO80:** Common Objects in Context, 80-class object taxonomy; Aurora semantic_labels use class IDs (e.g. 3=car, 8=truck).  
- **CUDA:** Compute Unified Device Architecture; NVIDIA’s parallel computing platform for GPU acceleration.  
- **cuDNN:** CUDA Deep Neural Network library; NVIDIA library for deep learning acceleration.  
- **LiDAR:** Light Detection and Ranging.  
- **NMS:** Non-maximum suppression; post-processing step in object detection to remove overlapping boxes.  
- **OOM:** Out of memory; CUDA OOM occurs when GPU memory is exhausted.  
- **ONNX:** Open Neural Network Exchange; format for model interoperability.  
- **PCL:** Point Cloud Library; used here for point-cloud clustering as a tyre detection fallback.  
- **ROS 2:** Robot Operating System 2; middleware for robotics applications.  
- **SLAM:** Simultaneous Localisation and Mapping.  
- **TF:** Transform; ROS 2 mechanism for publishing spatial relationships between coordinate frames.  
- **TPMS:** Tyre Pressure Monitoring System; Tyrecheck’s core business area.  
- **V4L2:** Video4Linux2; Linux API for video devices.  
- **YOLO:** You Only Look Once; real-time object detection architecture.

**Terms:**

- **Aurora / AORA:** SLAMTEC Aurora 6DOF SLAM and perception device; “AORA” is used in some project documentation.  
- **best_fallback.pt:** Custom YOLO segmentation model for wheel and vehicle component detection.  
- **best_fallback.engine:** TensorRT engine exported from the YOLO model for GPU inference.  
- **drive-over system:** Existing Tyrecheck system where vehicles drive over a fixed point to trigger tyre imaging.  
- **prefer_tensorrt_inspection:** Launch parameter controlling whether the TensorRT engine is preferred for tyre inference.  
- **slamware_map:** Map frame used by the Aurora and by the inspection pipeline for bounding boxes and goals.  
- **TensorRT:** NVIDIA inference runtime for optimised GPU execution.  
- **tire_merger:** Node that merges YOLO and PCL tyre detections.  
- **Tyrecheck:** Company for which this thesis was undertaken; TPMS and tyre inspection, Prague branch.  
- **use_cpu_inference:** Launch parameter (default true) enabling CPU-based ONNX Runtime inference for tyre detection.  
- **yolo_stale_s:** Parameter controlling how long before the merger falls back to PCL when YOLO detections are stale.

**Appendix B: Repository Structure and Key Documents**  
- ARCHITECTURE.md: Hardware and software layout.  
- scripts/cleanup.sh: Disk space management and removal of unused packages (Section 5.6).  
- TIRE_DETECTION_TROUBLESHOOTING.md: OOM workarounds, CPU inference usage, parameter explanations.  
- verify_system.py: Script for system verification; `--simulate` publishes fake detections for testing.  
- docs/MISSION_TIRE_ORDER_AND_SCENARIO.md: Mission flow and tyre order.  
- docs/TIRE_INSPECTION_APPROACH_SPEC.md: Behaviour for any robot placement angle.  
- docs/NAVIGATION_SAFETY.md: Costmap, detection alignment, fresh map, safety measures.  
- docs/AURORA_VEHICLE_DETECTION.md: Aurora semantic and fallback.  
- docs/VEHICLE_INSPECTION_FAILURE_MODES.md: Failure modes and mitigations.  
- docs/ACCEPTANCE_CRITERIA.md: Acceptance run criteria and validation.  
- RUNBOOK.md: Launch, topics, forensics, operational procedures.  
- PRODUCTION_CONFIG.yaml: Production-oriented parameters.

---

*This is a first draft. The autonomous rover is not yet working in all scenarios, and the thesis will be revised as the project moves towards full autonomy and production readiness. The work serves as a proof of concept for Tyrecheck, demonstrating that autonomous tyre inspection is achievable with the provided testing hardware so that the company can pursue production-grade deployment across bus depots, truck yards, trailer yards, and retail use worldwide. The author will add photographs of the robot, the disassembled Aurora, the Orviz unit, and the tyre detection unit when those figures are available.*
