# Aurora TF, localization, and launch — no room for error

This doc explains how the **Slamtec Aurora** (slamware_ros_sdk) provides **where the robot is** and **where it’s going**, and how to avoid TF/localization failures so the mission **always** sees the car, approaches it, and continues (no spinning, no getting stuck).

---

## 0. Is slamware_map → base_link the right approach?

**Yes.** This is the standard way to use Aurora with Nav2 and a single map frame:

- **Slamtec Aurora** (via slamware_ros_sdk) provides **odometry** and **map** from the device. The SDK publishes the robot pose as **odom → base_link** (TF) and **`/slamware_ros_sdk_server_node/odom`** (topic). There is no separate EKF or localization node; the robot pose comes from Aurora.
- The mission and Nav2 need a single chain from **map** to **base_link** so that goals (in map) and robot pose (base_link) are in the same frame. We use **slamware_map** as that map frame and add a **static** transform **slamware_map → odom** (identity), so the full chain is **slamware_map → odom → base_link**. That is exactly what Nav2 and the inspection manager expect (see `world_frame` / `map_frame` / `base_frame` in config).
- So the approach is correct and required. The only failure mode we've seen is **visibility**: if the terminal running the diagnostic (or mission) uses a **different RMW or ROS_DOMAIN_ID** than the terminal that started the launch, they don't see each other's nodes, topics, or TF — even though the SDK is publishing (you'll see "Published initial odom->base_link" in the launch log). Fix: use the **same RMW** in both terminals (see Section 5).

---

## 1. TF chain (robot always knows where it is)

The mission and Nav2 need **slamware_map → base_link** so that:

- Bounding boxes (vehicle/tire) from the segmentation pipeline are in **slamware_map**.
- The robot pose is in **slamware_map** (via base_link).
- Goals are built in slamware_map and sent to Nav2 as **map** (identity when map ≡ slamware_map).

**Full chain:**

| Segment | Publisher | Frame → child | When |
|--------|-----------|----------------|------|
| **slamware_map → odom** | aurora_bringup (static_transform_publisher) | Identity | Always, from launch |
| **odom → base_link** | slamware_ros_sdk_server_node (ServerOdometryWorker) | Robot pose from device | When Aurora is **connected** and streaming pose (odometry_pub_period 0.1 s) |

So **slamware_map → base_link** is valid only when:

1. aurora_bringup has started (static slamware_map→odom).
2. slamware_ros_sdk_server_node has **connected** to the Aurora (TCP 192.168.11.1:7447).
3. The device has sent at least one pose (odom and TF odom→base_link are published).

**Typical delay:** A few seconds after “Connected to the selected device” before the first pose/TF. If the device or network is slow, it can be longer.

---

## 2. What the Aurora (slamware_ros_sdk) provides

| Capability | Topic / TF | Use in mission |
|------------|------------|-----------------|
| **Robot pose (odom)** | `/slamware_ros_sdk_server_node/odom`, TF **odom → base_link** | Inspection manager: robot pose in slamware_map for goals; Nav2: odom for controller |
| **Robot pose (map)** | `/slamware_ros_sdk_server_node/robot_pose` (slamware_map) | Redundant with TF when slamware_map≡odom (identity) |
| **Map** | `/slamware_ros_sdk_server_node/map` | Nav2 occupancy grid |
| **Scan** | `/slamware_ros_sdk_server_node/scan` | Nav2 costmap |
| **Depth / semantic** | `/slamware_ros_sdk_server_node/depth_image_raw`, `/slamware_ros_sdk_server_node/semantic_segmentation`, `/slamware_ros_sdk_server_node/point_cloud` | 3D boxes, vehicle detection (aurora_semantic_fusion) |
| **Images** | `/slamware_ros_sdk_server_node/left_image_raw`, `/slamware_ros_sdk_server_node/right_image_raw` | YOLO, segmentation |

There is **no separate EKF node** in this stack: Aurora provides odom and map; the SDK publishes **odom → base_link** from the device. Nav2 uses that odom (and map/scan) for planning and control. So “robot always knows where it is” comes from **Aurora + static slamware_map→odom + SDK odom→base_link**.

---

## 3. What can go wrong and how we prevent it

| Failure | Cause | Prevention |
|--------|--------|------------|
| **TF slamware_map→base_link invalid** | Aurora not connected, not streaming pose, or network/device blip | 1) Start Aurora first; 2) Run `scripts/aurora_tf_diagnostic.sh` before full launch; 3) Mission waits in IDLE until TF is valid and **stable 3–5 s** (tf_stable_s); 4) If TF fails mid-mission, watchdog pauses and cancels in-flight Nav2 goal |
| **Mission starts then immediately pauses** | TF was valid for one tick then dropped (e.g. right after connect) | **tf_stable_s** (e.g. 5 s): we don’t leave IDLE until TF has been valid for 5 s in a row |
| **Robot spins / never approaches** | (1) TF invalid so tick never runs approach logic; (2) WAIT_VEHICLE_BOX timed out instead of dispatching approach | (1) Above; (2) We always dispatch approach on first tick in WAIT_VEHICLE_BOX when we have current_vehicle_box (and again after recovery rotation); comprehensive logging (approach_dispatch_attempt, wait_vehicle_timeout, rotation_dispatched) |
| **No vehicle detected** | Perception not running, or TF invalid so detection callback never drives state machine | Ensure segment_3d (and aurora_semantic_fusion) are running; ensure TF valid so inspection_manager tick runs |

---

## 4. Launch order and timing (full_bringup)

| T (s) | What starts | Note |
|-------|--------------|------|
| 0 | Aurora (slamware_ros_sdk_server_node + static TFs + navigation_permitted_publisher) | TF odom→base_link appears shortly after “Connected to the selected device” |
| 8 | segment_3d (YOLO, depth pipeline, 3D boxes, aurora_semantic_fusion) | Needs Aurora topics |
| 10 | Nav2 (costmaps, planner, controller) + depth_gate | Needs Aurora map, odom, scan |
| 35 | Nav2 lifecycle (configure/activate) | Nav2 becomes ready for goals |
| 70 | Inspection manager + photo capture | Waits for Nav2 navigate_to_pose (nav2_wait_timeout) and for TF stable (tf_stable_s) before starting mission |

So by 70 s, Aurora has had ~70 s to publish TF. The mission still does **not** start until:

- Nav2 action server is available, and  
- TF slamware_map→base_link has been valid for **tf_stable_s** (e.g. 5 s) in IDLE.

---

## 5. Diagnostic: Aurora only

Before or after full launch, you can verify Aurora and TF with only the Aurora launch running.

**Critical:** Both terminals must use the **same RMW**. If the launch terminal does not have `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, the diagnostic terminal will see **no nodes and no TF** (different DDS domain).

```bash
# Terminal 1: Aurora only — set RMW first, then launch
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1

# Terminal 2: Diagnostic (same RMW; script defaults to rmw_cyclonedds_cpp)
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash ~/ugv_ws/scripts/aurora_tf_diagnostic.sh
```

**Full audit (every node, every topic, /tf, TF lookup):**

```bash
# With Aurora running in Terminal 1 (and RMW set there), in Terminal 2:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash ~/ugv_ws/scripts/aurora_ros2_audit.sh
```

If the audit shows **0 nodes**, the launch and audit terminals are on different RMW or domain: set `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in the **launch** terminal and restart aurora_bringup.

The script:

- Waits up to 30 s for **slamware_map → base_link** (6 attempts × 5 s). The first attempt may report "frame does not exist" for slamware_map because **/tf_static** (where slamware_map→odom is published) needs a moment to be received by the TF buffer; the longer per-attempt timeout allows this.
- Lists nodes and key Aurora topics: `/slamware_ros_sdk_server_node/odom`, `/slamware_ros_sdk_server_node/robot_pose`, `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/left_image_raw`, `/slamware_ros_sdk_server_node/depth_image_raw`, `/slamware_ros_sdk_server_node/point_cloud`, `/slamware_ros_sdk_server_node/semantic_segmentation`.
- Echoes the transform a few times so you can confirm the pose is updating.

If this passes, the TF chain and Aurora publishing are good; the rest (Nav2, perception, mission) can then run without “TF invalid” at start.

---

## 6. Pre-mission checklist (full stack)

1. **Aurora on, Jetson can ping it**  
   `ping -c 2 192.168.11.1`

2. **Aurora-only diagnostic (optional but recommended)**  
   Start Aurora, run `scripts/aurora_tf_diagnostic.sh`; confirm TF and topics OK.

3. **Full launch**  
   `bash scripts/startup.sh` or `ros2 launch ugv_nav full_bringup.launch.py`

4. **Mission start**  
   Mission will not leave IDLE until Nav2 is ready and TF has been valid for **tf_stable_s** (5 s). Then: detect car → dispatch approach on first tick in WAIT_VEHICLE_BOX → APPROACH_VEHICLE → drive to car → tires → photos.

5. **If something fails**  
   Check `~/ugv_ws/logs/mission_latest.jsonl` for `idle_passed`, `tf_watchdog`, `tick_skipped_tf_invalid`, `approach_dispatch_attempt`, `wait_vehicle_timeout`, `rotation_dispatched`, `mission_heartbeat`. See `docs/MISSION_FORENSIC_REPORT_SPIN_2026-02-24.md` Section 8 (log events reference).

---

## 7. Summary

- **Robot always knows where it is:** Aurora (slamware_ros_sdk) publishes **odom → base_link**; we add **slamware_map → odom** (identity). So slamware_map→base_link is the single chain the mission and Nav2 use.
- **No EKF in this stack:** Localization is Aurora odom + map; Nav2 uses them as-is.
- **No room for error on launch:** We require TF **stable for 5 s** in IDLE, cancel in-flight goals when TF watchdog fires, and always dispatch approach when we have the vehicle box (no spin loop). Run `aurora_tf_diagnostic.sh` with Aurora only to verify TF and topics before or during debugging.
