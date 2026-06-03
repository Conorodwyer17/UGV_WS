# Jetson Engineering Brief — Complete Tyre Inspection Robot Overhaul

**Date**: 2026-06  
**Audience**: CloudCode agent running directly on the Jetson Orin Nano  
**GitHub repo**: https://github.com/Conorodwyer17/UGV_WS  
**Workspace on Jetson**: `~/ugv_ws`  
**Objective**: Make the robot autonomously drive to each tyre on a vehicle and photograph it. It currently does not do this.

---

## What You Are Working With

This is a real physical robot. It has three hardware components attached:

1. **SLAMTEC Aurora** — connected via Ethernet at IP `192.168.11.1`. This is the primary sensor: it provides 2D LiDAR scan, SLAM map, odometry (`odom → base_link`), stereo depth image, and COCO80 semantic segmentation (detects cars, trucks, buses). This is NOT the Aurora S. It is the base Aurora unit.

2. **Waveshare UGV Rover base** — connected via UART at `/dev/ttyTHS1` through an onboard ESP32. The ESP32 controls the motors. ROS sends `cmd_vel` (Twist) which the motor driver converts to Waveshare JSON protocol: `{"T":13,"X":<linear>,"Z":<angular>}`.

3. **Jetson Orin Nano** — the onboard computer running ROS 2 Humble. This is where everything runs.

The robot must:
1. Start up safely
2. See a parked vehicle using the Aurora's COCO80 semantic segmentation
3. Drive up to the vehicle (stopping ~2m away)
4. Detect the four tyres using a YOLO model (`tyre_detection_project/best.pt` or `best.onnx`)
5. Navigate to each tyre (~1m standoff, robot facing the tyre)
6. Photograph each tyre and save the image
7. Repeat for all four tyres
8. Complete and stop

The robot does NONE of this reliably today. You must fix it.

---

## Repository Setup — Do This First

```bash
cd ~/ugv_ws
git pull origin main
git log --oneline -5   # Confirm commit 2595361 is present
```

If not already built:
```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install 2>&1 | tail -30
source install/setup.bash
```

Check build errors immediately. Fix any package that fails to build before proceeding.

---

## Phase 0 — Hardware Verification (Do Before Touching Code)

You must verify each hardware component works independently before trying the full stack.

### 0.1 — Aurora Connectivity

```bash
ping -c 3 192.168.11.1
```

Expected: 0% packet loss, <5ms latency. If ping fails, the Aurora is not connected or powered. Do not proceed.

```bash
# Check Aurora IP and Ethernet interface
ip addr show   # find the interface with 192.168.11.x subnet
```

If ping succeeds, check that the Aurora ROS SDK node can connect:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run slamware_ros_sdk slamware_ros_sdk_server_node --ros-args -p ip_address:=192.168.11.1
```

Wait 10 seconds. Then in another terminal:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list | grep slamware
```

Expected topics (minimum):
- `/slamware_ros_sdk_server_node/odom`
- `/slamware_ros_sdk_server_node/scan`
- `/slamware_ros_sdk_server_node/map`
- `/slamware_ros_sdk_server_node/left_image_raw`
- `/slamware_ros_sdk_server_node/depth_image_raw`
- `/slamware_ros_sdk_server_node/semantic_labels`

If any of these are missing, check the Aurora firmware version. The full native mode requires firmware ≥ 2.11. Run:
```bash
ros2 topic echo /slamware_ros_sdk_server_node/odom --once
```
If this hangs, Aurora is not publishing. Kill the node and debug the connection.

### 0.2 — Motor Driver (ESP32 / UART)

```bash
ls -la /dev/ttyTHS1   # Must exist
```

If missing, check `/dev/ttyUSB0`, `/dev/ttyACM0`:
```bash
ls /dev/tty* | grep -E "THS|USB|ACM"
```

Test UART manually:
```bash
python3 -c "
import serial, json, time
s = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
# Config: UGV Rover
s.write(json.dumps({'T':900,'main':2,'module':0}).encode()+b'\n')
time.sleep(0.5)
# Send zero velocity
s.write(json.dumps({'T':13,'X':0.0,'Z':0.0}).encode()+b'\n')
time.sleep(0.2)
# Read any response
for _ in range(10):
    line = s.readline()
    if line: print(line.decode(errors='ignore').strip())
s.close()
print('Done')
"
```

Expected: JSON lines from ESP32 (T:1001 feedback with odometer values). If nothing is printed, check the UART port name.

Test robot actually moves (ONLY on flat safe surface with person watching):
```bash
python3 -c "
import serial, json, time
s = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
s.write(json.dumps({'T':900,'main':2,'module':0}).encode()+b'\n')
time.sleep(0.3)
print('Moving forward 0.5m/s for 1 second...')
s.write(json.dumps({'T':13,'X':0.1,'Z':0.0}).encode()+b'\n')
time.sleep(1.0)
s.write(json.dumps({'T':13,'X':0.0,'Z':0.0}).encode()+b'\n')
print('Stopped')
s.close()
"
```

If the robot does not move, the motor driver is broken. Fix the UART connection or port path first.

### 0.3 — YOLO Tyre Model

```bash
ls -la ~/ugv_ws/tyre_detection_project/
```

Required: `best.pt` AND `best.onnx` (for CPU inference).

If `best.onnx` is missing, generate it:
```bash
cd ~/ugv_ws
source install/setup.bash
MODEL_PT=tyre_detection_project/best.pt IMGSZ=480 bash scripts/export_onnx.sh
```

Quick model sanity check (confirm tyres can be detected):
```bash
python3 -c "
from ultralytics import YOLO
model = YOLO('tyre_detection_project/best.pt')
results = model.predict('tyre_detection_project/best.pt', save=False, verbose=True)
print([r.names for r in results])
"
```

This just checks the model loads. For actual detection, you need a camera image.

---

## Phase 1 — Identify What Is Currently Broken

Run the full stack and diagnose. In one terminal:
```bash
cd ~/ugv_ws && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ugv_nav full_bringup.launch.py 2>&1 | tee /tmp/full_bringup.log
```

Wait 120 seconds. In another terminal, run the diagnostic script:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash ~/ugv_ws/collect_mission_diagnostics.sh 2>&1 | tee /tmp/diagnostics.txt
```

Then check the mission log:
```bash
tail -50 ~/ugv_ws/logs/mission_latest.jsonl | python3 -c "
import sys,json
for line in sys.stdin:
    try: d=json.loads(line); print(d.get('event'), d.get('data',{}).get('state',''))
    except: pass
"
```

Check the state the mission is stuck in. Common failure states:

**Stuck in IDLE**: Nav2 is not ready, or TF is not available.
```bash
# Check Nav2 nodes
ros2 lifecycle nodes 2>/dev/null
ros2 node list | grep -E "controller|planner|bt_navigator|behavior"
# Check TF
ros2 run tf2_ros tf2_echo slamware_map base_link
```

**Stuck in SEARCH_VEHICLE**: Aurora is not publishing vehicle detections.
```bash
ros2 topic hz /aurora_semantic/vehicle_bounding_boxes
ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once
```

**Navigation goals dispatched but robot not moving**:
```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /stereo/navigation_permitted --once  # Must be True
```

---

## Phase 2 — Known Bugs Already Fixed in Current Code

The following bugs were fixed in commit `2595361`. Verify these fixes are present on the Jetson after `git pull`:

### Fix 1: FACE_TIRE timeout no longer silently skips captures

**Old (broken) behavior**: When `face_tire_timeout_s` (20s) expired, the mission went back to `WAIT_TIRE_BOX` without taking a photo. Robot physically arrived at the tyre but zero photos were captured.

**New behavior**: On face_tire timeout, robot proceeds to `WAIT_WHEEL_FOR_CAPTURE` and attempts the photo from current orientation.

**Verify**: In `~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/inspection_manager_node.py`, search for "face_tire_timeout":
```bash
grep -A3 "face_tire_timeout" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/inspection_manager_node.py | head -15
```
Expected: "WAIT_WHEEL_FOR_CAPTURE" appears, NOT "WAIT_TIRE_BOX".

### Fix 2: Aurora semantic segmentation parameter now reaches the SDK node

**Old behavior**: `enable_semantic_segmentation:=false` in launch did nothing. Aurora always ran semantic processing, consuming extra GPU memory.

**New behavior**: The flag is passed to `slamware_ros_sdk_server_node`.

**Verify**:
```bash
grep "enable_semantic_segmentation" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/launch/aurora_bringup.launch.py | head -5
```
Expected: `"enable_semantic_segmentation": LaunchConfiguration("enable_semantic_segmentation")` appears in the slamware_node parameters dict.

### Fix 3: TF conflict with laser frame removed

**Old behavior**: `world_frame_tf_publisher.py` published `base_link→laser` conflicting with Aurora's `slamware_map→laser`, causing costmap inconsistency.

**New behavior**: Only Aurora publishes the laser transform.

**Verify**:
```bash
grep "base_link.*laser\|child_frame_id.*laser" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/scripts/world_frame_tf_publisher.py
```
Expected: No matches (or a comment saying it was removed).

---

## Phase 3 — Critical Configuration Verification

These are the most likely causes of mission failure on real hardware. Fix them before running the full mission.

### 3.1 — PRODUCTION_CONFIG.yaml — Verify key values

```bash
cat ~/ugv_ws/PRODUCTION_CONFIG.yaml
```

**Critical parameters to check and correct if wrong:**

```yaml
# MUST match actual YOLO class name. Run this to check:
# python3 -c "from ultralytics import YOLO; m=YOLO('tyre_detection_project/best.pt'); print(m.names)"
tire_label: wheel   # Change to match actual class name from above command

# Tyre detection confidence - lower if tyres are not being detected
min_tire_probability: 0.35   # Start at 0.25 if missing detections

# Vehicle detection confidence
min_vehicle_probability: 0.5  # Lower to 0.3 if vehicle not detected

# These MUST match actual distances. Verify with a tape measure:
standoff_distance: 2.0  # meters from vehicle for vehicle approach goal
tire_offset: 1.0        # meters standoff when approaching a tyre (adjust to 0.8 if too far)

# TF frame names - these MUST match what Aurora publishes
world_frame: slamware_map
base_frame: base_link
map_frame: map

# Photo save location - MUST exist and be writable
save_directory: "~/ugv_ws/tire_inspection_photos"

# Key timeouts - increase if robot is timing out too fast
approach_timeout_s: 120.0       # Nav2 has 120s to reach vehicle
tire_search_timeout_s: 90.0     # 90s to find tyres after arriving at vehicle
face_tire_timeout_s: 20.0       # 20s to rotate and face tyre

# Distance gates for photo (0 = disabled; start with 0 and enable later)
photo_trigger_distance: 0.0     # Keep at 0 initially
capture_max_distance_to_tire_m: 0.0  # Keep at 0 initially
```

**To check actual YOLO class names:**
```bash
source ~/ugv_ws/install/setup.bash
python3 -c "
from ultralytics import YOLO
import os
model_path = os.path.expanduser('~/ugv_ws/tyre_detection_project/best.pt')
if os.path.exists(model_path):
    m = YOLO(model_path)
    print('Classes:', m.names)
else:
    print('Model not found at', model_path)
"
```

Update `tire_label` in `PRODUCTION_CONFIG.yaml` to EXACTLY match the class name shown.

### 3.2 — Verify UART port is correct

```bash
ls -la /dev/ttyTHS1 /dev/ttyUSB0 /dev/ttyACM0 2>/dev/null
```

In `full_bringup.launch.py`, the default UART port is `/dev/ttyTHS1`. If your port is different:
```bash
grep "uart_port" ~/ugv_ws/PRODUCTION_CONFIG.yaml
grep "uart_port" ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/launch/full_bringup.launch.py | head -5
```

If `/dev/ttyTHS1` is wrong, find the correct port:
```bash
dmesg | tail -20 | grep -i "tty\|serial\|uart"
# After plugging USB: dmesg | tail -5
```

Fix the default in `full_bringup.launch.py` or pass it at launch time:
```bash
ros2 launch ugv_nav full_bringup.launch.py uart_port:=/dev/ttyUSB0
```

### 3.3 — Verify Nav2 smac planner is installed

```bash
ros2 pkg list | grep smac
```

If empty, install it:
```bash
sudo apt install ros-humble-nav2-smac-planner
```

If not available, change the planner in `nav_aurora.yaml`:
```bash
nano ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/param/nav_aurora.yaml
```

Change:
```yaml
planner_plugins: ["SmacPlanner"]
SmacPlanner:
  plugin: "nav2_smac_planner/SmacPlanner2D"
```

To (fallback):
```yaml
planner_plugins: ["GridBased"]
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  tolerance: 1.0
  use_astar: false
  allow_unknown: true
```

And update the behavior tree files:
```bash
sed -i 's/planner_id="SmacPlanner"/planner_id="GridBased"/g' \
  ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/behavior_trees/*.xml
```

Rebuild after any YAML/XML change:
```bash
cd ~/ugv_ws && colcon build --symlink-install --packages-select ugv_nav && source install/setup.bash
```

---

## Phase 4 — Step-by-Step Mission Bring-Up Procedure

Do this in order. Do not skip steps.

### Step 1 — Start Aurora only (verify it works alone)

```bash
cd ~/ugv_ws && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
```

Wait 15 seconds. In another terminal:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Must see robot pose changing
ros2 topic echo /slamware_ros_sdk_server_node/odom --once

# Must see TF between world and robot
ros2 run tf2_ros tf2_echo slamware_map base_link

# Must see scan data
ros2 topic hz /slamware_ros_sdk_server_node/scan
# Expected: ~10 Hz

# Must see depth data
ros2 topic hz /slamware_ros_sdk_server_node/depth_image_raw
# Expected: ~5 Hz

# Check semantic labels
ros2 topic hz /slamware_ros_sdk_server_node/semantic_labels
# Expected: ~5 Hz (if Aurora firmware supports it)
```

**If TF is NOT available**: The Aurora is not publishing `odom→base_link`. This is a fatal problem. Check:
- `ping 192.168.11.1` still works while node is running
- Aurora is not resetting mid-session (check LED)
- `ros2 node list` shows `slamware_ros_sdk_server_node`

### Step 2 — Start Nav2 (verify planner works)

Keep aurora_bringup running. In a NEW terminal:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ugv_nav nav_aurora.launch.py
```

Wait 60 seconds for Nav2 lifecycle to complete. Then:
```bash
# Check Nav2 nodes are active
ros2 lifecycle nodes
# Expected: controller_server, planner_server, bt_navigator, etc. all in 'active'

# Check costmap is publishing
ros2 topic hz /local_costmap/costmap
# Expected: ~2 Hz
```

If Nav2 nodes are stuck in `unconfigured` or `inactive`:
```bash
# Run lifecycle startup manually
source ~/ugv_ws/install/setup.bash
python3 ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_nav/scripts/nav_lifecycle_startup.py
```

**Test navigation manually before running the mission**:
```bash
# Send a simple goal 1m ahead to verify Nav2 can plan
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
{header: {frame_id: 'map'},
 pose: {position: {x: 1.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Check logs for plan computation:
```bash
ros2 topic echo /plan --once
```

If no plan appears, the planner is broken. Fix the planner config (Phase 3.3).

### Step 3 — Start motor driver and verify motion

```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:=/dev/ttyTHS1
```

Then manually send a test velocity:
```bash
# ONLY DO THIS WITH THE ROBOT ON GROUND WITH SPACE TO MOVE
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
sleep 1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

The robot should move forward ~10cm then stop. If it doesn't move:
1. Check UART port
2. Check pyserial is installed: `python3 -c "import serial; print('OK')"`
3. Check `/dev/ttyTHS1` permissions: `sudo chmod 666 /dev/ttyTHS1`

### Step 4 — Verify the cmd_vel chain

The cmd_vel flow is: `Nav2 → cmd_vel_nav → depth_gate → cmd_vel → motor_driver → ESP32`

When the `enable_cmd_vel_mux:=true` (default), there's also a centroid servo mux before depth_gate.

**Simplified chain for debugging** (disable mux and depth_gate):
```bash
ros2 launch ugv_nav full_bringup.launch.py \
  enable_cmd_vel_mux:=false \
  enable_depth_gate:=false \
  require_nav_permitted:=false
```

This lets Nav2 drive `cmd_vel` directly to the motor driver with no gating.

**Normal chain** (what production should use):
```bash
ros2 topic echo /stereo/navigation_permitted --once
# Must be: data: True
# Published by navigation_permitted_publisher.py when use_bridge:=false
```

If `navigation_permitted` is not True, the depth_gate blocks ALL cmd_vel. Check:
```bash
ros2 node list | grep navigation_permitted
# Expected: /navigation_permitted_publisher
```

If not running, start it manually:
```bash
ros2 run ugv_nav navigation_permitted_publisher.py
```

### Step 5 — Start perception (tyre detection)

With Aurora still running:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch segmentation_3d segment_3d.launch.py \
  use_cpu_inference:=true \
  wheel_inspection_model:=$HOME/ugv_ws/tyre_detection_project/best.pt
```

Wait 30 seconds for model to load. Verify:
```bash
# Vehicle boxes from Aurora semantic
ros2 topic hz /aurora_semantic/vehicle_bounding_boxes
# Expected: >0.5 Hz when a vehicle is visible

# Tyre detection
ros2 topic hz /ultralytics_tire/segmentation/image
# Expected: >1 Hz (shows camera with detection overlay)

# Merged tyre boxes
ros2 topic hz /tire_bounding_boxes_merged
# Expected: >0.5 Hz when tyres are visible

# Tyre 3D positions
ros2 topic hz /tyre_3d_positions
# Expected: >0.5 Hz when tyres are visible and depth+detection aligned
```

**If no vehicle boxes**: Point the Aurora camera at a car or truck, then:
```bash
ros2 topic echo /slamware_ros_sdk_server_node/semantic_labels --once
# Should see mono8 image with non-zero pixels
ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once
```

If semantic_labels is empty/not received, Aurora semantic segmentation may be disabled. Check:
```bash
ros2 param get /slamware_ros_sdk_server_node enable_semantic_segmentation
```

If False, the parameter was passed correctly but Aurora disabled it. For the base Aurora (non-S), semantic segmentation is always on if firmware supports it.

**If no tyre detections**: Check model class names vs `tire_label` config (Phase 3.1).

---

## Phase 5 — Full Mission Run

Once all above steps pass, run the full mission:

```bash
cd ~/ugv_ws && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
./scripts/start_mission.sh
```

**While it runs, monitor in separate terminals:**

Terminal A — Mission state:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /inspection_manager/state
```

Terminal B — Live log:
```bash
tail -f ~/ugv_ws/logs/mission_latest.jsonl | python3 -c "
import sys,json
for line in sys.stdin:
    try:
        d=json.loads(line)
        e=d.get('event','')
        dat=d.get('data',{})
        print(f'{e:30s} {dat}')
    except: pass
"
```

Terminal C — Check if robot is moving:
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /cmd_vel
```

---

## Phase 6 — Fixing Specific Failure Modes

### Failure: Robot stays in IDLE

**Cause A**: Nav2 not available within `nav2_wait_timeout` (90s).
```bash
ros2 lifecycle nodes 2>/dev/null | grep -v "^$"
```
Fix: Increase timeout in PRODUCTION_CONFIG.yaml: `nav2_wait_timeout: 180.0`

**Cause B**: TF not stable for `tf_stable_s` (5s).
```bash
ros2 run tf2_ros tf2_echo slamware_map base_link
```
Should print continuously. If it errors or stops, Aurora is not streaming.

**Cause C**: Vehicle boxes topic not alive.
```bash
ros2 topic hz /aurora_semantic/vehicle_bounding_boxes
```
The mission requires this to have published at least once before starting. If not receiving vehicle boxes, the mission waits in IDLE until `startup_detection_wait_timeout_s` (30s) then errors.

Fix: Either point the robot at a vehicle before starting, OR set `require_detection_topic_at_startup: false` in PRODUCTION_CONFIG.yaml.

### Failure: Robot reaches SEARCH_VEHICLE but doesn't find vehicle

**Cause**: Vehicle is not within detection range or confidence too low.
```bash
ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once
# If no messages, Aurora semantic is failing
ros2 topic echo /slamware_ros_sdk_server_node/semantic_labels --once
# If empty image, Aurora is not seeing the vehicle
```

Fix 1: Move the robot within 5-8 metres of the vehicle.
Fix 2: Lower `min_vehicle_probability: 0.3` in PRODUCTION_CONFIG.yaml.
Fix 3: Check `vehicle_labels: "car,truck,bus"` includes the vehicle type visible.

### Failure: Vehicle detected but robot doesn't approach

**Cause A**: `require_nav_permitted: true` and `/stereo/navigation_permitted` is not True.
```bash
ros2 topic echo /stereo/navigation_permitted --once
```
Fix: If False or no message, check `navigation_permitted_publisher` is running. Or set `require_nav_permitted: false` in PRODUCTION_CONFIG.yaml temporarily.

**Cause B**: Goal generation fails (TF unavailable for goal transform).
```bash
tail -20 ~/ugv_ws/logs/mission_latest.jsonl | python3 -c "
import sys,json
for line in sys.stdin:
    try:
        d=json.loads(line)
        if 'goal_generation' in d.get('event','') or 'dispatch' in d.get('event',''):
            print(d)
    except: pass
"
```

**Cause C**: Nav2 goal rejected (costmap occupied at goal).
Fix: Clear costmaps and retry. Or reduce `inflate_radius` in nav_aurora.yaml.

### Failure: Robot approaches vehicle but doesn't find tyres

**Cause A**: Tyre detection model not working for this vehicle.
```bash
ros2 topic echo /tire_bounding_boxes_merged --once
# If no messages, tyre detection is failing
ros2 topic hz /ultralytics_tire/segmentation/image
```

Fix 1: Check model class name vs `tire_label` param.
Fix 2: Move robot to within 3m of tyre.
Fix 3: Verify model file: `ls -la ~/ugv_ws/tyre_detection_project/best.onnx`

**Cause B**: Tyre positions not projected to 3D.
```bash
ros2 topic hz /tyre_3d_positions
```

If 0 Hz, the `tyre_3d_projection_node` is failing. Check:
```bash
ros2 topic hz /slamware_ros_sdk_server_node/depth_image_raw
# If 0 Hz, no depth data from Aurora
```

Fix: If depth_image_raw is not publishing, check Aurora firmware 2.11+. Temporarily disable tyre_3d:
```bash
# In PRODUCTION_CONFIG.yaml:
use_tyre_3d_positions: false
# And enable planned corners fallback:
planned_tire_fallback_enabled: true
```

### Failure: Robot navigates but doesn't capture photo

**Cause A**: `photo_capture_service` is not running.
```bash
ros2 node list | grep photo_capture
```

**Cause B**: Camera topic not receiving images.
```bash
ros2 topic hz /slamware_ros_sdk_server_node/left_image_raw
```

Fix: Change `camera_topic` in PRODUCTION_CONFIG.yaml to the correct Aurora image topic.

**Cause C**: Photo trigger distance gate blocking.
Set in PRODUCTION_CONFIG.yaml:
```yaml
photo_trigger_distance: 0.0
capture_max_distance_to_tire_m: 0.0
capture_max_distance_to_tire_face_m: 0.0
```

**Cause D**: VERIFY_CAPTURE timeout (photo_capture_service never responded).
```bash
ros2 topic echo /inspection_manager/capture_result
# Send manual trigger to test:
ros2 service call /photo_capture_service/capture_photo std_srvs/srv/Trigger
```

Check the saved directory:
```bash
ls -lt ~/ugv_ws/tire_inspection_photos/ | head -5
```

---

## Phase 7 — Code Changes Required for Production

The following changes must be made in the actual source code. These are not configuration changes.

### 7.1 — Add explicit startup health check for Aurora

**Problem**: Currently the mission starts as soon as TF is available, but Aurora might be partially initialized. The semantic segmentation topic might not be streaming yet.

**Fix**: In `inspection_manager_node.py`, add a check that `/slamware_ros_sdk_server_node/semantic_labels` has published at least once before transitioning from IDLE. This prevents SEARCH_VEHICLE from running blind.

Location: `~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/inspection_manager_node.py`

In the IDLE → INIT startup check block (around line 3477), add alongside the existing `vehicle_boxes_alive` check:
```python
# In the startup invariants block, verify the semantic_labels topic is alive
# Add parameter: semantic_labels_wait_s (default 0.0 to not break existing behavior)
```

For now, the workaround is: wait until you see `/aurora_semantic/vehicle_bounding_boxes` publishing before running the mission. The `require_detection_topic_at_startup: true` + `vehicle_boxes_topic` check handles this.

### 7.2 — Simplify the cmd_vel chain for real hardware

The current chain has too many components: Nav2 → cmd_vel_nav_source → cmd_vel_mux → cmd_vel_nav → depth_gate → cmd_vel → motor_driver.

**Problem**: Any misconfigured topic name in this chain silently breaks motion. The robot appears to be navigating (Nav2 is active, goals are dispatched) but the physical robot never moves.

**Verify the full chain is correctly wired**:
```bash
# In one terminal
ros2 topic echo /cmd_vel --once    # What motor_driver receives

# In another terminal  
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}}"           # Simulate Nav2 output
```

If `/cmd_vel` does not immediately show the value, the depth_gate is blocking. Check:
```bash
ros2 node info /depth_gate 2>/dev/null | grep -A5 "Subscriptions\|Publishers"
```

**Quick fix for production**: Bypass depth_gate and cmd_vel_mux entirely for initial testing:
```bash
ros2 launch ugv_nav full_bringup.launch.py \
  enable_depth_gate:=false \
  enable_cmd_vel_mux:=false \
  require_nav_permitted:=false
```

This connects Nav2 directly to the motor driver. Use this to prove the robot can physically move when Nav2 commands it. Then re-enable the safety features once motion is confirmed.

### 7.3 — Reduce Nav2 goal tolerance for tyres (if robot parks too far away)

If the robot stops too far from the tyre for the camera to see it:

In `PRODUCTION_CONFIG.yaml`:
```yaml
xy_goal_tolerance: 0.10   # 10cm instead of 15cm for tyre approach
```

In `nav_aurora.yaml`, in the `goal_checker` section, ensure it reads:
```yaml
xy_goal_tolerance: 0.10
```

Rebuild is NOT required for YAML changes if using `--symlink-install`.

### 7.4 — Set correct image resolution for tyre detection

The Aurora left image is 416×224 pixels. If the YOLO model was trained at 640×640, it needs to be run at the appropriate size.

Check current config:
```bash
grep -E "wheel_imgsz|tyre_onnx_imgsz|imgsz" ~/ugv_ws/PRODUCTION_CONFIG.yaml ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/segment_3d/segmentation_3d/launch/segment_3d.launch.py | head -10
```

For CPU ONNX inference, the imgsz must match what the model was exported with:
```bash
python3 -c "
import onnx
m = onnx.load('tyre_detection_project/best.onnx')
for inp in m.graph.input:
    shape = [d.dim_value for d in inp.type.tensor_type.shape.dim]
    print('ONNX input shape:', shape)
"
```

If the shape is `[1, 3, 480, 480]`, the ONNX expects 480×480. Set `tyre_onnx_imgsz: 480` in the launch file or PRODUCTION_CONFIG.

---

## Phase 8 — Simplified Stack for First Successful Mission

If the full stack is still not working after all fixes, use this simplified launch approach. It removes non-essential components to prove the core mission path works.

### Minimal working launch sequence

**Terminal 1 — Aurora + TF:**
```bash
cd ~/ugv_ws && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
```

Wait 15s. Verify TF: `ros2 run tf2_ros tf2_echo slamware_map base_link`

**Terminal 2 — Nav2 (no EKF, direct Aurora odom):**
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ugv_nav nav_aurora.launch.py \
  enable_cmd_vel_mux:=false \
  enable_depth_gate:=false \
  enable_vehicle_speed_filter:=false
```

Wait 60s. Verify: `ros2 lifecycle nodes` shows all active.

**Terminal 3 — Motor driver:**
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run ugv_base_driver motor_driver_node --ros-args -p uart_port:=/dev/ttyTHS1
```

**Terminal 4 — Perception (tyre detection only, CPU):**
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch segmentation_3d segment_3d.launch.py \
  use_cpu_inference:=true \
  enable_tyre_3d_projection:=true \
  centroid_servo_enabled:=false \
  pcl_fallback_enabled:=false \
  minimal_perception:=false
```

**Terminal 5 — Navigation permitted publisher (needed for goal dispatch):**
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run ugv_nav navigation_permitted_publisher.py
```

**Terminal 6 — Inspection manager + photo capture (after waiting ~90s for Nav2 to finish starting):**
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch inspection_manager inspection_manager.launch.py \
  require_nav_permitted:=false \
  use_tyre_3d_positions:=true \
  require_detection_topic_at_startup:=false \
  params_file:=$HOME/ugv_ws/PRODUCTION_CONFIG.yaml
```

**Monitor in another terminal:**
```bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
watch -n1 "ros2 topic echo /inspection_manager/state --once 2>/dev/null"
```

This minimal stack removes: centroid servo, cmd_vel mux, depth gate, vehicle speed filter, PCL fallback, EKF. Each of these could be a silent failure point.

---

## Phase 9 — What a Successful Mission Looks Like

When the mission is working correctly, you will see this sequence:

**In the mission log (`tail -f ~/ugv_ws/logs/mission_latest.jsonl`):**
```
mission_start        {'robot_position': [x, y, z]}
state_transition     {'from': 'IDLE', 'to': 'SEARCH_VEHICLE'}
state_transition     {'from': 'SEARCH_VEHICLE', 'to': 'WAIT_VEHICLE_BOX'}
vehicle_detected     {'vehicle_id': 1, 'x': ..., 'y': ..., 'confidence': 0.8}
state_transition     {'from': 'WAIT_VEHICLE_BOX', 'to': 'APPROACH_VEHICLE'}
NAV_COMMAND_SENT     goal dispatched
nav_result           {'success': True, 'state': 'APPROACH_VEHICLE'}
state_transition     {'from': 'APPROACH_VEHICLE', 'to': 'WAIT_TIRE_BOX'}
tyre_3d_wait_tire_box_dispatch  {'target': [x, y, z], 'distance_m': 1.5}
state_transition     {'from': 'WAIT_TIRE_BOX', 'to': 'INSPECT_TIRE'}
nav_result           {'success': True, 'state': 'INSPECT_TIRE'}
state_transition     {'from': 'INSPECT_TIRE', 'to': 'FACE_TIRE'}
state_transition     {'from': 'FACE_TIRE', 'to': 'WAIT_WHEEL_FOR_CAPTURE'}
state_transition     {'from': 'WAIT_WHEEL_FOR_CAPTURE', 'to': 'VERIFY_CAPTURE'}
photo_captured       {'filename': 'tire_v1_n1_20260601_....jpg', 'bytes': 45123}
state_transition     {'from': 'VERIFY_CAPTURE', 'to': 'WAIT_TIRE_BOX'}
[repeats for tyres 2, 3, 4]
state_transition     {'from': 'VERIFY_CAPTURE', 'to': 'NEXT_VEHICLE'}
state_transition     {'from': 'NEXT_VEHICLE', 'to': 'DONE'}
mission_end          {'total_tires_captured': 4, 'total_vehicles': 1}
```

**Photos saved:**
```bash
ls -lt ~/ugv_ws/tire_inspection_photos/*.jpg | head -10
# Expected: 4 JPG files created during the mission
```

**Mission report:**
```bash
cat ~/ugv_ws/logs/mission_report_latest.json | python3 -m json.tool | grep -E "tires_captured|vehicles|error"
# Expected: total_tires_captured: 4, error_states_encountered: 0
```

---

## Phase 10 — Common Problems After Phase 8 Still Failing

If the robot still does not work after Phase 8, these are the remaining possibilities:

### Problem: Nav2 plans correctly but the robot goes to the wrong place

Aurora's map coordinate system may be different from what was expected. The standoff goal is computed in `slamware_map` frame. Check:
```bash
ros2 topic echo /inspection_manager/current_goal --once
ros2 run tf2_ros tf2_echo slamware_map base_link  # robot position
```

Compare the goal position with the vehicle position visually.

### Problem: Robot overshoots or drives past the vehicle

The `standoff_distance: 2.0` may need tuning. The goal is placed 2m from the vehicle bounding box face. If the bounding box is wrong (too small/large), the goal may be in the wrong place.

```bash
ros2 topic echo /aurora_semantic/vehicle_bounding_boxes --once
# Check: xmin, xmax, ymin, ymax should span the full vehicle footprint
```

### Problem: Tyre detection sees tyres but robot doesn't approach them

The `tyre_3d_positions` topic might be stale or have wrong frame_id:
```bash
ros2 topic echo /tyre_3d_positions --once
# Check: header.frame_id must be 'slamware_map'
# Check: poses array should have non-zero x,y values matching tyre positions
```

If frame is wrong, check `tyre_3d_projection_node.py` parameters in `segment_3d.launch.py`.

### Problem: Photo captured but file is empty or corrupt

```bash
ls -la ~/ugv_ws/tire_inspection_photos/
# Check: file size > 0 bytes

# View the image (if X11 forwarding available)
eog ~/ugv_ws/tire_inspection_photos/tire_v1_n1_*.jpg

# Check capture service logs
ros2 node info /photo_capture_service
```

The camera topic must be publishing at the time of capture. If `left_image_raw` is at very low Hz (<0.5 Hz), the capture will use a stale frame.

---

## Phase 11 — Emergency Simplified Mission (Nuclear Option)

If everything else fails and the robot still doesn't work, write a NEW simple mission node that bypasses all the complexity of `inspection_manager_node.py`.

Create `~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/inspection_manager/inspection_manager/simple_mission.py`:

```python
#!/usr/bin/env python3
"""
Simplified tyre inspection mission node.

This is a fallback when the full inspection_manager_node.py is not working.
It implements the core mission flow directly:
1. Wait for TF and Nav2
2. Look for a vehicle (Aurora semantic boxes)
3. Navigate to standoff position
4. Look for tyres (tyre_3d_positions)
5. Navigate to each tyre
6. Trigger photo capture
7. Move to next tyre

This node has NO state machine complexity, NO spin protection, NO retry budgets.
It just drives to the vehicle, drives to each tyre, takes a photo, and stops.
"""
import math
import time
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from geometry_msgs.msg import PoseArray
import tf2_ros
import cv2
from cv_bridge import CvBridge
from datetime import datetime


class SimpleMissionNode(Node):
    """Dead-simple mission: see vehicle, approach, find tyres, photograph."""

    def __init__(self):
        super().__init__("simple_mission")
        
        # Parameters
        self.declare_parameter("world_frame", "slamware_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("vehicle_standoff_m", 2.0)
        self.declare_parameter("tyre_standoff_m", 1.0)
        self.declare_parameter("save_directory", "~/ugv_ws/tire_inspection_photos")
        self.declare_parameter("min_vehicle_prob", 0.4)
        self.declare_parameter("min_tyre_count", 1)
        self.declare_parameter("tyres_per_vehicle", 4)

        self._world_frame = self.get_parameter("world_frame").value
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._vehicle_standoff = self.get_parameter("vehicle_standoff_m").value
        self._tyre_standoff = self.get_parameter("tyre_standoff_m").value
        self._save_dir = os.path.expanduser(self.get_parameter("save_directory").value)
        os.makedirs(self._save_dir, exist_ok=True)

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Subscriptions
        self._vehicle_boxes = None
        self._tyre_positions = None
        self._latest_image = None
        self._bridge = CvBridge()

        self.create_subscription(
            BoundingBoxes3d,
            "/aurora_semantic/vehicle_bounding_boxes",
            self._on_vehicle_boxes, 10
        )
        self.create_subscription(
            PoseArray,
            "/tyre_3d_positions",
            self._on_tyre_positions, 10
        )
        self.create_subscription(
            Image,
            "/slamware_ros_sdk_server_node/left_image_raw",
            self._on_image, 10
        )

        # Photo capture publisher
        self._capture_pub = self.create_publisher(Bool, "/inspection_manager/capture_photo", 10)
        self._state_pub = self.create_publisher(String, "/inspection_manager/state", 10)

        # State
        self._state = "STARTUP"
        self._photo_count = 0
        
        self.get_logger().info("Simple mission node started. Waiting 10s for systems to initialize...")
        self.create_timer(10.0, self._start_mission_timer)

    def _on_vehicle_boxes(self, msg):
        self._vehicle_boxes = msg

    def _on_tyre_positions(self, msg):
        self._tyre_positions = msg

    def _on_image(self, msg):
        self._latest_image = msg

    def _start_mission_timer(self):
        """One-shot timer to start after 10s."""
        # Run mission in a separate thread to avoid blocking spin
        import threading
        self._mission_thread = threading.Thread(target=self._run_mission, daemon=True)
        self._mission_thread.start()

    def _publish_state(self, state: str):
        self._state = state
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)
        self.get_logger().info(f"STATE: {state}")

    def _wait_for_tf(self, timeout=60.0):
        """Wait until slamware_map->base_link is available."""
        start = time.time()
        while time.time() - start < timeout:
            try:
                self._tf_buffer.lookup_transform(
                    self._world_frame, self._base_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.get_logger().info("TF available")
                return True
            except Exception:
                time.sleep(1.0)
        self.get_logger().error("TF not available after timeout")
        return False

    def _wait_for_nav2(self, timeout=90.0):
        """Wait for Nav2 action server."""
        self.get_logger().info("Waiting for Nav2...")
        if self._nav_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().info("Nav2 ready")
            return True
        self.get_logger().error("Nav2 not available")
        return False

    def _get_robot_pose_world(self):
        """Get current robot pose in world frame."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return tf.transform.translation
        except Exception:
            return None

    def _nav_to_pose(self, x, y, yaw, frame="map", timeout=120.0):
        """Navigate to (x, y, yaw) and wait for result. Returns True on success."""
        # Transform goal from world_frame to map_frame if needed
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"Navigating to ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)")
        future = self._nav_client.send_goal_async(goal)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 10.0:
            time.sleep(0.1)
        if not future.done():
            self.get_logger().error("Goal send timeout")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done() and (time.time() - start) < timeout:
            time.sleep(0.5)

        if not result_future.done():
            self.get_logger().error(f"Navigation timeout after {timeout}s")
            goal_handle.cancel_goal_async()
            return False

        status = result_future.result().status
        success = (status == 4)  # STATUS_SUCCEEDED
        self.get_logger().info(f"Navigation {'succeeded' if success else 'failed'} (status={status})")
        return success

    def _transform_goal_to_map(self, wx, wy):
        """Transform position from world_frame to map_frame for Nav2."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._world_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            # For identity transform (slamware_map == map), no change needed
            mx = wx + tf.transform.translation.x
            my = wy + tf.transform.translation.y
            return mx, my
        except Exception:
            # If no transform, assume identity
            return wx, wy

    def _standoff_from_target(self, tx, ty, robot_pos, offset_m):
        """Compute standoff position offset_m from target toward robot."""
        if robot_pos is None:
            return tx, ty, 0.0
        rx, ry = robot_pos.x, robot_pos.y
        dx, dy = rx - tx, ry - ty
        d = math.hypot(dx, dy)
        if d < 0.1:
            return tx + offset_m, ty, math.pi
        gx = tx + offset_m * dx / d
        gy = ty + offset_m * dy / d
        heading = math.atan2(ty - gy, tx - gx)
        return gx, gy, heading

    def _capture_photo(self, vehicle_id, tyre_num):
        """Capture and save a photo."""
        if self._latest_image is None:
            self.get_logger().error("No image available for capture!")
            return False
        try:
            cv_image = self._bridge.imgmsg_to_cv2(self._latest_image, "bgr8")
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"tire_v{vehicle_id}_n{tyre_num}_{timestamp}.jpg"
            filepath = os.path.join(self._save_dir, filename)
            ok = cv2.imwrite(filepath, cv_image)
            if ok and os.path.getsize(filepath) > 0:
                self.get_logger().info(f"Photo saved: {filename} ({os.path.getsize(filepath)} bytes)")
                self._photo_count += 1
                return True
            else:
                self.get_logger().error(f"Failed to write photo: {filepath}")
                return False
        except Exception as e:
            self.get_logger().error(f"Photo capture error: {e}")
            return False

    def _run_mission(self):
        """Main mission loop. Runs in background thread."""
        self.get_logger().info("Mission starting...")
        self._publish_state("INIT")

        # Wait for prerequisites
        if not self._wait_for_tf():
            self._publish_state("ERROR")
            return
        if not self._wait_for_nav2():
            self._publish_state("ERROR")
            return

        self._publish_state("SEARCH_VEHICLE")
        self.get_logger().info("Looking for vehicle. Point the robot toward a car/truck.")

        # Wait for vehicle detection
        vehicle_found = False
        search_start = time.time()
        vehicle_box = None
        while time.time() - search_start < 180.0:
            if self._vehicle_boxes and self._vehicle_boxes.bounding_boxes:
                for box in self._vehicle_boxes.bounding_boxes:
                    if box.probability >= self.get_parameter("min_vehicle_prob").value:
                        vehicle_box = box
                        vehicle_found = True
                        break
            if vehicle_found:
                break
            time.sleep(1.0)
            if int(time.time() - search_start) % 10 == 0:
                self.get_logger().info(f"Searching for vehicle... {int(time.time()-search_start)}s")

        if not vehicle_found:
            self.get_logger().error("No vehicle found after 180s. Mission aborted.")
            self._publish_state("ERROR")
            return

        # Vehicle found — compute approach goal
        cx = (vehicle_box.xmin + vehicle_box.xmax) / 2.0
        cy = (vehicle_box.ymin + vehicle_box.ymax) / 2.0
        self.get_logger().info(f"Vehicle detected at ({cx:.2f}, {cy:.2f}) confidence={vehicle_box.probability:.2f}")

        self._publish_state("APPROACH_VEHICLE")
        robot_pos = self._get_robot_pose_world()
        gx_w, gy_w, heading = self._standoff_from_target(cx, cy, robot_pos, self._vehicle_standoff)
        gx_m, gy_m = self._transform_goal_to_map(gx_w, gy_w)

        arrived = self._nav_to_pose(gx_m, gy_m, heading)
        if not arrived:
            self.get_logger().warn("Vehicle approach nav failed. Trying to continue anyway.")

        # At vehicle — look for tyres
        self._publish_state("WAIT_TIRE_BOX")
        self.get_logger().info("Looking for tyres...")
        tyres_to_visit = []
        
        tyre_wait_start = time.time()
        while time.time() - tyre_wait_start < 30.0:
            if self._tyre_positions and self._tyre_positions.poses:
                for pose in self._tyre_positions.poses:
                    p = (pose.position.x, pose.position.y)
                    # Skip if already in list (within 0.5m)
                    duplicate = any(math.hypot(p[0]-t[0], p[1]-t[1]) < 0.5 for t in tyres_to_visit)
                    if not duplicate:
                        tyres_to_visit.append(p)
            if len(tyres_to_visit) >= self.get_parameter("min_tyre_count").value:
                break
            time.sleep(0.5)

        if not tyres_to_visit:
            self.get_logger().warn("No tyres detected. Using estimated positions from vehicle box.")
            # Fallback: estimate 4 tyre positions from vehicle box corners
            vw = vehicle_box.xmax - vehicle_box.xmin
            vl = vehicle_box.ymax - vehicle_box.ymin
            margin = 0.1
            tyres_to_visit = [
                (vehicle_box.xmin + margin, vehicle_box.ymin + margin),
                (vehicle_box.xmax - margin, vehicle_box.ymin + margin),
                (vehicle_box.xmin + margin, vehicle_box.ymax - margin),
                (vehicle_box.xmax - margin, vehicle_box.ymax - margin),
            ]

        # Sort by distance to current robot position
        robot_pos = self._get_robot_pose_world()
        if robot_pos:
            tyres_to_visit.sort(
                key=lambda t: math.hypot(t[0] - robot_pos.x, t[1] - robot_pos.y)
            )

        self.get_logger().info(f"Found {len(tyres_to_visit)} tyre targets. Starting inspection.")

        # Inspect each tyre
        for i, (tx, ty) in enumerate(tyres_to_visit[:self.get_parameter("tyres_per_vehicle").value]):
            self._publish_state("INSPECT_TIRE")
            self.get_logger().info(f"Approaching tyre {i+1} at ({tx:.2f}, {ty:.2f})")

            robot_pos = self._get_robot_pose_world()
            gx_w, gy_w, heading = self._standoff_from_target(tx, ty, robot_pos, self._tyre_standoff)
            gx_m, gy_m = self._transform_goal_to_map(gx_w, gy_w)

            arrived = self._nav_to_pose(gx_m, gy_m, heading)
            if not arrived:
                self.get_logger().warn(f"Tyre {i+1} nav failed. Attempting capture from current position.")

            # Wait 2s for robot to settle, then capture
            time.sleep(2.0)

            self._publish_state("VERIFY_CAPTURE")
            ok = self._capture_photo(vehicle_id=1, tyre_num=i+1)
            if ok:
                self.get_logger().info(f"Tyre {i+1}/{len(tyres_to_visit)} photographed successfully.")
            else:
                self.get_logger().warn(f"Tyre {i+1} photo failed.")

            time.sleep(1.0)

        self._publish_state("DONE")
        self.get_logger().info(
            f"Mission complete. {self._photo_count} photos captured. "
            f"Saved to {self._save_dir}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Add this to `setup.py` entry_points:
```python
"simple_mission = inspection_manager.simple_mission:main",
```

Rebuild:
```bash
cd ~/ugv_ws && colcon build --symlink-install --packages-select inspection_manager
source install/setup.bash
```

Run with the minimal stack (Phase 8) but replace the inspection_manager launch with:
```bash
ros2 run inspection_manager simple_mission \
  --ros-args \
  -p world_frame:=slamware_map \
  -p map_frame:=map \
  -p vehicle_standoff_m:=2.0 \
  -p tyre_standoff_m:=1.0
```

This simple node has none of the state machine complexity. If this works, the problem was in the complex mission manager. If it still doesn't work, the problem is in the navigation/hardware layer.

---

## Phase 12 — Final Checklist Before Declaring Success

Before declaring the system production-ready, verify ALL of the following:

- [ ] `ping 192.168.11.1` works consistently
- [ ] `ros2 run tf2_ros tf2_echo slamware_map base_link` runs without errors
- [ ] `ros2 topic hz /cmd_vel` shows values when Nav2 is planning
- [ ] Robot physically moves when `ros2 topic pub /cmd_vel ... {linear: {x: 0.1}}` is run
- [ ] `ros2 topic hz /aurora_semantic/vehicle_bounding_boxes` shows >0.5 Hz when vehicle is visible
- [ ] `ros2 topic hz /tyre_3d_positions` shows >0.5 Hz when tyres are visible and within 3m
- [ ] Full mission runs from IDLE → DONE without ERROR state
- [ ] `ls ~/ugv_ws/tire_inspection_photos/*.jpg | wc -l` shows 4 files after one vehicle inspection
- [ ] Images are non-corrupt (viewable with `eog` or similar)
- [ ] `cat ~/ugv_ws/logs/mission_report_latest.json` shows `total_tires_captured: 4`

---

## Known Remaining Risks After This Fix

1. **Aurora semantic detection is not guaranteed to detect all vehicle types.** If the vehicle is an unusual truck variant, semantic segmentation may not fire. In that case, enable YOLO vehicle detection as fallback: `use_vehicle_yolo:=true` in the launch.

2. **Tyre detection at close range (<0.8m) is unreliable.** The YOLO model was likely trained on images from 1-3m range. If the robot gets too close, tyres may leave the camera FOV or appear as edge cases. Tune `tyre_standoff_m` to 1.0-1.5m.

3. **Nav2 planner fails near the vehicle.** The vehicle bounding box appears in the costmap as a large obstacle. The inflation radius (0.35m global, 0.4m local) may block plans to corners. Reduce to 0.2m if goals near the vehicle are consistently rejected.

4. **cmd_vel chain timing on first goal dispatch.** On the very first navigation goal, there can be a 2-5 second delay before the first cmd_vel is published. The `cmd_vel_timeout_s: 2.0` watchdog may fire and log a warning. This is not a bug — just set the timeout to 5.0 seconds.

5. **Aurora may reset its map after a while.** If the robot has been sitting for >5 minutes, the map may drift. Always run `reset_map_on_startup: true` (default) for each mission session.

---

## Quick Reference — Most Useful Commands

```bash
# Full system status check
source ~/ugv_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Check all key topics in one command
for t in /slamware_ros_sdk_server_node/odom \
          /aurora_semantic/vehicle_bounding_boxes \
          /tire_bounding_boxes_merged \
          /tyre_3d_positions \
          /cmd_vel \
          /stereo/navigation_permitted \
          /inspection_manager/state; do
  hz=$(ros2 topic hz $t --window 5 2>/dev/null | grep "average rate" | awk '{print $3}' &)
  echo "$t: $hz Hz"
  sleep 1
done

# Mission state (live)
ros2 topic echo /inspection_manager/state

# Live log stream
tail -f ~/ugv_ws/logs/mission_latest.jsonl | python3 -c "import sys,json; [print(json.loads(l).get('event'), json.loads(l).get('data',{}).get('state','')) for l in sys.stdin if l.strip()]"

# Emergency robot stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# Manual photo trigger
ros2 service call /photo_capture_service/capture_photo std_srvs/srv/Trigger

# Check photos saved
ls -lt ~/ugv_ws/tire_inspection_photos/*.jpg 2>/dev/null | head -5

# Rebuild only the packages that matter
cd ~/ugv_ws && colcon build --symlink-install --packages-select inspection_manager ugv_nav ugv_base_driver segmentation_3d && source install/setup.bash
```

---

*This document covers every known failure mode for this system. Start at Phase 0 and work through in order. Do not skip hardware verification. The most common cause of failure is a misconfigured parameter or a broken cmd_vel chain — both of which are visible within 5 minutes of running the diagnostic commands in Phase 1.*
