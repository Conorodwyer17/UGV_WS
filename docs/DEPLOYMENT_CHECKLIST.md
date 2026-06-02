# Pre-Deployment Checklist

Complete this checklist before each deployment to verify the robot is ready for an autonomous tyre inspection mission.

Time to complete: ~5 minutes.

---

## Hardware

- [ ] Aurora head is physically secure and level on the robot
- [ ] Aurora is powered and LED status shows active (not error)
- [ ] Aurora IP address reachable: `ping 192.168.11.1`
- [ ] Motor driver board connected via UART (`/dev/ttyTHS1`)
- [ ] Robot base battery >60% (check Waveshare base indicator lights)
- [ ] Jetson Orin Nano is on and SSH reachable
- [ ] Workspace has clear floor area >5m in front of the robot

---

## Software — Pre-Launch

- [ ] Workspace built and sourced:
  ```bash
  cd ~/ugv_ws && source install/setup.bash
  ```
- [ ] Tyre model exists:
  ```bash
  ls tyre_detection_project/best.pt   # or best.engine / best.onnx
  ```
- [ ] ONNX model exported (for CPU inference, default profile):
  ```bash
  ls tyre_detection_project/best.onnx
  # If missing: MODEL_PT=tyre_detection_project/best.pt IMGSZ=480 bash scripts/export_onnx.sh
  ```
- [ ] Disk space sufficient (>2 GB free):
  ```bash
  df -h ~/ugv_ws
  ```
- [ ] Photo save directory writable:
  ```bash
  ls ~/ugv_ws/tire_inspection_photos/ 2>/dev/null || mkdir -p ~/ugv_ws/tire_inspection_photos
  ```

---

## Launch and Initial Checks (after `start_mission.sh`)

Wait 90–120 seconds after launch, then verify:

**Aurora (TF and topics):**
- [ ] Robot TF available:
  ```bash
  ros2 run tf2_ros tf2_echo slamware_map base_link
  ```
  Expected: transforms printing at ~20 Hz with no "Lookup would require extrapolation" errors.

- [ ] Aurora publishing key topics:
  ```bash
  ros2 topic hz /slamware_ros_sdk_server_node/scan          # expect ~10 Hz
  ros2 topic hz /slamware_ros_sdk_server_node/odom          # expect ~20 Hz
  ros2 topic hz /slamware_ros_sdk_server_node/depth_image_raw  # expect ~5 Hz
  ros2 topic hz /slamware_ros_sdk_server_node/semantic_labels  # expect ~5 Hz
  ```

**Perception:**
- [ ] Tyre YOLO inference running:
  ```bash
  ros2 topic hz /ultralytics_tire/segmentation/image  # expect >1 Hz
  ```
- [ ] Vehicle detection publishing:
  ```bash
  ros2 topic hz /aurora_semantic/vehicle_bounding_boxes  # expect >0.5 Hz when vehicle visible
  ```
- [ ] Depth point cloud registered:
  ```bash
  ros2 topic hz /segmentation_processor/registered_pointcloud  # expect >1 Hz
  ```

**Navigation:**
- [ ] Nav2 lifecycle active:
  ```bash
  ros2 lifecycle nodes
  # Expected: controller_server, planner_server, bt_navigator all in 'active' state
  ```
- [ ] Costmap publishing:
  ```bash
  ros2 topic hz /local_costmap/costmap  # expect ~2 Hz
  ```

**Navigation permitted (depth gate):**
- [ ] Navigation gate open:
  ```bash
  ros2 topic echo /stereo/navigation_permitted --once
  # Expected: data: true
  ```

**Inspection manager:**
- [ ] Mission manager running:
  ```bash
  ros2 topic echo /inspection_manager/state --once
  # Expected: SEARCH_VEHICLE (if Nav2/TF ready) or IDLE (still waiting)
  ```
- [ ] Mission log directory writable:
  ```bash
  ls ~/ugv_ws/logs/mission_latest.jsonl 2>/dev/null && echo "OK" || echo "Log file not created yet"
  ```

---

## Pre-Mission Safety Check

- [ ] Area in front of robot is clear of personnel
- [ ] Target vehicle is stationary and will remain so during inspection
- [ ] Vehicle tyres are visible (not occluded by another vehicle)
- [ ] Emergency stop is accessible

---

## Quick Sensor Validation

Place the robot 3–5 m from a vehicle. Verify in the JSONL log:

```bash
tail -f ~/ugv_ws/logs/mission_latest.jsonl | python3 -c "
import sys, json
for line in sys.stdin:
    try: e = json.loads(line); print(e.get('event'), e.get('data', {}).get('state',''))
    except: pass
"
```

Expected sequence:
1. `mission_start` logged
2. `state_transition` IDLE → SEARCH_VEHICLE
3. When vehicle detected: `state_transition` → WAIT_VEHICLE_BOX
4. When vehicle confirmed: `state_transition` → APPROACH_VEHICLE

If state stays in SEARCH_VEHICLE >60 seconds with vehicle in view:
- Check `/aurora_semantic/vehicle_bounding_boxes` frequency
- Check `inspection_manager` logs for `vehicle_boxes_stream_stale`

---

## Known Hardware Assumptions

| Assumption | Consequence if Wrong |
|-----------|---------------------|
| Aurora firmware ≥ 2.11 | `use_bridge:=false` (default) fails; depth_image_raw not published; perception silently fails |
| Aurora at IP 192.168.11.1 | slamware_ros_sdk_server_node fails to connect; no TF, no scan |
| Aurora mounted at (0.0418, 0.03, 0) from base_link | Depth costmap obstacles appear at wrong positions; robot may collide |
| UART port `/dev/ttyTHS1` | Motor driver cannot send commands; robot does not move |
| RMW = `rmw_cyclonedds_cpp` on ALL terminals | Topics/TF not visible across terminals; mission state not observable |
| `tyre_detection_project/best.onnx` exists at IMGSZ=480 | CPU inference profile fails to load model |
| Disk has >100 MB free | Mission log writes fail; photo saves fail silently |

---

## Abort Criteria

Stop the mission immediately if:
- Robot moves toward a person
- Robot drives against a wall or obstacle without stopping
- Robot emits unusual sounds from base motors
- Aurora LED turns solid red (hardware fault)
- Mission state stays in ERROR for >10 seconds

To stop immediately:
```bash
# Stop robot motion
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'

# Kill mission
pkill -f inspection_manager_node
```

---

## Post-Mission Verification

After mission completes (`state: DONE`):

- [ ] Photos saved:
  ```bash
  ls -lt ~/ugv_ws/tire_inspection_photos/*.jpg | head -10
  ```
- [ ] Mission report generated:
  ```bash
  cat ~/ugv_ws/logs/mission_report_latest.json | python3 -m json.tool
  ```
- [ ] Expected fields in report:
  - `total_tires_captured` >= expected
  - `total_vehicles` >= 1
  - `error_states_encountered` = 0 (or explain non-zero)
