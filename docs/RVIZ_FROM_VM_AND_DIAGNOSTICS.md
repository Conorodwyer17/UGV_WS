# Viewing vehicle detection in RViz from a VM + diagnostics

## 1. Run RViz from your virtual machine (same ROS 2 network)

From the **VM**, use the same ROS_DOMAIN_ID and RMW as the robot/desktop so they see the same topics.

```bash
# On the VM (install ROS 2 Humble if needed)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rviz2 rviz2
```

- **Fixed Frame:** set to `slamware_map` (left panel → Global Options → Fixed Frame).
- **Add display:** Add → By topic → `/aurora_semantic/vehicle_markers` → MarkerArray → OK.
- Optional: Add → By topic → `/aurora_semantic/vehicle_bounding_boxes` if you have the `gb_visual_detection_3d_msgs` package on the VM (e.g. from a built workspace). For boxes in 3D you only need the MarkerArray.

**If the VM is on another host (different machine):** ensure the VM and the robot are on the same LAN. If discovery still fails, set Cyclone DDS to use the robot’s IP, e.g. on the VM:

```bash
export CYCLONEDDS_URI='<Discovery><Peers><Peer address="ROBOT_IP"/></Peers></Discovery>'
```

Replace `ROBOT_IP` with the robot/desktop IP.

---

## 2. Quick diagnostics (on the machine running the launch)

**Check that semantic and depth are published:**

```bash
ros2 topic hz /slamware_ros_sdk_server_node/semantic_labels
ros2 topic hz /slamware_ros_sdk_server_node/depth_image_raw
```

**Check TF (camera → map):**

```bash
ros2 run tf2_ros tf2_echo slamware_map camera_depth_optical_frame
```

**Check vehicle boxes (if any):**

```bash
ros2 topic echo /aurora_semantic/vehicle_bounding_boxes
```

**Check markers:**

```bash
ros2 topic hz /aurora_semantic/vehicle_markers
```

---

## 3. What the fusion node logs mean

After the changes in `aurora_semantic_fusion_node.py`:

- **"Receiving semantic: WxH"** / **"Receiving depth: WxH"** – fusion is getting both streams.
- **"Alignment: semantic ... -> depth ..."** – resolutions and scaling are as expected.
- **"Publishing N vehicle box(es): ..."** – at least one vehicle (car/truck/bus etc.) was detected and projected to the map.
- **"Vehicle mask(s) above min_area but 0 boxes"** – vehicle pixels exist but depth or TF failed (check depth and `camera_depth_optical_frame` → `slamware_map`).
- **"TF lookup failed (slamware_map <- camera_depth_optical_frame): ..."** – fix the TF tree (Aurora + static transforms so that `camera_depth_optical_frame` is connected to `slamware_map`).

TF was switched to “latest” time instead of the depth stamp to reduce clock-sync issues; if you still see no boxes, run the diagnostics above and fix TF or depth.
