# Aurora Demo → ROS Integration Feature Map

Generated: 2026-02-22

## Summary

Classic Aurora (this hardware) does **not** support hardware depth camera (`depthcam_view` reports "Depth camera is not supported"). The ROS stack uses **aurora_sdk_bridge** which subscribes to slamware left/right images and computes depth via **StereoSGBM** (software stereo). Demo artifacts relevant to this pipeline are: **frame_preview** (raw stereo acquisition), **calibration_exporter** (calibration format), and **depthcam_view** (reference for future hardware depth if available).

---

## 1. depthcam_view — Hardware Depth & PLY Export

**Location:** `aurora_remote_sdk_demo/demo/depthcam_view/src/depthcam_view.cpp`

**Status:** N/A for Classic Aurora (hardware depth not supported).

**Relevant code patterns for future porting:**
- `generatePointCloud()` — converts `RemoteEnhancedImagingFrame` + texture to PLY
- `savePointCloudToPLY()` — writes ASCII PLY with x,y,z,r,g,b
- Invalid point filtering: skip NaN, Inf, zero-depth
- Connection: auto-discover or `tcp://IP:1445` (Enhanced Imaging port)

**Adaptation:** Not applicable; use aurora_sdk_bridge software depth → point cloud.

---

## 2. frame_preview — Raw Stereo Image Acquisition

**Location:** `aurora_remote_sdk_demo/demo/frame_preview/`

**Purpose:** Captures raw left/right images via SDK callback `onRawCamImageData(timestamp_ns, left, right)`.

**Integration note:** slamware_ros_sdk_server_node already publishes `/slamware_ros_sdk_server_node/left_image_raw` and `right_image_raw`. aurora_sdk_bridge subscribes to these. No demo port needed for image acquisition.

**Snippet (for reference):**
```cpp
void onRawCamImageData(uint64_t timestamp_ns, 
                      const RemoteImageRef& left, 
                      const RemoteImageRef& right) override {
    cv::Mat leftImage, rightImage;
    left.toMat(leftImage);
    right.toMat(rightImage);
}
```

---

## 3. calibration_exporter — Camera Calibration Format

**Location:** `aurora_remote_sdk_demo/demo/calibration_exporter/`

**Output files:** `left_camera_calibration.xml/yml`, `right_camera_calibration.xml/yml`, `stereo_calibration.xml/yml`, `transform_calibration.xml/yml`.

**Classic Aurora:** `getCameraCalibration()` returns NOT_READY(-7). Exporter may fail; use checkerboard fisheye calibration instead.

**Useful:** OpenCV FileStorage format; can inform YAML structure for `equidistant_calibration.yaml`:
- camera_matrix, distortion_coefficients, image_width, image_height
- stereo: baseline, rotation, translation

---

## 4. aurora_sdk_bridge — Software Stereo Depth Pipeline

**Location:** `src/aurora_sdk_bridge/scripts/aurora_sdk_bridge_node.py`

**Behavior:**
- Subscribes to left/right images via ApproximateTimeSynchronizer (slop=0.1 s)
- Loads equidistant stereo calibration from YAML
- Uses `cv2.fisheye.stereoRectify` + `cv2.fisheye.initUndistortRectifyMap`
- StereoSGBM for disparity → depth
- Publishes: `/camera/depth/image`, `/camera/depth/camera_info`, `/camera/depth/points`, `/stereo/navigation_permitted`

**Ported logic from demo:** N/A — uses OpenCV stereo pipeline directly; calibration format aligns with demo exporter structure.

---

## 5. equidistant_calibration.yaml

**Location:** `src/aurora_sdk_bridge/config/equidistant_calibration.yaml`

**Required fields:** image_width, image_height, baseline_m, left/right camera_matrix, left/right dist_coeffs, rotation, translation.

**Source:** PHASE1_CALIBRATION_ANALYSIS recommends checkerboard fisheye calibration; current file may be placeholder.

---

## 6. slamware_ros_sdk — ROS Publisher of Left/Right Images

**Location:** `src/aurora_ros2_sdk_linux/src/slamware_ros_sdk/`

**Topics:** left_image_raw, right_image_raw, scan, odom, map, tf.

**getCameraCalibration:** ServerStereoCameraInfoWorker retries; Classic Aurora returns -7. Resolved by using aurora_sdk_bridge + YAML calibration.

---

## 7. depth_gate_node — Safety Gating

**Location:** `ugv_nav/scripts/depth_gate_node.py`

**Behavior:** Subscribes to `/stereo/navigation_permitted` and `cmd_vel_nav`; publishes to `/cmd_vel`. If navigation_permitted==False, publish zero twist.

---

## 8. nav_aurora.yaml — Costmap Obstacle Source

**Location:** `ugv_nav/param/nav_aurora.yaml`

**Integration:** Point cloud topic (e.g. `/camera/depth/points`) as obstacle source for local/global costmaps.

---

## 9. segmentation_processor — 3D Bounding Boxes

**Location:** `segmentation_3d/src/segmentation_processor.cpp`

**Inputs:** Point cloud, camera_info, detections. **Output:** `/darknet_ros_3d/bounding_boxes` (BoundingBoxes3d).

---

## 10. inspection_manager — Tire Approach & Photo Capture

**Location:** `inspection_manager/`

**Behavior:** Sends approach goals, queries `/stereo/pipeline_state`, triggers photo capture on arrival.

---

## Prioritized Demo → ROS Porting Candidates

| Priority | Demo/Artifact            | Port Target                         | Action                                        |
|----------|--------------------------|-------------------------------------|-----------------------------------------------|
| 1        | calibration_exporter     | equidistant_calibration.yaml        | Use output format; checkerboard if SDK fails  |
| 2        | depthcam_view PLY logic  | depth_to_registered_pointcloud_node | Reference for NaN/Inf filtering, PLY export   |
| 3        | frame_preview sync       | aurora_sdk_bridge                   | Reference timestamp handling; sync via sync   |
| 4        | StereoSGBM params        | aurora_sdk_bridge                   | Tune numDisparities, blockSize, P1, P2        |
| 5        | TF/transform_calibration | static_transform_publisher          | Use transform_calibration.xml if available    |

---

## Calibration Parameters Expected by aurora_sdk_bridge

```yaml
image_width: 640
image_height: 480
baseline_m: 0.06
left:
  camera_matrix: [[fx,0,cx],[0,fy,cy],[0,0,1]]
  dist_coeffs: [k1,k2,k3,k4]  # equidistant
right:
  camera_matrix: ...
  dist_coeffs: ...
rotation: [[1,0,0],[0,1,0],[0,0,1]]
translation: [-0.06, 0, 0]
```
