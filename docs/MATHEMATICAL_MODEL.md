# Mathematical Model – UGV Tyre Inspection Robot

This document describes the kinematic model, control limits, and detection-to-3D projection used in the system.

## 1. Robot Kinematics

### Differential Drive Model

The Waveshare UGV Rover uses differential drive. Wheel velocities relate to body velocities:

```
v = (v_left + v_right) / 2
omega = (v_right - v_left) / wheelbase
```

Where:
- `v`: linear velocity (m/s)
- `omega`: angular velocity (rad/s)
- `wheelbase`: distance between wheels (m)

### Odometry from Wheel Encoders

When ESP32 provides T:1001 feedback (wheel ticks), odometry is computed:

```
d_left = (ticks_left - prev_ticks_left) * metres_per_tick
d_right = (ticks_right - prev_ticks_right) * metres_per_tick
d_center = (d_left + d_right) / 2
d_theta = (d_right - d_left) / wheelbase

x += d_center * cos(theta)
y += d_center * sin(theta)
theta += d_theta
```

Published on `/wheel/odometry` when `publish_wheel_odom:=true`.

### Aurora Odometry

Aurora provides 6DOF pose directly. The `slamware_ros_sdk_server_node` publishes:
- `/slamware_ros_sdk_server_node/odom` – odometry in `odom` frame
- TF: `slamware_map` → `odom` → `base_link`

No wheel encoder integration when using Aurora-only localisation.

## 2. Control Limits

| Parameter | Value | Source |
|-----------|-------|--------|
| Max linear velocity | 0.3 m/s (configurable) | Nav2 controller |
| Max angular velocity | 0.5 rad/s | Nav2 controller |
| xy_goal_tolerance | 0.15 m (default), 0.1 m (tight) | nav_aurora.yaml |
| yaw_goal_tolerance | 0.25 rad (default) | nav_aurora.yaml |
| Vehicle buffer (speed filter) | 1.5 m | vehicle_speed_filter_node |

## 3. Detection to 3D Projection

### Camera Pinhole Model

Aurora depth uses intrinsics from `aurora_depth_intrinsics.yaml`:

```
u = fx * (X / Z) + cx
v = fy * (Y / Z) + cy
```

Inverse (pixel + depth to 3D):

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

### 2D Detection to 3D Bounding Box

For YOLO detection (bbox in image space):

1. Get depth at bbox centre (or median over bbox)
2. Unproject centre to 3D in camera frame
3. Compute bbox extent from pixel size and depth (scale with depth)
4. Transform to `slamware_map` via TF

The `segmentation_processor_tire` node performs this projection using `camera_info` and depth.

### Frame Conventions

- `slamware_map`: world frame for all detections
- `camera_depth_optical_frame`: depth image frame (optical centre, Z forward)
- `base_link`: robot base
