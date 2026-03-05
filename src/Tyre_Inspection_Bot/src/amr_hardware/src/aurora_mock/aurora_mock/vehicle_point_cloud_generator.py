"""
Vehicle point cloud generator for simulation realism (Phase 3).

Generates points on a simplified vehicle model: box body + 4 cylindrical wheels.
Used to produce realistic depth and point cloud for segment_3d and tire detection.
"""
import math
from typing import Any, List, Optional, Tuple

import numpy as np


def _sample_box_surface(
    xmin: float, ymin: float, zmin: float,
    xmax: float, ymax: float, zmax: float,
    points_per_face: int = 50,
) -> np.ndarray:
    """Sample points on the surface of an axis-aligned box."""
    points = []
    n = max(2, int(math.sqrt(points_per_face)))
    # Face x=xmin
    for i in range(n + 1):
        for j in range(n + 1):
            y = ymin + (ymax - ymin) * i / n
            z = zmin + (zmax - zmin) * j / n
            points.append([xmin, y, z])
    # Face x=xmax
    for i in range(n + 1):
        for j in range(n + 1):
            y = ymin + (ymax - ymin) * i / n
            z = zmin + (zmax - zmin) * j / n
            points.append([xmax, y, z])
    # Face y=ymin
    for i in range(n + 1):
        for j in range(n + 1):
            x = xmin + (xmax - xmin) * i / n
            z = zmin + (zmax - zmin) * j / n
            points.append([x, ymin, z])
    # Face y=ymax
    for i in range(n + 1):
        for j in range(n + 1):
            x = xmin + (xmax - xmin) * i / n
            z = zmin + (zmax - zmin) * j / n
            points.append([x, ymax, z])
    # Face z=zmin (ground)
    for i in range(n + 1):
        for j in range(n + 1):
            x = xmin + (xmax - xmin) * i / n
            y = ymin + (ymax - ymin) * j / n
            points.append([x, y, zmin])
    # Face z=zmax
    for i in range(n + 1):
        for j in range(n + 1):
            x = xmin + (xmax - xmin) * i / n
            y = ymin + (ymax - ymin) * j / n
            points.append([x, y, zmax])
    return np.array(points, dtype=np.float32)


def _sample_cylinder_surface(
    cx: float, cy: float, cz: float,
    radius: float, height: float,
    n_angle: int = 16, n_height: int = 8,
) -> np.ndarray:
    """Sample points on the surface of a vertical cylinder (axis along Z)."""
    points = []
    for i in range(n_angle + 1):
        angle = 2 * math.pi * i / n_angle
        x0 = cx + radius * math.cos(angle)
        y0 = cy + radius * math.sin(angle)
        for j in range(n_height + 1):
            z = cz + (j / n_height - 0.5) * height
            points.append([x0, y0, z])
    # Top and bottom caps
    for cap_z in (cz - height / 2, cz + height / 2):
        for i in range(n_angle):
            angle = 2 * math.pi * (i + 0.5) / n_angle
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append([x, y, cap_z])
    return np.array(points, dtype=np.float32)


def generate_vehicle_points(
    vehicle_x: float, vehicle_y: float, vehicle_z: float,
    extent_x: float, extent_y: float,
    wheelbase_m: float = 2.7, track_m: float = 1.6,
    tire_radius_m: float = 0.35, tire_height_m: float = 0.4,
    robot_x: float = 0.0, robot_y: float = 0.0,
) -> np.ndarray:
    """
    Generate points on vehicle body (box) + 4 tires (cylinders) in slamware_map frame.

    Vehicle is axis-aligned. Front = end closer to robot. Tire positions follow
    vehicle_modeler logic: FL, FR, RL, RR.
    """
    cx, cy, cz = vehicle_x, vehicle_y, vehicle_z
    ex, ey = extent_x, extent_y
    hw, ht = wheelbase_m * 0.5, track_m * 0.5
    z_ground = vehicle_z - 0.3  # box zmin

    # Infer orientation: front = closer to robot
    dx = robot_x - cx
    dy = robot_y - cy
    dist = math.hypot(dx, dy)
    if dist < 0.01:
        fwd_x, fwd_y = 1.0, 0.0
    else:
        fwd_x, fwd_y = dx / dist, dy / dist
    right_x, right_y = -fwd_y, fwd_x

    # Body box (axis-aligned in world; vehicle may be rotated - use AABB)
    half_x = max(ex, ey) / 2.0
    half_y = min(ex, ey) / 2.0
    if ex >= ey:
        xmin, xmax = cx - half_x, cx + half_x
        ymin, ymax = cy - half_y, cy + half_y
    else:
        xmin, xmax = cx - half_y, cx + half_y
        ymin, ymax = cy - half_x, cy + half_x
    zmin = z_ground
    zmax = cz + 0.3

    body_points = _sample_box_surface(xmin, ymin, zmin, xmax, ymax, zmax)

    # Tire positions (FL, FR, RL, RR)
    tire_centers = [
        (cx + fwd_x * hw - right_x * ht, cy + fwd_y * hw - right_y * ht, z_ground + tire_height_m / 2),
        (cx + fwd_x * hw + right_x * ht, cy + fwd_y * hw + right_y * ht, z_ground + tire_height_m / 2),
        (cx - fwd_x * hw - right_x * ht, cy - fwd_y * hw - right_y * ht, z_ground + tire_height_m / 2),
        (cx - fwd_x * hw + right_x * ht, cy - fwd_y * hw + right_y * ht, z_ground + tire_height_m / 2),
    ]

    tire_points = []
    for tx, ty, tz in tire_centers:
        tp = _sample_cylinder_surface(tx, ty, tz, tire_radius_m, tire_height_m)
        tire_points.append(tp)

    all_points = np.vstack([body_points] + tire_points)
    return all_points


def generate_vehicle_points_from_box(
    box: Any,
    robot_pos: Optional[Tuple[float, float, float]],
    wheelbase_m: float = 2.7,
    track_m: float = 1.6,
    tire_radius_m: float = 0.35,
    tire_height_m: float = 0.4,
) -> np.ndarray:
    """
    Generate points on vehicle body (box) + 4 tires (cylinders) from a BoundingBox3d.

    Uses box AABB for body; tire positions follow vehicle_modeler logic (FL, FR, RL, RR).
    """
    xmin = getattr(box, "xmin", 0.0)
    xmax = getattr(box, "xmax", 0.0)
    ymin = getattr(box, "ymin", 0.0)
    ymax = getattr(box, "ymax", 0.0)
    zmin = getattr(box, "zmin", 0.0)
    zmax = getattr(box, "zmax", 0.0)
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    ex = abs(xmax - xmin)
    ey = abs(ymax - ymin)
    if not (math.isfinite(cx) and math.isfinite(cy)) or (ex < 0.1 and ey < 0.1):
        return np.zeros((0, 3), dtype=np.float32)

    # Body: sample surface of AABB
    body_points = _sample_box_surface(xmin, ymin, zmin, xmax, ymax, zmax)

    # Tire positions (same logic as vehicle_modeler / simulated_detection_node)
    if robot_pos is None:
        return body_points
    rx, ry, _ = robot_pos
    dx, dy = rx - cx, ry - cy
    dist = math.hypot(dx, dy)
    if dist < 0.01:
        fwd_x, fwd_y = 1.0, 0.0
    elif abs(ex - ey) < 0.5:
        fwd_x, fwd_y = dx / dist, dy / dist
    elif ex >= ey:
        d_plus = math.hypot(cx + ex / 2 - rx, cy - ry)
        d_minus = math.hypot(cx - ex / 2 - rx, cy - ry)
        fwd_x, fwd_y = (1.0, 0.0) if d_plus < d_minus else (-1.0, 0.0)
    else:
        d_plus = math.hypot(cx - rx, cy + ey / 2 - ry)
        d_minus = math.hypot(cx - rx, cy - ey / 2 - ry)
        fwd_x, fwd_y = (0.0, 1.0) if d_plus < d_minus else (0.0, -1.0)
    right_x, right_y = -fwd_y, fwd_x
    hw, ht = wheelbase_m * 0.5, track_m * 0.5
    z_ground = zmin
    tire_centers = [
        (cx + fwd_x * hw - right_x * ht, cy + fwd_y * hw - right_y * ht, z_ground + tire_height_m / 2),
        (cx + fwd_x * hw + right_x * ht, cy + fwd_y * hw + right_y * ht, z_ground + tire_height_m / 2),
        (cx - fwd_x * hw - right_x * ht, cy - fwd_y * hw - right_y * ht, z_ground + tire_height_m / 2),
        (cx - fwd_x * hw + right_x * ht, cy - fwd_y * hw + right_y * ht, z_ground + tire_height_m / 2),
    ]

    tire_points = []
    for tx, ty, tz in tire_centers:
        tp = _sample_cylinder_surface(tx, ty, tz, tire_radius_m, tire_height_m)
        tire_points.append(tp)

    all_points = np.vstack([body_points] + tire_points)
    return all_points


def transform_points_to_frame(
    points: np.ndarray,
    transform_translation: Tuple[float, float, float],
    transform_rotation: Tuple[float, float, float, float],
) -> np.ndarray:
    """
    Transform 3D points using a transform (translation + quaternion xyzw).
    transform is from source frame to target frame: p_target = R @ p_source + t.
    """
    tx, ty, tz = transform_translation
    qx, qy, qz, qw = transform_rotation
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float32)
    out = (points @ R.T) + np.array([tx, ty, tz], dtype=np.float32)
    return out


def project_points_to_depth_image(
    points_cam: np.ndarray,
    depth_width: int, depth_height: int,
    fx: float = 180.0, fy: float = 180.0, cx: float = 208.0, cy: float = 112.0,
) -> np.ndarray:
    """
    Project 3D points in camera_depth_optical_frame to depth image.
    Returns depth image (height x width) with valid depth at projected pixels.
    points_cam: Nx3 array (x, y, z) in camera frame; z is depth.
    """
    depth = np.zeros((depth_height, depth_width), dtype=np.float32)
    for i in range(points_cam.shape[0]):
        x, y, z = points_cam[i, 0], points_cam[i, 1], points_cam[i, 2]
        if z <= 0.01:
            continue
        u = int(round(fx * x / z + cx))
        v = int(round(fy * y / z + cy))
        if 0 <= u < depth_width and 0 <= v < depth_height:
            if depth[v, u] == 0 or z < depth[v, u]:
                depth[v, u] = z
    return depth
