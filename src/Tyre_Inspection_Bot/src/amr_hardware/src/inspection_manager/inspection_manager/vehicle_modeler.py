"""Vehicle geometry and tire position estimation.

Uses bounding box dimensions and robot position to infer:
- Vehicle extent (front/rear extreme points)
- Orientation (longitudinal axis = hood-to-trunk; inferred from box, front = end closer to robot)
- Four tire positions (FL, FR, RL, RR) based on wheelbase and track.

Ground-plane: 3D AABB bottom face is projected to z = box.zmin (ground); footprint is the
oriented rectangle in XY. When box is axis-aligned (segment_3d output), length/width are
the longer/shorter box dimensions; front = end closer to robot. For rotated boxes (future),
orientation could come from PCA on point cloud inside the box.
"""
import logging
import math
from typing import List, Tuple, Optional, Any

_log = logging.getLogger(__name__)


def _box_extents(box: Any) -> Tuple[float, float, float, float, float]:
    """Return (center_x, center_y, center_z, extent_x, extent_y) from BoundingBox3d."""
    cx = (box.xmin + box.xmax) / 2.0
    cy = (box.ymin + box.ymax) / 2.0
    cz = (box.zmin + box.zmax) / 2.0
    ex = abs(box.xmax - box.xmin)
    ey = abs(box.ymax - box.ymin)
    return (cx, cy, cz, ex, ey)


def get_vehicle_footprint(
    box: Any,
    robot_pos: Optional[Tuple[float, float, float]],
) -> Optional[Tuple[float, float, float, float, float]]:
    """Return oriented footprint on ground plane: (center_x, center_y, length_m, width_m, yaw_rad).

    Projects box bottom (z = box.zmin) to ground; length = longer XY extent, width = shorter.
    Front = end of long axis closer to robot. yaw = atan2(fwd_y, fwd_x) in world frame.
    Returns None if box is degenerate.
    """
    cx, cy, cz, ex, ey = _box_extents(box)
    if not (math.isfinite(cx) and math.isfinite(cy)) or (ex < 0.01 and ey < 0.01):
        return None
    fwd_x, fwd_y = _infer_orientation(cx, cy, ex, ey, robot_pos)
    length = max(ex, ey)
    width = min(ex, ey)
    yaw = math.atan2(fwd_y, fwd_x)
    return (cx, cy, length, width, yaw)


ORIENTATION_AMBIGUOUS_THRESHOLD_M = 0.5


def _infer_orientation(
    center_x: float,
    center_y: float,
    extent_x: float,
    extent_y: float,
    robot_pos: Optional[Tuple[float, float, float]],
) -> Tuple[float, float]:
    """Infer vehicle longitudinal axis (fwd_x, fwd_y): hood direction = end closer to robot.

    - Longer box dimension = car length (hood to trunk).
    - Shorter dimension = car width (driver to passenger).
    - Front (hood) = the end of the long axis closer to the robot.
    - When box is square (extent_x ≈ extent_y, e.g. Aurora 1x1 m), bias toward
      front = robot direction (vehicle_center → robot) for better planned tire positions.
    """
    if robot_pos is None:
        return (1.0, 0.0)  # fallback: +x
    rx, ry, _ = robot_pos

    # When box is nearly square (Aurora semantic 1x1 m), use robot direction for front:
    # front = end closer to robot, so fwd = direction from vehicle center toward robot.
    # Log at debug to avoid flooding console when Aurora sends fixed 1x1 m boxes every callback.
    if abs(extent_x - extent_y) < ORIENTATION_AMBIGUOUS_THRESHOLD_M:
        _log.debug(
            "Orientation ambiguous: extent_x=%.2f m, extent_y=%.2f m (diff < %.1f m). "
            "Using robot direction for front (center → robot).",
            extent_x, extent_y, ORIENTATION_AMBIGUOUS_THRESHOLD_M,
        )
        dx = rx - center_x
        dy = ry - center_y
        dist = math.hypot(dx, dy)
        if dist >= 0.01:
            return (dx / dist, dy / dist)
        return (1.0, 0.0)

    if extent_x >= extent_y:
        # Long axis along x: front is either +x or -x
        end_plus = (center_x + extent_x / 2.0, center_y)
        end_minus = (center_x - extent_x / 2.0, center_y)
        d_plus = math.hypot(end_plus[0] - rx, end_plus[1] - ry)
        d_minus = math.hypot(end_minus[0] - rx, end_minus[1] - ry)
        fwd = (1.0, 0.0) if d_plus < d_minus else (-1.0, 0.0)
    else:
        # Long axis along y: front is either +y or -y
        end_plus = (center_x, center_y + extent_y / 2.0)
        end_minus = (center_x, center_y - extent_y / 2.0)
        d_plus = math.hypot(end_plus[0] - rx, end_plus[1] - ry)
        d_minus = math.hypot(end_minus[0] - rx, end_minus[1] - ry)
        fwd = (0.0, 1.0) if d_plus < d_minus else (0.0, -1.0)
    return fwd


def estimate_tire_positions_from_box(
    box: Any,
    robot_pos: Optional[Tuple[float, float, float]],
    wheelbase_m: float = 2.7,
    track_m: float = 1.6,
) -> List[Tuple[float, float, float]]:
    """Estimate 4 tire positions (FL, FR, RL, RR) from bounding box and robot position.

    Ground-plane: tire z is set to box.zmin (bottom of 3D AABB = ground contact).
    Uses box dimensions to infer vehicle orientation:
    - Longer dimension (x or y) = car length (hood to trunk).
    - Front (hood) = end of long axis closer to robot.
    - Tire anchors at ±(wheelbase/2) along long axis, ±(track/2) along short axis.
    When box dimensions exceed defaults (e.g. sim vehicle 4.5x2 m), uses box extents
    so tires align with box corners. Falls back to center+robot if box is degenerate.
    """
    cx, cy, cz, ex, ey = _box_extents(box)
    if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(cz)):
        return []
    # Ground plane: project footprint to z = bottom of box (tires on ground)
    z_ground = getattr(box, "zmin", cz)
    if not math.isfinite(z_ground):
        z_ground = cz
    if ex < 0.1 and ey < 0.1:
        # Degenerate box: use center + robot orientation
        return estimate_tire_positions((cx, cy, z_ground), robot_pos, wheelbase_m, track_m)
    fwd_x, fwd_y = _infer_orientation(cx, cy, ex, ey, robot_pos)
    # Right = perpendicular to fwd (right-hand rule: fwd x up)
    right_x = -fwd_y
    right_y = fwd_x
    # Use box dimensions when larger than defaults (e.g. sim vehicle 4.5x2 m)
    length = max(ex, ey)
    width = min(ex, ey)
    half_wb = max(wheelbase_m * 0.5, length * 0.5)
    half_tr = max(track_m * 0.5, width * 0.5)
    # FL, FR, RL, RR order; z = ground plane
    raw = [
        (cx + fwd_x * half_wb - right_x * half_tr, cy + fwd_y * half_wb - right_y * half_tr, z_ground),
        (cx + fwd_x * half_wb + right_x * half_tr, cy + fwd_y * half_wb + right_y * half_tr, z_ground),
        (cx - fwd_x * half_wb - right_x * half_tr, cy - fwd_y * half_wb - right_y * half_tr, z_ground),
        (cx - fwd_x * half_wb + right_x * half_tr, cy - fwd_y * half_wb + right_y * half_tr, z_ground),
    ]
    # Clamp tires to box so front tires are on front face, rear on rear face (small-box fix)
    length = max(ex, ey)
    width = min(ex, ey)
    half_len = length * 0.5
    half_wid = width * 0.5
    return [
        _clamp_tire_to_box_fwd_right(t, cx, cy, t[2], fwd_x, fwd_y, right_x, right_y, half_len, half_wid)
        for t in raw
    ]


def _clamp_tire_to_box_fwd_right(
    tire: Tuple[float, float, float],
    cx: float,
    cy: float,
    z: float,
    fwd_x: float,
    fwd_y: float,
    right_x: float,
    right_y: float,
    half_len: float,
    half_wid: float,
) -> Tuple[float, float, float]:
    """Clamp tire to box: longitudinal ±half_len, lateral ±half_wid (in fwd/right frame).

    Keeps front tires on the front face and rear on the rear face when box is small.
    """
    tx, ty, _ = tire
    dx = tx - cx
    dy = ty - cy
    a = dx * fwd_x + dy * fwd_y   # longitudinal
    b = dx * right_x + dy * right_y  # lateral
    inset = 0.12
    a_c = max(-half_len + inset, min(half_len - inset, a))
    b_c = max(-half_wid + inset, min(half_wid - inset, b))
    tx_c = cx + a_c * fwd_x + b_c * right_x
    ty_c = cy + a_c * fwd_y + b_c * right_y
    return (tx_c, ty_c, z)


def estimate_tire_positions(
    vehicle_center: Tuple[float, float, float],
    robot_pos: Optional[Tuple[float, float, float]],
    wheelbase_m: float = 2.7,
    track_m: float = 1.6,
) -> List[Tuple[float, float, float]]:
    """Estimate 4 tire positions (FL, FR, RL, RR) from vehicle center and robot position.

    Fallback when no box: assumes longitudinal axis = vehicle center → robot.
    """
    vx, vy, vz = vehicle_center
    if robot_pos is None:
        return []
    rx, ry, _ = robot_pos
    dx = vx - rx
    dy = vy - ry
    dist = math.hypot(dx, dy)
    if dist < 0.01:
        return []
    fwd_x = dx / dist
    fwd_y = dy / dist
    right_x = -fwd_y
    right_y = fwd_x
    half_wb = wheelbase_m * 0.5
    half_tr = track_m * 0.5
    # FL, FR, RL, RR order
    return [
        (vx + fwd_x * half_wb - right_x * half_tr, vy + fwd_y * half_wb - right_y * half_tr, vz),
        (vx + fwd_x * half_wb + right_x * half_tr, vy + fwd_y * half_wb + right_y * half_tr, vz),
        (vx - fwd_x * half_wb - right_x * half_tr, vy - fwd_y * half_wb - right_y * half_tr, vz),
        (vx - fwd_x * half_wb + right_x * half_tr, vy - fwd_y * half_wb + right_y * half_tr, vz),
    ]


def box_front_rear_points(
    box: Any,
    robot_pos: Optional[Tuple[float, float, float]],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Return (front_point, rear_point) in slamware_map frame.

    Front = hood end (closer to robot); rear = trunk end.
    Uses longer box dimension as car length.
    """
    cx, cy, cz, ex, ey = _box_extents(box)
    if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(cz)):
        return ((cx, cy, cz), (cx, cy, cz))
    fwd_x, fwd_y = _infer_orientation(cx, cy, ex, ey, robot_pos)
    half_len = max(ex, ey) / 2.0
    front = (cx + fwd_x * half_len, cy + fwd_y * half_len, cz)
    rear = (cx - fwd_x * half_len, cy - fwd_y * half_len, cz)
    return (front, rear)
