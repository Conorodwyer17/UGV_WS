"""Intermediate map poses to approach far-side tyres around the vehicle footprint.

Opposite-lateral case uses a short polyline (outside FL/FR or RL/RR corners) then standoff, not a single mid-bumper point.
"""

from __future__ import annotations

import math
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped

from .geometry_utils import (
    pose_stamped_from_standoff_xy,
    standoff_goal_robot_tyre_xy,
)
from .tyre_order import tyre_lon_lat


def _yaw_point_to_point(
    fx: float, fy: float, tx: float, ty: float
) -> float:
    return math.atan2(ty - fy, tx - fx)


def _same_lateral_side(lat_robot: float, lat_tyre: float, eps: float) -> bool:
    """True if robot and tyre are on the same side of vehicle centerline (outside)."""
    if abs(lat_robot) <= eps or abs(lat_tyre) <= eps:
        return True
    return (lat_robot > 0.0) == (lat_tyre > 0.0)


def build_tyre_approach_waypoints(
    map_frame: str,
    stamp,
    robot_xy: Tuple[float, float],
    tyre_xyz: Tuple[float, float, float],
    vehicle_center_xy: Tuple[float, float],
    fwd_xy: Tuple[float, float],
    right_xy: Tuple[float, float],
    half_length_m: float,
    half_width_m: float,
    standoff_m: float,
    clearance_m: float,
    lateral_eps_m: float = 0.2,
) -> List[PoseStamped]:
    """Return 1..N PoseStamped in map frame: optional perimeter waypoints then tyre standoff.

    When robot and tyre are on opposite sides (lateral), inserts two corner poses on the chosen
    bumper edge (expanded by ``clearance_m``), then standoff — reduces corner-cutting vs one bridge point.
    """
    rx, ry = robot_xy
    tx, ty, tz = tyre_xyz
    vx, vy = vehicle_center_xy
    fx, fy = fwd_xy
    rx_dir, ry_dir = right_xy

    _, lat_r = tyre_lon_lat((rx, ry, 0.0), vehicle_center_xy, fwd_xy, right_xy)
    _, lat_t = tyre_lon_lat(tyre_xyz, vehicle_center_xy, fwd_xy, right_xy)

    waypoints: List[PoseStamped] = []

    if _same_lateral_side(lat_r, lat_t, lateral_eps_m):
        gx, gy, heading, _ = standoff_goal_robot_tyre_xy(rx, ry, tx, ty, standoff_m)
        waypoints.append(
            pose_stamped_from_standoff_xy(map_frame, stamp, gx, gy, tz, heading)
        )
        return waypoints

    # Opposite lateral: polyline along the outside of the front or rear edge (two corners), then standoff.
    # Single mid-bumper bridge let Nav2 cut inside; corner waypoints force a wider path around the vehicle.
    c = max(clearance_m, 0.15)
    hlp = half_length_m + c
    hwp = half_width_m + c
    # tyre_order: FL = +fwd −right, FR = +fwd +right, RL = −fwd −right, RR = −fwd +right
    fl = (vx + fx * hlp - rx_dir * hwp, vy + fy * hlp - ry_dir * hwp)
    fr = (vx + fx * hlp + rx_dir * hwp, vy + fy * hlp + ry_dir * hwp)
    rl = (vx - fx * hlp - rx_dir * hwp, vy - fy * hlp - ry_dir * hwp)
    rr = (vx - fx * hlp + rx_dir * hwp, vy - fy * hlp + ry_dir * hwp)

    front_mid_x = vx + fx * (half_length_m + c)
    front_mid_y = vy + fy * (half_length_m + c)
    rear_mid_x = vx - fx * (half_length_m + c)
    rear_mid_y = vy - fy * (half_length_m + c)

    d_front = math.hypot(front_mid_x - rx, front_mid_y - ry) + math.hypot(
        front_mid_x - tx, front_mid_y - ty
    )
    d_rear = math.hypot(rear_mid_x - rx, rear_mid_y - ry) + math.hypot(
        rear_mid_x - tx, rear_mid_y - ty
    )

    if d_front <= d_rear:
        edge = [fl, fr]
    else:
        edge = [rl, rr]

    # Visit the nearer corner on this edge first (shorter path along the bumper).
    d0 = math.hypot(edge[0][0] - rx, edge[0][1] - ry)
    d1 = math.hypot(edge[1][0] - rx, edge[1][1] - ry)
    ordered = [edge[0], edge[1]] if d0 <= d1 else [edge[1], edge[0]]

    for i, (px, py) in enumerate(ordered):
        if i + 1 < len(ordered):
            nx, ny = ordered[i + 1][0], ordered[i + 1][1]
            yaw = _yaw_point_to_point(px, py, nx, ny)
        else:
            gx_s, gy_s, _, _ = standoff_goal_robot_tyre_xy(px, py, tx, ty, standoff_m)
            yaw = _yaw_point_to_point(px, py, gx_s, gy_s)
        waypoints.append(pose_stamped_from_standoff_xy(map_frame, stamp, px, py, tz, yaw))

    lx, ly = ordered[-1][0], ordered[-1][1]
    gx, gy, heading, _ = standoff_goal_robot_tyre_xy(lx, ly, tx, ty, standoff_m)
    waypoints.append(pose_stamped_from_standoff_xy(map_frame, stamp, gx, gy, tz, heading))
    return waypoints
