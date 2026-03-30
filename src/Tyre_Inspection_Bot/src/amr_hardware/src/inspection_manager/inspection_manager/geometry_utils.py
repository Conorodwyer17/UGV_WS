"""Geometry helpers for inspection goals: yaw/quaternion and tyre standoff poses."""

from __future__ import annotations

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import Header


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a geometry_msgs/Quaternion from a yaw angle (Z-axis rotation)."""
    half_yaw = yaw * 0.5
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    # roll = pitch = 0, so x = y = 0
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw angle from a geometry_msgs/Quaternion."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def standoff_goal_robot_tyre_xy(
    rx: float,
    ry: float,
    tx: float,
    ty: float,
    offset_m: float,
    min_sep_m: float = 0.08,
) -> Tuple[float, float, float, float]:
    """Standoff along tyre→robot: goal is offset_m from tyre center toward the robot.

    Matches tyre_3d_direct behaviour: robot stops in front of the tyre along the approach ray.
    Returns (goal_x, goal_y, yaw_facing_tyre, distance_robot_to_tyre_xy).
    """
    dx, dy = rx - tx, ry - ty
    d = math.hypot(dx, dy)
    if d < min_sep_m:
        dx, dy = 1.0, 0.0
        d = 1.0
    gx = tx + offset_m * dx / d
    gy = ty + offset_m * dy / d
    heading = math.atan2(ty - gy, tx - gx)
    dist_rt = math.hypot(rx - tx, ry - ty)
    return gx, gy, heading, dist_rt


def standoff_goal_vehicle_center_tire_xy(
    vcx: float,
    vcy: float,
    tx: float,
    ty: float,
    offset_m: float,
    min_radius_m: float = 0.05,
) -> Optional[Tuple[float, float, float, float]]:
    """Far-side standoff: from vehicle center through tyre center, offset_m past tyre (outside vehicle).

    Returns (goal_x, goal_y, yaw_facing_tyre, distance_vehicle_center_to_tire) or None if degenerate.
    """
    dx, dy = tx - vcx, ty - vcy
    d = math.hypot(dx, dy)
    if d < min_radius_m:
        return None
    gx = tx + offset_m * dx / d
    gy = ty + offset_m * dy / d
    heading = math.atan2(ty - gy, tx - gx)
    return gx, gy, heading, d


def pose_stamped_from_standoff_xy(
    map_frame: str,
    stamp,
    gx: float,
    gy: float,
    tz: float,
    heading: float,
) -> PoseStamped:
    """Build PoseStamped in map frame from standoff XY + tyre z + yaw facing tyre."""
    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = map_frame
    pose.header.stamp = stamp
    pose.pose.position = Point(x=gx, y=gy, z=tz)
    pose.pose.orientation = quaternion_from_yaw(heading)
    return pose
