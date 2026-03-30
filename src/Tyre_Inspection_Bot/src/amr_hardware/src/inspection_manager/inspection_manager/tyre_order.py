"""Tyre inspection visit order from robot position relative to vehicle (FL/FR/RL/RR convention).

Vehicle axes match vehicle_modeler: *fwd* = hood/toward front; *right* = lateral (FL is fwd − right).
"""

from __future__ import annotations

import math
from enum import IntEnum
from typing import List, Sequence, Tuple

# Slot indices: 0=FL, 1=FR, 2=RL, 3=RR (same as estimate_tire_positions_from_box)
SLOT_FL = 0
SLOT_FR = 1
SLOT_RL = 2
SLOT_RR = 3


class RobotSide(IntEnum):
    """Which region around the vehicle the robot occupies (vehicle-centric frame)."""

    LEFT = 0
    RIGHT = 1
    FRONT = 2
    REAR = 3


def classify_robot_side(
    lon_m: float,
    lat_m: float,
    lateral_dominance_ratio: float = 1.0,
) -> RobotSide:
    """Classify robot region from longitudinal/lateral offsets in vehicle frame.

    lon_m: dot(robot - center, fwd) — positive toward front (hood).
    lat_m: dot(robot - center, right) — positive toward vehicle right / passenger side (FL uses −right).
    """
    if lateral_dominance_ratio <= 0.0:
        lateral_dominance_ratio = 1.0
    # Prefer left/right when |lat| dominates; else front/rear.
    if abs(lat_m) * lateral_dominance_ratio >= abs(lon_m):
        if lat_m >= 0.0:
            return RobotSide.RIGHT
        return RobotSide.LEFT
    if lon_m >= 0.0:
        return RobotSide.FRONT
    return RobotSide.REAR


def inspection_order_indices(robot_side: RobotSide) -> List[int]:
    """Return tyre slot indices visit order (FL=0, FR=1, RL=2, RR=3) for perimeter-friendly inspection."""
    if robot_side == RobotSide.LEFT:
        return [SLOT_FL, SLOT_RL, SLOT_RR, SLOT_FR]
    if robot_side == RobotSide.RIGHT:
        return [SLOT_FR, SLOT_RR, SLOT_RL, SLOT_FL]
    if robot_side == RobotSide.FRONT:
        return [SLOT_FL, SLOT_FR, SLOT_RR, SLOT_RL]
    return [SLOT_RL, SLOT_RR, SLOT_FR, SLOT_FL]


def robot_lon_lat(
    robot_xy: Tuple[float, float],
    vehicle_center_xy: Tuple[float, float],
    fwd_xy: Tuple[float, float],
    right_xy: Tuple[float, float],
) -> Tuple[float, float]:
    """Project robot offset onto vehicle forward / right axes (no normalization required if axes unit)."""
    dx = robot_xy[0] - vehicle_center_xy[0]
    dy = robot_xy[1] - vehicle_center_xy[1]
    lon = dx * fwd_xy[0] + dy * fwd_xy[1]
    lat = dx * right_xy[0] + dy * right_xy[1]
    return lon, lat


def tyre_lon_lat(
    tyre_xyz: Tuple[float, float, float],
    vehicle_center_xy: Tuple[float, float],
    fwd_xy: Tuple[float, float],
    right_xy: Tuple[float, float],
) -> Tuple[float, float]:
    """Tyre centre offset in vehicle frame (xy only)."""
    return robot_lon_lat((tyre_xyz[0], tyre_xyz[1]), vehicle_center_xy, fwd_xy, right_xy)


def greedy_assign_poses_to_slots(
    pose_positions: Sequence[Tuple[float, float, float]],
    slot_centers: Sequence[Tuple[float, float, float]],
) -> List[int]:
    """Assign each slot index to a pose index (unique poses when possible).

    Returns list of length len(slot_centers): ``result[slot] = pose_index``.
    """
    n_slots = len(slot_centers)
    n_poses = len(pose_positions)
    if n_slots == 0 or n_poses == 0:
        return []
    pairs: List[Tuple[float, int, int]] = []
    for s in range(n_slots):
        sx, sy, _ = slot_centers[s]
        for p in range(n_poses):
            px, py, _ = pose_positions[p]
            pairs.append((math.hypot(px - sx, py - sy), s, p))
    pairs.sort(key=lambda t: t[0])
    slot_to_pose: dict[int, int] = {}
    used_pose = set()
    for _d, s, p in pairs:
        if s in slot_to_pose or p in used_pose:
            continue
        slot_to_pose[s] = p
        used_pose.add(p)
    # Fill missing slots (duplicate poses only if n_poses < n_slots)
    for s in range(n_slots):
        if s not in slot_to_pose:
            best_p = 0
            best_d = float("inf")
            sx, sy, _ = slot_centers[s]
            for p in range(n_poses):
                px, py, _ = pose_positions[p]
                d = math.hypot(px - sx, py - sy)
                if d < best_d:
                    best_d = d
                    best_p = p
            slot_to_pose[s] = best_p
    return [slot_to_pose[s] for s in range(n_slots)]


def ordered_tyres_from_slots(
    pose_positions: Sequence[Tuple[float, float, float]],
    slot_centers: Sequence[Tuple[float, float, float]],
    visit_slot_indices: Sequence[int],
) -> List[Tuple[float, float, float]]:
    """Build visit-ordered tyre positions using greedy slot assignment and visit order."""
    assign = greedy_assign_poses_to_slots(pose_positions, slot_centers)
    out: List[Tuple[float, float, float]] = []
    for slot_idx in visit_slot_indices:
        pi = assign[slot_idx]
        out.append(tuple(pose_positions[pi]))
    return out
