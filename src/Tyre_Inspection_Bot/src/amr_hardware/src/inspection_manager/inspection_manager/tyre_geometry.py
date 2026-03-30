"""Vehicle geometry and tyre slot classification from tyre-only 3D positions (no Aurora vehicle box)."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .tyre_order import RobotSide, classify_robot_side, inspection_order_indices

SLOT_FL = 0
SLOT_FR = 1
SLOT_RL = 2
SLOT_RR = 3


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.array([1.0, 0.0])
    return v / n


class TyreBasedVehicleGeometry:
    """Estimate vehicle axes, extents, and FL/FR/RL/RR slots from tyre XY positions."""

    def __init__(self, xy_points: np.ndarray):
        self._points = np.asarray(xy_points, dtype=float)
        if self._points.ndim != 2 or self._points.shape[1] != 2:
            raise ValueError("xy_points must be (N, 2)")
        self.n = self._points.shape[0]
        self.mean = np.mean(self._points, axis=0)
        self.longitudinal_axis = np.array([1.0, 0.0])
        self.right_axis = np.array([0.0, 1.0])
        self.front = 0.0
        self.rear = 0.0
        self.left = 0.0
        self.right = 0.0
        self.length = 1.0
        self.width = 1.0
        self.half_length = 0.5
        self.half_width = 0.5
        self.orientation = 0.0
        self.point_index_to_slot: List[int] = [SLOT_FL] * self.n
        self._compute_geometry()

    def _compute_geometry(self) -> None:
        pts = self._points
        mean = self.mean
        n = self.n
        X = pts - mean

        if n < 2:
            return

        if n == 2:
            vec = pts[1] - pts[0]
            self.length = float(np.linalg.norm(vec))
            self.longitudinal_axis = _unit(vec)
            self.right_axis = np.array([-self.longitudinal_axis[1], self.longitudinal_axis[0]])
            self.width = 0.5
            self.half_length = max(self.length * 0.5, 0.25)
            self.half_width = 0.25
            proj_long = X @ self.longitudinal_axis
            proj_lat = X @ self.right_axis
            self.front = float(np.max(proj_long))
            self.rear = float(np.min(proj_long))
            self.left = float(np.max(proj_lat))
            self.right = float(np.min(proj_lat))
            self.orientation = math.atan2(self.longitudinal_axis[1], self.longitudinal_axis[0])
            self.point_index_to_slot[0] = SLOT_RL
            self.point_index_to_slot[1] = SLOT_FR
            return

        if n >= 4:
            long_guess = _unit(np.array([1.0, 0.0]))
            proj = X @ long_guess
            idx = np.argsort(proj)
            rear_i = idx[: n // 2]
            front_i = idx[n // 2 :]
            rear_c = np.mean(pts[rear_i], axis=0)
            front_c = np.mean(pts[front_i], axis=0)
            self.longitudinal_axis = _unit(front_c - rear_c)
            self.right_axis = np.array([-self.longitudinal_axis[1], self.longitudinal_axis[0]])
        else:
            cov = np.cov(X.T)
            evals, evecs = np.linalg.eigh(cov)
            order = np.argsort(evals)[::-1]
            self.longitudinal_axis = _unit(evecs[:, order[0]])
            self.right_axis = _unit(evecs[:, order[1]])

        proj_long = (pts - mean) @ self.longitudinal_axis
        proj_lat = (pts - mean) @ self.right_axis
        self.front = float(np.max(proj_long))
        self.rear = float(np.min(proj_long))
        self.left = float(np.max(proj_lat))
        self.right = float(np.min(proj_lat))
        self.length = max(self.front - self.rear, 0.1)
        self.width = max(self.left - self.right, 0.1)
        self.half_length = self.length * 0.5
        self.half_width = self.width * 0.5
        self.orientation = math.atan2(self.longitudinal_axis[1], self.longitudinal_axis[0])

        if n == 4:
            order = np.argsort(proj_long)
            rear_i = order[:2]
            front_i = order[2:]
            fi = front_i[np.argsort(proj_lat[front_i])]
            ri = rear_i[np.argsort(proj_lat[rear_i])]
            FL_i, FR_i = int(fi[0]), int(fi[1])
            RL_i, RR_i = int(ri[0]), int(ri[1])
            for i, s in zip([FL_i, FR_i, RL_i, RR_i], [SLOT_FL, SLOT_FR, SLOT_RL, SLOT_RR]):
                self.point_index_to_slot[i] = s
        else:
            for i in range(n):
                long_c = float(proj_long[i])
                lat_c = float(proj_lat[i])
                ml = float(np.median(proj_long))
                mlat = float(np.median(proj_lat))
                front = long_c >= ml
                left = lat_c <= mlat
                if front:
                    self.point_index_to_slot[i] = SLOT_FL if left else SLOT_FR
                else:
                    self.point_index_to_slot[i] = SLOT_RL if left else SLOT_RR

    def classify_slot_for_point(self, x: float, y: float) -> int:
        vec = np.array([x - self.mean[0], y - self.mean[1]])
        long_c = float(vec @ self.longitudinal_axis)
        lat_c = float(vec @ self.right_axis)
        front = long_c >= (self.front + self.rear) * 0.5
        left = lat_c <= (self.left + self.right) * 0.5
        if front:
            return SLOT_FL if left else SLOT_FR
        return SLOT_RL if left else SLOT_RR

    def robot_side_enum(self, rx: float, ry: float) -> RobotSide:
        lon, lat = self.robot_lon_lat(rx, ry)
        return classify_robot_side(lon, lat)

    def robot_lon_lat(self, rx: float, ry: float) -> Tuple[float, float]:
        vec = np.array([rx - self.mean[0], ry - self.mean[1]])
        lon = float(vec @ self.longitudinal_axis)
        lat = float(vec @ self.right_axis)
        return lon, lat

    def visit_order_pose_indices(self, rx: float, ry: float) -> List[int]:
        """Indices into the original pose list (0..n-1) in inspection order."""
        side = self.robot_side_enum(rx, ry)
        slot_order = inspection_order_indices(side)
        slot_to_point: dict[int, int] = {}
        for i, slot in enumerate(self.point_index_to_slot):
            slot_to_point[slot] = i
        out: List[int] = []
        for slot in slot_order:
            if slot in slot_to_point:
                out.append(slot_to_point[slot])
        return out


def tyre_geometry_from_poses(
    poses: Sequence[object],
    min_points: int = 2,
) -> Optional[TyreBasedVehicleGeometry]:
    """Build geometry from geometry_msgs/Pose list. Returns None if too few points."""
    if len(poses) < min_points:
        return None
    pts = []
    for p in poses:
        pos = getattr(p, "position", p)
        x = float(getattr(pos, "x", 0.0))
        y = float(getattr(pos, "y", 0.0))
        pts.append([x, y])
    arr = np.array(pts, dtype=float)
    return TyreBasedVehicleGeometry(arr)
