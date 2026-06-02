"""Tests for geometry_utils: standoff pose computation and quaternion helpers."""

import math
import pytest

from inspection_manager.geometry_utils import (
    quaternion_from_yaw,
    yaw_from_quaternion,
    standoff_goal_robot_tyre_xy,
    standoff_goal_vehicle_center_tire_xy,
)


# --- quaternion_from_yaw and yaw_from_quaternion ---

def test_quaternion_roundtrip_zero():
    q = quaternion_from_yaw(0.0)
    assert abs(yaw_from_quaternion(q)) < 1e-9


def test_quaternion_roundtrip_positive():
    for yaw in [0.1, 0.5, math.pi / 2, math.pi, -math.pi / 4, -math.pi]:
        q = quaternion_from_yaw(yaw)
        recovered = yaw_from_quaternion(q)
        # Both should be equivalent (mod 2π, clamped to (-π, π])
        diff = abs(math.atan2(math.sin(yaw - recovered), math.cos(yaw - recovered)))
        assert diff < 1e-9, f"yaw={yaw:.3f} recovered={recovered:.3f}"


def test_quaternion_unit_norm():
    for yaw in [0.0, 1.0, math.pi]:
        q = quaternion_from_yaw(yaw)
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        assert abs(norm - 1.0) < 1e-9, f"quaternion not unit at yaw={yaw}"


# --- standoff_goal_robot_tyre_xy ---

def test_standoff_robot_tyre_goal_is_between_robot_and_tyre():
    # Robot at (3, 0), tyre at (0, 0), offset 1.0m
    # Goal should be at (1, 0) — 1m from tyre toward robot
    gx, gy, heading, dist = standoff_goal_robot_tyre_xy(3.0, 0.0, 0.0, 0.0, 1.0)
    assert abs(gx - 1.0) < 1e-6
    assert abs(gy - 0.0) < 1e-6
    assert abs(dist - 3.0) < 1e-6


def test_standoff_robot_tyre_heading_faces_tyre():
    # Robot at (2, 0), tyre at (0, 0): robot approaches from +x.
    # Goal at (1, 0), heading should face tyre at (0,0) from (1,0) → heading = π (west)
    gx, gy, heading, _dist = standoff_goal_robot_tyre_xy(2.0, 0.0, 0.0, 0.0, 1.0)
    # heading = atan2(ty - gy, tx - gx) = atan2(0 - 0, 0 - 1) = atan2(0, -1) = π
    assert abs(heading - math.pi) < 1e-6


def test_standoff_robot_tyre_diagonal():
    # Robot at (3, 3), tyre at (0, 0)
    gx, gy, heading, dist = standoff_goal_robot_tyre_xy(3.0, 3.0, 0.0, 0.0, math.sqrt(2))
    # offset = sqrt(2), direction = (1/sqrt(2), 1/sqrt(2))
    assert abs(gx - 1.0) < 1e-5
    assert abs(gy - 1.0) < 1e-5


def test_standoff_robot_tyre_degenerate_collocated():
    # Robot and tyre at same position: falls back to (1, 0) direction
    gx, gy, heading, dist = standoff_goal_robot_tyre_xy(0.0, 0.0, 0.0, 0.0, 1.0)
    # Should not crash; goal is offset in fallback direction
    assert math.isfinite(gx) and math.isfinite(gy)


# --- standoff_goal_vehicle_center_tire_xy ---

def test_standoff_vehicle_center_goal_outside_vehicle():
    # Vehicle center (0,0), tyre at (2,0), offset 1.0m → goal at (3, 0)
    result = standoff_goal_vehicle_center_tire_xy(0.0, 0.0, 2.0, 0.0, 1.0)
    assert result is not None
    gx, gy, heading, d = result
    assert abs(gx - 3.0) < 1e-6
    assert abs(gy - 0.0) < 1e-6
    assert abs(d - 2.0) < 1e-6


def test_standoff_vehicle_center_heading_faces_tyre():
    # Vehicle center (0,0), tyre at (2,0), goal at (3,0)
    # heading = atan2(ty-gy, tx-gx) = atan2(0-0, 2-3) = atan2(0, -1) = π
    result = standoff_goal_vehicle_center_tire_xy(0.0, 0.0, 2.0, 0.0, 1.0)
    assert result is not None
    _, _, heading, _ = result
    assert abs(heading - math.pi) < 1e-6


def test_standoff_vehicle_center_degenerate_returns_none():
    # Tyre at vehicle center: degenerate, should return None
    result = standoff_goal_vehicle_center_tire_xy(1.0, 1.0, 1.0, 1.0, 1.0)
    assert result is None
