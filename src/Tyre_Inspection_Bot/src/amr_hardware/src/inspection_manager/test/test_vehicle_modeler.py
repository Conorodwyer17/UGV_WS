from inspection_manager.vehicle_modeler import (
    estimate_tire_positions,
    estimate_tire_positions_from_box,
    box_front_rear_points,
)


def _make_box(xmin, xmax, ymin, ymax, zmin=0, zmax=1):
    """Fake BoundingBox3d-like object."""
    class Box:
        pass
    b = Box()
    b.xmin, b.xmax = xmin, xmax
    b.ymin, b.ymax = ymin, ymax
    b.zmin, b.zmax = zmin, zmax
    return b


def test_estimate_tire_positions_returns_four():
    vehicle_center = (5.0, 2.0, 0.0)
    robot_pos = (0.0, 0.0, 0.0)
    positions = estimate_tire_positions(vehicle_center, robot_pos, wheelbase_m=2.0, track_m=1.0)
    assert len(positions) == 4


def test_estimate_tire_positions_requires_robot_pose():
    vehicle_center = (1.0, 1.0, 0.0)
    assert estimate_tire_positions(vehicle_center, None) == []


def test_estimate_tire_positions_from_box_returns_four():
    # Box longer in x (5m) than y (2m) -> car length along x; robot at (0,0) -> front = +x (closer)
    box = _make_box(xmin=0, xmax=5, ymin=1, ymax=3)
    robot_pos = (0.0, 2.0, 0.0)
    positions = estimate_tire_positions_from_box(box, robot_pos, wheelbase_m=2.0, track_m=1.0)
    assert len(positions) == 4


def test_box_front_rear_points_uses_longer_dimension():
    # Box 5m x 2m, robot at (0,2) -> front = end closer to robot; xmin (0,2) closer than xmax (5,2)
    box = _make_box(xmin=0, xmax=5, ymin=1, ymax=3)
    robot_pos = (0.0, 2.0, 0.0)
    front, rear = box_front_rear_points(box, robot_pos)
    assert front[0] < rear[0]  # front is -x (closer to robot at x=0)


def test_estimate_tire_positions_from_box_clamps_small_box():
    # 1 m box (e.g. Aurora semantic): tires must lie on box boundary, not 1.35 m in front of center
    box = _make_box(xmin=-6.13, xmax=-5.13, ymin=1.6, ymax=2.6, zmin=0, zmax=1.2)
    robot_pos = (0.0, 0.0, 0.0)
    positions = estimate_tire_positions_from_box(box, robot_pos, wheelbase_m=2.7, track_m=1.6)
    assert len(positions) == 4
    cx, cy = -5.63, 2.1
    half_ex, half_ey = 0.5, 0.5
    for tx, ty, tz in positions:
        assert cx - half_ex <= tx <= cx + half_ex, f"Tire x {tx} outside box x [{cx - half_ex}, {cx + half_ex}]"
        assert cy - half_ey <= ty <= cy + half_ey, f"Tire y {ty} outside box y [{cy - half_ey}, {cy + half_ey}]"
