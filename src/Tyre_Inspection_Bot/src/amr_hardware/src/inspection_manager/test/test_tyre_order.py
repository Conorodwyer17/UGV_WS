"""Tests for tyre_order: robot side classification and visit-order generation."""

from inspection_manager.tyre_order import (
    classify_robot_side,
    inspection_order_indices,
    robot_lon_lat,
    RobotSide,
    SLOT_FL, SLOT_FR, SLOT_RL, SLOT_RR,
)


def test_classify_robot_left():
    # Robot far to vehicle's left (large negative lat)
    side = classify_robot_side(lon_m=0.2, lat_m=-3.0)
    assert side == RobotSide.LEFT


def test_classify_robot_right():
    # Robot far to vehicle's right (large positive lat)
    side = classify_robot_side(lon_m=0.1, lat_m=3.0)
    assert side == RobotSide.RIGHT


def test_classify_robot_front():
    # Robot directly in front (large positive lon, small lat)
    side = classify_robot_side(lon_m=3.0, lat_m=0.1)
    assert side == RobotSide.FRONT


def test_classify_robot_rear():
    # Robot directly behind (large negative lon, small lat)
    side = classify_robot_side(lon_m=-3.0, lat_m=0.1)
    assert side == RobotSide.REAR


def test_inspection_order_visits_all_four_tyres():
    for side in RobotSide:
        order = inspection_order_indices(side)
        assert len(order) == 4
        assert set(order) == {SLOT_FL, SLOT_FR, SLOT_RL, SLOT_RR}


def test_inspection_order_left_starts_near():
    # From LEFT, first tyre should be FL (front-left = near corner for left-side approach)
    order = inspection_order_indices(RobotSide.LEFT)
    assert order[0] == SLOT_FL


def test_inspection_order_right_starts_near():
    # From RIGHT, first tyre should be FR
    order = inspection_order_indices(RobotSide.RIGHT)
    assert order[0] == SLOT_FR


def test_robot_lon_lat_forward():
    # Robot 3m ahead of vehicle center, vehicle points along +x
    lon, lat = robot_lon_lat(
        robot_xy=(3.0, 0.0),
        vehicle_center_xy=(0.0, 0.0),
        fwd_xy=(1.0, 0.0),
        right_xy=(0.0, 1.0),
    )
    assert abs(lon - 3.0) < 1e-9
    assert abs(lat) < 1e-9


def test_robot_lon_lat_left_side():
    # Robot 2m to the left of vehicle center, vehicle points along +x
    lon, lat = robot_lon_lat(
        robot_xy=(0.0, -2.0),
        vehicle_center_xy=(0.0, 0.0),
        fwd_xy=(1.0, 0.0),
        right_xy=(0.0, 1.0),
    )
    assert abs(lon) < 1e-9
    assert abs(lat - (-2.0)) < 1e-9
