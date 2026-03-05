from types import SimpleNamespace

from inspection_manager.perception_handler import find_vehicle_box, find_tire_for_inspection


def _box(name, prob=0.9, xmin=0.0, xmax=1.0, ymin=0.0, ymax=1.0, zmin=0.0, zmax=1.0):
    return SimpleNamespace(
        object_name=name,
        probability=prob,
        xmin=xmin,
        xmax=xmax,
        ymin=ymin,
        ymax=ymax,
        zmin=zmin,
        zmax=zmax,
    )


class _Logger:
    def debug(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def test_find_vehicle_box_ignores_invalid_extents():
    boxes = [_box("car", xmin=1.0, xmax=1.0), _box("car", prob=0.8, xmin=2.0, xmax=3.0)]
    out = find_vehicle_box(boxes, ["car"], 0.5)
    assert out is not None
    assert out.xmin == 2.0


def test_find_tire_for_inspection_filters_invalid_boxes():
    boxes = [_box("car-tire", xmin=1.0, xmax=1.0), _box("car-tire", prob=0.7, xmin=3.0, xmax=3.4)]
    out = find_tire_for_inspection(
        boxes=boxes,
        tire_label="car-tire",
        min_prob=0.3,
        inspected_positions=[],
        tolerance=0.5,
        vehicle_pos=None,
        max_tire_distance_from_vehicle=5.0,
        robot_pos=(0.0, 0.0, 0.0),
        target_position=None,
        logger=_Logger(),
    )
    assert out is not None
    assert out.xmin == 3.0
