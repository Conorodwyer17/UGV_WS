from inspection_manager.tire_enumerator import TireEnumerator


def test_tire_enumerator_default_layout():
    enum = TireEnumerator()
    tires = enum.enumerate_tires({"vehicle_class": "car", "center": {"x": 1.0, "y": 2.0, "z": 0.0}})
    assert len(tires) == 4
    assert tires[0]["tire_id"].startswith("axle1_")

