from inspection_manager.tire_enumerator import TireEnumerator


def test_enumerator_generates_tires():
    enum = TireEnumerator()
    tires = enum.enumerate_tires({"vehicle_class": "car", "center": {"x": 0.0, "y": 0.0, "z": 0.0}})
    assert len(tires) == 4
    assert tires[0]["tire_id"].startswith("axle")
