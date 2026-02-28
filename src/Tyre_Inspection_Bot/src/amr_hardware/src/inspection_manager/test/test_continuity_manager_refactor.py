import importlib.util
from pathlib import Path


_MODULE_PATH = Path(__file__).resolve().parents[1] / "inspection_manager" / "continuity_manager.py"
_SPEC = importlib.util.spec_from_file_location("canonical_continuity_module", str(_MODULE_PATH))
_MOD = importlib.util.module_from_spec(_SPEC)
assert _SPEC is not None and _SPEC.loader is not None
_SPEC.loader.exec_module(_MOD)
ContinuityManager = _MOD.ContinuityManager


def _t(tire_id, x, y):
    return {"tire_id": tire_id, "position": {"x": x, "y": y, "z": 0.0}}


def test_direction_lock_persists_after_initial_order():
    mgr = ContinuityManager()
    tires = [_t("t1", 1.0, 0.0), _t("t2", 0.0, 1.0), _t("t3", -1.0, 0.0), _t("t4", 0.0, -1.0)]
    ordered_1 = mgr.order_tires(tires, robot_pose={"x": 2.0, "y": 0.0, "z": 0.0})
    assert mgr.direction_lock in ("cw", "ccw")
    ordered_2 = mgr.order_tires(tires, robot_pose={"x": -2.0, "y": 0.0, "z": 0.0})
    assert [t["tire_id"] for t in ordered_1] == [t["tire_id"] for t in ordered_2]


def test_flip_direction_changes_order():
    mgr = ContinuityManager()
    tires = [_t("t1", 1.0, 0.0), _t("t2", 0.0, 1.0), _t("t3", -1.0, 0.0), _t("t4", 0.0, -1.0)]
    order_a = [t["tire_id"] for t in mgr.order_tires(tires, robot_pose={"x": 2.0, "y": 0.0, "z": 0.0})]
    mgr.flip_direction()
    order_b = [t["tire_id"] for t in mgr.order_tires(tires, robot_pose={"x": 2.0, "y": 0.0, "z": 0.0})]
    assert order_a != order_b
