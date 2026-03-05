from types import SimpleNamespace

from builtin_interfaces.msg import Time

from inspection_manager.goal_generator import compute_box_goal


class _DummyClock:
    class _Now:
        def to_msg(self):
            return Time(sec=0, nanosec=0)

    def now(self):
        return self._Now()


class _DummyNode:
    def __init__(self, params, robot_pose=None):
        self._params = params
        self._robot_pose = robot_pose
        self.tf_buffer = SimpleNamespace(lookup_transform=lambda *args, **kwargs: (_ for _ in ()).throw(Exception("no tf")))
        self._last_detection_msg_time = None

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def _get_current_pose(self):
        return self._robot_pose

    def get_clock(self):
        return _DummyClock()


def _make_box(**kwargs):
    d = {
        "xmin": 0.0,
        "xmax": 1.0,
        "ymin": 0.0,
        "ymax": 1.0,
        "zmin": 0.0,
        "zmax": 1.0,
    }
    d.update(kwargs)
    return SimpleNamespace(**d)


def _make_pose(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )


def test_compute_box_goal_rejects_invalid_box():
    node = _DummyNode(
        params={
            "world_frame": "slamware_map",
            "map_frame": "map",
            "base_frame": "base_link",
            "require_goal_transform": True,
            "goal_max_age_s": 5.0,
            "detection_stamp_max_age_s": 0.5,
        },
        robot_pose=_make_pose(),
    )
    out = compute_box_goal(node, _make_box(xmin=1.0, xmax=1.0), offset=0.5)
    assert out["failure_code"] == "target_invalid"


def test_compute_box_goal_rejects_stale_detection():
    node = _DummyNode(
        params={
            "world_frame": "slamware_map",
            "map_frame": "map",
            "base_frame": "base_link",
            "require_goal_transform": True,
            "goal_max_age_s": 0.001,
            "detection_stamp_max_age_s": 0.5,
        },
        robot_pose=_make_pose(),
    )
    node._last_detection_msg_time = 0.0
    out = compute_box_goal(node, _make_box(), offset=0.5)
    assert out["failure_code"] == "target_stale"


def test_compute_box_goal_requires_tf_when_configured():
    node = _DummyNode(
        params={
            "world_frame": "slamware_map",
            "map_frame": "map",
            "base_frame": "base_link",
            "require_goal_transform": True,
            "goal_max_age_s": 5.0,
            "detection_stamp_max_age_s": 0.5,
        },
        robot_pose=_make_pose(),
    )
    out = compute_box_goal(node, _make_box(), offset=0.5)
    assert out["failure_code"] == "tf_unavailable"
