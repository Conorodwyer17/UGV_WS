"""
Stub ROS 2 message modules so unit tests run on any machine without a ROS install.

Only the attributes that the tested modules actually access are stubbed.
The stubs are registered in sys.modules before any test file imports inspection_manager code.
"""
import sys
import types
import math


def _make_simple_module(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    return mod


# ---- geometry_msgs.msg ----

class _Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x; self.y = y; self.z = z; self.w = w

class _PoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()

class _Pose:
    def __init__(self):
        self.position = _Point()
        self.orientation = _Quaternion()

class _Twist:
    def __init__(self):
        class _V: x = y = z = 0.0
        self.linear = _V(); self.angular = _V()

class _PoseArray:
    def __init__(self):
        self.header = _Header()
        self.poses = []

geometry_msgs_mod = _make_simple_module("geometry_msgs")
class _Vector3Stamped:
    def __init__(self):
        self.header = _Header()
        self.vector = type("V3", (), {"x": 0.0, "y": 0.0, "z": 0.0})()

class _TransformStamped:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.transform = type("T", (), {
            "translation": _Point(),
            "rotation": _Quaternion(),
        })()

geometry_msgs_msg = _make_simple_module(
    "geometry_msgs.msg",
    Point=_Point,
    PoseStamped=_PoseStamped,
    Pose=_Pose,
    Quaternion=_Quaternion,
    Twist=_Twist,
    PoseArray=_PoseArray,
    Vector3Stamped=_Vector3Stamped,
    TransformStamped=_TransformStamped,
)
sys.modules.setdefault("geometry_msgs", geometry_msgs_mod)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)


# ---- std_msgs.msg ----

class _Bool:
    def __init__(self):
        self.data = False

class _String:
    def __init__(self):
        self.data = ""

class _Empty:
    pass

class _Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None

std_msgs_mod = _make_simple_module("std_msgs")
std_msgs_msg = _make_simple_module(
    "std_msgs.msg",
    Bool=_Bool,
    String=_String,
    Empty=_Empty,
    Header=_Header,
)
sys.modules.setdefault("std_msgs", std_msgs_mod)
sys.modules.setdefault("std_msgs.msg", std_msgs_msg)


# ---- nav_msgs.msg ----

class _Odometry:
    def __init__(self):
        self.header = _Header()
        self.pose = type("PC", (), {"pose": _Pose(), "covariance": [0.0] * 36})()

nav_msgs_mod = _make_simple_module("nav_msgs")
nav_msgs_msg = _make_simple_module("nav_msgs.msg", Odometry=_Odometry)
sys.modules.setdefault("nav_msgs", nav_msgs_mod)
sys.modules.setdefault("nav_msgs.msg", nav_msgs_msg)


# ---- builtin_interfaces.msg ----

class _Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec; self.nanosec = nanosec

builtin_mod = _make_simple_module("builtin_interfaces")
builtin_msg = _make_simple_module("builtin_interfaces.msg", Time=_Time)
sys.modules.setdefault("builtin_interfaces", builtin_mod)
sys.modules.setdefault("builtin_interfaces.msg", builtin_msg)


# ---- gb_visual_detection_3d_msgs.msg ----

class _BoundingBox3d:
    def __init__(self, object_name="", probability=0.9,
                 xmin=0.0, xmax=1.0, ymin=0.0, ymax=1.0, zmin=0.0, zmax=1.0):
        self.object_name = object_name
        self.probability = probability
        self.xmin = xmin; self.xmax = xmax
        self.ymin = ymin; self.ymax = ymax
        self.zmin = zmin; self.zmax = zmax
        self.header = _Header()

class _BoundingBoxes3d:
    def __init__(self):
        self.header = _Header()
        self.bounding_boxes = []

gb_mod = _make_simple_module("gb_visual_detection_3d_msgs")
gb_msg = _make_simple_module(
    "gb_visual_detection_3d_msgs.msg",
    BoundingBox3d=_BoundingBox3d,
    BoundingBoxes3d=_BoundingBoxes3d,
)
sys.modules.setdefault("gb_visual_detection_3d_msgs", gb_mod)
sys.modules.setdefault("gb_visual_detection_3d_msgs.msg", gb_msg)


# ---- visualization_msgs.msg ----

vis_mod = _make_simple_module("visualization_msgs")
vis_msg = _make_simple_module(
    "visualization_msgs.msg",
    Marker=type("Marker", (), {}),
    MarkerArray=type("MarkerArray", (), {}),
)
sys.modules.setdefault("visualization_msgs", vis_mod)
sys.modules.setdefault("visualization_msgs.msg", vis_msg)


# ---- sensor_msgs.msg ----

sensor_mod = _make_simple_module("sensor_msgs")
sensor_msg = _make_simple_module(
    "sensor_msgs.msg",
    Image=type("Image", (), {"header": None, "data": b"", "encoding": "bgr8",
                             "width": 0, "height": 0, "step": 0}),
)
sys.modules.setdefault("sensor_msgs", sensor_mod)
sys.modules.setdefault("sensor_msgs.msg", sensor_msg)


# ---- action_msgs.msg ----

class _GoalStatus:
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6

action_mod = _make_simple_module("action_msgs")
action_msg = _make_simple_module("action_msgs.msg", GoalStatus=_GoalStatus)
sys.modules.setdefault("action_msgs", action_mod)
sys.modules.setdefault("action_msgs.msg", action_msg)


# ---- rclpy stubs ----

class _Logger:
    def info(self, *a, **kw): pass
    def warn(self, *a, **kw): pass
    def error(self, *a, **kw): pass
    def debug(self, *a, **kw): pass

class _Duration:
    def __init__(self, seconds=0.0, nanoseconds=0):
        self.seconds = seconds

class _Time:
    def __init__(self): pass

rclpy_mod = _make_simple_module(
    "rclpy",
    time=_make_simple_module("rclpy.time", Time=_Time),
    duration=_make_simple_module("rclpy.duration", Duration=_Duration),
)
sys.modules.setdefault("rclpy", rclpy_mod)
sys.modules.setdefault("rclpy.time", rclpy_mod.time)
sys.modules.setdefault("rclpy.duration", rclpy_mod.duration)
sys.modules.setdefault("rclpy.node", _make_simple_module("rclpy.node", Node=object))
sys.modules.setdefault("rclpy.action", _make_simple_module("rclpy.action", ActionClient=object))
sys.modules.setdefault("rclpy.qos", _make_simple_module(
    "rclpy.qos",
    QoSProfile=type("QoSProfile", (), {}),
    QoSReliabilityPolicy=type("QoSReliabilityPolicy", (), {"BEST_EFFORT": 0, "RELIABLE": 1}),
    QoSHistoryPolicy=type("QoSHistoryPolicy", (), {"KEEP_LAST": 0}),
))


# ---- tf2_ros stubs ----

class _Buffer:
    def lookup_transform(self, *a, **kw):
        raise Exception("no tf in test stub")

tf2_ros_mod = _make_simple_module(
    "tf2_ros",
    Buffer=_Buffer,
    TransformListener=lambda *a, **kw: None,
    TransformBroadcaster=type("TransformBroadcaster", (), {"sendTransform": lambda *a: None}),
    TransformException=Exception,
    StaticTransformBroadcaster=type("StaticTransformBroadcaster", (), {}),
)
sys.modules.setdefault("tf2_ros", tf2_ros_mod)


# ---- tf2_geometry_msgs ----

tf2_geom_mod = _make_simple_module("tf2_geometry_msgs")
sys.modules.setdefault("tf2_geometry_msgs", tf2_geom_mod)


# ---- nav2_msgs.action ----

nav2_msgs_mod = _make_simple_module("nav2_msgs")
nav2_msgs_action = _make_simple_module(
    "nav2_msgs.action",
    NavigateToPose=type("NavigateToPose", (), {
        "Goal": type("Goal", (), {"pose": None}),
        "Result": type("Result", (), {}),
        "Feedback": type("Feedback", (), {}),
    }),
    FollowWaypoints=type("FollowWaypoints", (), {
        "Goal": type("Goal", (), {"poses": []}),
    }),
)
nav2_msgs_srv = _make_simple_module(
    "nav2_msgs.srv",
    ClearEntireCostmap=type("ClearEntireCostmap", (), {}),
    GetCosts=type("GetCosts", (), {
        "Request": type("Request", (), {}),
        "Response": type("Response", (), {}),
    }),
)
sys.modules.setdefault("nav2_msgs", nav2_msgs_mod)
sys.modules.setdefault("nav2_msgs.action", nav2_msgs_action)
sys.modules.setdefault("nav2_msgs.srv", nav2_msgs_srv)


# ---- ament_index_python ----

ament_mod = _make_simple_module("ament_index_python")
ament_packages = _make_simple_module(
    "ament_index_python.packages",
    get_package_share_directory=lambda pkg: f"/tmp/share/{pkg}",
)
sys.modules.setdefault("ament_index_python", ament_mod)
sys.modules.setdefault("ament_index_python.packages", ament_packages)


# ---- yaml (already available; just ensure import doesn't fail) ----

import yaml  # noqa: E402 F401 — confirm yaml is importable
