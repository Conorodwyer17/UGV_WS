import math
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from typing import Optional, Dict, Any

import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf2_ros import TransformException

from .geometry_utils import quaternion_from_yaw, yaw_from_quaternion
from .transformer import check_tf_validity

# Nav2 costmap cost constants (nav2_costmap_2d)
COSTMAP_FREE_SPACE = 0
COSTMAP_INSCRIBED_INFLATED = 253  # Reject if cost >= this (obstacle or inflated)


def _is_finite_box(box) -> bool:
    values = [box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax]
    return all(math.isfinite(v) for v in values)


MIN_SAFE_OFFSET_M = 0.5  # Floor for vehicle/large objects
MIN_SAFE_OFFSET_TIRE_M = 0.25  # Floor for tire (small box); allows ~30 cm standoff for front-on capture


def _check_goal_in_free_space(node, goal: PoseStamped) -> Optional[bool]:
    """Query Nav2 GetCosts service; return True if free, False if occupied, None if check skipped.
    Uses a thread to avoid blocking the executor (no spin in callback)."""
    try:
        from nav2_msgs.srv import GetCosts
        svc_name = str(node.get_parameter("goal_costmap_get_cost_service").value)
    except Exception:
        return None

    def _do_call():
        client = node.create_client(GetCosts, svc_name)
        if not client.wait_for_service(timeout_sec=0.3):
            return None
        req = GetCosts.Request()
        req.use_footprint = False
        req.poses = [goal]
        resp = client.call(req)
        if not resp.success or len(resp.costs) == 0:
            return None
        return float(resp.costs[0]) < COSTMAP_INSCRIBED_INFLATED

    try:
        with ThreadPoolExecutor(max_workers=1) as ex:
            future = ex.submit(_do_call)
            return future.result(timeout=1.0)
    except (FuturesTimeoutError, Exception):
        return None


def compute_box_goal(
    node, box, offset: float, detection_stamp=None, vehicle_box=None
) -> Optional[Dict[str, Any]]:
    """Compute a navigation goal for a detected box. Returns goal + metadata or None on failure.
    Goal = object_center - offset * dir(robot->object), so robot stops offset m in front of object.
    For far-side tires: when vehicle_box is provided and target is a tire (small box), places goal
    along vehicle_center->tire_center vector (outside vehicle) to avoid "goal inside vehicle" rejection.
    Enforces a minimum offset (tire boxes: 0.25 m; vehicle: 0.5 m) for safety.
    detection_stamp: optional rclpy.time.Time from message header; if set, TF and age are checked at that time.
    vehicle_box: optional BoundingBox3d; when set for tire goals, uses vehicle-center vector for far-side placement.
    """
    world_frame = node.get_parameter("world_frame").value
    map_frame = node.get_parameter("map_frame").value
    base_frame = node.get_parameter("base_frame").value
    require_tf = bool(node.get_parameter("require_goal_transform").value)
    goal_max_age_s = float(node.get_parameter("goal_max_age_s").value)
    try:
        detection_stamp_max_age_s = float(node.get_parameter("detection_stamp_max_age_s").value)
    except Exception:
        detection_stamp_max_age_s = 0.5

    if not _is_finite_box(box):
        return {"failure_code": "target_invalid"}
    if not (box.xmin < box.xmax and box.ymin < box.ymax and box.zmin < box.zmax):
        return {"failure_code": "target_invalid"}

    detection_age = None
    if hasattr(node, "_last_detection_msg_time") and node._last_detection_msg_time is not None:
        detection_age = max(0.0, (time.time() - node._last_detection_msg_time))
        if detection_age > goal_max_age_s:
            return {"failure_code": "target_stale", "detection_age_s": detection_age}

    # Strict stamp-based age: reject detections older than detection_stamp_max_age_s before creating goals
    if detection_stamp is not None and detection_stamp_max_age_s > 0:
        try:
            now = node.get_clock().now()
            age_s = (now.nanoseconds - detection_stamp.nanoseconds) / 1e9
            if age_s > detection_stamp_max_age_s:
                return {
                    "failure_code": "target_stale",
                    "detection_age_s": age_s,
                    "detection_stamp_age_s": age_s,
                }
        except Exception:
            pass

    # TF validity at detection time: do not use stale poses; reject if transform not available at stamp
    if detection_stamp is not None and require_tf:
        tf_ok, tf_code = check_tf_validity(
            node.tf_buffer,
            map_frame,
            base_frame,
            stamp=detection_stamp,
            timeout_s=0.2,
        )
        if not tf_ok:
            return {"failure_code": tf_code}

    center_x = (box.xmin + box.xmax) / 2.0
    center_y = (box.ymin + box.ymax) / 2.0
    center_z = (box.zmin + box.zmax) / 2.0

    # Use lower minimum for small (tire) boxes so we can get ~30 cm front-on to tire
    box_w = abs(box.xmax - box.xmin)
    box_h = abs(box.ymax - box.ymin)
    is_tire = max(box_w, box_h) < 0.6
    min_offset = MIN_SAFE_OFFSET_TIRE_M if is_tire else MIN_SAFE_OFFSET_M
    offset = max(offset, min_offset)

    robot_pose = node._get_current_pose()
    if robot_pose is None:
        return {"failure_code": "robot_pose_unavailable"}

    robot_x = robot_pose.pose.position.x
    robot_y = robot_pose.pose.position.y
    robot_z = robot_pose.pose.position.z

    # Far-side tire placement (research: IEEE obstacle-inflation-free path planning):
    # When vehicle_box provided and target is a tire, place goal along vehicle_center->tire_center
    # vector so robot stops *outside* the vehicle (avoids "goal_inside_vehicle_box" rejection).
    use_vehicle_center_goal = False
    if vehicle_box is not None and is_tire and _is_finite_box(vehicle_box):
        vc_x = (vehicle_box.xmin + vehicle_box.xmax) / 2.0
        vc_y = (vehicle_box.ymin + vehicle_box.ymax) / 2.0
        vx = center_x - vc_x
        vy = center_y - vc_y
        vnorm = math.sqrt(vx * vx + vy * vy)
        if vnorm > 1e-3:
            ux = vx / vnorm
            uy = vy / vnorm
            goal_x = center_x + ux * offset
            goal_y = center_y + uy * offset
            heading = math.atan2(center_y - goal_y, center_x - goal_x)
            dist = math.sqrt((center_x - robot_x) ** 2 + (center_y - robot_y) ** 2)
            np_x, np_y = center_x, center_y
            dist_to_nearest = 0.0
            use_vehicle_center_goal = True

    if not use_vehicle_center_goal:
        # Nearest point on the box (rectangle) to the robot: ensures standoff is from the nearest face, not center.
        # Avoids driving too close when offset is from center (e.g. 0.7m from center of 1m-deep vehicle = 0.2m from face).
        np_x = max(box.xmin, min(box.xmax, robot_x))
        np_y = max(box.ymin, min(box.ymax, robot_y))
        dnx = robot_x - np_x
        dny = robot_y - np_y
        dist_to_nearest = math.sqrt(dnx * dnx + dny * dny)

        if dist_to_nearest < 1e-3:
            # Robot effectively inside or on the box; use center-based goal and keep at least offset from center.
            dx = center_x - robot_x
            dy = center_y - robot_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 1e-3:
                heading = yaw_from_quaternion(robot_pose.pose.orientation)
                goal_x = robot_x + offset * math.cos(heading)
                goal_y = robot_y + offset * math.sin(heading)
            else:
                goal_x = center_x - offset * math.cos(math.atan2(dy, dx))
                goal_y = center_y - offset * math.sin(math.atan2(dy, dx))
                heading = math.atan2(center_y - goal_y, center_x - goal_x)
        else:
            # Place goal exactly offset meters from the nearest face (along robot -> nearest_pt).
            goal_x = np_x + offset * dnx / dist_to_nearest
            goal_y = np_y + offset * dny / dist_to_nearest
            dist = math.sqrt((center_x - robot_x) ** 2 + (center_y - robot_y) ** 2)
            # Orientation: face the object from the goal position (front-on view), not from robot position
            heading = math.atan2(center_y - goal_y, center_x - goal_x)

    goal_z = center_z
    # Distance from goal to nearest point on box (for validation: should match offset when using nearest-face).
    goal_to_surface_m = math.sqrt((goal_x - np_x) ** 2 + (goal_y - np_y) ** 2) if dist_to_nearest >= 1e-3 else math.sqrt((goal_x - center_x) ** 2 + (goal_y - center_y) ** 2)

    if not all(
        math.isfinite(v)
        for v in (goal_x, goal_y, goal_z, robot_x, robot_y, robot_z)
    ):
        return {"failure_code": "goal_or_robot_nan_inf"}

    goal_slamware = PoseStamped()
    goal_slamware.header = Header()
    goal_slamware.header.frame_id = world_frame
    goal_slamware.header.stamp = node.get_clock().now().to_msg()
    goal_slamware.pose.position.x = goal_x
    goal_slamware.pose.position.y = goal_y
    goal_slamware.pose.position.z = goal_z
    goal_slamware.pose.orientation = quaternion_from_yaw(heading)

    transform_ok = True
    try:
        transform = node.tf_buffer.lookup_transform(
            map_frame, world_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
        )
        goal = tf2_geometry_msgs.do_transform_pose_stamped(goal_slamware, transform)
    except (TransformException, Exception):
        transform_ok = False
        if require_tf:
            return {"failure_code": "tf_unavailable"}
        goal = PoseStamped()
        goal.header = Header()
        goal.header.frame_id = map_frame
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = goal_z
        goal.pose.orientation = quaternion_from_yaw(heading)

    # Optional: reject goal if it lies in occupied/inflated costmap cell (failure_injection_defense_report #7)
    try:
        if bool(node.get_parameter("goal_costmap_precheck").value):
            precheck_result = _check_goal_in_free_space(node, goal)
            if precheck_result is False:
                return {"failure_code": "goal_in_occupied_cell"}
    except Exception:
        pass  # Param or service unavailable; skip precheck, allow goal

    return {
        "goal": goal,
        "goal_world": (goal_x, goal_y, goal_z),
        "heading": heading,
        "distance_to_object": dist,
        "goal_to_surface_m": goal_to_surface_m,
        "center": (center_x, center_y, center_z),
        "robot_pos": (robot_x, robot_y, robot_z),
        "world_frame": world_frame,
        "transform_ok": transform_ok,
        "detection_age_s": detection_age,
        "failure_code": None,
    }
