import math
from typing import Optional, Dict, Any

import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf2_ros import TransformException

from .geometry_utils import quaternion_from_yaw, yaw_from_quaternion


def compute_box_goal(node, box, offset: float) -> Optional[Dict[str, Any]]:
    """Compute a navigation goal for a detected box. Returns goal + metadata or None on failure."""
    world_frame = node.get_parameter("world_frame").value
    map_frame = node.get_parameter("map_frame").value
    require_tf = bool(node.get_parameter("require_goal_transform").value)

    center_x = (box.xmin + box.xmax) / 2.0
    center_y = (box.ymin + box.ymax) / 2.0
    center_z = (box.zmin + box.zmax) / 2.0

    robot_pose = node._get_current_pose()
    if robot_pose is None:
        return None

    robot_x = robot_pose.pose.position.x
    robot_y = robot_pose.pose.position.y
    robot_z = robot_pose.pose.position.z

    dx = center_x - robot_x
    dy = center_y - robot_y
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < 0.01:
        # If we're essentially at the object center, keep current heading.
        heading = yaw_from_quaternion(robot_pose.pose.orientation)
    else:
        heading = math.atan2(dy, dx)

    goal_x = center_x - offset * math.cos(heading)
    goal_y = center_y - offset * math.sin(heading)
    goal_z = center_z

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
            return None
        goal = PoseStamped()
        goal.header = Header()
        goal.header.frame_id = map_frame
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = goal_z
        goal.pose.orientation = quaternion_from_yaw(heading)

    return {
        "goal": goal,
        "goal_world": (goal_x, goal_y, goal_z),
        "heading": heading,
        "distance_to_object": dist,
        "center": (center_x, center_y, center_z),
        "robot_pos": (robot_x, robot_y, robot_z),
        "world_frame": world_frame,
        "transform_ok": transform_ok,
    }
