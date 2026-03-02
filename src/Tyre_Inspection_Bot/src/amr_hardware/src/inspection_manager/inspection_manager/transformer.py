import time
from typing import Optional, Tuple

import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException


# TF validity error codes for goal validation and logging (Nav2/docs: use exact stamp when available)
TF_VALID = "tf_ok"
TF_UNAVAILABLE = "tf_unavailable"
TF_UNAVAILABLE_AT_STAMP = "tf_unavailable_at_detection_time"
TF_STALE = "tf_stale"
TF_FALLBACK_LATEST = "tf_fallback_latest"  # Stamp was future; used latest available


def lookup_transform(tf_buffer, target_frame: str, source_frame: str, timeout_s: float = 0.2):
    """Lookup a transform with a bounded timeout. Returns TransformStamped or None."""
    try:
        return tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=float(timeout_s)),
        )
    except TransformException:
        return None


def lookup_transform_at_stamp(
    tf_buffer,
    target_frame: str,
    source_frame: str,
    stamp,  # rclpy.time.Time
    timeout_s: float = 0.2,
    logger=None,
):
    """Lookup transform at a specific time (e.g. detection message stamp). Returns TransformStamped or None.
    On future-timestamp extrapolation failure, falls back to latest available transform and logs."""
    try:
        return tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            stamp,
            timeout=rclpy.duration.Duration(seconds=float(timeout_s)),
        )
    except TransformException:
        # Fallback: stamp may be in future; retry with latest (avoids extrapolation exception)
        try:
            t = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=float(timeout_s)),
            )
            if logger:
                logger.warn(
                    "TF lookup at stamp failed (likely future); using latest transform (tf_future_stamp_fallback)"
                )
            return t
        except TransformException:
            return None


def check_tf_validity(
    tf_buffer,
    map_frame: str,
    base_frame: str,
    stamp=None,
    timeout_s: float = 0.2,
    fallback_to_latest: bool = True,
) -> Tuple[bool, str]:
    """
    Central TF validity check for goal dispatch.
    Verifies transform map_frame -> base_frame is available.
    If stamp is provided (rclpy.time.Time), checks availability at that exact time;
    do not transform stale poses — reject goals if transform not available at stamp.
    When stamp fails (e.g. future timestamp) and fallback_to_latest is True, retries with latest.
    Returns (ok: bool, error_code: str). error_code is TF_VALID or TF_FALLBACK_LATEST on success.
    """
    use_stamp = stamp is not None
    if stamp is None:
        stamp = rclpy.time.Time()
    try:
        ok = tf_buffer.can_transform(map_frame, base_frame, stamp)
        if ok:
            return True, TF_VALID
        if use_stamp and fallback_to_latest:
            ok_latest = tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time())
            if ok_latest:
                return True, TF_FALLBACK_LATEST
        return False, TF_UNAVAILABLE_AT_STAMP if use_stamp else TF_UNAVAILABLE
    except Exception:
        if use_stamp and fallback_to_latest:
            try:
                ok_latest = tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time())
                if ok_latest:
                    return True, TF_FALLBACK_LATEST
            except Exception:
                pass
        return False, TF_UNAVAILABLE_AT_STAMP if use_stamp else TF_UNAVAILABLE


def get_current_pose(
    tf_buffer,
    world_frame: str,
    base_frame: str,
    timeout_s: float = 0.2,
    logger=None,
) -> Optional[PoseStamped]:
    """Get current robot pose in world_frame. Returns PoseStamped or None."""
    transform = lookup_transform(tf_buffer, world_frame, base_frame, timeout_s=timeout_s)
    if transform is None:
        if logger:
            logger.warn(f"TF lookup failed: {world_frame}->{base_frame}")
        return None
    pose = PoseStamped()
    pose.header.frame_id = world_frame
    pose.header.stamp = transform.header.stamp
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


def transform_pose(
    tf_buffer,
    pose: PoseStamped,
    target_frame: str,
    timeout_s: float = 0.2,
    logger=None,
) -> Optional[PoseStamped]:
    """Transform PoseStamped to target_frame. Returns transformed pose or None."""
    transform = lookup_transform(tf_buffer, target_frame, pose.header.frame_id, timeout_s=timeout_s)
    if transform is None:
        if logger:
            logger.warn(f"TF lookup failed: {target_frame}->{pose.header.frame_id}")
        return None
    try:
        return tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)
    except Exception as e:
        if logger:
            logger.warn(f"TF transform failed: {e}")
        return None


def monotonic_time_s() -> float:
    """Return monotonic time in seconds (stable for timeout comparisons)."""
    return time.monotonic()
