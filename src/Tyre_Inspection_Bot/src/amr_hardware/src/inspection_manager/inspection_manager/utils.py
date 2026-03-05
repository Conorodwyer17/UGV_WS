import math
from typing import Optional


def distance_point_to_aabb_2d(px: float, py: float, xmin: float, xmax: float, ymin: float, ymax: float) -> float:
    """Distance from point (px, py) to nearest edge/face of axis-aligned 2D box.
    Reference: jackal_3d_slam point cloud filtering; ACC QCar2 distance for braking."""
    nearest_x = max(xmin, min(xmax, px))
    nearest_y = max(ymin, min(ymax, py))
    return math.sqrt((px - nearest_x) ** 2 + (py - nearest_y) ** 2)


def should_trigger_photo(distance_m: Optional[float], threshold_m: float) -> bool:
    """Return True if photo should trigger based on distance threshold.
    When threshold > 0, we require being within threshold of the goal; if distance is unknown, do not trigger."""
    if threshold_m <= 0:
        return True  # disabled
    if distance_m is None:
        return False  # cannot validate; block capture to avoid taking photo from wrong position
    return distance_m <= threshold_m
