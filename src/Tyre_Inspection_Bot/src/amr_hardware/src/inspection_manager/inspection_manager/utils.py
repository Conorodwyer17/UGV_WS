from typing import Optional


def should_trigger_photo(distance_m: Optional[float], threshold_m: float) -> bool:
    """Return True if photo should trigger based on distance threshold."""
    if threshold_m <= 0:
        return True  # disabled
    if distance_m is None:
        return True  # cannot validate; do not block capture
    return distance_m <= threshold_m
