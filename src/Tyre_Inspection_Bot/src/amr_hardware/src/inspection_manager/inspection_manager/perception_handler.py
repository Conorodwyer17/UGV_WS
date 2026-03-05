import math
from typing import List, Optional

from gb_visual_detection_3d_msgs.msg import BoundingBox3d


def _is_valid_box(box: BoundingBox3d) -> bool:
    vals = [box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax, box.probability]
    if not all(math.isfinite(v) for v in vals):
        return False
    if not (box.xmin < box.xmax and box.ymin < box.ymax and box.zmin < box.zmax):
        return False
    return True


def parse_vehicle_labels(value) -> List[str]:
    if isinstance(value, str):
        labels = [label.strip() for label in value.split(",") if label.strip()]
    elif isinstance(value, list):
        labels = [str(label).strip() for label in value if str(label).strip()]
    else:
        labels = []
    return labels if labels else ["car", "truck"]


def log_bounding_box(logger, box: BoundingBox3d, label: str) -> None:
    """Log detailed bounding box information."""
    center_x = (box.xmin + box.xmax) / 2.0
    center_y = (box.ymin + box.ymax) / 2.0
    center_z = (box.zmin + box.zmax) / 2.0
    width = box.xmax - box.xmin
    height = box.ymax - box.ymin
    depth = box.zmax - box.zmin

    logger.info(
        f"[{label}] Bounding Box Details:\n"
        f"  Object: {box.object_name}\n"
        f"  Probability: {box.probability:.3f}\n"
        f"  Center: ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})\n"
        f"  Dimensions: W={width:.3f}, H={height:.3f}, D={depth:.3f}\n"
        f"  Bounds: X=[{box.xmin:.3f}, {box.xmax:.3f}], "
        f"Y=[{box.ymin:.3f}, {box.ymax:.3f}], "
        f"Z=[{box.zmin:.3f}, {box.zmax:.3f}]"
    )


def find_vehicle_box(
    boxes: List[BoundingBox3d],
    vehicle_labels: List[str],
    min_prob: float,
    target_position: Optional[tuple] = None,
) -> Optional[BoundingBox3d]:
    """Find a vehicle box matching labels and min probability.
    If target_position (x,y,z) is given, return the box whose center is closest to it.
    Otherwise return highest probability."""
    filtered = []
    labels_lower = [label.lower() for label in vehicle_labels]
    for box in boxes:
        if not _is_valid_box(box):
            continue
        if box.object_name.lower() in labels_lower and box.probability >= min_prob:
            filtered.append(box)

    if not filtered:
        return None

    if target_position is not None and len(target_position) >= 2:
        def dist_sq(b):
            cx = (b.xmin + b.xmax) / 2.0
            cy = (b.ymin + b.ymax) / 2.0
            return (cx - target_position[0]) ** 2 + (cy - target_position[1]) ** 2
        return min(filtered, key=dist_sq)
    return sorted(filtered, key=lambda b: b.probability, reverse=True)[0]


def tire_position_label(
    tire_center: tuple,
    vehicle_center: Optional[tuple],
    robot_pos: Optional[tuple],
) -> str:
    """Compute FL/FR/RL/RR from vehicle geometry (vehicle center, robot position, tire center in world frame)."""
    if not vehicle_center or not robot_pos:
        return ""
    vx, vy = vehicle_center[0], vehicle_center[1]
    rx, ry = robot_pos[0], robot_pos[1]
    dx = vx - rx
    dy = vy - ry
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.01:
        return ""
    # Vehicle "forward" = direction from vehicle to robot (we observe from this side)
    fwd_x = dx / dist
    fwd_y = dy / dist
    # Right = forward × z_up
    right_x = -fwd_y
    right_y = fwd_x
    tx, ty = tire_center[0], tire_center[1]
    along_fwd = (tx - vx) * fwd_x + (ty - vy) * fwd_y
    along_right = (tx - vx) * right_x + (ty - vy) * right_y
    thresh = 0.05
    if along_fwd > thresh and along_right < -thresh:
        return "FL"
    if along_fwd > thresh and along_right > thresh:
        return "FR"
    if along_fwd < -thresh and along_right < -thresh:
        return "RL"
    if along_fwd < -thresh and along_right > thresh:
        return "RR"
    return ""


def find_tire_for_inspection(
    boxes: List[BoundingBox3d],
    tire_label: str,
    min_prob: float,
    inspected_positions: List[tuple],
    tolerance: float,
    vehicle_pos: Optional[tuple],
    max_tire_distance_from_vehicle: float,
    robot_pos: Optional[tuple],
    target_position: Optional[tuple],
    logger,
) -> Optional[BoundingBox3d]:
    """Find a tire that matches label and filters, favoring closest to robot."""
    filtered = [
        b for b in boxes
        if _is_valid_box(b)
        and b.object_name.lower() == tire_label.lower()
        and b.probability >= min_prob
    ]

    if not filtered:
        return None

    if robot_pos is None:
        logger.warn("Cannot get robot pose for tire selection")

    un_inspected = []
    for box in filtered:
        tire_center = (
            (box.xmin + box.xmax) / 2.0,
            (box.ymin + box.ymax) / 2.0,
            (box.zmin + box.zmax) / 2.0
        )

        already_inspected = False
        for inspected_pos in inspected_positions:
            dist = math.sqrt(
                (tire_center[0] - inspected_pos[0])**2 +
                (tire_center[1] - inspected_pos[1])**2 +
                (tire_center[2] - inspected_pos[2])**2
            )
            if dist < tolerance:
                already_inspected = True
                break

        if already_inspected:
            continue

        if vehicle_pos:
            dist_to_vehicle = math.sqrt(
                (tire_center[0] - vehicle_pos[0])**2 +
                (tire_center[1] - vehicle_pos[1])**2 +
                (tire_center[2] - vehicle_pos[2])**2
            )
            if dist_to_vehicle > max_tire_distance_from_vehicle:
                logger.debug(
                    f"Tire at ({tire_center[0]:.2f}, {tire_center[1]:.2f}) is "
                    f"{dist_to_vehicle:.2f}m from vehicle (max: {max_tire_distance_from_vehicle}m) - skipping"
                )
                continue

        un_inspected.append(box)

    if not un_inspected:
        logger.debug(
            f"No un-inspected tires found. Total tires: {len(filtered)}, "
            f"Already inspected: {len(inspected_positions)}"
        )
        return None

    if target_position is not None and len(target_position) >= 2:
        def distance_to_target(box: BoundingBox3d) -> float:
            tire_center = (
                (box.xmin + box.xmax) / 2.0,
                (box.ymin + box.ymax) / 2.0,
                (box.zmin + box.zmax) / 2.0
            )
            return math.sqrt(
                (tire_center[0] - target_position[0])**2 +
                (tire_center[1] - target_position[1])**2
            )
        return min(un_inspected, key=lambda b: (distance_to_target(b), -b.probability))

    if robot_pos:
        def distance_to_robot(box: BoundingBox3d) -> float:
            tire_center = (
                (box.xmin + box.xmax) / 2.0,
                (box.ymin + box.ymax) / 2.0,
                (box.zmin + box.zmax) / 2.0
            )
            return math.sqrt(
                (tire_center[0] - robot_pos[0])**2 +
                (tire_center[1] - robot_pos[1])**2 +
                (tire_center[2] - robot_pos[2])**2
            )
        return min(un_inspected, key=lambda b: (distance_to_robot(b), -b.probability))

    return max(un_inspected, key=lambda b: b.probability)
