#!/usr/bin/env python3
"""Perception-driven world model for vehicles and tires."""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
import tf2_geometry_msgs  # noqa: F401 - registers geometry conversions
from tf2_ros import Buffer


@dataclass
class Track:
    track_id: str
    x: float
    y: float
    z: float
    probability: float
    updated_at: float
    freshness_s: float
    confidence: float = 0.0
    side: str = ""
    axle_index: int = 0


@dataclass
class VehicleModel:
    vehicle_id: str
    x: float
    y: float
    z: float
    probability: float
    updated_at: float
    tires: Dict[str, Track] = field(default_factory=dict)


class WorldModel:
    def __init__(self, node, tf_buffer: Buffer) -> None:
        self.node = node
        self.tf_buffer = tf_buffer
        self._vehicles: Dict[str, VehicleModel] = {}
        self._next_vehicle = 1
        self._next_tire = 1

    def _center(self, box) -> Tuple[float, float, float]:
        return (
            0.5 * (box.xmin + box.xmax),
            0.5 * (box.ymin + box.ymax),
            0.5 * (box.zmin + box.zmax),
        )

    def _to_map(self, msg: BoundingBoxes3d, xyz: Tuple[float, float, float]) -> Optional[Tuple[float, float, float]]:
        frame = msg.header.frame_id or "map"
        if frame == "map":
            return xyz
        p = PoseStamped()
        p.header = msg.header
        p.pose.position.x = xyz[0]
        p.pose.position.y = xyz[1]
        p.pose.position.z = xyz[2]
        p.pose.orientation.w = 1.0
        try:
            out = self.tf_buffer.transform(p, "map", timeout=Duration(seconds=0.1))
            return (out.pose.position.x, out.pose.position.y, out.pose.position.z)
        except Exception:
            return None

    @staticmethod
    def _dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy

    def _associate_vehicle(self, pos: Tuple[float, float, float], prob: float, now_s: float) -> str:
        best_id = ""
        best_d2 = 1e9
        for vid, v in self._vehicles.items():
            d2 = self._dist2((v.x, v.y), (pos[0], pos[1]))
            if d2 < best_d2:
                best_d2 = d2
                best_id = vid
        if best_id and best_d2 <= 2.0 * 2.0:
            v = self._vehicles[best_id]
            alpha = 0.4
            v.x = alpha * pos[0] + (1.0 - alpha) * v.x
            v.y = alpha * pos[1] + (1.0 - alpha) * v.y
            v.z = alpha * pos[2] + (1.0 - alpha) * v.z
            v.probability = max(v.probability, prob)
            v.updated_at = now_s
            return best_id
        vid = f"vehicle_{self._next_vehicle}"
        self._next_vehicle += 1
        self._vehicles[vid] = VehicleModel(
            vehicle_id=vid,
            x=pos[0],
            y=pos[1],
            z=pos[2],
            probability=prob,
            updated_at=now_s,
        )
        return vid

    def update_vehicles(self, msg: BoundingBoxes3d, labels: List[str], min_prob: float) -> None:
        labels_lower = {x.lower() for x in labels}
        now_s = time.time()
        for box in msg.bounding_boxes:
            if box.object_name.lower() not in labels_lower or box.probability < min_prob:
                continue
            p = self._center(box)
            p_map = self._to_map(msg, p)
            if p_map is None:
                continue
            self._associate_vehicle(p_map, float(box.probability), now_s)

    def update_tires(self, msg: BoundingBoxes3d, tire_label: str, min_prob: float, max_vehicle_dist_m: float = 6.0) -> None:
        now_s = time.time()
        for box in msg.bounding_boxes:
            if box.object_name.lower() != tire_label.lower() or box.probability < min_prob:
                continue
            p = self._center(box)
            p_map = self._to_map(msg, p)
            if p_map is None:
                continue
            if not self._vehicles:
                continue
            nearest_vid = min(self._vehicles.keys(), key=lambda vid: self._dist2((self._vehicles[vid].x, self._vehicles[vid].y), (p_map[0], p_map[1])))
            v = self._vehicles[nearest_vid]
            if math.sqrt(self._dist2((v.x, v.y), (p_map[0], p_map[1]))) > max_vehicle_dist_m:
                continue

            # Associate tire track under this vehicle.
            best_tid = ""
            best_d2 = 1e9
            for tid, t in v.tires.items():
                d2 = self._dist2((t.x, t.y), (p_map[0], p_map[1]))
                if d2 < best_d2:
                    best_d2 = d2
                    best_tid = tid
            if best_tid and best_d2 <= 0.8 * 0.8:
                t = v.tires[best_tid]
                alpha = 0.5
                t.x = alpha * p_map[0] + (1.0 - alpha) * t.x
                t.y = alpha * p_map[1] + (1.0 - alpha) * t.y
                t.z = alpha * p_map[2] + (1.0 - alpha) * t.z
                t.probability = max(t.probability, float(box.probability))
                t.updated_at = now_s
                t.freshness_s = 0.0
                t.confidence = min(1.0, t.confidence + 0.15)
            else:
                tid = f"tire_{self._next_tire}"
                self._next_tire += 1
                v.tires[tid] = Track(
                    track_id=tid,
                    x=p_map[0],
                    y=p_map[1],
                    z=p_map[2],
                    probability=float(box.probability),
                    updated_at=now_s,
                    freshness_s=0.0,
                    confidence=0.3,
                )

    def _annotate_tire_layout(self, vehicle: VehicleModel) -> List[Track]:
        tires = list(vehicle.tires.values())
        if not tires:
            return []
        # Determine a lateral axis from centroid spread to label left/right deterministically.
        cx = sum(t.x for t in tires) / len(tires)
        cy = sum(t.y for t in tires) / len(tires)
        angles = [(math.atan2(t.y - cy, t.x - cx), t) for t in tires]
        angles.sort(key=lambda x: x[0])
        ordered = [t for _, t in angles]
        # clockwise deterministic sweep
        for idx, t in enumerate(ordered):
            t.axle_index = idx // 2 + 1
            t.side = "left" if idx % 2 == 0 else "right"
        return ordered

    def resolve_vehicle_id(self, requested_id: str = "") -> Optional[str]:
        if requested_id and requested_id in self._vehicles:
            return requested_id
        if not self._vehicles:
            return None
        return max(self._vehicles.keys(), key=lambda vid: self._vehicles[vid].probability)

    def get_vehicle(self, vehicle_id: str) -> Optional[VehicleModel]:
        return self._vehicles.get(vehicle_id)

    def get_tires_for_vehicle(self, vehicle_id: str, max_stale_s: float = 1.0) -> List[Dict]:
        v = self._vehicles.get(vehicle_id)
        if v is None:
            return []
        now_s = time.time()
        for t in v.tires.values():
            t.freshness_s = max(0.0, now_s - t.updated_at)
        valid = [t for t in self._annotate_tire_layout(v) if t.freshness_s <= max_stale_s and t.confidence >= 0.3]
        out: List[Dict] = []
        for t in valid:
            out.append(
                {
                    "tire_id": t.track_id,
                    "axle_index": t.axle_index,
                    "side": t.side,
                    "confidence": t.confidence,
                    "freshness_s": t.freshness_s,
                    "position": {"x": t.x, "y": t.y, "z": t.z},
                }
            )
        return out

    def get_tire(self, vehicle_id: str, tire_id: str) -> Optional[Dict]:
        v = self._vehicles.get(vehicle_id)
        if v is None or tire_id not in v.tires:
            return None
        t = v.tires[tire_id]
        return {
            "tire_id": t.track_id,
            "side": t.side,
            "confidence": t.confidence,
            "freshness_s": max(0.0, time.time() - t.updated_at),
            "position": {"x": t.x, "y": t.y, "z": t.z},
        }
