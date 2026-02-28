#!/usr/bin/env python3
"""Tire candidate generation skeleton."""

from typing import Any, Dict, List


class TireEnumerator:
    """
    Heuristic tire enumerator for parked vehicles.

    Expected input keys (object_record):
    - object_id: str
    - center: {x, y, z}
    - yaw_rad: float (optional)
    - length_m: float (vehicle longitudinal size, optional)
    - width_m: float (vehicle lateral size, optional)
    - axle_count: int (optional, defaults from vehicle class)
    - vehicle_class: str (car|truck|bus)
    """

    @staticmethod
    def _default_axles(vehicle_class: str) -> int:
        if vehicle_class in ("truck", "bus"):
            return 3
        return 2

    @staticmethod
    def _rotation(yaw: float, x_local: float, y_local: float):
        import math

        c = math.cos(yaw)
        s = math.sin(yaw)
        return (x_local * c - y_local * s, x_local * s + y_local * c)

    def enumerate_tires(self, object_record: Dict[str, Any]) -> List[Dict[str, Any]]:
        center = object_record.get("center", {})
        cx = float(center.get("x", 0.0))
        cy = float(center.get("y", 0.0))
        cz = float(center.get("z", 0.0))
        yaw = float(object_record.get("yaw_rad", 0.0))
        vehicle_class = str(object_record.get("vehicle_class", "car"))

        length_m = float(object_record.get("length_m", 4.6))
        width_m = float(object_record.get("width_m", 1.9))
        axle_count = int(object_record.get("axle_count", self._default_axles(vehicle_class)))
        axle_count = max(2, axle_count)

        longitudinal_margin = 0.15 * length_m
        lateral_half = 0.5 * width_m
        usable = max(0.5, length_m - 2.0 * longitudinal_margin)
        step = usable / max(1, axle_count - 1)
        start_x = -0.5 * usable

        tires: List[Dict[str, Any]] = []
        for axle_idx in range(axle_count):
            x_local = start_x + axle_idx * step
            for side, y_local in (("left", lateral_half), ("right", -lateral_half)):
                dx, dy = self._rotation(yaw, x_local, y_local)
                tire_id = f"axle{axle_idx + 1}_{side}"
                tires.append(
                    {
                        "tire_id": tire_id,
                        "axle_index": axle_idx + 1,
                        "side": side,
                        "position": {"x": cx + dx, "y": cy + dy, "z": cz},
                    }
                )
        return tires

