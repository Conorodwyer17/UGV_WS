#!/usr/bin/env python3
"""Continuity policy for tyre sweep direction and transition prefetch."""

import math
from typing import Dict, List, Optional


class ContinuityManager:
    def __init__(self) -> None:
        self.direction_lock: Optional[str] = None  # "cw" or "ccw"

    @staticmethod
    def _angle(cx: float, cy: float, tire: Dict) -> float:
        tx = float(tire["position"]["x"])
        ty = float(tire["position"]["y"])
        return math.atan2(ty - cy, tx - cx)

    def order_tires(self, tires: List[Dict], robot_pose: Optional[Dict] = None) -> List[Dict]:
        if not tires:
            return []
        cx = sum(float(t["position"]["x"]) for t in tires) / len(tires)
        cy = sum(float(t["position"]["y"]) for t in tires) / len(tires)
        with_angles = [{"angle": self._angle(cx, cy, t), **t} for t in tires]
        ccw = sorted(with_angles, key=lambda t: t["angle"])
        cw = list(reversed(ccw))
        if self.direction_lock is None:
            # Choose direction minimizing entry distance from robot to first tire.
            if robot_pose is None:
                self.direction_lock = "cw"
                ordered = cw
            else:
                rx = float(robot_pose["x"])
                ry = float(robot_pose["y"])
                d_ccw = math.hypot(float(ccw[0]["position"]["x"]) - rx, float(ccw[0]["position"]["y"]) - ry)
                d_cw = math.hypot(float(cw[0]["position"]["x"]) - rx, float(cw[0]["position"]["y"]) - ry)
                self.direction_lock = "cw" if d_cw <= d_ccw else "ccw"
                ordered = cw if self.direction_lock == "cw" else ccw
        else:
            ordered = cw if self.direction_lock == "cw" else ccw
        return ordered

    def flip_direction(self) -> None:
        if self.direction_lock == "cw":
            self.direction_lock = "ccw"
        elif self.direction_lock == "ccw":
            self.direction_lock = "cw"
