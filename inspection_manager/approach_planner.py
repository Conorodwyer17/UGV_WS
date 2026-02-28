#!/usr/bin/env python3
"""Approach planning adapter skeleton."""

import math
from typing import Dict, Any


class ApproachPlanner:
    def __init__(self, standoff_m: float = 0.6) -> None:
        self.standoff_m = standoff_m

    def compute_goal(self, tire_record: Dict[str, Any]) -> Dict[str, Any]:
        p = tire_record.get("position", {})
        tx = float(p.get("x", 0.0))
        ty = float(p.get("y", 0.0))
        side = str(tire_record.get("side", "left"))

        approach_sign = -1.0 if side == "left" else 1.0
        goal_x = tx
        goal_y = ty + approach_sign * self.standoff_m
        yaw = -math.pi / 2.0 if side == "left" else math.pi / 2.0
        return {
            "frame_id": "map",
            "x": goal_x,
            "y": goal_y,
            "yaw_rad": yaw,
            "standoff_m": self.standoff_m,
        }

