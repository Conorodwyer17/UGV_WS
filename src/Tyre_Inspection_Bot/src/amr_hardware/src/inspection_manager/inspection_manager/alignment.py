#!/usr/bin/env python3
"""Convergence-check helpers for tyre alignment.

AlignmentController.is_converged() is a pure utility used by tests and
any future fine-positioning logic that needs to know when the robot has
settled at a tyre inspection pose.

Note: VisualServoAlignServer was removed — the mission manager drives
fine positioning through centroid_servo_node instead.
"""

import math


class AlignmentController:
    @staticmethod
    def is_converged(position_rms_m: float, angular_deg: float, control_quality: float) -> bool:
        """Return True when robot is settled: position < 5 cm, heading < 3°, quality ≥ 0.9."""
        return position_rms_m <= 0.05 and angular_deg <= 3.0 and control_quality >= 0.9
