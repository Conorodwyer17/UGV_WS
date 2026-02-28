#!/usr/bin/env python3
"""Live alignment convergence utility using perception-derived error."""

import time
from typing import Callable


class AlignmentController:
    def converge_until(self, error_fn: Callable[[], float], tolerance_cm: float, timeout_s: float) -> bool:
        """
        Aligns until live error from `error_fn` is below threshold.

        error_fn must return current lateral/pose error in meters.
        """
        start = time.time()
        stable_hits = 0
        tol_m = max(0.0, tolerance_cm) / 100.0
        while time.time() - start < timeout_s:
            err_m = float(error_fn())
            if err_m <= tol_m:
                stable_hits += 1
                if stable_hits >= 3:
                    return True
            else:
                stable_hits = 0
            time.sleep(0.08)
        return False
