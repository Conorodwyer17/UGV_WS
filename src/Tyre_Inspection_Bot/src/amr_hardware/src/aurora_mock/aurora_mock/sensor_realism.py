"""
Sensor realism utilities for Aurora mock simulation.

Provides configurable noise models, dropout, and optional latency to stress-test
perception and state estimation. All features can be disabled for deterministic testing.
"""
import random
from typing import Callable, Optional


def gaussian_noise(std: float, mean: float = 0.0) -> float:
    """Add Gaussian noise. Returns mean + N(0, std^2)."""
    if std <= 0:
        return mean
    return mean + random.gauss(0.0, std)


def apply_odom_noise(
    x: float, y: float, theta: float,
    vx: float, vtheta: float,
    position_std: float, velocity_std: float, drift_rate: float,
) -> tuple:
    """
    Apply realistic odometry noise: Gaussian on pose/velocity, optional drift.
    drift_rate: multiplicative drift per update (e.g. 0.001 = 0.1% per step).
    Returns (x, y, theta, vx, vtheta).
    """
    x_n = gaussian_noise(position_std, x)
    y_n = gaussian_noise(position_std, y)
    theta_n = gaussian_noise(position_std * 0.1, theta)  # rad
    vx_n = gaussian_noise(velocity_std, vx)
    vtheta_n = gaussian_noise(velocity_std * 0.1, vtheta)
    if drift_rate != 0:
        scale = 1.0 + drift_rate
        x_n *= scale
        y_n *= scale
    return (x_n, y_n, theta_n, vx_n, vtheta_n)


def apply_scan_noise(
    ranges: list, noise_std: float, dropout_prob: float, range_max: float = float("inf")
) -> list:
    """
    Apply LiDAR-like noise: Gaussian on range, optional dropout (replace with inf).
    """
    out = []
    for r in ranges:
        if r == float("inf") or r <= 0:
            out.append(r)
            continue
        if random.random() < dropout_prob:
            out.append(float("inf"))
        else:
            rn = gaussian_noise(noise_std, r)
            if rn < 0 or rn > range_max:
                rn = float("inf")
            out.append(rn)
    return out


def apply_depth_noise(depth_values: list, noise_std: float) -> list:
    """Apply Gaussian noise to depth values (meters). Only modifies valid depth (d > 0); invalid stay as-is."""
    out = []
    for d in depth_values:
        if d <= 0 or d != d:  # nan or invalid
            out.append(d)
        else:
            dn = gaussian_noise(noise_std, d)
            out.append(max(0.01, dn))
    return out


def should_dropout(prob: float) -> bool:
    """Return True if this message should be dropped (for dropout simulation)."""
    if prob <= 0:
        return False
    return random.random() < prob


def next_period_with_jitter(base_period: float, jitter_std: float) -> float:
    """
    Return next timer period with Gaussian jitter.
    jitter_std: fraction of base_period (e.g. 0.05 = 5% variation).
    """
    if jitter_std <= 0:
        return base_period
    jitter = random.gauss(0, jitter_std) * base_period
    return max(0.001, base_period + jitter)


class LatencyQueue:
    """
    Queue for delayed publishing. Stores (publish_fn, publish_at_ns).
    publish_fn: zero-arg callable that performs the publish.
    publish_at_ns: nanoseconds (monotonic or sim time) when to publish.
    """

    def __init__(self):
        self._entries: list = []

    def add(self, publish_fn: Callable[[], None], publish_at_ns: int):
        self._entries.append((publish_at_ns, publish_fn))

    def process_due(self, now_ns: int) -> int:
        """Call publish for all entries where now_ns >= publish_at_ns. Returns count published."""
        remaining = []
        published = 0
        for pub_at, fn in self._entries:
            if now_ns >= pub_at:
                try:
                    fn()
                    published += 1
                except Exception:
                    pass
            else:
                remaining.append((pub_at, fn))
        self._entries = remaining
        return published
