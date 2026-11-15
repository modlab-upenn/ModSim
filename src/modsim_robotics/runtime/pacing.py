"""Dependency-free wall-clock pacing helpers for runtime presentations."""

from __future__ import annotations

import math


def wall_clock_deadline_s(
    wall_started_s: float,
    simulated_elapsed_s: float,
    real_time_factor: float,
) -> float:
    """Return the absolute wall-clock deadline for a simulated-time sample.

    A factor of one preserves real-time playback, a factor greater than one
    speeds presentation up, and a factor between zero and one slows it down.
    The helper only maps clocks; it does not alter physics or controller time.
    """
    if not math.isfinite(wall_started_s):
        raise ValueError("wall_started_s must be finite")
    if not math.isfinite(simulated_elapsed_s) or simulated_elapsed_s < 0.0:
        raise ValueError("simulated_elapsed_s must be finite and nonnegative")
    if not math.isfinite(real_time_factor) or real_time_factor <= 0.0:
        raise ValueError("real_time_factor must be finite and positive")
    return wall_started_s + simulated_elapsed_s / real_time_factor


__all__ = ["wall_clock_deadline_s"]
