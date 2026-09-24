"""Wall-clock pacing remains separate from simulated physics time."""

from __future__ import annotations

import math

import pytest

from modsim.runtime.pacing import wall_clock_deadline_s


@pytest.mark.parametrize(
    ("real_time_factor", "expected_deadline_s"),
    ((1.0, 112.0), (4.0, 103.0), (0.5, 124.0)),
)
def test_wall_clock_deadline_scales_only_elapsed_presentation_time(
    real_time_factor: float,
    expected_deadline_s: float,
) -> None:
    assert wall_clock_deadline_s(100.0, 12.0, real_time_factor) == pytest.approx(
        expected_deadline_s
    )


@pytest.mark.parametrize(
    ("wall_started_s", "simulated_elapsed_s", "real_time_factor", "message"),
    (
        (math.nan, 1.0, 1.0, "wall_started_s must be finite"),
        (0.0, math.inf, 1.0, "simulated_elapsed_s must be finite and nonnegative"),
        (0.0, -1.0, 1.0, "simulated_elapsed_s must be finite and nonnegative"),
        (0.0, 1.0, math.nan, "real_time_factor must be finite and positive"),
        (0.0, 1.0, 0.0, "real_time_factor must be finite and positive"),
    ),
)
def test_wall_clock_deadline_rejects_unsafe_clock_inputs(
    wall_started_s: float,
    simulated_elapsed_s: float,
    real_time_factor: float,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        wall_clock_deadline_s(wall_started_s, simulated_elapsed_s, real_time_factor)
