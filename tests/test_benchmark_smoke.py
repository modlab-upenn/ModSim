"""Smoke tests: each benchmark script runs a few ticks against the current library.

The scripts reach into runtime internals (for example the overload check) to
time individual stages, so a library change can break them without breaking any
other test. These runs are kept tiny; they check that the scripts still work,
not how fast they are.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _load(relative: str, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Dataclasses resolve their module through sys.modules while executing.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_the_mblocks_parallel_scaling_benchmark_runs_a_few_ticks() -> None:
    pytest.importorskip("mujoco")
    scaling = _load("examples/scenarios/mblocks_parallel_scaling.py", "mblocks_parallel_scaling")

    result: Any = scaling.run(
        1,
        duration_s=0.005,
        dt_s=0.0005,
        spacing_x_m=0.40,
        spacing_y_m=0.70,
        progress_every_s=0.0,
    )

    assert result.ticks >= 10
    assert result.failed == 0
    assert result.stop_reason == "duration reached"


def test_the_complexity_benchmark_stages_run_a_few_ticks() -> None:
    pytest.importorskip("numpy")
    pytest.importorskip("mujoco")
    bench = _load("scripts/complexity/bench_complexity.py", "bench_complexity")
    rows: list[dict[str, Any]] = []

    bench.exp_tick([2], 3, rows)
    bench.exp_detect(4, [1], 1, rows)

    stages = {row["stage"] for row in rows}
    assert {"physics", "overloads", "docking_pass", "detect"} <= stages
    assert all(row["ms"] >= 0.0 for row in rows)
