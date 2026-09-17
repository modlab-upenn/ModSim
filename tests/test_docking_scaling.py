"""Computational-cost scaling of the docking pass as module count grows.

The paper argues that ModSim's per-step docking overhead is near-linear in the
number of modules: a naive pass evaluates all ``O(n^2)`` connector pairs, while
the uniform spatial hash in ``broadphase.py`` (cell size equal to the acceptance
radius) culls non-adjacent pairs so that only ``O(n)`` geometrically adjacent
pairs reach the compatibility, acceptance, and guard checks. This module turns
that argument into evidence at two levels of strength:

* **Deterministic regression tests** (always run, no timing): the spatial hash
  returns exactly the same near pairs as a brute-force scan, and the number of
  candidate pairs the semantic checks actually evaluate grows linearly, not
  quadratically, with module count. These are timing-free so they are stable in
  CI.
* **A wall-clock benchmark** (``python tests/test_docking_scaling.py`` or with
  ``MODSIM_RUN_BENCHMARKS=1``): it times the spatial hash against a naive scan
  and the full detection pass across a size sweep, fits an empirical log-log
  slope, and writes a CSV for the paper's figure.

Everything runs on the kinematic mock backend, which is exactly what isolates
ModSim's own docking overhead from the backend's physics-integration cost.
"""

from __future__ import annotations

import csv
import functools
import math
import os
import time
from collections.abc import Callable
from pathlib import Path

import pytest

from modsim.connectors.broadphase import candidate_pairs
from modsim.core.ids import ConnectorInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.transforms import Vec3, vec_norm, vec_sub
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "generic_cube"
SPACING_M = 0.1

# A linear chain of N cubes has N-1 pairs of coincident facing connectors, so
# the number of candidate pairs is expected to be O(N) while the total number of
# free-connector pairs is O(N^2). The upper bound below is deliberately loose:
# the claim is linearity, not an exact count.
LINEAR_CANDIDATE_FACTOR = 3

_EXAMPLE_PACK = Path(__file__).resolve().parents[1] / "examples" / "robot_packs" / "generic_cube"


def _build_session(module_count: int) -> RuntimeSession:
    """Return a mock-backend session with ``module_count`` cubes in a row."""
    loaded = RobotPackLoader().load(_EXAMPLE_PACK)
    scene = SceneSpec.grid(MODULE_TYPE, module_count, spacing_m=SPACING_M)
    return RuntimeSession.create(loaded, scene, "mock")


def _free_connector_positions(
    session: RuntimeSession,
) -> list[tuple[ConnectorInstanceId, Vec3]]:
    """Return the world positions of every free connector, as detection sees them."""
    return [
        (connector.id, connector.world_pose.translation)
        for connector in session.world.connectors.values()
        if connector.resolved and not connector.is_engaged
    ]


def _naive_pairs(
    positions: list[tuple[ConnectorInstanceId, Vec3]],
    radius_m: float,
) -> set[tuple[ConnectorInstanceId, ConnectorInstanceId]]:
    """Return every within-radius pair by an explicit O(c^2) scan."""
    found: set[tuple[ConnectorInstanceId, ConnectorInstanceId]] = set()
    for i in range(len(positions)):
        key_i, point_i = positions[i]
        for j in range(i + 1, len(positions)):
            key_j, point_j = positions[j]
            if vec_norm(vec_sub(point_i, point_j)) <= radius_m:
                found.add((key_i, key_j) if key_i < key_j else (key_j, key_i))
    return found


# ----------------------------------------------------------------------
# deterministic regressions (no timing)
# ----------------------------------------------------------------------


@pytest.mark.parametrize("module_count", [2, 5, 10, 25])
def test_spatial_hash_returns_the_same_pairs_as_a_naive_scan(module_count: int) -> None:
    """The broad-phase optimisation must not drop or invent any candidate pair."""
    session = _build_session(module_count)
    radius = session.docking.detection_radius(session.world)
    positions = _free_connector_positions(session)

    hashed = set(candidate_pairs(positions, radius))
    naive = _naive_pairs(positions, radius)

    assert hashed == naive, "spatial hash disagrees with the brute-force scan"


@pytest.mark.parametrize("module_count", [4, 8, 16, 32, 64])
def test_candidate_pairs_scale_linearly_not_quadratically(module_count: int) -> None:
    """The pairs that reach the semantic checks grow linearly with module count."""
    session = _build_session(module_count)
    connector_count = len(_free_connector_positions(session))
    total_pairs = connector_count * (connector_count - 1) // 2
    candidate_count = len(session.proposals())

    # Linear in module count...
    assert candidate_count <= LINEAR_CANDIDATE_FACTOR * module_count, (
        f"{candidate_count} candidate pairs for {module_count} modules is super-linear"
    )
    # ...and a vanishing fraction of the quadratic all-pairs set as N grows.
    assert candidate_count * 4 < total_pairs, (
        f"{candidate_count} candidates is not clearly below the {total_pairs} all-pairs count"
    )


def test_candidate_fraction_shrinks_with_scale() -> None:
    """The culled fraction improves with size, the signature of O(n) vs O(n^2)."""
    small = _build_session(8)
    large = _build_session(64)

    def fraction(session: RuntimeSession) -> float:
        connector_count = len(_free_connector_positions(session))
        total_pairs = connector_count * (connector_count - 1) // 2
        return len(session.proposals()) / total_pairs

    assert fraction(large) < fraction(small), (
        "the candidate fraction should shrink as the world grows"
    )


# ----------------------------------------------------------------------
# wall-clock benchmark (opt-in; skipped in ordinary CI)
# ----------------------------------------------------------------------

DEFAULT_SIZES = (16, 32, 64, 128, 256, 512, 1024)
NAIVE_SIZE_CAP = 512
BENCHMARK_REPEATS = 5


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _time_call(call: Callable[[], object], repeats: int) -> float:
    """Return the median wall-clock seconds of ``call`` over ``repeats`` runs."""
    samples: list[float] = []
    for _ in range(repeats):
        start = time.perf_counter()
        call()
        samples.append(time.perf_counter() - start)
    return _median(samples)


def _loglog_slope(sizes: list[int], seconds: list[float]) -> float:
    """Return the least-squares slope of log(time) against log(size)."""
    xs = [math.log(size) for size in sizes]
    ys = [math.log(value) for value in seconds]
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    numerator = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys, strict=True))
    denominator = sum((x - mean_x) ** 2 for x in xs)
    return numerator / denominator


def measure_scaling(
    sizes: tuple[int, ...] = DEFAULT_SIZES,
    repeats: int = BENCHMARK_REPEATS,
) -> list[dict[str, float]]:
    """Measure hash, naive, and full-pass timings across a module-count sweep."""
    rows: list[dict[str, float]] = []
    for module_count in sizes:
        session = _build_session(module_count)
        radius = session.docking.detection_radius(session.world)
        positions = _free_connector_positions(session)

        hash_s = _time_call(functools.partial(candidate_pairs, positions, radius), repeats)
        pass_s = _time_call(session.proposals, repeats)
        naive_s = (
            _time_call(functools.partial(_naive_pairs, positions, radius), repeats)
            if module_count <= NAIVE_SIZE_CAP
            else math.nan
        )
        rows.append(
            {
                "modules": float(module_count),
                "connectors": float(len(positions)),
                "candidate_pairs": float(len(candidate_pairs(positions, radius))),
                "hash_ms": hash_s * 1e3,
                "naive_ms": naive_s * 1e3,
                "detect_pass_ms": pass_s * 1e3,
            }
        )
    return rows


@pytest.mark.skipif(
    not os.environ.get("MODSIM_RUN_BENCHMARKS"),
    reason="set MODSIM_RUN_BENCHMARKS=1 to run the timing benchmark",
)
def test_spatial_hash_scales_near_linearly() -> None:
    """Opt-in timing check: the hash is near-linear and beats the naive scan."""
    sizes = (32, 64, 128, 256, 512)
    rows = measure_scaling(sizes, repeats=BENCHMARK_REPEATS)
    hash_slope = _loglog_slope([int(r["modules"]) for r in rows], [r["hash_ms"] for r in rows])
    naive_slope = _loglog_slope([int(r["modules"]) for r in rows], [r["naive_ms"] for r in rows])

    assert hash_slope < 1.5, f"spatial hash scaled with slope {hash_slope:.2f} (expected ~1)"
    assert naive_slope > 1.6, f"naive scan scaled with slope {naive_slope:.2f} (expected ~2)"


def _write_csv(rows: list[dict[str, float]], path: Path) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:  # pragma: no cover - manual benchmark entry point
    """Run the full sweep, print a table, fit slopes, and write a CSV."""
    rows = measure_scaling()
    cols = ("modules", "conns", "cands", "hash_ms", "naive_ms", "pass_ms")
    header = f"{cols[0]:>8} {cols[1]:>7} {cols[2]:>7} {cols[3]:>10} {cols[4]:>10} {cols[5]:>10}"
    print(header)
    print("-" * len(header))
    for row in rows:
        naive = "n/a" if math.isnan(row["naive_ms"]) else f"{row['naive_ms']:.3f}"
        print(
            f"{int(row['modules']):>8} {int(row['connectors']):>7} "
            f"{int(row['candidate_pairs']):>7} {row['hash_ms']:>10.3f} "
            f"{naive:>10} {row['detect_pass_ms']:>10.3f}"
        )

    sizes = [int(row["modules"]) for row in rows]
    hash_slope = _loglog_slope(sizes, [r["hash_ms"] for r in rows])
    pass_slope = _loglog_slope(sizes, [r["detect_pass_ms"] for r in rows])
    print()
    print(f"hash        log-log slope: {hash_slope:.2f}")
    print(f"detect pass log-log slope: {pass_slope:.2f}")
    naive_rows = [r for r in rows if not math.isnan(r["naive_ms"])]
    if len(naive_rows) >= 2:
        naive_sizes = [int(r["modules"]) for r in naive_rows]
        naive_slope = _loglog_slope(naive_sizes, [r["naive_ms"] for r in naive_rows])
        print(f"naive scan  log-log slope: {naive_slope:.2f}")

    out_path = Path(os.environ.get("MODSIM_BENCHMARK_OUT", "docking_scaling_results.csv"))
    _write_csv(rows, out_path)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":  # pragma: no cover - manual benchmark entry point
    main()
