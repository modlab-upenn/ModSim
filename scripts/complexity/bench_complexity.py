"""Per-stage cost of a ModSim runtime step, measured against world size.

Each experiment varies one quantity and holds the others fixed, so a fitted
log-log slope estimates the exponent of that quantity in the stage's cost:

* ``tick``      per-tick stages vs. module count N (M-Blocks pack, MuJoCo,
                no connections, no candidates): physics step, snapshot,
                ingest, overload check, passive docking pass, and reading every
                module pose once (a controller touching the whole world).
* ``tick_conn`` the same with docked rows, so the connection count E ~ N.
* ``detect``    docking detection vs. the number K of candidate pairs within
                the acceptance radius, at fixed N (mock backend).
* ``event_n``   one dock and one undock vs. N at fixed assembly size.
* ``event_a``   one dock and one undock vs. assembly size A at fixed N.

Events are timed on the mock backend (ModSim core only) and on MuJoCo (core
plus constraint activation), in both of MuJoCo's connection-change modes:
``mujoco`` (the default, a one-step delay) and ``mujoco_immediate`` (rows
rebuilt with ``mj_step1`` on the next step). Because the immediate mode moves
work into the following physics step, each event is also charged its
``*_next_step_extra``: the median time of the step right after the event minus
the median time of an ordinary step. Results go to ``OUT/measurements.csv``
and the fitted slopes to ``OUT/fits.json``.

    python scripts/complexity/bench_complexity.py --out results/complexity
    python scripts/complexity/bench_complexity.py --out results/complexity --quick
"""

from __future__ import annotations

import argparse
import csv
import gc
import json
import math
import platform
import statistics
import time
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any

import numpy as np

from modsim.core.ids import ModuleInstanceId, connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime.mblocks_lattice import TABLETOP_ROTATION
from modsim.runtime.session import RuntimeSession

ROOT = Path(__file__).resolve().parents[2]
MBLOCKS = ROOT / "examples/robot_packs/mblocks_3d"
CUBE = ROOT / "examples/robot_packs/generic_cube"
FAR_M = 0.5  # spacing that keeps every connector pair outside every acceptance radius


def _median_ms(
    fn: Callable[[], object], repeats: int, *, setup: Callable[[], object] | None = None
) -> float:
    samples = []
    for _ in range(repeats):
        if setup is not None:
            setup()
        start = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - start) * 1000)
    return statistics.median(samples)


def _grid(n: int, spacing: float) -> list[tuple[float, float]]:
    side = math.ceil(math.sqrt(n))
    return [((k % side) * spacing, (k // side) * spacing) for k in range(n)]


def _mblocks_scene(n: int) -> SceneSpec:
    return SceneSpec.of(
        ModulePlacement(
            instance_id=ModuleInstanceId(f"m{k:04d}"),
            module_type_id="mblocks_3d",
            pose=Transform(translation=(x, y, 0.025), rotation=TABLETOP_ROTATION),
        )
        for k, (x, y) in enumerate(_grid(n, FAR_M))
    )


def _cube_rows(rows: Iterable[tuple[int, float]], prefix: str = "c") -> SceneSpec:
    """Rows of generic cubes along x; ``spacing`` 0.1 puts neighbouring faces together."""
    placements: list[ModulePlacement] = []
    for r, (length, spacing) in enumerate(rows):
        for i in range(length):
            placements.append(
                ModulePlacement(
                    instance_id=ModuleInstanceId(f"{prefix}{r:04d}_{i:04d}"),
                    module_type_id="generic_cube",
                    pose=Transform.from_translation((i * spacing, r * FAR_M, 1.0)),
                )
            )
    return SceneSpec.of(placements)


def _dock_all_aligned(session: RuntimeSession) -> None:
    for proposal in session.proposals():
        if proposal.compatibility.compatible and proposal.acceptance.satisfied:
            session.request_dock(proposal.connector_a, proposal.connector_b)
    session.process_docking()


def _session(
    pack: LoadedRobotPack, scene: SceneSpec, backend: str, **options: Any
) -> RuntimeSession:
    if backend == "mujoco":
        options.setdefault("gravity", (0.0, 0.0, 0.0))
    return RuntimeSession.create(pack, scene, backend, **options)


# ----------------------------------------------------------------------
# experiments
# ----------------------------------------------------------------------


def tick_stages(session: RuntimeSession, dt: float, ticks: int) -> dict[str, float]:
    adapter, world = session.adapter, session.world
    for _ in range(20):
        session.step(dt, process_connectors=False)
    totals = {
        k: []
        for k in ("physics", "snapshot", "ingest", "overloads", "docking_pass", "read_all_poses")
    }
    for _ in range(ticks):
        t0 = time.perf_counter()
        adapter.step(dt)
        t1 = time.perf_counter()
        snapshot = adapter.snapshot()
        t2 = time.perf_counter()
        world.ingest(snapshot)
        t3 = time.perf_counter()
        session._evaluate_overloads(snapshot)
        t4 = time.perf_counter()
        session.process_docking()
        t5 = time.perf_counter()
        for module in world.modules.values():
            _ = module.pose
        t6 = time.perf_counter()
        for key, (a, b) in zip(
            totals, ((t0, t1), (t1, t2), (t2, t3), (t3, t4), (t4, t5), (t5, t6)), strict=True
        ):
            totals[key].append((b - a) * 1000)
    return {k: statistics.median(v) for k, v in totals.items()}


def exp_tick(sizes: list[int], ticks: int, rows: list[dict[str, Any]]) -> None:
    pack = RobotPackLoader().load(MBLOCKS)
    for n in sizes:
        session = _session(
            pack,
            _mblocks_scene(n),
            "mujoco",
            ground=True,
            gravity=(0.0, 0.0, -9.81),
            timestep_s=0.0005,
            weld_pool_size=max(12, 3 * n),
            hinge_pool_size=2,
        )
        try:
            world = session.world
            stats = tick_stages(session, 0.0005, ticks)
            for stage, ms in stats.items():
                rows.append(
                    dict(
                        experiment="tick",
                        backend="mujoco",
                        stage=stage,
                        x_name="N",
                        x=n,
                        N=n,
                        C=len(world.connectors),
                        E=len(world.connections),
                        ms=ms,
                    )
                )
        finally:
            session.shutdown()
        print(f"tick N={n}: " + ", ".join(f"{k} {v:.3f}" for k, v in stats.items()), flush=True)


def exp_tick_conn(sizes: list[int], ticks: int, rows: list[dict[str, Any]]) -> None:
    pack = RobotPackLoader().load(CUBE)
    for n in sizes:
        length = 8
        scene = _cube_rows([(length, 0.1)] * (n // length))
        session = _session(pack, scene, "mujoco", timestep_s=0.002, weld_pool_size=n)
        try:
            _dock_all_aligned(session)
            world = session.world
            stats = tick_stages(session, 0.002, ticks)
            for stage, ms in stats.items():
                rows.append(
                    dict(
                        experiment="tick_conn",
                        backend="mujoco",
                        stage=stage,
                        x_name="N",
                        x=n,
                        N=n,
                        C=len(world.connectors),
                        E=len(world.connections),
                        ms=ms,
                    )
                )
        finally:
            session.shutdown()
        print(
            f"tick_conn N={n} E={len(world.connections)}: "
            + ", ".join(f"{k} {v:.3f}" for k, v in stats.items()),
            flush=True,
        )


def exp_detect(n: int, pair_counts: list[int], repeats: int, rows: list[dict[str, Any]]) -> None:
    """K aligned-but-undocked pairs among n cubes; time one detection pass."""
    pack = RobotPackLoader().load(CUBE)
    for k in pair_counts:
        scene = _cube_rows([(2, 0.1)] * k + [(1, 0.1)] * (n - 2 * k))
        session = _session(pack, scene, "mock")
        try:
            found = len(session.proposals())
            ms = _median_ms(session.proposals, repeats)
            rows.append(
                dict(
                    experiment="detect",
                    backend="mock",
                    stage="detect",
                    x_name="K",
                    x=max(found, 0),
                    N=n,
                    C=len(session.world.connectors),
                    E=0,
                    ms=ms,
                )
            )
        finally:
            session.shutdown()
        print(f"detect N={n} K={found}: {ms:.3f} ms", flush=True)
    # and against N with no candidates
    for size in sorted({max(16, n // 8), max(32, n // 4), max(64, n // 2), n}):
        session = _session(pack, _cube_rows([(1, 0.1)] * size), "mock")
        try:
            ms = _median_ms(session.proposals, repeats)
            rows.append(
                dict(
                    experiment="detect_n",
                    backend="mock",
                    stage="detect",
                    x_name="N",
                    x=size,
                    N=size,
                    C=len(session.world.connectors),
                    E=0,
                    ms=ms,
                )
            )
        finally:
            session.shutdown()
        print(f"detect N={size} K=0: {ms:.3f} ms", flush=True)


EVENT_BACKENDS = ("mock", "mujoco", "mujoco_immediate")
EVENT_DT_S = 0.002


def _event_costs(
    pack: LoadedRobotPack, scene: SceneSpec, backend: str, joint: tuple[str, str], repeats: int
) -> dict[str, float]:
    """Dock everything aligned, then time undock and re-dock of the ``joint`` pair.

    Returns medians in ms for ``dock`` and ``undock`` (the calls themselves) and
    ``dock_next_step_extra`` / ``undock_next_step_extra`` (how much longer the
    physics step after the event took than an ordinary step).
    """
    options: dict[str, Any] = {}
    engine = backend
    if backend.startswith("mujoco"):
        engine = "mujoco"
        options = dict(
            timestep_s=EVENT_DT_S,
            weld_pool_size=len(scene.placements),
            immediate_connection_changes=backend == "mujoco_immediate",
        )
    session = _session(pack, scene, engine, **options)
    try:
        _dock_all_aligned(session)
        world, manager, adapter = session.world, session.docking, session.adapter
        a = connector_instance_id(ModuleInstanceId(joint[0]), "front")
        b = connector_instance_id(ModuleInstanceId(joint[1]), "rear")
        connection = world.connector(a).connection_id
        assert connection is not None, "benchmark joint did not dock"

        def timed_step() -> float:
            gc.disable()
            start = time.perf_counter()
            adapter.step(EVENT_DT_S)
            elapsed = time.perf_counter() - start
            gc.enable()
            return elapsed * 1000

        for _ in range(3):
            timed_step()  # warm up
        plain_ms = statistics.median(timed_step() for _ in range(max(10, repeats)))
        undock_ms, dock_ms, after_undock_ms, after_dock_ms = [], [], [], []
        for _ in range(repeats):
            gc.disable()
            t0 = time.perf_counter()
            manager.undock(world, adapter, connection)
            t1 = time.perf_counter()
            gc.enable()
            after_undock_ms.append(timed_step())
            for c in (a, b):
                world.connector(c).available_at_s = 0.0  # skip the redock cooldown
            proposal = manager._evaluate(world, *sorted((a, b)), requested=True)
            assert proposal is not None and proposal.viable, proposal
            gc.disable()
            t2 = time.perf_counter()
            manager.commit(world, adapter, proposal)
            t3 = time.perf_counter()
            gc.enable()
            after_dock_ms.append(timed_step())
            connection = world.connector(a).connection_id
            assert connection is not None
            undock_ms.append((t1 - t0) * 1000)
            dock_ms.append((t3 - t2) * 1000)
        return {
            "dock": statistics.median(dock_ms),
            "undock": statistics.median(undock_ms),
            "dock_next_step_extra": statistics.median(after_dock_ms) - plain_ms,
            "undock_next_step_extra": statistics.median(after_undock_ms) - plain_ms,
            "plain_step": plain_ms,
        }
    finally:
        session.shutdown()


def exp_events(
    sizes: list[int],
    assembly_sizes: list[int],
    fixed_n: int,
    fixed_a: int,
    repeats: int,
    rows: list[dict[str, Any]],
) -> None:
    pack = RobotPackLoader().load(CUBE)
    for backend in EVENT_BACKENDS:
        # vary N, assembly of fixed_a cubes, joint in its middle; the rest free
        for n in sizes:
            scene = _cube_rows([(fixed_a, 0.1)] + [(1, 0.1)] * (n - fixed_a))
            mid = fixed_a // 2
            costs = _event_costs(
                pack, scene, backend, (f"c0000_{mid - 1:04d}", f"c0000_{mid:04d}"), repeats
            )
            dock, undock = costs["dock"], costs["undock"]
            for stage, ms in costs.items():
                rows.append(
                    dict(
                        experiment="event_n",
                        backend=backend,
                        stage=stage,
                        x_name="N",
                        x=n,
                        N=n,
                        C=2 * n,
                        E=fixed_a - 1,
                        ms=ms,
                    )
                )
            print(
                f"event_n [{backend}] N={n} A={fixed_a}: dock {dock:.3f} undock {undock:.3f} "
                f"next-step extra {costs['dock_next_step_extra']:+.3f}/"
                f"{costs['undock_next_step_extra']:+.3f} (step {costs['plain_step']:.3f})",
                flush=True,
            )
        # vary assembly size, N fixed
        for a_size in assembly_sizes:
            scene = _cube_rows([(a_size, 0.1)] + [(1, 0.1)] * (fixed_n - a_size))
            mid = a_size // 2
            costs = _event_costs(
                pack, scene, backend, (f"c0000_{mid - 1:04d}", f"c0000_{mid:04d}"), repeats
            )
            dock, undock = costs["dock"], costs["undock"]
            for stage, ms in costs.items():
                rows.append(
                    dict(
                        experiment="event_a",
                        backend=backend,
                        stage=stage,
                        x_name="A",
                        x=a_size,
                        N=fixed_n,
                        C=2 * fixed_n,
                        E=a_size - 1,
                        ms=ms,
                    )
                )
            print(
                f"event_a [{backend}] N={fixed_n} A={a_size}: dock {dock:.3f} undock {undock:.3f} "
                f"next-step extra {costs['dock_next_step_extra']:+.3f}/"
                f"{costs['undock_next_step_extra']:+.3f} (step {costs['plain_step']:.3f})",
                flush=True,
            )


# ----------------------------------------------------------------------
# fitting
# ----------------------------------------------------------------------


def fit_slopes(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Least-squares log-log slope per (experiment, backend, stage), upper half of x."""
    groups: dict[tuple[str, str, str], list[tuple[float, float]]] = {}
    for r in rows:
        if r["x"] > 0 and r["ms"] > 0:
            groups.setdefault((r["experiment"], r["backend"], r["stage"]), []).append(
                (r["x"], r["ms"])
            )
    fits = []
    for (experiment, backend, stage), points in sorted(groups.items()):
        points.sort()
        tail = points[len(points) // 2 - 1 :] if len(points) >= 4 else points
        if len(tail) < 2:
            continue
        x = np.log([p[0] for p in tail])
        y = np.log([p[1] for p in tail])
        slope, _intercept = np.polyfit(x, y, 1)
        fits.append(
            dict(
                experiment=experiment,
                backend=backend,
                stage=stage,
                slope=round(float(slope), 3),
                x_range=[tail[0][0], tail[-1][0]],
                ms_range=[round(tail[0][1], 4), round(tail[-1][1], 4)],
            )
        )
    return fits


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--quick", action="store_true", help="Smaller sizes and fewer repeats")
    parser.add_argument(
        "--only", nargs="*", default=None, choices=["tick", "tick_conn", "detect", "events"]
    )
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    quick = args.quick
    rows: list[dict[str, Any]] = []
    run = set(args.only or ["tick", "tick_conn", "detect", "events"])
    if "tick" in run:
        exp_tick(
            [8, 16, 32, 64, 128] if quick else [8, 16, 32, 64, 128, 256, 512],
            50 if quick else 200,
            rows,
        )
    if "tick_conn" in run:
        exp_tick_conn(
            [16, 32, 64, 128] if quick else [16, 32, 64, 128, 256, 512, 1024],
            50 if quick else 200,
            rows,
        )
    if "detect" in run:
        n = 256 if quick else 1024
        exp_detect(n, [0, 4, 16, 64] if quick else [0, 8, 32, 128, 512], 10 if quick else 30, rows)
    if "events" in run:
        exp_events(
            [16, 64, 256] if quick else [16, 32, 64, 128, 256, 512, 1024],
            [4, 16, 64] if quick else [4, 8, 16, 32, 64, 128, 256],
            256 if quick else 1024,
            4,
            10 if quick else 30,
            rows,
        )
    import mujoco

    meta = dict(
        python=platform.python_version(),
        machine=platform.machine(),
        system=platform.system(),
        processor=platform.processor(),
        mujoco=mujoco.__version__,
        quick=quick,
    )
    with (args.out / "measurements.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["experiment", "backend", "stage", "x_name", "x", "N", "C", "E", "ms"],
        )
        writer.writeheader()
        writer.writerows(rows)
    fits = fit_slopes(rows)
    (args.out / "fits.json").write_text(json.dumps(dict(meta=meta, fits=fits), indent=2) + "\n")
    print("\nfitted log-log slopes (upper half of each sweep):")
    for f in fits:
        print(
            f"  {f['experiment']:10s} {f['backend']:7s} {f['stage']:15s} slope {f['slope']:+.2f}  "
            f"x {f['x_range']}  ms {f['ms_range']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
