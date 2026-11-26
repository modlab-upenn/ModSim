"""Scalability benchmark: X independent six-cube M-Blocks demos in ONE runtime session.

Every copy is the ``mblocks_online_lattice_large`` demo (2-by-3 rectangle to a
six-cube line, 21 generated pivots). All copies share a single RuntimeSession,
one MuJoCo model, one WorldState, one DockingManager and one physics clock. Each
copy keeps its own planner and feedback executor, which only sees its own six
modules through a thin scoped view of the shared session. Physics is stepped
exactly once per tick for the whole world.

Examples::

    # one run, 8 copies (48 modules), print a JSON summary
    python examples/scenarios/mblocks_parallel_scaling.py --copies 8

    # sweep, one fresh process per point, append rows to a CSV
    python examples/scenarios/mblocks_parallel_scaling.py \
        --sweep 1,2,4,8,16,32 --csv scaling.csv

    # control condition: same X copies, but one separate session per copy
    python examples/scenarios/mblocks_parallel_scaling.py \
        --sweep 1,2,4,8,16,32 --isolated --csv scaling.csv

    # watch one run in the MuJoCo viewer (macOS: use mjpython instead of python)
    mjpython examples/scenarios/mblocks_parallel_scaling.py --copies 8 --view --speed 1

    # quick throughput-only sweep: fixed 5 s of simulated time per point
    python examples/scenarios/mblocks_parallel_scaling.py \
        --sweep 1,2,4,8,16,32,64 --duration 5 --csv throughput.csv

No library code is modified: the scoped view only filters ``world.modules`` and
``world.connections`` to one copy, turns that copy's ``session.step`` into a
read of the shared tick, and narrows "clear all joint commands" to that copy's
own actuators so one copy failing cannot zero another copy's flywheels.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import json
import math
import os
import resource
import statistics
import subprocess
import sys
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modsim.connectors.docking import DockingManager
from modsim.core.events import Event
from modsim.core.ids import JointInstanceId, ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.planning.mblocks import LatticeBlock, LatticeState
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.mblocks_lattice import block_pose, larger_state, line_goal
from modsim.runtime.mblocks_online_planning import OnlineLatticeScenario
from modsim.runtime.session import RuntimeSession

PACK = Path(__file__).parents[1] / "robot_packs/mblocks_3d"
PITCH_M = 0.05
HEIGHT_M = 0.025
MODULES_PER_COPY = 6


# ----------------------------------------------------------------------
# Scoped view of one shared session
# ----------------------------------------------------------------------


class ScopedWorld:
    """WorldState view restricted to one copy's modules and connections."""

    __slots__ = ("_conn_cache", "_conn_key", "_connector_ids", "_ids", "_idset", "_world")

    def __init__(self, world: Any, ids: tuple[ModuleInstanceId, ...]) -> None:
        # Keep scene order: the planner compares LatticeState tuples by order.
        self._world = world
        self._ids = ids
        self._idset = frozenset(ids)
        self._connector_ids = tuple(
            cid for cid, c in world.connectors.items() if c.module_id in self._idset
        )
        self._conn_key: object = None
        self._conn_cache: dict[Any, Any] = {}

    @property
    def connectors(self) -> Mapping[Any, Any]:
        all_connectors = self._world.connectors
        return {cid: all_connectors[cid] for cid in self._connector_ids}

    @property
    def modules(self) -> Mapping[ModuleInstanceId, Any]:
        all_modules = self._world.modules
        return {i: all_modules[i] for i in self._ids}

    @property
    def connections(self) -> Mapping[Any, Any]:
        # Connections change only through DockCommitted/UndockCommitted, which bump
        # topology_revision. Rebuilding only then keeps the per-tick cost
        # O(own connections) instead of O(all connections in the world).
        world = self._world
        key = world.revision.topology_revision
        if key != self._conn_key:
            self._conn_cache = {
                cid: c
                for cid, c in world.connections.items()
                if world.connector(c.connector_a).module_id in self._idset
            }
            self._conn_key = key
        return dict(self._conn_cache)

    def __getattr__(self, name: str) -> Any:  # time_s, revision, assemblies, ...
        return getattr(self._world, name)


class ScopedSession:
    """RuntimeSession facade for one copy. Never advances physics itself.

    Each copy owns a DockingManager that only sees its own connectors. The
    shared manager's passive broad-phase would otherwise let one copy's
    ``process_docking`` capture faces in another copy, bypassing that copy's
    measured capture gate.
    """

    def __init__(self, shared: RuntimeSession, ids: Iterable[str], joints: Iterable[str]) -> None:
        self._shared = shared
        self._docking = DockingManager()
        self.world = ScopedWorld(shared.world, tuple(ModuleInstanceId(i) for i in ids))
        self._joints = tuple(JointInstanceId(j) for j in joints)
        self.tick_events: tuple[Event, ...] = ()
        self.step_calls_this_tick = 0

    @property
    def adapter(self) -> Any:
        return self._shared.adapter

    def step(self, dt_s: float, *, process_connectors: bool = True) -> tuple[Event, ...]:
        # The driver already advanced the shared world by exactly one tick.
        self.step_calls_this_tick += 1
        if self.step_calls_this_tick > 1:
            raise RuntimeError("a scenario tried to advance physics twice in one tick")
        events = self.tick_events
        if process_connectors:
            events = events + self.process_docking()
        return events

    def request_dock(self, connector_a: Any, connector_b: Any) -> None:
        self._docking.request_dock(connector_a, connector_b)

    def request_undock(self, connection: Any) -> None:
        self._docking.request_undock(connection)

    def process_docking(self) -> tuple[Event, ...]:
        # Record the changes on the shared session, whose step counter decides
        # connection_change_pending (reached through __getattr__ below).
        return self._shared.record_connection_changes(
            self._docking.run(self.world, self._shared.adapter)  # type: ignore[arg-type]
        )

    def proposals(self) -> tuple[Any, ...]:
        # Detect only among this copy's connectors. Falling through to the shared
        # session scanned every copy's connectors on each capture check, which
        # made landing capture cost O(X^2) per tick.
        return self._docking.detect(self.world)  # type: ignore[arg-type]

    def clear_joint_commands(self, joints: Iterable[JointInstanceId] | None = None) -> None:
        self._shared.clear_joint_commands(self._joints if joints is None else joints)

    def __getattr__(self, name: str) -> Any:  # set_joint_commands, metrics, ...
        # Anything that inspects the world must be overridden above, or it sees all copies.
        return getattr(self._shared, name)


# ----------------------------------------------------------------------
# Scene construction
# ----------------------------------------------------------------------


def copy_state(index: int) -> LatticeState:
    base = larger_state()
    return LatticeState(
        blocks=tuple(
            LatticeBlock(id=f"d{index:03d}_{b.id}", cell=b.cell, quarter_turns=b.quarter_turns)
            for b in base.blocks
        )
    )


def copy_origins(copies: int, spacing_x_m: float, spacing_y_m: float) -> list[tuple[float, float]]:
    columns = math.ceil(math.sqrt(copies))
    return [((k % columns) * spacing_x_m, (k // columns) * spacing_y_m) for k in range(copies)]


def build_scene(states: list[LatticeState], origins: list[tuple[float, float]]) -> SceneSpec:
    placements: list[ModulePlacement] = []
    for state, (x, y) in zip(states, origins, strict=True):
        frame = Transform.from_translation((x, y, HEIGHT_M))
        placements.extend(
            ModulePlacement(
                instance_id=ModuleInstanceId(b.id),
                module_type_id="mblocks_3d",
                pose=block_pose(b, frame, PITCH_M),
            )
            for b in state.blocks
        )
    return SceneSpec.of(placements)


# ----------------------------------------------------------------------
# One measured run
# ----------------------------------------------------------------------


@dataclass
class CopyResult:
    index: int
    origin_m: tuple[float, float] = (0.0, 0.0)
    phase: str = "running"
    completed_at_s: float | None = None
    verified_pivots: int = 0
    replans: int = 0
    detail: str = ""


@dataclass
class RunResult:
    copies: int
    modules: int
    mujoco_version: str
    mode: str = "shared"
    nbody: int = 0
    ngeom: int = 0
    neq: int = 0
    nu: int = 0
    setup_wall_s: float = 0.0
    sim_time_s: float = 0.0
    ticks: int = 0
    wall_s: float = 0.0
    physics_wall_s: float = 0.0
    sync_wall_s: float = 0.0
    controller_wall_s: float = 0.0
    real_time_factor: float = 0.0
    module_steps_per_s: float = 0.0
    tick_ms_mean: float = 0.0
    tick_ms_p95: float = 0.0
    peak_rss_mb: float = 0.0
    completed: int = 0
    failed: int = 0
    pivots_verified_total: int = 0
    viewer_wall_s: float = 0.0
    unfinished: int = 0
    completion_s_mean: float | None = None
    completion_s_max: float | None = None
    stop_reason: str = ""
    per_copy: list[CopyResult] = field(default_factory=list)


def _peak_rss_mb() -> float:
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return peak / (1024 * 1024) if sys.platform == "darwin" else peak / 1024


def _make_session(
    loaded: Any, states: list[LatticeState], origins: list[tuple[float, float]], dt_s: float
) -> RuntimeSession:
    n = len(states)
    return RuntimeSession.create(
        loaded,
        build_scene(states, origins),
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        timestep_s=dt_s,
        # Same per-copy pool budget as the single demo (max(12, 6 * 3) welds, 2 hinges).
        weld_pool_size=n * max(12, MODULES_PER_COPY * 3),
        hinge_pool_size=n * 2,
        constraint_time_constant_s=0.002,
    )


def run(
    copies: int,
    *,
    duration_s: float,
    dt_s: float,
    spacing_x_m: float,
    spacing_y_m: float,
    progress_every_s: float,
    isolated: bool = False,
    use_viewer: bool = False,
    speed: float = 0.0,
    view_fps: float = 30.0,
    hold: bool = True,
) -> RunResult:
    """Run X copies. ``isolated`` is the control: X separate sessions, same placements.

    ``use_viewer`` opens the MuJoCo passive viewer on the shared model (shared mode
    only). The viewer only reads ``MjData``; it is synced at ``view_fps`` of
    wall-clock time and, when ``speed`` > 0, the loop sleeps so simulated time
    advances at ``speed`` times real time. Rendering and pacing make the timing
    fields meaningless as a benchmark, so such runs are labelled ``shared-viewer``.
    """
    import mujoco

    if use_viewer and isolated:
        raise ValueError("--view needs the shared session; it cannot show isolated sessions")

    result = RunResult(
        copies=copies,
        modules=copies * MODULES_PER_COPY,
        mujoco_version=mujoco.__version__,
        mode="isolated" if isolated else ("shared-viewer" if use_viewer else "shared"),
    )
    t_setup = time.perf_counter()
    states = [copy_state(k) for k in range(copies)]
    origins = copy_origins(copies, spacing_x_m, spacing_y_m)
    loaded = RobotPackLoader().load(PACK)
    if isolated:
        sessions = [
            _make_session(loaded, [st], [o], dt_s) for st, o in zip(states, origins, strict=True)
        ]
        owner = list(range(copies))
    else:
        sessions = [_make_session(loaded, states, origins, dt_s)]
        owner = [0] * copies
    viewer_stack = contextlib.ExitStack()
    try:
        for session in sessions:
            m = getattr(session.adapter, "_compiled", None)
            if m is not None:
                m = m.model
                result.nbody += m.nbody
                result.ngeom += m.ngeom
                result.neq += m.neq
                result.nu += m.nu

        scoped: list[ScopedSession] = []
        scenarios: list[OnlineLatticeScenario] = []
        for k, state in enumerate(states):
            view = ScopedSession(
                sessions[owner[k]],
                (b.id for b in state.blocks),
                (f"{b.id}/flywheel_spin" for b in state.blocks),
            )
            scenarios.append(
                OnlineLatticeScenario.create(view, line_goal(state), initial=state, dt_s=dt_s)  # type: ignore[arg-type]
            )
            scoped.append(view)
        result.setup_wall_s = time.perf_counter() - t_setup
        viewer = None
        if use_viewer:
            viewer = viewer_stack.enter_context(_open_viewer(sessions[0].adapter))
        next_frame = 0.0
        stop_reason = ""

        copy_results = [CopyResult(index=k, origin_m=origins[k]) for k in range(copies)]
        tick_ms: list[float] = []
        clock = sessions[0].world
        next_progress = progress_every_s
        active = copies
        t_run = time.perf_counter()
        while clock.time_s < duration_s:
            t0 = time.perf_counter()
            for session in sessions:
                session.adapter.step(dt_s)  # MuJoCo integration only
            t1 = time.perf_counter()
            tick_events = []
            for session in sessions:  # ModSim state sync (snapshot -> WorldState)
                snapshot = session.adapter.snapshot()
                session.world.ingest(snapshot)
                tick_events.append(session._evaluate_overloads(snapshot))
            t2 = time.perf_counter()
            active = 0
            for k, (view, scenario) in enumerate(zip(scoped, scenarios, strict=True)):
                record = copy_results[k]
                if record.phase in {"complete", "failed"}:
                    continue
                view.tick_events = tick_events[owner[k]]
                view.step_calls_this_tick = 0
                scenario.step()
                phase = scenario.status.phase.value
                if phase in {"complete", "failed"}:
                    record.phase = phase
                    record.detail = scenario.status.detail
                    if phase == "complete":
                        record.completed_at_s = clock.time_s
                else:
                    active += 1
            t3 = time.perf_counter()
            result.physics_wall_s += t1 - t0
            result.sync_wall_s += t2 - t1
            result.controller_wall_s += t3 - t2
            tick_ms.append((t3 - t0) * 1000)
            result.ticks += 1
            if progress_every_s > 0 and clock.time_s >= next_progress:
                done = sum(r.phase == "complete" for r in copy_results)
                bad = sum(r.phase == "failed" for r in copy_results)
                rtf = clock.time_s / (time.perf_counter() - t_run)
                print(
                    f"[X={copies:3d} {result.mode}] sim {clock.time_s:7.2f}s  RTF {rtf:6.3f}  "
                    f"complete {done}/{copies}  failed {bad}",
                    file=sys.stderr,
                    flush=True,
                )
                next_progress += progress_every_s
            if viewer is not None:
                tv = time.perf_counter()
                if not viewer.is_running():
                    stop_reason = "viewer closed"
                    break
                if speed > 0:  # hold simulated time to `speed` x real time
                    ahead = (t_run + clock.time_s / speed) - time.perf_counter()
                    if ahead > 0:
                        time.sleep(ahead)
                now = time.perf_counter()
                if now >= next_frame:
                    viewer.sync()
                    _overlay(viewer, result.mode, copies, clock.time_s, copy_results, t_run)
                    next_frame = now + 1.0 / view_fps
                result.viewer_wall_s += time.perf_counter() - tv
            if active == 0:
                break
        result.wall_s = time.perf_counter() - t_run
        result.sim_time_s = clock.time_s
        result.stop_reason = stop_reason or (
            "all copies finished" if active == 0 else "duration reached"
        )

        for k, scenario in enumerate(scenarios):
            snap = scenario.planning_snapshot
            copy_results[k].verified_pivots = snap.completed_actions
            copy_results[k].replans = snap.replans
            if copy_results[k].phase == "running":
                copy_results[k].detail = scenario.status.detail
        result.per_copy = copy_results
        result.completed = sum(r.phase == "complete" for r in copy_results)
        result.failed = sum(r.phase == "failed" for r in copy_results)
        result.unfinished = copies - result.completed - result.failed
        result.pivots_verified_total = sum(r.verified_pivots for r in copy_results)
        done_times = [r.completed_at_s for r in copy_results if r.completed_at_s is not None]
        if done_times:
            result.completion_s_mean = statistics.fmean(done_times)
            result.completion_s_max = max(done_times)
        if result.wall_s:
            result.real_time_factor = result.sim_time_s / result.wall_s
            result.module_steps_per_s = result.modules * result.ticks / result.wall_s
        result.tick_ms_mean = statistics.fmean(tick_ms) if tick_ms else 0.0
        result.tick_ms_p95 = statistics.quantiles(tick_ms, n=20)[-1] if len(tick_ms) >= 20 else 0.0
        result.peak_rss_mb = _peak_rss_mb()
        if viewer is not None and hold and viewer.is_running():
            viewer.sync()
            _overlay(viewer, result.mode, copies, clock.time_s, copy_results, t_run, done=True)
            print("Viewer: final state shown; close the window to exit.", file=sys.stderr)
            while viewer.is_running():
                time.sleep(0.05)
        return result
    finally:
        viewer_stack.close()  # join the viewer before the backend is torn down
        for session in sessions:
            session.shutdown()


def _open_viewer(adapter: Any) -> contextlib.AbstractContextManager[Any]:
    # Reuse the backend's managed passive viewer: it joins the render thread on
    # exit and explains the mjpython requirement on macOS.
    from modsim_backend_mujoco.viewer import (
        _managed_passive_viewer,  # pyright: ignore[reportPrivateUsage]
    )

    return _managed_passive_viewer(adapter, None)


def _overlay(
    viewer: Any,
    mode: str,
    copies: int,
    sim_time_s: float,
    records: list[CopyResult],
    t_run: float,
    *,
    done: bool = False,
) -> None:
    set_texts = getattr(viewer, "set_texts", None)  # absent in older MuJoCo releases
    if not callable(set_texts):
        return
    import mujoco

    complete = sum(r.phase == "complete" for r in records)
    failed = sum(r.phase == "failed" for r in records)
    rtf = sim_time_s / max(time.perf_counter() - t_run, 1e-9)
    status = "FINISHED - close the window to exit" if done else f"RTF {rtf:.3f}"
    set_texts(
        (
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            f"M-Blocks x{copies} ({copies * MODULES_PER_COPY} modules)",
            f"sim {sim_time_s:6.2f} s   complete {complete}/{copies}   failed {failed}\n{status}",
        )
    )


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------

CSV_FIELDS = [
    "mode", "copies", "modules", "mujoco_version", "nbody", "ngeom", "neq", "nu",
    "setup_wall_s", "sim_time_s", "ticks", "wall_s", "physics_wall_s", "sync_wall_s",
    "controller_wall_s", "real_time_factor", "module_steps_per_s", "tick_ms_mean",
    "tick_ms_p95", "peak_rss_mb", "completed", "failed", "unfinished", "pivots_verified_total",
    "completion_s_mean", "completion_s_max", "stop_reason",
]  # fmt: skip


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--copies", type=int, help="Number X of six-cube demos in one session")
    group.add_argument("--sweep", type=str, help="Comma list of X values, each in a fresh process")
    parser.add_argument(
        "--duration",
        type=float,
        default=180.0,
        help="Simulated-time budget (s); 180 s matches the single demo",
    )
    parser.add_argument("--dt", type=float, default=0.0005)
    parser.add_argument("--spacing-x", type=float, default=0.40, help="Copy-to-copy spacing (m)")
    parser.add_argument(
        "--spacing-y",
        type=float,
        default=0.70,
        help="Row spacing (m); the line grows ~0.35 m in +y",
    )
    parser.add_argument(
        "--progress",
        type=float,
        default=5.0,
        help="Progress print interval in sim seconds (0 = off)",
    )
    parser.add_argument(
        "--isolated",
        action="store_true",
        help="Control condition: one separate session per copy (same placements, same process)",
    )
    parser.add_argument(
        "--view",
        action="store_true",
        help="Watch the shared session in the MuJoCo viewer (macOS: run with mjpython)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.0,
        help="With --view: simulated seconds per real second (0 = as fast as possible)",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="With --view: frame rate")
    parser.add_argument(
        "--no-hold",
        action="store_true",
        help="With --view: exit at the end instead of keeping the final state on screen",
    )
    parser.add_argument("--csv", type=Path, help="Append one summary row per run")
    parser.add_argument("--json", type=Path, help="Write the full result (with per-copy outcomes)")
    args = parser.parse_args()

    if args.view and (args.sweep or args.isolated):
        parser.error("--view works with a single shared run: use --copies X without --isolated")
    if args.speed < 0 or args.fps <= 0:
        parser.error("--speed must be >= 0 and --fps > 0")
    if args.sweep:
        values = [int(v) for v in args.sweep.split(",") if v.strip()]
        status = 0
        for x in values:
            cmd = [
                sys.executable, __file__, "--copies", str(x), "--duration", str(args.duration),
                "--dt", str(args.dt), "--spacing-x", str(args.spacing_x),
                "--spacing-y", str(args.spacing_y), "--progress", str(args.progress),
            ]  # fmt: skip
            if args.isolated:
                cmd.append("--isolated")
            if args.csv:
                cmd += ["--csv", str(args.csv)]
            if args.json:
                tag = "iso" if args.isolated else "shared"
                name = f"{args.json.stem}_{tag}_x{x}{args.json.suffix}"
                cmd += ["--json", str(args.json.with_name(name))]
            print(f"=== X = {x} ({x * MODULES_PER_COPY} modules) ===", file=sys.stderr, flush=True)
            status |= subprocess.call(cmd, env=os.environ.copy())
        return status

    if args.copies < 1:
        parser.error("--copies must be >= 1")
    result = run(
        args.copies,
        duration_s=args.duration,
        dt_s=args.dt,
        spacing_x_m=args.spacing_x,
        spacing_y_m=args.spacing_y,
        progress_every_s=args.progress,
        isolated=args.isolated,
        use_viewer=args.view,
        speed=args.speed,
        view_fps=args.fps,
        hold=not args.no_hold,
    )
    if args.view:
        print(
            f"Viewer run: timings include {result.viewer_wall_s:.1f} s of rendering/pacing "
            "and are not benchmark numbers (mode = shared-viewer).",
            file=sys.stderr,
        )
    row = {k: getattr(result, k) for k in CSV_FIELDS}
    print(json.dumps(row, indent=2))
    if args.csv:
        new = not args.csv.exists()
        with args.csv.open("a", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            if new:
                writer.writeheader()
            writer.writerow(row)
    if args.json:
        payload = dict(row, per_copy=[vars(c) for c in result.per_copy])
        args.json.write_text(json.dumps(payload, indent=2) + "\n")
    # Exit 0 whenever the benchmark itself ran; per-copy failures are data, not errors.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
