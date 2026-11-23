"""Run one ``modsim runtime`` / ``modsim run --gui`` demo headlessly and record it.

The real CLI parses the arguments and resolves every per-demo default (dt,
duration, gravity, ground, connector gap, viewer rules). Instead of opening
the Qt Runtime Inspector, the resolved ``RuntimeInspectorConfig`` is driven
through ``RuntimeInspectorRunner`` — the same Qt-free engine the GUI worker
uses — with the worker's stop rule (step budget or terminal scenario phase).
Inspector frames are published on a fixed simulated-time cadence
(``--publish-hz``) instead of wall-clock pacing, so the run is reproducible.

Usage::

    python scripts/regression/run_demo.py --out DIR [--every N] -- \\
        runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice

Writes ``DIR/trace.txt`` (exact world state, see state_trace.py),
``DIR/frames.jsonl`` (one normalised inspector frame per publish) and
``DIR/summary.json``. Wall-clock-only fields (``planning_ms``) are dropped.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys
import time
import types
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import state_trace

WALL_CLOCK_KEYS = {"planning_ms"}


def _normalise(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _normalise(v) for k, v in value.items() if k not in WALL_CLOCK_KEYS}
    if isinstance(value, list):
        return [_normalise(v) for v in value]
    if isinstance(value, float):
        return value.hex()
    return value


def _drive(config: Any, out: Path, tracer: state_trace.Tracer) -> int:
    from modsim.runtime.inspector_runner import RuntimeInspectorRunner

    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(config, status_callback=statuses.append)
    perturb = os.environ.get("MODSIM_REGRESSION_PERTURB")
    if perturb:  # self-test only: nudge one generalized coordinate by a relative epsilon
        data = getattr(runner.session.adapter, "_data", None)
        if data is None:
            raise SystemExit("MODSIM_REGRESSION_PERTURB needs the MuJoCo backend")
        # Scale the largest translational coordinate. Scaling a zero changes
        # nothing, and MuJoCo renormalises free-joint quaternions every step.
        model = runner.session.adapter._compiled.model  # type: ignore[attr-defined]
        positions: list[int] = []
        for joint in range(model.njnt):
            address = int(model.jnt_qposadr[joint])
            if model.jnt_type[joint] == 0:  # mjJNT_FREE: x, y, z then a quaternion
                positions.extend(range(address, address + 3))
            elif model.jnt_type[joint] in (2, 3):  # mjJNT_SLIDE, mjJNT_HINGE
                positions.append(address)
        index = max(positions, key=lambda i: abs(float(data.qpos[i])))
        before = float(data.qpos[index])
        data.qpos[index] = before * (1.0 + float(perturb))
        print(f"self-test: qpos[{index}] {before!r} -> {float(data.qpos[index])!r}")
    publish_every = max(1, round(1.0 / (config.publish_hz * config.dt_s)))
    frames = (out / "frames.jsonl").open("w")
    started = time.perf_counter()
    completed = 0
    try:
        frames.write(json.dumps(_normalise(runner.frame().model_dump(mode="json"))) + "\n")
        raised = None
        try:
            while completed < runner.step_count and not runner.execution_finished:
                runner.step()
                completed += 1
                if completed % publish_every == 0:
                    frame = runner.frame()
                    frames.write(json.dumps(_normalise(frame.model_dump(mode="json"))) + "\n")
        except Exception as error:  # a scenario failure is an outcome to compare, not a crash
            raised = f"{type(error).__module__}.{type(error).__name__}: {error}"
        final = runner.frame()
        frames.write(json.dumps(_normalise(final.model_dump(mode="json"))) + "\n")
        status = runner.scenario.status
        summary = {
            "demo": str(getattr(config.demo, "value", config.demo)),
            "backend": config.backend,
            "dt_s": config.dt_s,
            "duration_s": config.duration_s,
            "gravity": config.gravity,
            "ground": config.ground,
            "steps": completed,
            "step_budget": runner.step_count,
            "sim_time_s": runner.session.world.time_s,
            "phase": status.phase.value,
            "detail": status.detail,
            "action_index": status.action_index,
            "action_count": status.action_count,
            "connections": sorted(runner.session.world.connections),
            "events": len(runner.session.world.event_log.events),
            "raised": raised,
            "metrics": _normalise(dataclasses.asdict(final.metrics)),
            "setup_status_messages": statuses,
        }
    finally:
        frames.close()
        runner.shutdown()  # writes the final traced state
    wall = time.perf_counter() - started
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    (out / "wall_seconds.txt").write_text(f"{wall:.3f}\n")
    if summary["raised"]:
        print(f"scenario raised at step {completed}: {summary['raised']}")
    print(
        f"{summary['demo']} [{config.backend}] {summary['phase']}: steps={completed}/"
        f"{runner.step_count} t={summary['sim_time_s']:.4f}s events={summary['events']} "
        f"trace_records={tracer.records} wall={wall:.1f}s"
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--every", type=int, default=250, help="Trace every Nth step (0 = final only)"
    )
    parser.add_argument("cli", nargs=argparse.REMAINDER, help="-- then modsim CLI arguments")
    args = parser.parse_args()
    cli = args.cli[1:] if args.cli[:1] == ["--"] else args.cli
    if not cli:
        parser.error("pass the modsim CLI arguments after --")
    args.out.mkdir(parents=True, exist_ok=True)
    tracer = state_trace.install(args.out / "trace.txt", args.every)

    captured: dict[str, int] = {}

    def fake_runtime_main(config: Any) -> int:
        captured["code"] = _drive(config, args.out, tracer)
        return captured["code"]

    # Intercept the GUI launch: the CLI imports this name lazily.
    stub = types.ModuleType("modsim_studio.runtime_app")
    stub.main = fake_runtime_main  # type: ignore[attr-defined]
    sys.modules["modsim_studio.runtime_app"] = stub

    import modsim
    from modsim.cli import app

    (args.out / "modsim_path.txt").write_text(str(Path(modsim.__file__).resolve()) + "\n")
    try:
        app(args=cli, prog_name="modsim", standalone_mode=False)
    except SystemExit as exit_:
        code = exit_.code if isinstance(exit_.code, int) else 1
        if "code" not in captured:
            return code or 1
        return code
    except Exception as error:  # Typer re-raises typer.Exit as click exceptions
        exit_code = getattr(error, "exit_code", None)
        if exit_code is not None and "code" in captured:
            return int(exit_code)
        raise
    if "code" not in captured:
        print("The CLI did not launch a Runtime Inspector session", file=sys.stderr)
        return 1
    return captured["code"]


if __name__ == "__main__":
    raise SystemExit(main())
