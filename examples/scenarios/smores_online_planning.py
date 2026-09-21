"""Headless runner for the online planner, with optional final Inspector snapshot."""

from __future__ import annotations

import argparse
from pathlib import Path

from modsim.runtime.demos import RuntimeDemo
from modsim.runtime.inspector_runner import RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim.runtime.reconfiguration import ReconfigurationPhase


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        choices=("smores_online_assembly", "smores_online_driver_to_snake"),
        default="smores_online_assembly",
    )
    parser.add_argument(
        "--duration", type=float, default=360.0, help="Simulation budget in seconds"
    )
    parser.add_argument("--snapshot", type=Path, help="Write the final Inspector frame as JSON")
    args = parser.parse_args()
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=Path(__file__).resolve().parents[1] / "robot_packs/smores_ep",
            demo=RuntimeDemo(args.demo),
            backend="mujoco",
            duration_s=args.duration,
            dt_s=0.002,
            gravity=True,
            ground=True,
            height_m=0.05,
        )
    )
    try:
        for step in range(runner.step_count):
            runner.step()
            status = runner.scenario.status
            if step % 5000 == 0:
                print(f"{status.time_s:.1f} s: {status.detail}", flush=True)
            if status.phase in {ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED}:
                break
        status = runner.scenario.status
        if args.snapshot is not None:
            args.snapshot.write_text(runner.frame().model_dump_json(indent=2) + "\n")
        print(f"{status.phase.value} at {status.time_s:.3f} s: {status.detail}")
        if status.phase is not ReconfigurationPhase.COMPLETE:
            print("The requested topology was not completed within the simulation budget.")
            return 1
        return 0
    finally:
        runner.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
