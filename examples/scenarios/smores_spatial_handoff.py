"""Run the experimental fixture-supported 3D SMORES handoff.

Install the mujoco extra. From the repository root:
  python examples/scenarios/smores_spatial_handoff.py --view --speed 0.5
  python examples/scenarios/smores_spatial_handoff.py --snapshot /tmp/handoff.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("expected a finite positive number")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pack", type=Path, default=Path(__file__).parents[1] / "robot_packs/smores_ep"
    )
    parser.add_argument("--view", action="store_true", help="open the native MuJoCo viewer")
    parser.add_argument("--speed", type=positive, default=0.5)
    parser.add_argument("--no-hold", action="store_true", help="close viewer on completion")
    parser.add_argument(
        "--plan-only", action="store_true", help="validate paths without stepping physics"
    )
    parser.add_argument(
        "--torque-limit", type=positive, default=1.2, help="provisional servo limit in Nm"
    )
    parser.add_argument(
        "--snapshot", type=Path, help="write plan, result, and canonical event history as JSON"
    )
    args = parser.parse_args()
    if args.speed > 10:
        parser.error("--speed must not exceed 10")
    try:
        from modsim_backend_mujoco.spatial_experiment import SpatialExperiment
    except ImportError as exc:
        parser.exit(2, f"The MuJoCo extra is required: pip install -e '.[mujoco]'\n{exc}\n")
    from modsim.planning.spatial import SpatialPlanningError

    try:
        experiment = SpatialExperiment(args.pack, torque_limit_nm=args.torque_limit)
    except SpatialPlanningError as exc:
        parser.exit(1, f"Planning refused: {exc}\n")
    try:
        if args.plan_only:
            result = experiment.report()
            result["phase"] = "planned"
            result["detail"] = "Sampled geometric plan only; physical execution not evaluated"
        else:
            if args.view:
                from modsim_backend_mujoco.spatial_viewer import run_spatial_viewer

                run_spatial_viewer(experiment, speed=args.speed, hold=not args.no_hold)
            else:
                while not experiment.terminal:
                    experiment.step()
            result = experiment.report()
        text = json.dumps(result, indent=2, allow_nan=False)
        if args.snapshot:
            args.snapshot.write_text(text + "\n", encoding="utf-8")
        print(text)
        return 0 if args.plan_only or experiment.phase in {"complete", "stopped"} else 1
    except KeyboardInterrupt:
        experiment.stop()
        return 130
    finally:
        experiment.close()


if __name__ == "__main__":
    raise SystemExit(main())
