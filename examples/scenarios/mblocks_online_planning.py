"""Generate and execute the Sung 2015 planar M-Blocks construction.

Run with --plan-only to inspect geometry without simulator dependencies.
Physical execution uses the rotated one-plane pack and ideal attachments.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from modsim.planning.mblocks import LatticeGoal, LatticeState, plan_reconfiguration
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.mblocks_lattice import (
    elbow_goal,
    larger_state,
    line_goal,
    starter_state,
    tabletop_scene,
)
from modsim.runtime.mblocks_online_planning import OnlineLatticeScenario
from modsim.runtime.session import RuntimeSession


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pack", type=Path, default=Path(__file__).parents[1] / "robot_packs/mblocks_3d"
    )
    parser.add_argument("--goal", choices=("line", "elbow"), default="line")
    parser.add_argument(
        "--preset",
        choices=("small", "large"),
        default="small",
        help="Starting cluster: small has 4 cubes; large has 6 cubes",
    )
    parser.add_argument("--goal-file", type=Path, help="LatticeGoal JSON; overrides --goal")
    parser.add_argument("--initial-file", type=Path, help="LatticeState JSON; overrides --preset")
    parser.add_argument(
        "--plan-only", action="store_true", help="Print a geometric plan; no physics"
    )
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--dt", type=float, default=0.0005)
    parser.add_argument("--quarter-rpm", type=float, default=14000.0)
    parser.add_argument("--half-rpm", type=float, default=17000.0)
    parser.add_argument(
        "--snapshot", type=Path, help="Write final immutable planner diagnostics as JSON"
    )
    args = parser.parse_args()
    initial = (
        LatticeState.model_validate_json(args.initial_file.read_text())
        if args.initial_file
        else larger_state()
        if args.preset == "large"
        else starter_state()
    )
    goal = (
        LatticeGoal.model_validate_json(args.goal_file.read_text())
        if args.goal_file
        else elbow_goal(initial)
        if args.goal == "elbow"
        else line_goal(initial)
    )
    plan = plan_reconfiguration(initial, goal)
    if args.plan_only:
        print(plan.model_dump_json(indent=2))
        return 0
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    loaded = RobotPackLoader().load(args.pack)
    session = RuntimeSession.create(
        loaded,
        tabletop_scene(initial, pitch_m=goal.pitch_m),
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        timestep_s=args.dt,
        weld_pool_size=max(12, len(initial.blocks) * 3),
        hinge_pool_size=2,
        constraint_time_constant_s=0.002,
    )
    try:
        scenario = OnlineLatticeScenario.create(
            session,
            goal,
            initial=initial,
            dt_s=args.dt,
            quarter_rpm=args.quarter_rpm,
            half_rpm=args.half_rpm,
        )
        decision_cursor = 0
        while session.world.time_s < args.duration:
            scenario.step()
            # Read only at a modest diagnostic cadence, not every physics tick.
            if int(session.world.time_s / args.dt) % 100 == 0 or scenario.status.phase.value in {
                "complete",
                "failed",
            }:
                snapshot = scenario.planning_snapshot
                for decision in snapshot.decisions:
                    if decision.sequence >= decision_cursor:
                        print(
                            f"{decision.time_s:8.3f}s  {decision.kind}: {decision.detail}",
                            flush=True,
                        )
                        decision_cursor = decision.sequence + 1
            if scenario.status.phase.value in {"complete", "failed"}:
                break
        if args.snapshot:
            args.snapshot.write_text(scenario.planning_snapshot.model_dump_json(indent=2) + "\n")
        print(f"{scenario.status.phase.value}: {scenario.status.detail}")
        if scenario.status.phase.value == "complete":
            return 0
        if scenario.status.phase.value != "failed":
            print("Time budget exhausted before reaching the target")
        return 1
    finally:
        session.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
