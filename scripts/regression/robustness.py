#!/usr/bin/env python3
"""Success rate of a Runtime Inspector demo under tiny initial perturbations.

Physical reconfiguration demos are chaotic: a 1e-7 m change in where a module
starts can decide whether a run completes. A single run therefore says little;
this script runs the same demo many times, each with the free-joint x/y
positions of every module nudged by Gaussian noise (seeded, so reproducible),
and reports how many runs complete.

Run 0 is always unperturbed. Runs execute in parallel worker processes.

Examples:
  python scripts/regression/robustness.py --runs 6 --jobs 4 -- \\
      runtime examples/robot_packs/smores_ep --backend mujoco \\
      --demo smores_physical_driver_to_snake --model-view smores_topology --no-viewer

  python scripts/regression/robustness.py --runs 20 -- \\
      runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice --no-viewer

  # Compare MuJoCo's two connection-change modes on the same seeds:
  python scripts/regression/robustness.py --runs 20 --immediate 1 -- ...

Results go to --out (default: results/robustness/<demo>-<time>/): one line per
run in runs.csv and a summary.json. Each run's inspector frames (frames.jsonl,
about 25 MB for a SMORES run) are deleted once the run is summarised; pass
--keep-frames to keep them for replay.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def _worker(seed: int, sigma_m: float, immediate: str | None, out: Path, cli: list[str]) -> int:
    """Run one perturbed demo in this process (invoked through --worker)."""
    import mujoco
    import numpy as np

    from modsim_backend_mujoco import adapter as mujoco_adapter

    adapter_class = mujoco_adapter.MuJoCoBackendAdapter
    if immediate is not None:
        original_init = adapter_class.__init__

        def init(self: object, *args: object, **kwargs: object) -> None:
            kwargs.setdefault("immediate_connection_changes", immediate == "1")
            original_init(self, *args, **kwargs)  # type: ignore[arg-type]

        adapter_class.__init__ = init  # type: ignore[method-assign]

    original_load = adapter_class.load

    def load(self: object, *args: object, **kwargs: object) -> object:
        result = original_load(self, *args, **kwargs)  # type: ignore[arg-type]
        if seed:
            model = self._require_compiled().model  # type: ignore[attr-defined]
            data = self._data  # type: ignore[attr-defined]
            rng = np.random.default_rng(seed)
            for joint in range(model.njnt):
                if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
                    address = model.jnt_qposadr[joint]
                    data.qpos[address : address + 2] += rng.normal(0.0, sigma_m, 2)
            mujoco.mj_forward(model, data)
            self._after_forward(model, data)  # type: ignore[attr-defined]
        return result

    adapter_class.load = load  # type: ignore[method-assign]

    sys.path.insert(0, str(HERE))
    sys.argv = ["run_demo.py", "--out", str(out), "--every", "0", "--", *cli]
    import runpy

    try:
        runpy.run_path(str(HERE / "run_demo.py"), run_name="__main__")
    except SystemExit as exit_:
        return int(exit_.code or 0)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--runs", type=int, default=6, help="number of runs (run 0 unperturbed)")
    parser.add_argument("--first-seed", type=int, default=0)
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--sigma", type=float, default=1e-7, help="perturbation std dev, m")
    parser.add_argument(
        "--immediate",
        choices=("0", "1"),
        default=None,
        help="force MuJoCo immediate_connection_changes off (0) or on (1)",
    )
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument(
        "--keep-frames",
        action="store_true",
        help="keep each run's frames.jsonl (large) instead of deleting it",
    )
    parser.add_argument("--worker", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("cli", nargs=argparse.REMAINDER, help="-- then modsim CLI arguments")
    args = parser.parse_args()
    cli = args.cli[1:] if args.cli[:1] == ["--"] else args.cli
    if not cli:
        parser.error("pass the modsim CLI arguments after --")

    if args.worker is not None:
        assert args.out is not None
        return _worker(args.worker, args.sigma, args.immediate, args.out, cli)

    demo = cli[cli.index("--demo") + 1] if "--demo" in cli else "demo"
    out = args.out or REPO / "results" / "robustness" / f"{demo}-{time.strftime('%Y%m%d-%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)
    seeds = list(range(args.first_seed, args.first_seed + args.runs))
    print(f"{demo}: {len(seeds)} runs, {args.jobs} at a time -> {out}", flush=True)

    def run(seed: int) -> dict[str, object]:
        run_dir = out / f"seed_{seed}"
        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--worker",
            str(seed),
            "--sigma",
            str(args.sigma),
            "--out",
            str(run_dir),
            *(["--immediate", args.immediate] if args.immediate else []),
            "--",
            *cli,
        ]
        started = time.monotonic()
        completed = subprocess.run(command, cwd=REPO, capture_output=True, text=True, check=False)
        (out / f"seed_{seed}.log").write_text(completed.stdout + completed.stderr)
        summary_path = run_dir / "summary.json"
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        if not args.keep_frames:
            (run_dir / "frames.jsonl").unlink(missing_ok=True)
        row = {
            "seed": seed,
            "phase": summary.get("phase", "crashed"),
            "sim_time_s": summary.get("sim_time_s"),
            "events": summary.get("events"),
            "wall_s": round(time.monotonic() - started, 1),
            "detail": str(summary.get("detail", completed.stderr.strip()[-200:]))[:200],
        }
        print(
            f"  seed {seed}: {row['phase']} at t={row['sim_time_s']} s ({row['wall_s']} s wall)",
            flush=True,
        )
        return row

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        rows = sorted(pool.map(run, seeds), key=lambda row: int(str(row["seed"])))

    with (out / "runs.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    completed_runs = [row for row in rows if row["phase"] == "complete"]
    summary = {
        "demo": demo,
        "cli": cli,
        "runs": len(rows),
        "complete": len(completed_runs),
        "failed_seeds": [row["seed"] for row in rows if row["phase"] != "complete"],
        "sigma_m": args.sigma,
        "immediate_connection_changes": args.immediate,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    failed = summary["failed_seeds"]
    print(f"{demo}: {len(completed_runs)}/{len(rows)} complete; failed seeds {failed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
