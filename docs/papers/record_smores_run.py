"""Record a nominal spatial run without changing the planner or controller.

Requires the ModSim MuJoCo extra. An explicit output directory keeps the
archived paper data from being overwritten accidentally.
"""

import argparse
import csv
import hashlib
import json
import platform
import subprocess
import time
from datetime import UTC, datetime
from importlib.metadata import version
from pathlib import Path

import numpy as np

from modsim.core.transforms import quat_angle
from modsim_backend_mujoco.spatial_experiment import CAPTURE, MATE, SpatialExperiment


def record(output: Path) -> None:
    root = Path(__file__).resolve().parents[2]
    out = output
    output.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    e = SpatialExperiment(root / "examples/robot_packs/smores_ep")
    planning_s = time.perf_counter() - started
    rows = []
    capture = {}
    saturation_steps = 0
    lastphase = ""
    steps = 0
    try:
        while not e.terminal:
            e.step()
            steps += 1
            force = float(np.max(np.abs(e.data.actuator_force)))
            if force >= e.torque_limit_nm - 1e-09:
                saturation_steps += 1
            if steps % 10 == 0 or e.phase != lastphase:
                row = {
                    "time_s": float(e.data.time),
                    "phase": e.phase,
                    "joint_error_rad": e.scenario.joint_error_rad,
                    "target_error_m": e.target_error_m(),
                    "effort_nm": force,
                    "penetration_m": e._penetration(e.data),
                }
                for m in ("receiver", "arm", "upper", "payload"):
                    p = (
                        e.data.site(f"{m}/connector/left").xpos
                        + e.data.site(f"{m}/connector/right").xpos
                    ) / 2
                    for ax, v in zip("xyz", p, strict=True):
                        row[f"{m}_{ax}_m"] = float(v)
                rows.append(row)
            if e.phase == "transfer" and lastphase != "transfer":
                from modsim.core.ids import ConnectorInstanceId

                a = e.session.world.connectors[ConnectorInstanceId(CAPTURE.a)]
                b = e.session.world.connectors[ConnectorInstanceId(CAPTURE.b)]
                rel = b.world_pose.relative_to(a.world_pose)
                capture = {
                    "connector_pair": [CAPTURE.a, CAPTURE.b],
                    "position_error_m": float(np.linalg.norm(rel.translation)),
                    "rotation_error_rad": quat_angle(MATE.inverse().compose(rel).rotation),
                    "height_m": float(
                        (a.world_pose.translation[2] + b.world_pose.translation[2]) / 2
                    ),
                }
            if e.phase != lastphase:
                print(e.phase, e.data.time, flush=True)
            lastphase = e.phase
        report = e.report()
        report["paper_measurements"] = {
            "planning_wall_s": planning_s,
            "elapsed_wall_s": time.perf_counter() - started,
            "capture": capture,
            "torque_saturation_time_s": saturation_steps * e.dt_s,
            "module_mass_kg": float(sum(e.model.body_mass)) / 5,
            "final_center_positions_m": {
                m: [rows[-1][f"{m}_{a}_m"] for a in "xyz"]
                for m in ("receiver", "arm", "upper", "payload")
            },
        }
        (out / "reference_run.json").write_text(json.dumps(report, indent=2) + "\n")
        with (out / "reference_trace.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        files = sorted(
            p
            for folder in (
                "src/modsim",
                "src/modsim_backend_mujoco",
                "examples/robot_packs/smores_ep",
            )
            for p in (root / folder).rglob("*")
            if p.is_file()
            and "__pycache__" not in p.parts
            and (p.suffix in {".py", ".yaml", ".urdf", ".obj", ".stl"})
        )
        manifest = {
            "recorded_at_utc": datetime.now(UTC).isoformat(),
            "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
            "worktree_dirty": bool(
                subprocess.check_output(["git", "status", "--porcelain"], text=True).strip()
            ),
            "python": platform.python_version(),
            "platform": platform.platform(),
            "packages": {p: version(p) for p in ("modsim-robotics", "mujoco", "numpy")},
            "source_sha256": {
                str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files
            },
        }
        (out / "reference_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(
            json.dumps(
                {
                    k: report[k]
                    for k in (
                        "phase",
                        "time_s",
                        "peak_effort_nm",
                        "peak_penetration_m",
                        "target_error_m",
                        "paper_measurements",
                    )
                },
                indent=2,
            )
        )
    finally:
        e.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    record(args.output)


if __name__ == "__main__":
    main()
