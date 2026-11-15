"""Exercise a built ModSim distribution in a fresh environment outside its checkout."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

from verify_artifacts import verify_artifact_contents


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("distribution", type=Path)
    parser.add_argument(
        "--python", default=sys.executable, help="Python interpreter for the new venv"
    )
    arguments = parser.parse_args()
    distribution = arguments.distribution.resolve(strict=True)
    verify_artifact_contents(distribution)
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"

    with TemporaryDirectory(prefix="modsim-installed-") as directory:
        work = Path(directory)
        venv = work / "venv"
        home = work / "modsim-home"
        environment["MODSIM_PATH"] = str(home)
        environment.pop("MODSIM_HOME", None)

        def run(*command: str) -> subprocess.CompletedProcess[str]:
            return subprocess.run(
                command,
                cwd=work,
                env=environment,
                text=True,
                capture_output=True,
                check=True,
            )

        try:
            run("uv", "venv", "--python", arguments.python, str(venv))
            python = venv / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
            cli = venv / (
                "Scripts/modsim-robotics.exe" if os.name == "nt" else "bin/modsim-robotics"
            )
            studio_cli = venv / (
                "Scripts/modsim-robotics-studio.exe"
                if os.name == "nt"
                else "bin/modsim-robotics-studio"
            )
            run("uv", "pip", "install", "--python", str(python), str(distribution))
            site_packages = Path(
                run(
                    str(python),
                    "-c",
                    "import sysconfig; print(sysconfig.get_path('purelib'))",
                ).stdout.strip()
            )
            # Simulate another project's import name in this temporary venv.
            # It must remain independent of our package and CLI entry points.
            with (site_packages / "modsim.py").open("x", encoding="utf-8") as stream:
                stream.write("COLLISION_SENTINEL = True\n")
            run(
                str(python),
                "-c",
                "from importlib.metadata import distribution; "
                "from importlib.resources import files; from importlib.util import find_spec; "
                "import modsim_robotics, sys; "
                "entry_points = {entry.name: entry.value "
                "for entry in distribution('modsim-robotics').entry_points "
                "if entry.group == 'console_scripts'}; "
                "assert entry_points == {"
                "'modsim-robotics': 'modsim_robotics.cli:app', "
                "'modsim-robotics-studio': 'modsim_robotics_studio.launcher:cli'}; "
                "assert files('modsim_robotics').joinpath('data', 'robot_packs', 'generic_cube', "
                "'robot_pack.yaml').is_file(); "
                "assert 'mujoco' not in sys.modules; assert 'PySide6' not in sys.modules; "
                "assert find_spec('ruff') is None, 'Base installation pulled in Ruff'; "
                "import modsim; assert modsim.COLLISION_SENTINEL is True; "
                "print(modsim_robotics.__file__)",
            )
            run(str(cli), "--version")
            run(str(python), "-m", "modsim_robotics", "--version")
            run(str(studio_cli), "--help")
            if home.exists():
                raise RuntimeError("Importing ModSim or printing its version created user data.")
            result = json.loads(
                run(str(cli), "setup", "--home", str(home), "--output", "json").stdout
            )
            pack = Path(result["starter_pack"])
            if not pack.is_relative_to(home) or not (pack / "robot_pack.yaml").is_file():
                raise RuntimeError("Setup did not materialize the bundled starter pack.")
            catalog = json.loads(run(str(cli), "pack", "list", "--output", "json").stdout)
            if not any(
                entry["id"] == "generic_cube"
                and entry["version"] == "0.1.0"
                and Path(entry["path"]) == pack
                for entry in catalog["packs"]
            ):
                raise RuntimeError("The installed starter is missing from the pack library.")
            run(str(cli), "pack", "inspect", "generic_cube")
            run(str(cli), "pack", "validate", "generic_cube@0.1.0", "--profile", "simulation")
            docking = json.loads(
                run(
                    str(cli),
                    "dock",
                    "generic_cube",
                    "--backend",
                    "mock",
                    "--count",
                    "2",
                    "--output",
                    "json",
                ).stdout
            )
            if len(docking["connections"]) != 1:
                raise RuntimeError("Installed mock example did not commit exactly one connection.")
            if not any(event["kind"] == "DockCommitted" for event in docking["events"]):
                raise RuntimeError("Installed mock example did not emit DockCommitted.")
            repeated = json.loads(
                run(str(cli), "setup", "--home", str(home), "--output", "json").stdout
            )
            if repeated["created"] or Path(repeated["starter_pack"]) != pack:
                raise RuntimeError("Repeated setup did not preserve the existing starter pack.")
        except subprocess.CalledProcessError as error:
            print(error.stdout, file=sys.stderr)
            print(error.stderr, file=sys.stderr)
            raise

    print(f"Installed-distribution smoke test passed: {distribution.name}")


if __name__ == "__main__":
    main()
