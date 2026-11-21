"""Dependency-friendly launcher for the bundled ``modsim-robotics-studio`` command."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from modsim_robotics import __version__
from modsim_robotics.user_data import UserDataError, resolve_robot_pack


def cli() -> None:
    """Launch a blank Builder or open an explicitly selected RobotPack.

    Raises:
        SystemExit: With the application exit status, or status 2 for invalid
            options, an unresolved selector, or missing Studio dependencies.
    """
    parser = argparse.ArgumentParser(
        prog="modsim-robotics-studio",
        description="Open a blank Studio workspace, or select a RobotPack by ID or path.",
    )
    parser.add_argument(
        "pack_path",
        nargs="?",
        metavar="PACK",
        help="Installed pack ID, ID@VERSION, or explicit path; omit for a blank workspace.",
    )
    parser.add_argument("--pack", help="Select a pack ID, ID@VERSION, or explicit path.")
    parser.add_argument(
        "--home",
        type=Path,
        help="ModSim home; defaults to MODSIM_PATH, legacy MODSIM_HOME, then ~/.modsim.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    arguments = parser.parse_args()
    if arguments.pack_path is not None and arguments.pack is not None:
        parser.error("Pass either a positional RobotPack path or --pack, not both.")
    selector = arguments.pack if arguments.pack is not None else arguments.pack_path
    try:
        initial_path = (
            resolve_robot_pack(selector, home=arguments.home) if selector is not None else None
        )
    except UserDataError as error:
        parser.error(str(error))

    try:
        from modsim_robotics_studio.app import main
    except ImportError as error:
        print(
            "ModSim Studio dependencies are not installed. "
            "Install with: pip install modsim-robotics",
            file=sys.stderr,
        )
        raise SystemExit(2) from error
    raise SystemExit(main(initial_path, home=arguments.home))
