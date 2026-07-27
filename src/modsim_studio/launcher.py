"""Dependency-friendly console launcher for optional ModSim Studio."""

from __future__ import annotations

import sys
from pathlib import Path


def cli() -> None:
    """Launch Studio or explain how to install its optional dependencies."""
    try:
        from modsim_studio.app import main
    except ImportError as error:
        print(
            "ModSim Studio dependencies are not installed. "
            "Install with: pip install 'modsim-robotics[studio]'",
            file=sys.stderr,
        )
        raise SystemExit(2) from error
    initial_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    raise SystemExit(main(initial_path))
