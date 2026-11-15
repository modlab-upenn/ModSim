"""Check a stable release tag against the package's single version source."""

from __future__ import annotations

import argparse
import ast
import re
from pathlib import Path


def read_package_version(source: str) -> str:
    """Read the literal version without importing the package or its dependencies."""
    module = ast.parse(source)
    for statement in module.body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "__version__"
            for target in statement.targets
        ):
            if isinstance(statement.value, ast.Constant) and isinstance(statement.value.value, str):
                return statement.value.value
            break
    raise RuntimeError("The package must define a literal __version__ string.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", required=True)
    arguments = parser.parse_args()
    match = re.fullmatch(r"v((?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\.(?:0|[1-9]\d*))", arguments.tag)
    if match is None:
        parser.error("stable releases require a tag of the form vX.Y.Z")

    version_file = Path(__file__).resolve().parents[2] / "src/modsim_robotics/__init__.py"
    version = read_package_version(version_file.read_text(encoding="utf-8"))
    if version != match[1]:
        parser.error(f"tag {arguments.tag!r} does not match package version {version!r}")
    print(f"Release tag matches modsim-robotics {version}.")


if __name__ == "__main__":
    main()
