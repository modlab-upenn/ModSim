"""Check ModSim's built distributions without installing or modifying them."""

from __future__ import annotations

import argparse
import configparser
import re
import tarfile
import tomllib
import zipfile
from collections.abc import Callable
from email import policy
from email.parser import BytesParser
from pathlib import Path, PurePosixPath

from verify_tag import read_package_version

PROJECT_ROOT = Path(__file__).resolve().parents[2]
NAMESPACES = {"modsim_robotics", "modsim_robotics_studio", "modsim_robotics_backend_mujoco"}
LEGACY_NAMESPACES = {"modsim", "modsim_studio", "modsim_backend_mujoco"}
ENTRY_POINTS = {
    "modsim-robotics": "modsim_robotics.cli:app",
    "modsim-robotics-studio": "modsim_robotics_studio.launcher:cli",
}
DEVELOPMENT_TOOLS = {
    "build",
    "ruff",
    "pytest",
    "pytest-cov",
    "pyright",
    "pre-commit",
    "twine",
    "hatchling",
}
STUDIO_DEPENDENCIES = {"numpy", "pyqtgraph", "pyside6", "pyvista", "pyvistaqt"}


def _check_dependency_policy(requirements: list[str], extras: list[str]) -> None:
    """Require the default desktop stack and keep MuJoCo behind its extra."""
    required: set[str] = set()
    mujoco_markers: list[str] = []
    for requirement in requirements:
        match = re.match(r"[A-Za-z0-9][A-Za-z0-9_.-]*", requirement)
        if match is None:
            raise RuntimeError(f"Malformed dependency metadata: {requirement!r}")
        name = re.sub(r"[-_.]+", "-", match[0]).lower()
        marker = requirement.partition(";")[2].strip()
        if not marker:
            required.add(name)
        if name == "mujoco":
            mujoco_markers.append(marker)
        if name in DEVELOPMENT_TOOLS and not re.fullmatch(r"extra\s*==\s*(['\"])dev\1", marker):
            raise RuntimeError(f"Development tool appears outside the dev extra: {requirement!r}")

    missing = STUDIO_DEPENDENCIES - required
    if missing:
        raise RuntimeError(
            "Studio dependencies must be installed by default; missing unconditional "
            f"requirements: {sorted(missing)}"
        )
    if "mujoco" in required:
        raise RuntimeError("MuJoCo must remain an optional backend, outside default dependencies.")
    if "mujoco" not in extras or len(mujoco_markers) != 1:
        raise RuntimeError("Distribution must declare the mujoco extra and its backend dependency.")
    # This project uses one extra-only marker for the backend. Restricting that
    # expression also rejects an OR branch that leaks MuJoCo into a base install.
    marker = mujoco_markers[0]
    while marker.startswith("(") and marker.endswith(")"):
        marker = marker[1:-1].strip()
    if not re.fullmatch(r"extra\s*==\s*(['\"])mujoco\1", marker):
        raise RuntimeError("MuJoCo must be selected only by the mujoco extra.")


def _check_members(members: set[PurePosixPath], starter_root: PurePosixPath) -> None:
    forbidden_directories = {
        ".git",
        ".github",
        ".venv",
        ".modsim",
        ".pytest_cache",
        ".ruff_cache",
        ".test-tmp",
        "__pycache__",
        "node_modules",
        "tests",
        "website",
    }
    development_members = {
        str(member)
        for member in members
        if forbidden_directories.intersection(member.parts)
        or member.suffix in {".pyc", ".pyo", ".log"}
        or member.name in {".coverage", "MJDATA.TXT", "MJMODEL.TXT"}
        or member.is_relative_to("docs/archive")
        or member.is_relative_to("docs/technical_report")
    }
    if development_members:
        raise RuntimeError(
            f"Distribution contains development files: {sorted(development_members)}"
        )

    legacy_members = {
        str(member)
        for member in members
        for relative in (member.relative_to("src") if member.parts[0] == "src" else member,)
        if relative.parts and relative.parts[0].removesuffix(".py") in LEGACY_NAMESPACES
    }
    if legacy_members:
        raise RuntimeError(f"Distribution contains legacy import names: {sorted(legacy_members)}")

    pack_roots = (
        PurePosixPath("examples/robot_packs"),
        PurePosixPath("modsim_robotics/data/robot_packs"),
        PurePosixPath("src/modsim_robotics/data/robot_packs"),
    )
    unexpected = {
        str(member)
        for member in members
        for root in pack_roots
        if member.is_relative_to(root) and member.relative_to(root).parts[0] != "generic_cube"
    }
    unexpected.update(
        str(member)
        for member in members
        if member.name == "robot_pack.yaml" and member != starter_root / "robot_pack.yaml"
    )
    if unexpected:
        raise RuntimeError(f"Distribution contains unapproved Robot Packs: {sorted(unexpected)}")

    source = PROJECT_ROOT / "examples/robot_packs/generic_cube"
    if not (source / "robot_pack.yaml").is_file():
        raise RuntimeError("The authoritative Generic Cube starter pack is missing.")
    expected = {
        starter_root / path.relative_to(source).as_posix()
        for path in source.rglob("*")
        if path.is_file()
    }
    missing = expected - members
    if missing:
        raise RuntimeError(
            f"Distribution is missing starter pack files: {sorted(map(str, missing))}"
        )


def _check_metadata(
    read: Callable[[str], bytes], *, metadata_path: str, license_path: str, version_path: str
) -> None:
    project = tomllib.loads((PROJECT_ROOT / "pyproject.toml").read_text(encoding="utf-8"))[
        "project"
    ]
    metadata = BytesParser(policy=policy.default).parsebytes(read(metadata_path))
    version = read_package_version(
        (PROJECT_ROOT / "src/modsim_robotics/__init__.py").read_text(encoding="utf-8")
    )
    expected_fields = {
        "Name": project["name"],
        "Version": version,
        "Requires-Python": project["requires-python"],
        "License-Expression": project["license"],
    }
    for field, expected in expected_fields.items():
        if metadata.get_all(field) != [expected]:
            raise RuntimeError(f"Distribution {field} must match {expected!r}.")
    if metadata.get_all("License-File") != ["LICENSE"]:
        raise RuntimeError("Distribution metadata must declare the project LICENSE file.")
    if read(license_path) != (PROJECT_ROOT / "LICENSE").read_bytes():
        raise RuntimeError("Distribution LICENSE does not match the repository license.")
    if read_package_version(read(version_path).decode("utf-8")) != version:
        raise RuntimeError("Packaged code and distribution metadata have different versions.")

    _check_dependency_policy(
        metadata.get_all("Requires-Dist", []), metadata.get_all("Provides-Extra", [])
    )


def verify_artifact_contents(distribution: Path) -> None:
    """Check published identity, metadata, license, code, and starter-pack contents."""
    version = read_package_version(
        (PROJECT_ROOT / "src/modsim_robotics/__init__.py").read_text(encoding="utf-8")
    )
    distribution_stem = f"modsim_robotics-{version}"
    if distribution.suffix == ".whl":
        if not distribution.name.startswith(distribution_stem + "-"):
            raise RuntimeError("Wheel filename does not match the project name and version.")
        with zipfile.ZipFile(distribution) as archive:
            members = {PurePosixPath(name) for name in archive.namelist() if not name.endswith("/")}
            starter_root = PurePosixPath("modsim_robotics/data/robot_packs/generic_cube")
            _check_members(members, starter_root)
            dist_info = distribution_stem + ".dist-info"
            if any(member.parts[0] not in NAMESPACES | {dist_info} for member in members):
                raise RuntimeError(
                    "Wheel contains files outside the public Python packages and metadata."
                )
            for namespace in NAMESPACES:
                if PurePosixPath(namespace, "__init__.py") not in members:
                    raise RuntimeError(f"Wheel is missing the {namespace} package.")
            _check_metadata(
                archive.read,
                metadata_path=f"{dist_info}/METADATA",
                license_path=f"{dist_info}/licenses/LICENSE",
                version_path="modsim_robotics/__init__.py",
            )
            entry_points = configparser.ConfigParser(interpolation=None)
            entry_points.read_string(archive.read(f"{dist_info}/entry_points.txt").decode("utf-8"))
            if dict(entry_points["console_scripts"]) != ENTRY_POINTS:
                raise RuntimeError("Wheel console scripts do not match the public CLI commands.")
    elif distribution.name.endswith(".tar.gz"):
        if distribution.name != distribution_stem + ".tar.gz":
            raise RuntimeError(
                "Source archive filename does not match the project name and version."
            )
        with tarfile.open(distribution) as archive:
            files = {member.name: member for member in archive.getmembers() if member.isfile()}
            if any(PurePosixPath(name).parts[0] != distribution_stem for name in files):
                raise RuntimeError("Source archive must contain one versioned project directory.")
            members = {PurePosixPath(*PurePosixPath(name).parts[1:]) for name in files}
            _check_members(members, PurePosixPath("examples/robot_packs/generic_cube"))

            def read(name: str) -> bytes:
                stream = archive.extractfile(files[name])
                if stream is None:
                    raise RuntimeError(f"Source archive is missing file contents: {name}")
                return stream.read()

            _check_metadata(
                read,
                metadata_path=f"{distribution_stem}/PKG-INFO",
                license_path=f"{distribution_stem}/LICENSE",
                version_path=f"{distribution_stem}/src/modsim_robotics/__init__.py",
            )
            project = tomllib.loads(read(f"{distribution_stem}/pyproject.toml").decode("utf-8"))
            if project["project"]["scripts"] != ENTRY_POINTS:
                raise RuntimeError(
                    "Source archive console scripts do not match the public CLI commands."
                )
    else:
        raise RuntimeError("Expected a wheel or .tar.gz source distribution.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("distributions", nargs="+", type=Path)
    arguments = parser.parse_args()
    for distribution in arguments.distributions:
        verify_artifact_contents(distribution.resolve(strict=True))
        print(f"Distribution metadata and contents passed: {distribution.name}")


if __name__ == "__main__":
    main()
