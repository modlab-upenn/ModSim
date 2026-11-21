"""Built metadata must install Studio by default and leave physics optional."""

from __future__ import annotations

import io
import subprocess
import sys
import tarfile
import tomllib
import zipfile
from pathlib import Path

import pytest

from modsim_robotics import __version__

_ROOT = Path(__file__).resolve().parents[1]
_REQUIREMENTS = (
    "numpy>=1.26,<3",
    "pyqtgraph>=0.13,<1",
    "PySide6>=6.7,<7",
    "pyvista>=0.45,<1",
    "pyvistaqt>=0.11,<1",
    'mujoco>=3.3.3,<4; extra == "mujoco"',
    'ruff>=0.16,<1; extra == "dev"',
)


def _artifact(
    directory: Path,
    kind: str,
    requirements: tuple[str, ...] = _REQUIREMENTS,
    extras: tuple[str, ...] = ("mujoco", "studio", "dev"),
) -> Path:
    """Write a minimal real wheel/sdist that satisfies the other release checks."""
    project = tomllib.loads((_ROOT / "pyproject.toml").read_text())["project"]
    metadata = (
        "Metadata-Version: 2.4\n"
        f"Name: {project['name']}\n"
        f"Version: {__version__}\n"
        f"Requires-Python: {project['requires-python']}\n"
        f"License-Expression: {project['license']}\n"
        "License-File: LICENSE\n"
        + "".join(f"Provides-Extra: {extra}\n" for extra in extras)
        + "".join(f"Requires-Dist: {requirement}\n" for requirement in requirements)
        + "\nMinimal metadata fixture.\n"
    ).encode()
    stem = f"modsim_robotics-{__version__}"
    files = {
        f"{namespace}/__init__.py": (_ROOT / "src" / namespace / "__init__.py").read_bytes()
        for namespace in (
            "modsim_robotics",
            "modsim_robotics_studio",
            "modsim_robotics_backend_mujoco",
        )
    }
    starter = _ROOT / "examples/robot_packs/generic_cube"
    if kind == "wheel":
        dist_info = f"{stem}.dist-info"
        files[f"{dist_info}/METADATA"] = metadata
        files[f"{dist_info}/licenses/LICENSE"] = (_ROOT / "LICENSE").read_bytes()
        scripts = "\n".join(f"{name} = {value}" for name, value in project["scripts"].items())
        files[f"{dist_info}/entry_points.txt"] = f"[console_scripts]\n{scripts}\n".encode()
        for source in starter.rglob("*"):
            if source.is_file():
                relative = source.relative_to(starter).as_posix()
                name = f"modsim_robotics/data/robot_packs/generic_cube/{relative}"
                files[name] = source.read_bytes()
        destination = directory / f"{stem}-py3-none-any.whl"
        with zipfile.ZipFile(destination, "w") as archive:
            for name, content in files.items():
                archive.writestr(name, content)
    else:
        files = {f"src/{name}": content for name, content in files.items()}
        files["PKG-INFO"] = metadata
        files["LICENSE"] = (_ROOT / "LICENSE").read_bytes()
        files["pyproject.toml"] = (_ROOT / "pyproject.toml").read_bytes()
        for source in starter.rglob("*"):
            if source.is_file():
                relative = source.relative_to(starter).as_posix()
                files[f"examples/robot_packs/generic_cube/{relative}"] = source.read_bytes()
        destination = directory / f"{stem}.tar.gz"
        with tarfile.open(destination, "w:gz") as archive:
            for name, content in files.items():
                entry = tarfile.TarInfo(f"{stem}/{name}")
                entry.size = len(content)
                archive.addfile(entry, io.BytesIO(content))
    return destination


def _verify(path: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(_ROOT / "tools/release/verify_artifacts.py"), str(path)],
        cwd=path.parent,
        text=True,
        capture_output=True,
        check=False,
    )


@pytest.mark.parametrize("kind", ("wheel", "sdist"))
def test_default_studio_and_optional_backend_metadata_pass(tmp_path: Path, kind: str) -> None:
    result = _verify(_artifact(tmp_path, kind))
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Distribution metadata and contents passed" in result.stdout


@pytest.mark.parametrize("kind", ("wheel", "sdist"))
@pytest.mark.parametrize("dependency", ("numpy", "pyqtgraph", "PySide6", "pyvista", "pyvistaqt"))
def test_optional_desktop_dependency_is_rejected(
    tmp_path: Path, kind: str, dependency: str
) -> None:
    requirements = tuple(
        requirement + '; extra == "studio"'
        if requirement.startswith(dependency + ">=")
        else requirement
        for requirement in _REQUIREMENTS
    )
    result = _verify(_artifact(tmp_path, kind, requirements))
    assert result.returncode != 0
    assert "Studio dependencies must be installed by default" in result.stderr
    assert dependency.lower() in result.stderr


@pytest.mark.parametrize("kind", ("wheel", "sdist"))
@pytest.mark.parametrize(
    "backend_requirement",
    (
        "mujoco>=3.3.3,<4",
        'mujoco>=3.3.3,<4; extra == "mujoco" or python_version >= "3.11"',
        'mujoco>=3.3.3,<4; extra == "studio"',
    ),
)
def test_default_or_wrong_extra_backend_is_rejected(
    tmp_path: Path, kind: str, backend_requirement: str
) -> None:
    requirements = tuple(
        backend_requirement if requirement.startswith("mujoco") else requirement
        for requirement in _REQUIREMENTS
    )
    result = _verify(_artifact(tmp_path, kind, requirements))
    assert result.returncode != 0
    assert "MuJoCo must" in result.stderr


@pytest.mark.parametrize("kind", ("wheel", "sdist"))
def test_missing_backend_extra_declaration_is_rejected(tmp_path: Path, kind: str) -> None:
    result = _verify(_artifact(tmp_path, kind, extras=("studio", "dev")))
    assert result.returncode != 0
    assert "must declare the mujoco extra" in result.stderr


@pytest.mark.parametrize("kind", ("wheel", "sdist"))
def test_default_development_tool_remains_rejected(tmp_path: Path, kind: str) -> None:
    requirements = tuple(
        "ruff>=0.16,<1" if requirement.startswith("ruff") else requirement
        for requirement in _REQUIREMENTS
    )
    result = _verify(_artifact(tmp_path, kind, requirements))
    assert result.returncode != 0
    assert "Development tool appears outside the dev extra" in result.stderr
