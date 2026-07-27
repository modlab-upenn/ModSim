"""CLI smoke and behavior tests."""

from __future__ import annotations

import json
from pathlib import Path

from typer.testing import CliRunner

from modsim import __version__
from modsim.cli import app

runner = CliRunner()


def test_cli_help_succeeds() -> None:
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "Robot Packs" in result.stdout
    assert "studio" in result.stdout


def test_cli_version_succeeds() -> None:
    result = runner.invoke(app, ["--version"])
    assert result.exit_code == 0
    assert result.stdout.strip() == f"modsim {__version__}"


def test_cli_validate_valid_pack_exits_zero(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        ["pack", "validate", str(example_pack_dir), "--profile", "simulation"],
    )
    assert result.exit_code == 0
    assert "VALID" in result.stdout
    assert "generic_cube@0.1.0" in result.stdout


def test_cli_validate_invalid_pack_exits_one(copied_pack: Path) -> None:
    (copied_pack / "assets" / "urdf" / "generic_cube.urdf").unlink()

    result = runner.invoke(app, ["pack", "validate", str(copied_pack)])

    assert result.exit_code == 1
    assert "INVALID" in result.stdout
    assert "asset.missing" in result.stdout
    assert "robot_pack.yaml:/assets/urdf/generic_cube" in result.stdout


def test_cli_json_output_is_machine_readable(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        ["pack", "validate", str(example_pack_dir), "--output", "json"],
    )

    assert result.exit_code == 0
    payload = json.loads(result.stdout)
    assert payload == {
        "valid": True,
        "profile": "authoring",
        "pack": "generic_cube@0.1.0",
        "summary": {"errors": 0, "warnings": 0, "infos": 0},
        "issues": [],
    }


def test_cli_inspect_reports_aggregate_counts(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["pack", "inspect", str(example_pack_dir)])
    assert result.exit_code == 0
    assert "Module types" in result.stdout
    assert "Connector types" in result.stdout


def test_cli_pack_init_creates_draft(tmp_path: Path, example_pack_dir: Path) -> None:
    destination = tmp_path / "draft"
    urdf = example_pack_dir / "assets" / "urdf" / "generic_cube.urdf"

    result = runner.invoke(
        app,
        ["pack", "init", "--from-urdf", str(urdf), "--out", str(destination)],
    )

    assert result.exit_code == 0
    assert "Created Robot Pack" in result.stdout
    assert (destination / "robot_pack.yaml").is_file()
