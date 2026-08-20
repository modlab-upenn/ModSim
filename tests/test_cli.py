"""CLI smoke and behavior tests."""

from __future__ import annotations

import json
from pathlib import Path
from typing import cast

import pytest
from typer.testing import CliRunner

from modsim import __version__
from modsim.backends.registry import BACKEND_ENV_VAR
from modsim.cli import app
from modsim.core.scene import SceneSpec
from modsim.core.state import WorldState
from modsim.model_views import ModelViewContext, ModelViewFactory
from modsim.robot_packs import RobotPackLoader

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
    assert "Model views" in result.stdout


def test_cli_views_lists_pack_recipes_and_registered_builders(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["views", str(example_pack_dir), "--output", "json"])

    assert result.exit_code == 0, result.output
    payload = json.loads(result.stdout)
    assert payload["pack"] == "generic_cube@0.1.0"
    assert [recipe["id"] for recipe in payload["recipes"]] == ["module_topology"]
    assert {builder["builder"] for builder in payload["builders"]} == {"module_topology_graph"}


def test_cli_views_generates_exact_json_for_three_isolated_modules(
    example_pack_dir: Path,
) -> None:
    result = runner.invoke(
        app,
        [
            "views",
            str(example_pack_dir),
            "--view",
            "module_topology",
            "--output",
            "json",
        ],
    )

    assert result.exit_code == 0, result.output
    payload = json.loads(result.stdout)
    loaded = RobotPackLoader().load(example_pack_dir)
    recipe = loaded.pack.manifest.model_views[0]
    world = WorldState.from_scene(
        loaded.pack,
        SceneSpec.grid("generic_cube", 3, spacing_m=0.1),
    )
    expected = ModelViewFactory().build(
        recipe,
        ModelViewContext(pack=loaded.pack, world=world),
    )
    assert payload == expected.model_dump(mode="json")
    assert [node["id"] for node in payload["nodes"]] == [
        "generic_cube_0",
        "generic_cube_1",
        "generic_cube_2",
    ]
    assert payload["edges"] == []


def test_cli_views_rejects_an_unknown_recipe(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        ["views", str(example_pack_dir), "--view", "missing_view"],
    )

    assert result.exit_code == 2
    assert "Unknown model-view recipe 'missing_view'" in result.output
    assert "Available recipes: module_topology" in result.output


def test_cli_views_reports_an_unregistered_builder(copied_pack: Path) -> None:
    manifest = copied_pack / "robot_pack.yaml"
    manifest.write_text(
        manifest.read_text().replace(
            "builder: module_topology_graph",
            "builder: unavailable_builder",
            1,
        )
    )

    result = runner.invoke(
        app,
        ["views", str(copied_pack), "--view", "module_topology"],
    )

    assert result.exit_code == 2
    assert "unavailable_builder" in result.output
    assert "is not registered" in result.output
    assert "available: module_topology_graph" in result.output


def test_cli_views_rejects_a_recipe_not_enabled_for_runtime(copied_pack: Path) -> None:
    manifest = copied_pack / "robot_pack.yaml"
    manifest.write_text(manifest.read_text().replace("      - runtime\n", "      - authoring\n", 1))

    result = runner.invoke(
        app,
        ["views", str(copied_pack), "--view", "module_topology"],
    )

    assert result.exit_code == 2
    assert "not enabled for runtime generation" in result.output
    assert "configured modes: authoring" in result.output


@pytest.mark.parametrize("spacing", ("nan", "inf", "-inf"))
def test_cli_views_rejects_non_finite_spacing(
    example_pack_dir: Path,
    spacing: str,
) -> None:
    result = runner.invoke(
        app,
        [
            "views",
            str(example_pack_dir),
            "--view",
            "module_topology",
            "--spacing",
            spacing,
        ],
    )

    assert result.exit_code == 2
    assert "--spacing must be a finite number" in result.output


def test_cli_dock_runs_a_mock_session(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--count", "3"])

    assert result.exit_code == 0
    assert "2 dock(s), 0 failure(s)" in result.stdout
    assert "DockCommitted" in result.stdout
    assert "assembly:generic_cube_0" in result.stdout


def test_cli_dock_reports_why_nothing_latched(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--no-latch"])

    assert result.exit_code == 0
    assert "0 dock(s)" in result.stdout
    assert "auto-latching" in result.stdout
    assert "Pass --latch" in result.stdout


def test_cli_dock_undock_restores_free_modules(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        ["dock", str(example_pack_dir), "--count", "3", "--undock", "--output", "json"],
    )

    assert result.exit_code == 0
    payload = json.loads(result.stdout)
    assert payload["pack"] == "generic_cube@0.1.0"
    assert payload["connections"] == []
    assert len(payload["assemblies"]) == 3
    assert payload["metrics"]["undocking_success_count"] == 2


def test_cli_dock_reports_modules_placed_out_of_range(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--spacing", "1.0"])

    assert result.exit_code == 0
    assert "0 dock(s)" in result.stdout
    assert "detection radius" in result.stdout


def test_cli_dock_reports_a_pair_blocked_by_policy(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--no-latch"])

    assert result.exit_code == 0
    assert "Pairs that did not dock" in result.stdout
    assert "generic_cube_0/front" in result.stdout


def test_cli_dock_rejects_an_unknown_module_type(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--module-type", "nope"])

    assert result.exit_code == 2
    assert "unknown module type" in result.output


def test_cli_dock_rejects_an_invalid_pack(copied_pack: Path) -> None:
    (copied_pack / "assets" / "urdf" / "generic_cube.urdf").unlink()

    result = runner.invoke(app, ["dock", str(copied_pack)])

    assert result.exit_code == 1
    assert "INVALID" in result.stdout


def test_cli_backends_lists_the_mock(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(BACKEND_ENV_VAR, raising=False)
    result = runner.invoke(app, ["backends"])

    assert result.exit_code == 0
    assert "mock (active)" in result.stdout
    assert "mujoco" in result.stdout


def test_cli_backends_json_reports_the_active_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(BACKEND_ENV_VAR, "mock")
    result = runner.invoke(app, ["backends", "--output", "json"])

    assert result.exit_code == 0
    payload = json.loads(result.stdout)
    assert payload["active"] == "mock"
    assert {entry["name"] for entry in payload["backends"]} >= {"mock", "mujoco"}


def test_cli_dock_reports_the_backend_it_used(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--backend", "mock"])

    assert result.exit_code == 0
    assert "on the mock backend" in result.stdout


def test_cli_dock_honours_the_backend_environment_variable(
    example_pack_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(BACKEND_ENV_VAR, "mock")
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--output", "json"])

    assert result.exit_code == 0
    assert json.loads(result.stdout)["backend"] == "mock"


def test_cli_dock_rejects_an_unknown_backend(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--backend", "nope"])

    assert result.exit_code == 2
    assert "unknown backend" in result.output


@pytest.mark.parametrize(
    ("command", "option", "value"),
    (
        ("dock", "--dt", "0"),
        ("dock", "--dt", "nan"),
        ("run", "--dt", "0"),
        ("run", "--duration", "0"),
        ("run", "--duration", "nan"),
    ),
)
def test_cli_rejects_unsafe_time_arguments(
    example_pack_dir: Path,
    command: str,
    option: str,
    value: str,
) -> None:
    result = runner.invoke(app, [command, str(example_pack_dir), option, value])

    assert result.exit_code == 2
    assert f"{option} must be a finite number greater than 0" in result.output


def test_cli_no_gravity_notes_that_the_mock_has_none(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app, ["dock", str(example_pack_dir), "--backend", "mock", "--no-gravity"]
    )

    assert result.exit_code == 0
    assert "has no gravity" in result.output


def test_cli_run_executes_a_scripted_scenario(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        [
            "run",
            str(example_pack_dir),
            "--count",
            "2",
            "--spacing",
            "0.1",
            "--duration",
            "0.2",
            "--output",
            "json",
        ],
    )

    assert result.exit_code == 0
    payload = json.loads(result.stdout)
    assert payload["backend"] == "mock"
    assert len(payload["connections"]) == 1


def test_cli_run_arranges_a_selected_connector_pair(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        [
            "run",
            str(example_pack_dir),
            "--fixed-connector",
            "front",
            "--moving-connector",
            "front",
            "--connector-gap",
            "0.02",
            "--duration",
            "1",
            "--output",
            "json",
        ],
    )

    assert result.exit_code == 0, result.output
    payload = json.loads(result.stdout)
    assert len(payload["connections"]) == 1
    assert payload["metrics"]["docking_success_count"] == 1


def test_cli_run_requires_both_selected_connectors(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        ["run", str(example_pack_dir), "--fixed-connector", "front"],
    )

    assert result.exit_code == 2
    assert "must be supplied together" in result.output


def test_cli_run_reports_an_unknown_selected_connector(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        [
            "run",
            str(example_pack_dir),
            "--fixed-connector",
            "missing",
            "--moving-connector",
            "front",
        ],
    )

    assert result.exit_code == 2
    assert "Could not arrange the connector pair" in result.output
    assert "unknown connector" in result.output


def test_cli_run_releases_at_the_scheduled_time(example_pack_dir: Path) -> None:
    result = runner.invoke(
        app,
        [
            "run",
            str(example_pack_dir),
            "--count",
            "2",
            "--spacing",
            "0.1",
            "--duration",
            "0.4",
            "--undock-at",
            "0.2",
            "--output",
            "json",
        ],
    )

    assert result.exit_code == 0
    payload = json.loads(result.stdout)
    assert payload["connections"] == []
    assert payload["metrics"]["undocking_success_count"] == 1


def run_scenario(example_pack_dir: Path, *extra: str) -> dict[str, object]:
    """Run a scripted scenario and return its JSON report."""
    result = runner.invoke(
        app,
        [
            "run",
            str(example_pack_dir),
            "--count",
            "2",
            "--spacing",
            "0.1",
            "--output",
            "json",
            *extra,
        ],
    )
    assert result.exit_code == 0, result.output
    payload: dict[str, object] = json.loads(result.stdout)
    return payload


def module_gap(payload: dict[str, object]) -> float:
    """Return the x separation between the two modules in a scenario report."""
    positions = cast("dict[str, list[float]]", payload["module_positions"])
    first, second = (positions[name][0] for name in sorted(positions))
    return abs(second - first)


def test_cli_run_retracts_the_driver_after_release(example_pack_dir: Path) -> None:
    """Releasing a weld does not separate anything by itself.

    Two welded modules share a velocity, so on release they coast along
    together. The retract step is what makes an undock visible.
    """
    payload = run_scenario(
        example_pack_dir, "--duration", "3", "--undock-at", "1", "--retract", "0.05"
    )

    assert payload["connections"] == []
    assert module_gap(payload) > 0.15


def test_cli_run_with_zero_retract_leaves_the_pair_coasting(example_pack_dir: Path) -> None:
    payload = run_scenario(
        example_pack_dir, "--duration", "3", "--undock-at", "1", "--retract", "0"
    )

    assert payload["connections"] == []
    # Still adjacent: the constraint is gone but nothing pushed them apart.
    assert module_gap(payload) == pytest.approx(0.1, abs=0.02)


def test_cli_run_requires_mujoco_for_the_viewer(example_pack_dir: Path) -> None:
    result = runner.invoke(app, ["run", str(example_pack_dir), "--backend", "mock", "--view"])

    assert result.exit_code == 2
    assert "requires the MuJoCo backend" in result.output


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
