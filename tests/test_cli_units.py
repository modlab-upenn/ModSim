"""CLI quantity input and local display choices preserve canonical runtime data."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest
from typer.testing import CliRunner

from modsim_robotics.cli import app
from modsim_robotics.runtime import RuntimeInspectorConfig
from modsim_robotics.unit_settings import load_unit_preferences, save_unit_preferences
from modsim_robotics.units import QuantityKind, UnitPreferences

runner = CliRunner()


@pytest.fixture(autouse=True)
def isolated_preferences(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    home = tmp_path / "preferences-home"
    monkeypatch.setenv("MODSIM_PATH", str(home))
    monkeypatch.delenv("MODSIM_HOME", raising=False)
    return home


@pytest.mark.parametrize(
    ("command", "canonical", "readable"),
    [
        (
            "dock",
            ["--spacing", "0.1", "--dt", "0.01", "--steps", "2"],
            ["--spacing", "100 mm", "--timestep", "10 ms", "--steps", "2"],
        ),
        (
            "views",
            ["--view", "module_topology", "--spacing", "0.1"],
            ["--view", "module_topology", "--spacing", "10 cm"],
        ),
        (
            "run",
            [
                "--duration",
                "0.01",
                "--dt",
                "0.001",
                "--spacing",
                "0.15",
                "--approach",
                "0.03",
                "--height",
                "0.05",
                "--orientation",
                "0",
            ],
            [
                "--duration",
                "10 ms",
                "--timestep",
                "1 ms",
                "--spacing",
                "15 cm",
                "--approach-speed",
                "3 cm/s",
                "--initial-height",
                "50 mm",
                "--orientation",
                "0 deg",
            ],
        ),
    ],
)
def test_explicit_quantities_match_canonical_runtime_results(
    example_pack_dir: Path, command: str, canonical: list[str], readable: list[str]
) -> None:
    baseline = runner.invoke(app, [command, str(example_pack_dir), *canonical, "--output", "json"])
    converted = runner.invoke(app, [command, str(example_pack_dir), *readable, "--output", "json"])
    assert baseline.exit_code == 0, baseline.output
    assert converted.exit_code == 0, converted.output
    assert json.loads(converted.stdout) == json.loads(baseline.stdout)


@pytest.mark.parametrize(
    "arguments",
    [
        ["--duration", "5 mm"],
        ["--orientation", "5 cm"],
        ["--approach-speed", "90 deg/s"],
        ["--gap", "nan m"],
        ["--timestep", "1e999 ms"],
    ],
)
def test_incompatible_or_nonfinite_inputs_fail_before_loading(
    arguments: list[str], isolated_preferences: Path
) -> None:
    result = runner.invoke(app, ["run", "./missing-pack", *arguments])
    assert result.exit_code == 2
    assert "finite" in result.output and "compatible quantity" in result.output
    assert not isolated_preferences.exists()


@pytest.mark.parametrize(
    "arguments",
    [
        ["--dt", "0.01", "--timestep", "1 ms"],
        ["--height=0.1", "--initial-height=50mm"],
        ["--connector-gap", "0.01", "--gap", "5 mm"],
        ["--approach", "0.01", "--approach-speed", "5 mm/s"],
        ["--spacing", "0.1", "-s", "20 cm"],
    ],
)
def test_duplicate_physical_aliases_are_rejected(arguments: list[str]) -> None:
    result = runner.invoke(app, ["run", "./missing-pack", *arguments])
    assert result.exit_code == 2
    assert "only once" in result.output


def test_json_remains_si_regardless_of_saved_or_explicit_display_choices(
    example_pack_dir: Path, isolated_preferences: Path
) -> None:
    arguments = ["dock", str(example_pack_dir), "--output", "json"]
    baseline = runner.invoke(app, arguments)
    save_unit_preferences(UnitPreferences(length_unit="cm", angle_unit="rad", time_unit="ms"))
    converted = runner.invoke(
        app, [*arguments, "--length-unit", "mm", "--angle-unit", "deg", "--time-unit", "ms"]
    )
    assert baseline.exit_code == converted.exit_code == 0
    assert json.loads(baseline.stdout) == json.loads(converted.stdout)
    payload = json.loads(converted.stdout)
    assert payload["module_positions"]["generic_cube_1"][0] == pytest.approx(0.1)
    assert isolated_preferences.is_dir()


def test_json_does_not_depend_on_a_broken_local_display_file(
    example_pack_dir: Path, isolated_preferences: Path
) -> None:
    isolated_preferences.mkdir()
    (isolated_preferences / "units.toml").write_text("invalid [", encoding="utf-8")
    result = runner.invoke(app, ["dock", str(example_pack_dir), "--output", "json"])
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["pack"] == "generic_cube@0.1.0"


def test_human_view_output_formats_positions_and_rpy_in_selected_units(
    example_pack_dir: Path,
) -> None:
    result = runner.invoke(
        app,
        [
            "views",
            str(example_pack_dir),
            "--view",
            "module_topology",
            "--length-unit",
            "mm",
            "--angle-unit",
            "deg",
            "--time-unit",
            "ms",
        ],
    )
    assert result.exit_code == 0, result.output
    assert "World position (mm)" in result.stdout
    assert "World RPY (deg)" in result.stdout
    assert "(100, 0, 0)" in result.stdout


@pytest.mark.parametrize("command", [("runtime",), ("run", "--gui")])
def test_runtime_receives_si_values_and_ephemeral_display_profile(
    example_pack_dir: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    command: tuple[str, ...],
) -> None:
    runtime = ModuleType("modsim_robotics_studio.runtime_app")
    received: list[tuple[RuntimeInspectorConfig, Path | None, UnitPreferences | None]] = []

    def launch(
        config: RuntimeInspectorConfig,
        *,
        home: Path | None = None,
        preferences: UnitPreferences | None = None,
    ) -> int:
        received.append((config, home, preferences))
        return 0

    runtime.__dict__["main"] = launch
    monkeypatch.setitem(sys.modules, runtime.__name__, runtime)
    home = tmp_path / "selected-home"
    result = runner.invoke(
        app,
        [
            *command,
            str(example_pack_dir),
            "--backend",
            "mock",
            "--no-viewer",
            "--home",
            str(home),
            "--duration",
            "500 ms",
            "--timestep",
            "1 ms",
            "--orientation",
            "90 deg",
            "--gap",
            "5 mm",
            "--approach-speed",
            "20 mm/s",
            "--retract-speed",
            "1 cm/s",
            "--undock-at",
            "200 ms",
            "--initial-height",
            "50 mm",
            "--publish-hz",
            "0.02 kHz",
            "--length-unit",
            "cm",
            "--angle-unit",
            "rad",
            "--time-unit",
            "ms",
        ],
    )
    assert result.exit_code == 0, result.output
    config, actual_home, profile = received[0]
    assert actual_home == home
    assert profile is not None and profile.length_unit == "cm" and profile.angle_unit == "rad"
    assert profile.time_unit == "ms"
    assert config.duration_s == 0.5
    assert config.dt_s == 0.001
    assert config.connector_gap_m == 0.005
    assert config.orientation_rad == pytest.approx(math.pi / 2)
    assert config.approach_m_s == 0.02
    assert config.retract_m_s == 0.01
    assert config.undock_at_s == 0.2
    assert config.height_m == 0.05
    assert config.publish_hz == 20.0
    assert not home.exists()


def test_units_show_lists_choices_without_initializing_home(isolated_preferences: Path) -> None:
    result = runner.invoke(app, ["units", "show", "--supported", "--output", "json"])
    assert result.exit_code == 0, result.output
    payload = json.loads(result.stdout)
    assert payload["effective"]["length"] == "mm"
    assert "cm" in payload["supported"]["length"]
    assert "deg" in payload["supported"]["angle"]
    assert not isolated_preferences.exists()


def test_units_set_merges_saves_and_resets_local_choices(isolated_preferences: Path) -> None:
    result = runner.invoke(
        app,
        [
            "units",
            "set",
            "--length-unit",
            "cm",
            "--angle-unit",
            "rad",
            "--time-unit",
            "ms",
            "--torque-unit",
            "N*mm",
            "--output",
            "json",
        ],
    )
    assert result.exit_code == 0, result.output
    profile = load_unit_preferences()
    assert profile.length_unit == "cm" and profile.angle_unit == "rad"
    assert profile.time_unit == "ms" and profile.torque_unit == "N*mm"
    assert (isolated_preferences / "units.toml").is_file()
    changed = runner.invoke(app, ["units", "set", "--mass-unit", "g"])
    assert changed.exit_code == 0
    assert load_unit_preferences().length_unit == "cm"
    assert load_unit_preferences().mass_unit == "g"
    reset = runner.invoke(app, ["units", "set", "--reset"])
    assert reset.exit_code == 0
    assert load_unit_preferences() == UnitPreferences()


def test_invalid_preference_choices_leave_existing_file_unchanged(
    isolated_preferences: Path,
) -> None:
    save_unit_preferences(UnitPreferences())
    before = (isolated_preferences / "units.toml").read_bytes()
    result = runner.invoke(app, ["units", "set", "--length-unit", "deg"])
    assert result.exit_code == 2
    assert (isolated_preferences / "units.toml").read_bytes() == before


def test_speed_override_can_return_to_derivation_without_resetting_other_choices() -> None:
    saved = runner.invoke(
        app, ["units", "set", "--length-unit", "cm", "--linear-velocity-unit", "m/s"]
    )
    assert saved.exit_code == 0
    assert load_unit_preferences().unit_for(QuantityKind.LINEAR_VELOCITY) == "m/s"
    restored = runner.invoke(app, ["units", "set", "--linear-velocity-unit", "auto"])
    assert restored.exit_code == 0
    profile = load_unit_preferences()
    assert profile.length_unit == "cm"
    assert profile.linear_velocity_unit is None
    assert profile.unit_for(QuantityKind.LINEAR_VELOCITY) == "cm/s"


def test_help_does_not_import_studio_or_create_preferences(isolated_preferences: Path) -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "from typer.testing import CliRunner; "
            "from modsim_robotics.cli import app; import sys; "
            "assert CliRunner().invoke(app,['units','set','--help']).exit_code == 0; "
            "assert CliRunner().invoke(app,['run','--help']).exit_code == 0; "
            "assert not any(name.startswith(('PySide6','pyvista','modsim_robotics_studio')) "
            "for name in sys.modules)",
        ],
        capture_output=True,
        text=True,
        timeout=20,
        env=dict(os.environ),
    )
    assert result.returncode == 0, result.stderr
    assert not isolated_preferences.exists()
