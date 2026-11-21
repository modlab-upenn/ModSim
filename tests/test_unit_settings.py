"""Local unit choices persist independently of simulation and pack data."""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim_robotics import unit_settings
from modsim_robotics.unit_settings import (
    UnitPreferencesError,
    load_unit_preferences,
    save_unit_preferences,
)
from modsim_robotics.units import UnitPreferences


def test_reading_missing_preferences_does_not_initialize_home(tmp_path: Path) -> None:
    home = tmp_path / "not-created"
    assert load_unit_preferences(home) == UnitPreferences()
    assert not home.exists()


def test_explicit_save_round_trips_and_preserves_library_configuration(tmp_path: Path) -> None:
    config = tmp_path / "config.toml"
    original = b"# keep my library location\nschema_version = 1\n[paths]\nrobot_packs = 'custom'\n"
    config.write_bytes(original)
    preferences = UnitPreferences(length_unit="cm", angle_unit="rad")
    save_unit_preferences(preferences, tmp_path)
    assert load_unit_preferences(tmp_path) == preferences
    assert config.read_bytes() == original
    assert sorted(path.name for path in tmp_path.iterdir()) == ["config.toml", "units.toml"]


def test_preferences_follow_modsim_home_environment(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("MODSIM_PATH", str(tmp_path / "selected"))
    save_unit_preferences(UnitPreferences(length_unit="m"))
    assert load_unit_preferences().length_unit == "m"
    assert (tmp_path / "selected" / "units.toml").is_file()


@pytest.mark.parametrize(
    "text",
    [
        "broken TOML [",
        "schema_version = 2\n[units]\nlength = 'mm'\n",
        "schema_version = true\n[units]\nlength = 'mm'\n",
        "schema_version = 1\nunits = 'mm'\n",
        "schema_version = 1\n[units]\nlength = 10\n",
        "schema_version = 1\n[units]\nlength = 'radians'\n",
        "schema_version = 1\n[units]\nangle = 'mm'\n",
    ],
)
def test_invalid_preferences_are_reported_without_rewriting(tmp_path: Path, text: str) -> None:
    path = tmp_path / "units.toml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(UnitPreferencesError):
        load_unit_preferences(tmp_path)
    assert path.read_text(encoding="utf-8") == text


def test_preferences_refuse_symbolic_link_file(tmp_path: Path) -> None:
    target = tmp_path / "untouched.toml"
    target.write_text("not preferences", encoding="utf-8")
    path = tmp_path / "units.toml"
    try:
        path.symlink_to(target)
    except OSError:
        pytest.skip("Creating symbolic links requires platform privileges")
    with pytest.raises(UnitPreferencesError, match="symbolic-link"):
        load_unit_preferences(tmp_path)
    with pytest.raises(UnitPreferencesError, match="symbolic-link"):
        save_unit_preferences(UnitPreferences(), tmp_path)
    assert target.read_text(encoding="utf-8") == "not preferences"


def test_failed_atomic_replace_keeps_previous_preferences(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = UnitPreferences(length_unit="m", angle_unit="rad")
    save_unit_preferences(original, tmp_path)
    before = (tmp_path / "units.toml").read_bytes()

    def fail_replace(_source: Path, _target: Path) -> None:
        raise OSError("simulated write failure")

    monkeypatch.setattr(unit_settings.os, "replace", fail_replace)
    with pytest.raises(UnitPreferencesError, match="write failure"):
        save_unit_preferences(UnitPreferences(), tmp_path)
    assert (tmp_path / "units.toml").read_bytes() == before
    assert list(tmp_path.glob(".units-*")) == []
