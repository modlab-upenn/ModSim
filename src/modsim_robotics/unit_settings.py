"""Persist local display-unit preferences without modifying Robot Packs.

Reading preferences is side-effect free. Only an explicit save creates
``units.toml`` in the selected ModSim data home; setup and the pack-library
configuration remain independent of display preferences.
"""

from __future__ import annotations

import os
import tempfile
import tomllib
from dataclasses import asdict, fields
from pathlib import Path
from typing import cast

from modsim_robotics.units import UnitPreferences
from modsim_robotics.user_data import resolve_user_home


class UnitPreferencesError(ValueError):
    """Report invalid or inaccessible local unit preferences."""


def load_unit_preferences(home: str | Path | None = None) -> UnitPreferences:
    """Read local display choices, returning defaults when no file exists.

    Args:
        home: ModSim data home. None follows MODSIM_PATH, legacy MODSIM_HOME,
            then ``~/.modsim``. Reading never initializes that directory.

    Returns:
        Validated display choices for all supported physical quantities. Legacy
        files containing only length/angle preferences retain the other defaults.

    Raises:
        UnitPreferencesError: The file is unsafe, unreadable, or invalid.
    """
    try:
        path = resolve_user_home(home) / "units.toml"
        if path.is_symlink():
            raise UnitPreferencesError(f"Refusing symbolic-link preferences: '{path}'.")
        if not path.exists():
            return UnitPreferences()
        with path.open("rb") as stream:
            settings: dict[str, object] = tomllib.load(stream)
        if type(settings.get("schema_version")) is not int or settings["schema_version"] != 1:
            raise UnitPreferencesError(f"Unsupported units schema in '{path}'.")
        units = settings.get("units")
        if not isinstance(units, dict):
            raise UnitPreferencesError(f"Preferences '{path}' require a [units] table.")
        choices = cast(dict[str, object], units)
        values: dict[str, str] = {}
        for field in fields(UnitPreferences):
            key = field.name.removesuffix("_unit")
            if key in choices:
                value = choices[key]
                if not isinstance(value, str):
                    raise UnitPreferencesError(f"Unit {key!r} in '{path}' must be a string.")
                values[field.name] = value
        return UnitPreferences(**values)
    except (OSError, ValueError, RuntimeError) as error:
        if isinstance(error, UnitPreferencesError):
            raise
        raise UnitPreferencesError(f"Cannot read unit preferences: {error}") from error


def save_unit_preferences(
    preferences: UnitPreferences,
    home: str | Path | None = None,
) -> None:
    """Atomically save an explicit display choice to local units.toml.

    Args:
        preferences: Validated local display choices. Automatic speed/acceleration
            preferences are omitted so they continue following length/angle choices.
        home: ModSim data home, resolved with the same rules as reading.

    Raises:
        UnitPreferencesError: The destination is unsafe or cannot be written.
    """
    staged: Path | None = None
    try:
        root = resolve_user_home(home)
        root.mkdir(parents=True, exist_ok=True)
        path = root / "units.toml"
        if path.is_symlink():
            raise UnitPreferencesError(f"Refusing symbolic-link preferences: '{path}'.")
        text = (
            "# Local display preferences; Robot Packs and runtime values remain SI.\n"
            "schema_version = 1\n\n[units]\n"
        )
        for name, value in asdict(preferences).items():
            if value is not None:
                text += f'{name.removesuffix("_unit")} = "{value}"\n'
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix=".units-", suffix=".toml", dir=root, delete=False
        ) as stream:
            staged = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, path)
        staged = None
    except (OSError, ValueError, RuntimeError) as error:
        if isinstance(error, UnitPreferencesError):
            raise
        raise UnitPreferencesError(f"Cannot save unit preferences: {error}") from error
    finally:
        if staged is not None:
            staged.unlink(missing_ok=True)
