"""Share display preferences across Studio controls without changing documents."""

from __future__ import annotations

import logging
from pathlib import Path

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from modsim_robotics.unit_settings import (
    UnitPreferencesError,
    load_unit_preferences,
    save_unit_preferences,
)
from modsim_robotics.units import UnitPreferences
from modsim_robotics.user_data import resolve_user_home

_LOGGER = logging.getLogger("modsim_robotics.studio.units")


class UnitManager(QObject):
    """Notify editors about local display choices, retaining canonical SI values.

    Args:
        parent: QObject owning this manager, normally the application.
        home: Selected ModSim data home for units.toml.
        preferences: Explicit initial choices; None restores local preferences.
    """

    changed = Signal()

    def __init__(
        self,
        parent: QObject | None = None,
        *,
        home: str | Path | None = None,
        preferences: UnitPreferences | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("studioUnits")
        self.home = resolve_user_home(home)
        if preferences is None:
            try:
                preferences = load_unit_preferences(self.home)
            except UnitPreferencesError as error:
                _LOGGER.warning("%s; using default display units for this session", error)
                preferences = UnitPreferences()
        self.preferences = preferences

    def set_preferences(self, preferences: UnitPreferences, *, persist: bool = True) -> None:
        """Apply validated choices after optionally saving them atomically.

        Args:
            preferences: New local display choices.
            persist: Save to units.toml when True. False changes this session only.

        Raises:
            UnitPreferencesError: Saving fails; the active choices remain unchanged.
        """
        if persist:
            save_unit_preferences(preferences, self.home)
        if preferences == self.preferences:
            return
        self.preferences = preferences
        self.changed.emit()


def units_manager(home: str | Path | None = None) -> UnitManager:
    """Return the application-owned display-unit manager for a data home.

    Args:
        home: Explicit ModSim data home, or None for environment/default lookup.

    Returns:
        Shared manager for Studio windows using the same resolved data home.

    Raises:
        RuntimeError: No QApplication exists.
    """
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        raise RuntimeError("Create a QApplication before requesting display preferences.")
    root = resolve_user_home(home)
    for manager in application.findChildren(UnitManager, "studioUnits"):
        if manager.home == root:
            return manager
    return UnitManager(application, home=root)
