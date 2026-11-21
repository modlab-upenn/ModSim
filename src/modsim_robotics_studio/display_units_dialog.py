"""Shared display-unit chooser for Studio and the standalone Inspector."""

from __future__ import annotations

from dataclasses import fields
from typing import cast

from PySide6.QtCore import QSize
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QMessageBox,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from modsim_robotics.units import QuantityKind, UnitPreferences, get_units
from modsim_robotics_studio._window_sizing import manage_window
from modsim_robotics_studio.unit_preferences import UnitManager

_KINDS = {
    "length_unit": QuantityKind.LENGTH,
    "angle_unit": QuantityKind.ANGLE,
    "time_unit": QuantityKind.TIME,
    "mass_unit": QuantityKind.MASS,
    "force_unit": QuantityKind.FORCE,
    "torque_unit": QuantityKind.TORQUE,
    "inertia_unit": QuantityKind.INERTIA,
    "linear_velocity_unit": QuantityKind.LINEAR_VELOCITY,
    "angular_velocity_unit": QuantityKind.ANGULAR_VELOCITY,
    "linear_acceleration_unit": QuantityKind.LINEAR_ACCELERATION,
    "linear_stiffness_unit": QuantityKind.LINEAR_STIFFNESS,
    "angular_stiffness_unit": QuantityKind.ANGULAR_STIFFNESS,
    "linear_damping_unit": QuantityKind.LINEAR_DAMPING,
    "angular_damping_unit": QuantityKind.ANGULAR_DAMPING,
    "frequency_unit": QuantityKind.FREQUENCY,
}


def show_display_units_dialog(parent: QWidget, manager: UnitManager) -> bool:
    """Choose shared local display units without editing a pack or simulation.

    Args:
        parent: Window owning the modal dialog.
        manager: Shared preferences for this ModSim home.

    Returns:
        Whether an explicit choice was saved and applied.
    """
    dialog = QDialog(parent)
    dialog.setWindowTitle("Display units")
    layout = QVBoxLayout(dialog)
    introduction = QLabel(
        "Choose units for authoring and inspection. Saved Robot Packs and runtime values "
        "remain canonical SI. Orientation is displayed as fixed-axis roll (X), pitch (Y), "
        "yaw (Z); runtime orientation remains a quaternion."
    )
    introduction.setWordWrap(True)
    layout.addWidget(introduction)
    content = QWidget()
    form = QFormLayout(content)
    form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
    choices: dict[str, QComboBox] = {}
    preferences = manager.preferences
    for field in fields(UnitPreferences):
        kind = _KINDS[field.name]
        picker = QComboBox()
        picker.setObjectName(field.name)
        picker.setAccessibleName(field.name.removesuffix("_unit").replace("_", " ").title())
        automatic = {
            "linear_velocity_unit": "Follow length unit (/s)",
            "angular_velocity_unit": "Follow angle unit (/s)",
            "linear_acceleration_unit": "Follow length unit (/s²)",
        }.get(field.name)
        if automatic is not None:
            picker.addItem(automatic, None)
        for definition in get_units(kind):
            picker.addItem(definition.symbol, definition.symbol)
        current = getattr(preferences, field.name)
        picker.setCurrentIndex(picker.findData(current))
        form.addRow(field.name.removesuffix("_unit").replace("_", " ").title(), picker)
        choices[field.name] = picker
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setWidget(content)
    layout.addWidget(scroll, 1)
    buttons = QDialogButtonBox(
        QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
    )
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout.addWidget(buttons)
    sizing = manage_window(dialog, "display-units", preferred=QSize(600, 700))
    try:
        while dialog.exec() == int(QDialog.DialogCode.Accepted):
            try:
                selected = {
                    name: cast(str | None, picker.currentData()) for name, picker in choices.items()
                }
                manager.set_preferences(UnitPreferences(**selected))  # type: ignore[arg-type]
            except ValueError as error:
                QMessageBox.warning(dialog, "Cannot save display units", str(error))
                continue
            return True
        return False
    finally:
        del sizing
        dialog.deleteLater()
