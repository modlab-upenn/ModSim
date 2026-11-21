# pyright: reportPrivateUsage=false
"""Runtime unit choices preserve canonical launch, plot, and event records."""

from __future__ import annotations

import math
import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtCore import Qt, QTimer
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QComboBox, QDialog, QPushButton, QVBoxLayout, QWidget

from modsim_robotics.runtime.inspection import RuntimeEventRow
from modsim_robotics.units import QuantityKind, UnitError, UnitPreferences
from modsim_robotics_studio.display_units_dialog import show_display_units_dialog
from modsim_robotics_studio.project import StudioProject
from modsim_robotics_studio.runtime_events import RuntimeEventTableModel
from modsim_robotics_studio.runtime_planning import ActionHistory, PlanningWorkspace
from modsim_robotics_studio.runtime_quantity import QuantityAxis, QuantitySpinBox
from modsim_robotics_studio.runtime_workspace import RuntimeWorkspace
from modsim_robotics_studio.unit_preferences import UnitManager


@pytest.fixture(scope="module")
def application() -> QApplication:
    current = QApplication.instance()
    return current if isinstance(current, QApplication) else QApplication([])


def test_spinbox_keeps_canonical_range_and_value_when_display_changes(
    application: QApplication,
) -> None:
    field = QuantitySpinBox(kind=QuantityKind.LENGTH, unit="mm")
    field.setRange(-1.0, 1.0)
    field.setValue(0.125)
    assert field.value() == field.value_si() == 0.125
    assert field.text() == "125 mm"
    for unit in ("cm", "m", "mm"):
        assert field.set_unit(unit)
        assert field.value() == field.value_si() == 0.125
        assert field.minimum() == -1.0 and field.maximum() == 1.0


def test_spinbox_accepts_explicit_suffix_and_retains_invalid_pending_input(
    application: QApplication,
) -> None:
    field = QuantitySpinBox(kind=QuantityKind.ANGLE, unit="deg")
    field.setRange(-1000, 1000)
    field.lineEdit().setText("1.5707963267948966 rad")
    assert field.value_si() == pytest.approx(math.pi / 2)
    assert field.set_unit("rad")
    assert field.value_si() == pytest.approx(math.pi / 2)
    field.lineEdit().setText("unfinished")
    assert not field.set_unit("deg")
    assert field.unit == "rad" and field.lineEdit().text() == "unfinished"
    with pytest.raises(UnitError):
        field.value_si()


def test_spinbox_preserves_sub_decimal_timestep_precision(application: QApplication) -> None:
    field = QuantitySpinBox(kind=QuantityKind.TIME, unit="s")
    field.setRange(1e-9, 1.0)
    value = 1 / 240
    field.setValue(value)
    assert field.value_si() == value
    for unit in ("ms", "s") * 20:
        assert field.set_unit(unit)
        assert field.value() == field.value_si() == value
        field.interpretText()
        assert field.value() == value
    field.setStyleSheet("QDoubleSpinBox { padding: 3px; }")
    application.processEvents()
    field.interpretText()
    assert field.value() == value
    assert len(field.text()) < 25


@pytest.mark.parametrize("draft", ["1e-", "unfinished"])
def test_native_focus_change_and_enter_preserve_invalid_spinbox_draft(
    application: QApplication, draft: str
) -> None:
    host = QWidget()
    layout = QVBoxLayout(host)
    field = QuantitySpinBox(kind=QuantityKind.ANGLE, unit="deg")
    field.setRange(-1000, 1000)
    other = QPushButton("Display units")
    layout.addWidget(field)
    layout.addWidget(other)
    host.show()
    try:
        field.setFocus()
        application.processEvents()
        field.lineEdit().setText(draft)
        QTest.keyClick(field, Qt.Key.Key_Return)
        assert field.text() == draft
        other.setFocus()
        application.processEvents()
        assert field.text() == draft
        assert not field.set_unit("rad")
        assert field.unit == "deg" and field.text() == draft
        with pytest.raises(UnitError):
            field.value_si()
    finally:
        host.close()


def test_native_focus_change_commits_explicit_angle_once(application: QApplication) -> None:
    host = QWidget()
    layout = QVBoxLayout(host)
    field = QuantitySpinBox(kind=QuantityKind.ANGLE, unit="deg")
    field.setRange(-1000, 1000)
    other = QPushButton("Next")
    layout.addWidget(field)
    layout.addWidget(other)
    host.show()
    try:
        field.setFocus()
        application.processEvents()
        field.lineEdit().setText("1.5707963267948966 rad")
        other.setFocus()
        application.processEvents()
        assert field.value_si() == pytest.approx(math.pi / 2)
        assert field.text() == "90 deg"
        field.set_unit("rad")
        assert field.value_si() == pytest.approx(math.pi / 2)
    finally:
        host.close()


def test_runtime_form_unit_changes_preserve_literal_launch_request(
    application: QApplication, tmp_path: Path, example_pack_dir: Path
) -> None:
    project = StudioProject.open(example_pack_dir)
    manager = UnitManager(application, home=tmp_path, preferences=UnitPreferences())
    workspace = RuntimeWorkspace(lambda: project, lambda: True, units=manager)
    try:
        workspace.duration.setValue(12.5)
        workspace.timestep.setValue(0.005)
        workspace.initial_height.setValue(0.125)
        workspace.orientation.setValue(math.pi / 2)
        workspace.approach.setValue(0.05)
        workspace.gap.setText("25 mm")
        workspace.undock_at.setText("2500 ms")
        workspace.retract.setText("2 cm/s")
        expected = workspace.launch_config()
        assert expected.connector_gap_m == 0.025
        assert expected.undock_at_s == 2.5
        assert expected.retract_m_s == 0.02
        before_readiness = workspace.readiness_label.text()
        for profile in (
            UnitPreferences("cm", "rad", time_unit="ms"),
            UnitPreferences("m", "deg", linear_velocity_unit="cm/s"),
            UnitPreferences(),
        ):
            manager.set_preferences(profile, persist=False)
            assert workspace.launch_config() == expected
            assert workspace.readiness_label.text() == before_readiness
        assert workspace.initial_height.text() == "125 mm"
        assert not project.dirty
        assert not (tmp_path / "units.toml").exists()
    finally:
        workspace.close()
        workspace.deleteLater()
        manager.deleteLater()


def test_axis_maps_ticks_without_changing_coordinate_scale(application: QApplication) -> None:
    axis = QuantityAxis("bottom", kind=QuantityKind.LENGTH, unit="mm")
    assert axis.tickStrings([0.0, 0.025, 0.1], 1.0, 0.025) == ["0", "25", "100"]
    axis.set_unit("cm")
    assert axis.tickStrings([0.0, 0.025, 0.1], 1.0, 0.025) == ["0", "2.5", "10"]
    assert axis.scale == 1.0


def test_planning_display_preferences_keep_manual_camera_ranges(
    application: QApplication,
) -> None:
    widget = PlanningWorkspace()
    history = ActionHistory()
    try:
        widget.workspace.setXRange(-0.3, 0.7, padding=0)
        widget.workspace.setYRange(-0.2, 0.8, padding=0)
        before = widget.workspace.viewRange()
        history.plot.setXRange(0.5, 2.5, padding=0)
        previous_history = history.plot.viewRange()[0]
        preferences = UnitPreferences("cm", "rad", time_unit="ms")
        widget.set_units(preferences)
        history.set_units(preferences)
        assert widget.workspace.viewRange() == before
        assert history.plot.viewRange()[0] == previous_history
        assert widget.x_axis.tickStrings([0.1], 1.0, 0.1) == ["10"]
        assert history.time_axis.tickStrings([0.1], 1.0, 0.1) == ["100"]
    finally:
        widget.close()
        history.close()


def test_event_display_converts_typed_force_and_time_without_changing_raw_record(
    application: QApplication,
) -> None:
    event = RuntimeEventRow(
        sequence=1,
        time_s=0.125,
        kind="ConnectorOverloaded",
        detail="canonical diagnostic",
        subject="alpha/front",
        measured_force_n=0.02,
        force_limit_n=0.01,
    )
    model = RuntimeEventTableModel()
    model.set_events((event,))
    model.set_units(UnitPreferences(time_unit="ms", force_unit="mN"))
    assert model.data(model.index(0, 1)) == "125"
    assert model.data(model.index(0, 3)) == "alpha/front: 20 mN > 10 mN"
    assert model.events == (event,)
    assert event.detail == "canonical diagnostic" and event.measured_force_n == 0.02


def test_shared_unit_dialog_persists_choices_for_all_runtime_quantities(
    application: QApplication, tmp_path: Path
) -> None:
    manager = UnitManager(application, home=tmp_path, preferences=UnitPreferences())

    def choose() -> None:
        dialog = application.activeModalWidget()
        assert isinstance(dialog, QDialog)
        for name, value in (("length_unit", "cm"), ("angle_unit", "rad"), ("time_unit", "ms")):
            picker = dialog.findChild(QComboBox, name)
            assert picker is not None
            picker.setCurrentIndex(picker.findData(value))
        dialog.accept()

    QTimer.singleShot(0, choose)
    assert show_display_units_dialog(QDialog(), manager)
    assert manager.preferences.length_unit == "cm"
    assert manager.preferences.angle_unit == "rad"
    assert manager.preferences.time_unit == "ms"
    assert (tmp_path / "units.toml").is_file()
    manager.deleteLater()
