# pyright: reportPrivateUsage=false
"""Connector pose authoring converts at the UI boundary without moving geometry."""

from __future__ import annotations

import math
import os
from collections.abc import Iterator
from dataclasses import replace
from pathlib import Path
from typing import cast

import pytest
from pydantic import JsonValue

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyvistaqt")

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QLineEdit,
    QPushButton,
    QWidget,
)

from modsim_robotics.core.transforms import quat_from_rpy
from modsim_robotics.robot_packs import JointSpec, JointType
from modsim_robotics.unit_settings import load_unit_preferences
from modsim_robotics.units import QuantityKind, UnitPreferences
from modsim_robotics_studio import display_units_dialog, main_window
from modsim_robotics_studio.quantity_widgets import (
    OptionalQuantityEdit,
    QuantityListEdit,
    QuantityVectorEdit,
)
from modsim_robotics_studio.unit_preferences import UnitManager, units_manager


class _ViewportStub(QWidget):
    entity_selected = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.render_count = 0

    def render_module(self, *_args: object) -> None:
        self.render_count += 1

    def set_layer_visibility(self, **_kwargs: bool) -> None:
        pass

    def select_entity(self, *_args: str) -> None:
        pass

    def fit_module(self) -> None:
        pass


@pytest.fixture(scope="module")
def application() -> QApplication:
    current = QApplication.instance()
    return current if isinstance(current, QApplication) else QApplication([])


@pytest.fixture
def builder(
    application: QApplication, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, copied_pack: Path
) -> Iterator[main_window.MainWindow]:
    manager = UnitManager(application, home=tmp_path / "home", preferences=UnitPreferences())

    def manager_for_home(_home: str | Path | None = None) -> UnitManager:
        return manager

    monkeypatch.setattr(main_window, "units_manager", manager_for_home)
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)
    window = main_window.MainWindow(initial_path=copied_pack)
    window.show()
    application.processEvents()
    yield window
    window.project = None
    window.close()
    window.deleteLater()
    manager.deleteLater()
    application.processEvents()


def _select_connector(window: main_window.MainWindow) -> tuple[str, str]:
    assert window.project is not None
    module = next(iter(window.project.pack.hardware_catalog.module_types.values()))
    connector = module.connectors[0]
    assert window._select_tree_entity(("connector", connector.id, module.id))
    return module.id, connector.id


def test_connector_apply_converts_mm_and_degrees_to_persisted_si(
    builder: main_window.MainWindow,
) -> None:
    module_id, connector_id = _select_connector(builder)
    xyz, rpy = builder.properties.findChildren(QuantityVectorEdit)
    xyz.fields[0].setText("50 mm")
    rpy.fields[2].setText("90 deg")
    apply = next(
        button
        for button in builder.properties.findChildren(QPushButton)
        if button.text() == "Apply connector"
    )
    apply.click()
    assert builder.project is not None and builder.project.dirty
    updated = next(
        connector
        for connector in builder.project.pack.hardware_catalog.module_types[module_id].connectors
        if connector.id == connector_id
    )
    assert updated.local_pose is not None
    assert updated.local_pose.xyz_m[0] == 0.05
    assert updated.local_pose.rpy_rad[2] == pytest.approx(math.pi / 2)
    assert set(updated.local_pose.model_dump()) == {"xyz_m", "rpy_rad"}


def test_unit_switch_keeps_pending_edits_project_selection_and_render(
    builder: main_window.MainWindow,
) -> None:
    _select_connector(builder)
    project = builder.project
    selection = builder._current_selection
    viewport = cast(_ViewportStub, builder.viewport)
    rendered = viewport.render_count
    xyz, rpy = builder.properties.findChildren(QuantityVectorEdit)
    xyz.fields[0].setText("50")
    rpy.fields[2].setText("90")
    for length, angle in (("cm", "rad"), ("m", "deg"), ("mm", "deg")):
        builder._units.set_preferences(UnitPreferences(length, angle), persist=False)
        assert xyz.value_si()[0] == 0.05
        assert rpy.value_si()[2] == pytest.approx(math.pi / 2)
        assert builder.project is project and project is not None and not project.dirty
        assert builder._current_selection == selection
        assert viewport.render_count == rendered
    assert "mm" in xyz.fields[0].text()


def test_invalid_pending_pose_is_preserved_in_its_original_units(
    builder: main_window.MainWindow,
) -> None:
    _select_connector(builder)
    xyz, _rpy = builder.properties.findChildren(QuantityVectorEdit)
    xyz.fields[0].setText("unfinished")
    project = builder.project
    builder._units.set_preferences(UnitPreferences("cm", "rad"), persist=False)
    assert xyz.fields[0].text() == "unfinished"
    assert xyz.unit == "mm"
    assert "previous units" in builder.statusBar().currentMessage()
    assert builder.project is project and project is not None and not project.dirty


def test_display_units_menu_persists_preferences(
    builder: main_window.MainWindow, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    def accept_units(dialog: QDialog) -> int:
        length = dialog.findChild(QComboBox, "length_unit")
        angle = dialog.findChild(QComboBox, "angle_unit")
        assert length is not None and angle is not None
        length.setCurrentIndex(length.findData("cm"))
        angle.setCurrentIndex(angle.findData("rad"))
        return int(QDialog.DialogCode.Accepted)

    monkeypatch.setattr(display_units_dialog.QDialog, "exec", accept_units)
    builder.units_action.trigger()
    assert load_unit_preferences(tmp_path / "home") == UnitPreferences("cm", "rad")
    assert builder.units_status.text() == "Units: cm · rad · s"


def test_add_connector_dialog_uses_the_same_quantity_controls(
    builder: main_window.MainWindow, monkeypatch: pytest.MonkeyPatch
) -> None:
    assert builder.project is not None
    module = next(iter(builder.project.pack.hardware_catalog.module_types.values()))

    def accept_connector(dialog: main_window._FormDialog) -> int:
        identifier = next(
            field
            for field in dialog.findChildren(QLineEdit)
            if field.placeholderText() == "for example: front_face"
        )
        identifier.setText("units_test")
        xyz, rpy = dialog.findChildren(QuantityVectorEdit)
        xyz.fields[2].setText("2.5 cm")
        rpy.fields[0].setText("180 deg")
        return int(QDialog.DialogCode.Accepted)

    monkeypatch.setattr(main_window._FormDialog, "exec", accept_connector)
    builder._add_connector_dialog(module.id, module.root_link)
    assert builder.project is not None
    connector = next(
        item
        for item in builder.project.pack.hardware_catalog.module_types[module.id].connectors
        if item.id == "units_test"
    )
    assert connector.local_pose is not None
    assert connector.local_pose.xyz_m[2] == 0.025
    assert connector.local_pose.rpy_rad[0] == pytest.approx(math.pi)


def test_application_shares_managers_only_for_the_same_data_home(
    application: QApplication, tmp_path: Path
) -> None:
    first = units_manager(tmp_path / "first-home")
    second = units_manager(tmp_path / "second-home")
    try:
        assert units_manager(tmp_path / "first-home") is first
        assert first is not second
        first.set_preferences(UnitPreferences("cm", "rad"), persist=False)
        assert second.preferences == UnitPreferences()
        assert not (tmp_path / "first-home").exists()
        assert not (tmp_path / "second-home").exists()
    finally:
        first.deleteLater()
        second.deleteLater()


def test_module_mass_authoring_respects_mass_preference(builder: main_window.MainWindow) -> None:
    assert builder.project is not None
    module = next(iter(builder.project.pack.hardware_catalog.module_types.values()))
    builder._units.set_preferences(UnitPreferences(mass_unit="g"), persist=False)
    builder._show_module_properties(module.id)
    field = builder.properties.findChild(OptionalQuantityEdit, "mass_kg")
    assert field is not None and field.unit == "g"
    field.setText("750 g")
    next(
        button
        for button in builder.properties.findChildren(QPushButton)
        if button.text() == "Apply module metadata"
    ).click()
    assert builder.project.pack.hardware_catalog.module_types[module.id].mass_kg == 0.75


def test_connector_rules_convert_lists_tolerances_loads_and_policy(
    builder: main_window.MainWindow,
) -> None:
    assert builder.project is not None
    type_id = next(iter(builder.project.pack.hardware_catalog.connector_types))
    builder._show_connector_type_properties(type_id)
    project = builder.project
    fields = {
        field.objectName(): field for field in builder.properties.findChildren(OptionalQuantityEdit)
    }
    fields["position_tolerance_m"].setText("6 mm")
    fields["orientation_tolerance_rad"].setText("8 deg")
    fields["max_relative_velocity_m_s"].setText("50 mm/s")
    fields["max_bending_moment_nm"].setText("1800 N*mm")
    fields["redock_cooldown_s"].setText("250 ms")
    fields["break_force_n"].setText("20 N")
    values = builder.properties.findChild(QuantityListEdit, "values_rad")
    assert values is not None
    values.setText("0 deg, 90 deg, 180 deg")
    mode = builder.properties.findChild(QComboBox, "orientation_mode")
    assert mode is not None
    mode.setCurrentText("discrete")
    declare = builder.properties.findChild(QCheckBox, "declare_policy")
    assert declare is not None
    declare.setChecked(True)
    builder._units.set_preferences(
        replace(builder._units.preferences, length_unit="cm", angle_unit="rad", time_unit="ms"),
        persist=False,
    )
    assert builder.project is project and not project.dirty
    assert fields["position_tolerance_m"].unit == "cm"
    assert fields["max_relative_velocity_m_s"].unit == "cm/s"
    next(
        button
        for button in builder.properties.findChildren(QPushButton)
        if button.text() == "Apply connector type"
    ).click()
    assert builder.project is not None
    connector_type = builder.project.pack.hardware_catalog.connector_types[type_id]
    assert connector_type.allowed_orientations is not None
    assert connector_type.allowed_orientations.values_rad == pytest.approx(
        (0, math.pi / 2, math.pi)
    )
    assert connector_type.acceptance_region is not None
    assert connector_type.acceptance_region.position_tolerance_m == 0.006
    assert connector_type.acceptance_region.orientation_tolerance_rad == pytest.approx(
        math.radians(8)
    )
    assert connector_type.acceptance_region.max_relative_velocity_m_s == 0.05
    assert connector_type.limits is not None
    assert connector_type.limits.max_bending_moment_nm == 1.8
    assert connector_type.docking_policy is not None
    assert connector_type.docking_policy.redock_cooldown_s == 0.25
    assert connector_type.docking_policy.break_force_n == 20


@pytest.mark.parametrize(
    (
        "joint_type",
        "position_field",
        "position_input",
        "position_si",
        "effort_field",
        "effort_kind",
    ),
    [
        (
            JointType.REVOLUTE,
            "lower_position_rad",
            "-90 deg",
            -math.pi / 2,
            "max_effort_nm",
            QuantityKind.TORQUE,
        ),
        (
            JointType.PRISMATIC,
            "lower_position_m",
            "-25 mm",
            -0.025,
            "max_effort_n",
            QuantityKind.FORCE,
        ),
    ],
)
def test_joint_limit_quantity_depends_on_joint_type(
    builder: main_window.MainWindow,
    joint_type: JointType,
    position_field: str,
    position_input: str,
    position_si: float,
    effort_field: str,
    effort_kind: QuantityKind,
) -> None:
    project = builder.project
    assert project is not None
    module = next(iter(project.pack.hardware_catalog.module_types.values()))
    joint = JointSpec(
        id="units_joint",
        source_joint_name="units_joint",
        type=joint_type,
        parent_link=module.root_link,
        child_link="test_child",
        axis=(0.0, 0.0, 1.0),
    )
    module = module.model_copy(update={"joints": (joint,)})
    hardware = project.pack.hardware_catalog.model_copy(
        update={"module_types": {module.id: module}}
    )
    pack = project.pack.model_copy(update={"hardware_catalog": hardware})
    builder._set_project(replace(project, loaded=replace(project.loaded, pack=pack)))
    builder._show_joint_properties(joint.id, module.id)
    field = builder.properties.findChild(OptionalQuantityEdit, position_field)
    effort = builder.properties.findChild(OptionalQuantityEdit, effort_field)
    assert field is not None and effort is not None
    assert effort.kind == effort_kind
    field.setText(position_input)
    effort.setText("500 N*mm" if joint_type is JointType.REVOLUTE else "5 N")
    velocity_name = (
        "max_velocity_rad_per_s" if joint_type is JointType.REVOLUTE else "max_velocity_m_per_s"
    )
    velocity = builder.properties.findChild(OptionalQuantityEdit, velocity_name)
    assert velocity is not None
    velocity.setText("60 rpm" if joint_type is JointType.REVOLUTE else "100 mm/s")
    next(
        button
        for button in builder.properties.findChildren(QPushButton)
        if button.text() == "Apply joint metadata"
    ).click()
    assert builder.project is not None
    limits = builder.project.pack.hardware_catalog.module_types[module.id].joints[0].limits
    assert limits is not None
    assert getattr(limits, position_field) == pytest.approx(position_si)
    assert getattr(limits, effort_field) == (0.5 if joint_type is JointType.REVOLUTE else 5.0)
    assert getattr(limits, velocity_name) == pytest.approx(
        math.tau if joint_type is JointType.REVOLUTE else 0.1
    )


def test_lattice_configuration_edits_known_quantities_and_preserves_quaternion(
    application: QApplication,
) -> None:
    quaternion = quat_from_rpy((0.5, math.pi / 2, -0.3))
    data: dict[str, JsonValue] = {
        "pitch_m": 0.05,
        "position_tolerance_m": 0.002,
        "orientation_tolerance_rad": math.radians(3),
        "origin_world_m": [0.0, 0.0, 0.0],
        "orientation_world_wxyz": list(quaternion),
    }
    editor = main_window._CubicLatticeFields(data, UnitPreferences())
    assert editor.value()["orientation_world_wxyz"] == list(quaternion)
    editor.scalars["pitch_m"].setText("60 mm")
    editor.origin.fields[0].setText("2 cm")
    editor.orientation.fields[0].setText("15 deg")
    result = editor.value()
    assert result["pitch_m"] == 0.06
    assert result["origin_world_m"] == [0.02, 0.0, 0.0]
    assert result["orientation_world_wxyz"] != list(quaternion)
    expected = quat_from_rpy(editor.orientation.value_si())
    assert result["orientation_world_wxyz"] == list(expected)


def test_custom_builder_json_is_not_guessed_or_converted(application: QApplication) -> None:
    data: dict[str, JsonValue] = {"custom_threshold": 250, "unknown_pose": [90, 0, 0]}
    editor = main_window._ModelViewConfigurationEditor(data, "custom_builder", UnitPreferences)
    assert editor.value() == data
    assert not editor.findChildren(OptionalQuantityEdit)


@pytest.mark.parametrize("constraint_kind", ["compliant", "hinge"])
def test_connection_parameters_map_stiffness_and_separation_correctly(
    builder: main_window.MainWindow, constraint_kind: str
) -> None:
    assert builder.project is not None
    type_id = next(iter(builder.project.pack.hardware_catalog.connector_types))
    builder._show_connector_type_properties(type_id)
    constraint = builder.properties.findChild(QComboBox, "constraint")
    assert constraint is not None
    constraint.setCurrentText(constraint_kind)
    fields = {
        field.objectName(): field for field in builder.properties.findChildren(OptionalQuantityEdit)
    }
    if constraint_kind == "compliant":
        fields["translational_stiffness_n_per_m"].setText("1 N/mm")
        fields["rotational_stiffness_nm_per_rad"].setText("2 N*m/deg")
    else:
        axis = builder.properties.findChild(QLineEdit, "hinge_axis")
        assert axis is not None
        axis.setText("0, 0, 1")
        fields["anchor_separation_m"].setText("40 mm")
    next(
        button
        for button in builder.properties.findChildren(QPushButton)
        if button.text() == "Apply connector type"
    ).click()
    connection = builder.project.pack.hardware_catalog.connector_types[type_id].physical_connection
    assert connection is not None
    if constraint_kind == "compliant":
        assert connection.compliance is not None
        assert connection.compliance.translational_stiffness_n_per_m == 1000
        assert connection.compliance.rotational_stiffness_nm_per_rad == pytest.approx(360 / math.pi)
    else:
        assert connection.hinge is not None
        assert connection.hinge.anchor_separation_m == 0.04
        assert connection.hinge.axis == (0.0, 0.0, 1.0)


def test_lattice_raw_toggle_retains_unfinished_quantity_draft(application: QApplication) -> None:
    data: dict[str, JsonValue] = {
        "pitch_m": 0.05,
        "position_tolerance_m": 0.002,
        "orientation_tolerance_rad": math.radians(3),
    }
    editor = main_window._ModelViewConfigurationEditor(data, "cubic_lattice", UnitPreferences)
    field = editor.findChild(OptionalQuantityEdit, "pitch_m")
    assert field is not None
    field.setText("unfinished")
    editor._mode_button.click()
    assert editor.value() == data
    editor._mode_button.click()
    assert field.text() == "unfinished"
    with pytest.raises(ValueError):
        editor.value()
