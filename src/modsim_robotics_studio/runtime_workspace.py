"""Configure and run a Robot Pack inside Studio's Runtime workspace."""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from modsim_robotics.backends.registry import available_backends
from modsim_robotics.robot_packs import RobotPackLoader, ValidationReport
from modsim_robotics.runtime import RuntimeInspectorConfig
from modsim_robotics.runtime.demos import RuntimeDemo
from modsim_robotics.units import QuantityKind, UnitError
from modsim_robotics_studio.chrome import DetailsSection, property_group
from modsim_robotics_studio.project import StudioProject
from modsim_robotics_studio.quantity_widgets import OptionalQuantityEdit
from modsim_robotics_studio.runtime_launch import (
    add_default_topology_view,
    runtime_config_defaults,
    runtime_recipes,
    validate_runtime_launch,
)
from modsim_robotics_studio.runtime_quantity import QuantitySpinBox
from modsim_robotics_studio.unit_preferences import UnitManager, units_manager

if TYPE_CHECKING:
    from modsim_robotics_studio.runtime_window import RuntimeInspectorWindow

_LOGGER = logging.getLogger("modsim_robotics.studio.runtime_workspace")


def _choice(name: str) -> QComboBox:
    field = QComboBox()
    field.setAccessibleName(name)
    field.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
    field.setMinimumContentsLength(12)
    field.setMinimumWidth(0)
    field.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    field.currentTextChanged.connect(field.setToolTip)
    return field


def _number(name: str, minimum: float, maximum: float, decimals: int = 3) -> QDoubleSpinBox:
    field = QDoubleSpinBox()
    field.setAccessibleName(name)
    field.setDecimals(decimals)
    field.setRange(minimum, maximum)
    field.setSingleStep(10 ** -min(decimals, 2))
    return field


def _optional_number(field: OptionalQuantityEdit, name: str) -> float | None:
    try:
        return field.value_si()
    except UnitError as error:
        raise ValueError(
            f"{name} must be a number with a compatible unit, or left empty for automatic settings"
        ) from error


class RuntimeWorkspace(QWidget):
    """Own launch settings and one embedded Inspector for the open Studio pack.

    Args:
        project_provider: Return the current in-memory authoring project.
        save_project: Save that project transactionally and report success.
        parent: Studio widget owning this workspace.
        units: Shared display preferences for Studio's selected ModSim home.

    Signals report explicit recipe edits and execution lifetime to the Studio
    shell. The runtime always loads the saved pack; edited authoring objects
    never cross the simulation thread or process boundary.
    """

    project_changed = Signal(object)
    execution_active_changed = Signal(bool)

    def __init__(
        self,
        project_provider: Callable[[], StudioProject | None],
        save_project: Callable[[], bool],
        parent: QWidget | None = None,
        *,
        units: UnitManager | None = None,
    ) -> None:
        super().__init__(parent)
        self._units = units or units_manager()
        self._project_provider = project_provider
        self._save_project = save_project
        self._pack_root: Path | None = None
        self._refreshing = False
        self._active = False
        self.inspector: RuntimeInspectorWindow | None = None
        self.setObjectName("RuntimeWorkspace")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.pages = QStackedWidget()
        layout.addWidget(self.pages)

        self.setup_page = QWidget()
        self.setup_page.setObjectName("Panel")
        setup_layout = QVBoxLayout(self.setup_page)
        title = QLabel("Configure your simulation")
        title.setObjectName("SectionTitle")
        setup_layout.addWidget(title)
        self.pack_label = QLabel("Open or create a Robot Pack in the Robot Pack workspace first.")
        self.pack_label.setWordWrap(True)
        setup_layout.addWidget(self.pack_label)
        guidance = QLabel(
            "Choose a demonstration, backend, and generated model view. "
            "Runtime validation checks this launch separately from authoring validation."
        )
        guidance.setWordWrap(True)
        guidance.setObjectName("Muted")
        setup_layout.addWidget(guidance)
        self.units_notice = QLabel()
        self.units_notice.setWordWrap(True)
        self.units_notice.setObjectName("Muted")
        self.units_notice.hide()
        setup_layout.addWidget(self.units_notice)

        self.settings = QWidget()
        settings_layout = QVBoxLayout(self.settings)
        settings_layout.setContentsMargins(0, 0, 0, 0)
        form = property_group("Simulation", settings_layout)
        self.backend = _choice("Backend")
        for backend in available_backends():
            self.backend.addItem("MuJoCo" if backend == "mujoco" else backend.title(), backend)
        self.backend.setCurrentIndex(self.backend.findData("mujoco"))
        form.addRow("Backend", self.backend)
        self.demo = _choice("Demonstration")
        for demo in RuntimeDemo:
            self.demo.addItem(demo.value.replace("_", " ").capitalize(), demo.value)
        self.demo.setCurrentIndex(self.demo.findData(RuntimeDemo.DOCK_UNDOCK.value))
        form.addRow("Demonstration", self.demo)
        self.module_type = _choice("Module type")
        form.addRow("Module type", self.module_type)
        self.model_view = _choice("Model view")
        form.addRow("Model view", self.model_view)
        self.add_view_button = QPushButton("Add topology view")
        self.add_view_button.setToolTip(
            "Add a generic runtime topology recipe to the open Robot Pack. Save persists it."
        )
        self.add_view_button.clicked.connect(self.add_topology_view)
        form.addRow("Missing a model view?", self.add_view_button)
        self.viewer = QCheckBox("Open MuJoCo's native 3D companion")
        form.addRow("Native 3D viewer", self.viewer)
        self.duration = self._quantity("Duration", QuantityKind.TIME, 0.001, 1_000_000)
        self.duration.setSingleStep(1)
        form.addRow("Duration", self.duration)
        self.speed = _number("Playback speed", 0.001, 1000)
        self.speed.setSingleStep(0.25)
        form.addRow("Playback speed", self.speed)

        advanced = QWidget()
        advanced_form = QFormLayout(advanced)
        advanced_form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        advanced_form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.timestep = self._quantity("Timestep", QuantityKind.TIME, 0.000001, 1)
        advanced_form.addRow("Timestep", self.timestep)
        self.publish_hz = self._quantity("UI update rate", QuantityKind.FREQUENCY, 0.1, 1000)
        advanced_form.addRow("UI update rate", self.publish_hz)
        self.gravity = QCheckBox("Enable gravity")
        advanced_form.addRow("Gravity", self.gravity)
        self.ground = QCheckBox("Add ground plane")
        advanced_form.addRow("Ground", self.ground)
        self.initial_height = self._quantity("Initial height", QuantityKind.LENGTH, -1000, 1000)
        advanced_form.addRow("Initial height", self.initial_height)
        self.fixed_connector = _choice("Fixed connector")
        self.moving_connector = _choice("Moving connector")
        advanced_form.addRow("Fixed connector", self.fixed_connector)
        advanced_form.addRow("Moving connector", self.moving_connector)
        self.gap = self._optional_quantity("Connector gap", QuantityKind.LENGTH)
        self.gap.setPlaceholderText(f"Automatic ({self.gap.unit})")
        advanced_form.addRow("Connector gap", self.gap)
        self.orientation = self._quantity("Orientation", QuantityKind.ANGLE, -1000, 1000)
        advanced_form.addRow("Orientation", self.orientation)
        self.approach = self._quantity(
            "Approach speed", QuantityKind.LINEAR_VELOCITY, 0.000001, 1000
        )
        advanced_form.addRow("Approach speed", self.approach)
        self.undock_at = self._optional_quantity("Undock at", QuantityKind.TIME)
        self.undock_at.setPlaceholderText(f"Automatic ({self.undock_at.unit})")
        advanced_form.addRow("Undock at", self.undock_at)
        self.retract = self._optional_quantity("Retraction speed", QuantityKind.LINEAR_VELOCITY)
        self.retract.setPlaceholderText(f"Automatic ({self.retract.unit})")
        advanced_form.addRow("Retraction speed", self.retract)
        self.advanced = DetailsSection("Advanced runtime settings", advanced)
        settings_layout.addWidget(self.advanced)
        defaults = QPushButton("Reset demonstration settings")
        defaults.clicked.connect(self._apply_defaults)
        settings_layout.addWidget(defaults)
        settings_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(self.settings)
        setup_layout.addWidget(scroll, 1)

        self.readiness_label = QLabel("Runtime has not been validated")
        self.readiness_label.setWordWrap(True)
        self.readiness_label.setObjectName("ValidationSummary")
        setup_layout.addWidget(self.readiness_label)
        self.readiness = QPlainTextEdit()
        self.readiness.setReadOnly(True)
        self.readiness.setMaximumHeight(150)
        self.readiness.setPlaceholderText(
            "Validation checks simulation metadata, the selected backend, "
            "demonstration, and model-view builder. Backend initialization happens on Start."
        )
        setup_layout.addWidget(self.readiness)
        actions = QHBoxLayout()
        self.validate_button = QPushButton("Validate runtime")
        self.start_button = QPushButton("Start simulation")
        self.start_button.setProperty("primary", True)
        self.validate_button.clicked.connect(self.validate_runtime)
        self.start_button.clicked.connect(self.start_simulation)
        actions.addStretch(1)
        actions.addWidget(self.validate_button)
        actions.addWidget(self.start_button)
        setup_layout.addLayout(actions)
        self.pages.addWidget(self.setup_page)

        self.live_page = QWidget()
        live_layout = QVBoxLayout(self.live_page)
        live_layout.setContentsMargins(0, 0, 0, 0)
        live_header = QHBoxLayout()
        self.run_label = QLabel()
        self.run_label.setWordWrap(True)
        live_header.addWidget(self.run_label, 1)
        self.back_button = QPushButton("Back to setup")
        self.back_button.clicked.connect(self._back_to_setup)
        live_header.addWidget(self.back_button)
        live_layout.addLayout(live_header)
        self.inspector_layout = QVBoxLayout()
        self.inspector_layout.setContentsMargins(0, 0, 0, 0)
        live_layout.addLayout(self.inspector_layout, 1)
        self.pages.addWidget(self.live_page)
        self.pages.currentChanged.connect(self._size_current_page)
        self._size_current_page()

        self.backend.currentIndexChanged.connect(self._backend_changed)
        self.demo.currentIndexChanged.connect(self._apply_defaults)
        self.module_type.currentIndexChanged.connect(self._refresh_connectors)
        for combo in (
            self.backend,
            self.demo,
            self.module_type,
            self.model_view,
            self.fixed_connector,
            self.moving_connector,
        ):
            combo.currentIndexChanged.connect(self._invalidate_readiness)
        for spin in (
            self.duration,
            self.speed,
            self.timestep,
            self.publish_hz,
            self.initial_height,
            self.orientation,
            self.approach,
        ):
            spin.valueChanged.connect(self._invalidate_readiness)
        for check in (self.viewer, self.gravity, self.ground):
            check.toggled.connect(self._invalidate_readiness)
        for field in (self.gap, self.undock_at, self.retract):
            field.textChanged.connect(self._invalidate_readiness)
        self._units.changed.connect(self._refresh_units)
        self.refresh_project()

    def _quantity(
        self, name: str, kind: QuantityKind, minimum: float, maximum: float
    ) -> QuantitySpinBox:
        field = QuantitySpinBox(kind=kind, unit=self._units.preferences.unit_for(kind))
        field.setAccessibleName(name)
        field.setRange(minimum, maximum)
        field.setSingleStep(0.01)
        return field

    def _optional_quantity(self, name: str, kind: QuantityKind) -> OptionalQuantityEdit:
        field = OptionalQuantityEdit(None, kind=kind, unit=self._units.preferences.unit_for(kind))
        field.setAccessibleName(name)
        return field

    def _refresh_units(self) -> None:
        preserved = False
        for field in (
            self.duration,
            self.timestep,
            self.publish_hz,
            self.initial_height,
            self.orientation,
            self.approach,
            self.gap,
            self.undock_at,
            self.retract,
        ):
            try:
                if not field.set_unit(self._units.preferences.unit_for(field.kind)):
                    preserved = True
            except UnitError:
                preserved = True
                field.setToolTip(f"This input retains its previous unit: {field.unit}.")
            if isinstance(field, OptionalQuantityEdit):
                field.setPlaceholderText(f"Automatic ({field.unit})")
        self.units_notice.setText(
            "Unfinished inputs retain their previous units. "
            "Correct them before changing their display units again."
        )
        self.units_notice.setVisible(preserved)

    @property
    def is_running(self) -> bool:
        """Return whether a simulation still owns execution resources."""
        return self._active

    def _size_current_page(self, *_args: object) -> None:
        for index in range(self.pages.count()):
            page = self.pages.widget(index)
            if page is not None:
                policy = (
                    QSizePolicy.Policy.Expanding
                    if page is self.pages.currentWidget()
                    else QSizePolicy.Policy.Ignored
                )
                page.setSizePolicy(policy, policy)
        self.pages.updateGeometry()

    def refresh_project(self) -> None:
        """Refresh recipe and module selectors after an authoring edit or save."""
        project = self._project_provider()
        self.settings.setEnabled(project is not None and not self._active)
        self.validate_button.setEnabled(project is not None and not self._active)
        self.start_button.setEnabled(project is not None and not self._active)
        if project is None:
            return
        self.pack_label.setText(f"{project.pack.manifest.name}  ·  {project.loaded.root}")
        new_pack = project.loaded.root != self._pack_root
        if new_pack:
            self._pack_root = project.loaded.root
            self._discard_inspector()
            self.pages.setCurrentWidget(self.setup_page)
        module_id = self.module_type.currentData()
        view_id = self.model_view.currentData()
        self._refreshing = True
        try:
            self.module_type.clear()
            for module_id_value in project.pack.hardware_catalog.module_types:
                self.module_type.addItem(module_id_value, module_id_value)
            if not new_pack and self.module_type.findData(module_id) >= 0:
                self.module_type.setCurrentIndex(self.module_type.findData(module_id))
            self.model_view.clear()
            recipes = runtime_recipes(project.pack)
            if not recipes:
                self.model_view.addItem("No runtime model view — add one below", None)
            for recipe in recipes:
                self.model_view.addItem(f"{recipe.name or recipe.id} ({recipe.builder})", recipe.id)
            preferred = next((recipe.id for recipe in recipes if recipe.default), None)
            selected = (
                view_id if not new_pack and self.model_view.findData(view_id) >= 0 else preferred
            )
            if self.model_view.findData(selected) >= 0:
                self.model_view.setCurrentIndex(self.model_view.findData(selected))
        finally:
            self._refreshing = False
        self._refresh_connectors()
        if new_pack:
            self._apply_defaults()
        self._invalidate_readiness()

    def _refresh_connectors(self) -> None:
        project = self._project_provider()
        module_id = self.module_type.currentData()
        module = (
            None if project is None else project.pack.hardware_catalog.module_types.get(module_id)
        )
        for combo in (self.fixed_connector, self.moving_connector):
            selected = combo.currentData()
            blocked = combo.blockSignals(True)
            combo.clear()
            combo.addItem("Automatic / demonstration default", None)
            if module is not None:
                for connector in module.connectors:
                    combo.addItem(connector.id, connector.id)
            if combo.findData(selected) >= 0:
                combo.setCurrentIndex(combo.findData(selected))
            combo.blockSignals(blocked)

    def _apply_defaults(self, *_args: object) -> None:
        if self._refreshing:
            return
        project = self._project_provider()
        if project is None:
            return
        config = runtime_config_defaults(
            project.loaded.root, RuntimeDemo(self.demo.currentData()), self.backend.currentData()
        )
        self.duration.setValue(config.duration_s)
        self.speed.setValue(config.real_time_factor)
        self.timestep.setValue(config.dt_s)
        self.publish_hz.setValue(config.publish_hz)
        self.gravity.setChecked(config.gravity)
        self.ground.setChecked(config.ground)
        self.initial_height.setValue(config.height_m)
        self.viewer.setChecked(config.viewer_enabled)
        self.viewer.setEnabled(config.backend == "mujoco")
        self.orientation.setValue(config.orientation_rad)
        self.approach.setValue(config.approach_m_s)
        for field, value in (
            (self.gap, config.connector_gap_m),
            (self.undock_at, config.undock_at_s),
            (self.retract, config.retract_m_s),
        ):
            field.set_value_si(value)
        self.fixed_connector.setCurrentIndex(0)
        self.moving_connector.setCurrentIndex(0)
        if config.view_id is not None:
            index = self.model_view.findData(config.view_id)
            if index >= 0:
                self.model_view.setCurrentIndex(index)
        self._invalidate_readiness()

    def _backend_changed(self, *_args: object) -> None:
        # Comparing engines should preserve the user's scenario and tuning.
        # Only changing the demonstration or explicitly resetting replaces them.
        native_viewer = self.backend.currentData() == "mujoco"
        self.viewer.setEnabled(native_viewer)
        self.viewer.setChecked(native_viewer)
        self._invalidate_readiness()

    def launch_config(self) -> RuntimeInspectorConfig:
        """Return the form's literal launch request, with SI units throughout.

        Raises:
            ValueError: No pack is open or an optional numeric input is invalid.
        """
        project = self._project_provider()
        if project is None:
            raise ValueError("Open or create a Robot Pack first")
        return replace(
            runtime_config_defaults(project.loaded.root),
            demo=RuntimeDemo(self.demo.currentData()),
            backend=self.backend.currentData(),
            module_type=self.module_type.currentData(),
            view_id=self.model_view.currentData(),
            duration_s=self.duration.value_si(),
            real_time_factor=self.speed.value(),
            dt_s=self.timestep.value_si(),
            publish_hz=self.publish_hz.value_si(),
            viewer_enabled=self.viewer.isChecked(),
            gravity=self.gravity.isChecked(),
            ground=self.ground.isChecked(),
            height_m=self.initial_height.value_si(),
            fixed_connector=self.fixed_connector.currentData(),
            moving_connector=self.moving_connector.currentData(),
            connector_gap_m=_optional_number(self.gap, "Connector gap"),
            orientation_rad=self.orientation.value_si(),
            approach_m_s=self.approach.value_si(),
            undock_at_s=_optional_number(self.undock_at, "Undock time"),
            retract_m_s=_optional_number(self.retract, "Retraction speed"),
        )

    def add_topology_view(self) -> None:
        """Explicitly add a runtime topology recipe as an unsaved authoring edit."""
        project = self._project_provider()
        if project is None or self._active:
            return
        updated = add_default_topology_view(project)
        self.project_changed.emit(updated)
        self.refresh_project()
        self.model_view.setCurrentIndex(
            self.model_view.findData(updated.pack.manifest.model_views[-1].id)
        )
        self.readiness_label.setText("Topology recipe added — save it with the Robot Pack")

    def _invalidate_readiness(self, *_args: object) -> None:
        if self._refreshing:
            return
        self.readiness_label.setText("Runtime settings changed — validate before starting")
        self._set_readiness_state("")
        self.readiness.clear()

    def _set_readiness_state(self, state: str) -> None:
        self.readiness_label.setProperty("state", state)
        self.readiness_label.style().unpolish(self.readiness_label)
        self.readiness_label.style().polish(self.readiness_label)

    def _show_report(self, report: ValidationReport) -> None:
        self.readiness_label.setText(
            "Runtime preflight passed" if report.valid else "Runtime needs attention"
        )
        self._set_readiness_state(
            "error" if report.errors else "warning" if report.warnings else "success"
        )
        lines = [f"{len(report.errors)} errors · {len(report.warnings)} warnings"]
        for issue in report.issues:
            line = (
                f"{issue.severity.value.upper()} · {issue.code} · {issue.location}\n{issue.message}"
            )
            if issue.suggested_fix:
                line += f"\n{issue.suggested_fix}"
            lines.append(line)
        if report.valid:
            lines.append(
                "Ready to initialize the selected backend. Physics is not run by validation."
            )
        project = self._project_provider()
        if project is not None and project.dirty:
            lines.append("Unsaved Robot Pack edits must be saved before starting.")
        self.readiness.setPlainText("\n\n".join(lines))

    def validate_runtime(self) -> ValidationReport | None:
        """Check current in-memory data and launch settings without starting physics."""
        project = self._project_provider()
        if project is None or self._active:
            return None
        try:
            report = validate_runtime_launch(project.loaded, self.launch_config())
        except Exception as error:
            _LOGGER.exception("Runtime preflight could not complete")
            self.readiness_label.setText("Runtime needs attention")
            self._set_readiness_state("error")
            self.readiness.setPlainText(str(error))
            return None
        self._show_report(report)
        _LOGGER.info(
            "Runtime preflight: errors=%d warnings=%d", len(report.errors), len(report.warnings)
        )
        return report

    def _confirm_save(self) -> bool:
        dialog = QMessageBox(self)
        dialog.setWindowTitle("Save and start")
        dialog.setText("Save Robot Pack edits before starting the simulation?")
        dialog.setInformativeText(
            "The runtime loads the saved pack, including its model-view recipe."
        )
        save = dialog.addButton("Save and start", QMessageBox.ButtonRole.AcceptRole)
        dialog.addButton(QMessageBox.StandardButton.Cancel)
        dialog.setDefaultButton(save)
        dialog.exec()
        return dialog.clickedButton() is save

    def start_simulation(self) -> None:
        """Revalidate, save confirmed edits, and start one embedded Inspector."""
        if self._active:
            return
        report = self.validate_runtime()
        if report is None or not report.valid:
            return
        project = self._project_provider()
        if project is None:
            return
        if project.dirty and (not self._confirm_save() or not self._save_project()):
            return
        try:
            config = self.launch_config()
            # A worker reads files independently. Recheck the exact on-disk
            # input after saving, rather than launching stale in-memory data.
            loaded = RobotPackLoader().load(config.pack_path)
            report = validate_runtime_launch(loaded, config)
            self._show_report(report)
            if not report.valid:
                return
            from modsim_robotics_studio.runtime_window import RuntimeInspectorWindow

            self._discard_inspector()
            self.inspector = RuntimeInspectorWindow(
                config, self.live_page, embedded=True, auto_start=False, units=self._units
            )
            self.inspector.execution_finished.connect(self._execution_finished)
            self.inspector.execution_failed.connect(self._execution_failed)
            self.inspector_layout.addWidget(self.inspector)
            self.run_label.setText(
                f"{project.pack.manifest.name} · {config.backend} · {config.demo.value} · "
                "Saved pack snapshot; authoring edits apply on the next launch."
            )
            self.pages.setCurrentWidget(self.live_page)
            self._set_active(True)
            _LOGGER.info(
                "Starting embedded runtime pack=%s backend=%s demo=%s view=%s",
                config.pack_path,
                config.backend,
                config.demo.value,
                config.view_id,
            )
            self.inspector.start()
        except Exception as error:
            _LOGGER.exception("Could not start embedded runtime")
            # A controller may fail after acquiring resources. Keep its widget
            # alive and persistence locked until cooperative shutdown succeeds.
            stopped = self.stop_and_wait()
            self._set_active(not stopped)
            if stopped:
                self.pages.setCurrentWidget(self.setup_page)
            else:
                self.run_label.setText(f"Could not start simulation · {error}")
            self.readiness_label.setText("Could not start simulation")
            self._set_readiness_state("error")
            self.readiness.setPlainText(str(error))

    def _set_active(self, active: bool) -> None:
        self._active = active
        self.back_button.setEnabled(not active)
        self.settings.setEnabled(not active)
        self.validate_button.setEnabled(not active)
        self.start_button.setEnabled(not active)
        self.execution_active_changed.emit(active)

    def _execution_finished(self) -> None:
        self._set_active(False)

    def _execution_failed(self, message: str) -> None:
        self.run_label.setText(f"Simulation failed · {message}")

    def _back_to_setup(self) -> None:
        if not self._active:
            self.pages.setCurrentWidget(self.setup_page)
            self.refresh_project()

    def _discard_inspector(self) -> None:
        if self.inspector is not None:
            if not self.inspector.stop_and_wait():
                raise RuntimeError("The previous runtime is still stopping. Try again shortly.")
            self.inspector_layout.removeWidget(self.inspector)
            self.inspector.close()
            self.inspector.deleteLater()
            self.inspector = None

    def stop_and_wait(self, timeout_ms: int = 10_000) -> bool:
        """Stop execution before Studio closes or replaces the live Inspector.

        Args:
            timeout_ms: Maximum cooperative shutdown wait in milliseconds.

        Returns:
            Whether the runtime released its execution resources in time.
        """
        if self.inspector is None:
            return True
        stopped = self.inspector.stop_and_wait(timeout_ms)
        if stopped:
            self._set_active(False)
        return stopped


__all__ = ["RuntimeWorkspace"]
