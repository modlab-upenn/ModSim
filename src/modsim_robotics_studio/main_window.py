# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
"""Main ModSim Studio Robot Pack Builder window."""

from __future__ import annotations

import json
import logging
import math
from collections.abc import Callable, Mapping
from enum import StrEnum
from pathlib import Path
from typing import cast

from pydantic import JsonValue
from PySide6.QtCore import QDir, QObject, QSettings, QSize, Qt, Signal
from PySide6.QtGui import QAction, QCloseEvent, QFontDatabase, QIcon, QShowEvent
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDockWidget,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolBar,
    QToolButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from modsim_robotics.core.transforms import quat_from_rpy, quat_to_rpy
from modsim_robotics.importers import URDFImporter
from modsim_robotics.robot_packs import (
    AcceptanceRegion,
    AcceptanceShape,
    AlignmentMode,
    AllowedOrientations,
    ComplianceSpec,
    ConnectorGender,
    ConnectorLimits,
    ConnectorSpec,
    ConnectorTypeSpec,
    ControlMode,
    DockingPolicySpec,
    HingeConstraintSpec,
    JointLimits,
    JointSpec,
    JointType,
    ModelViewMode,
    ModelViewSpec,
    OrientationMode,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    PoseSpec,
    ValidationProfile,
)
from modsim_robotics.units import QuantityKind, UnitError, UnitPreferences
from modsim_robotics_studio._loading import BusyDialog, LoadingPanel
from modsim_robotics_studio._window_sizing import WindowSizingPolicy, manage_window
from modsim_robotics_studio.appearance import theme_manager
from modsim_robotics_studio.chrome import DetailsSection, StudioHeader, property_group
from modsim_robotics_studio.display_units_dialog import show_display_units_dialog
from modsim_robotics_studio.icons import studio_icon
from modsim_robotics_studio.project import StudioProject
from modsim_robotics_studio.quantity_widgets import (
    OptionalQuantityEdit,
    QuantityEdit,
    QuantityListEdit,
    QuantityVectorEdit,
)
from modsim_robotics_studio.runtime_workspace import RuntimeWorkspace
from modsim_robotics_studio.unit_preferences import units_manager
from modsim_robotics_studio.user_errors import (
    error_log_details,
    user_error_message,
    validate_identifier,
)
from modsim_robotics_studio.viewport import RobotViewport

_KIND_ROLE = int(Qt.ItemDataRole.UserRole)
_ID_ROLE = _KIND_ROLE + 1
_MODULE_ROLE = _KIND_ROLE + 2
# Keep file pickers inside Qt: native GTK dialogs can abort the process when
# launched from a Snap terminal with incompatible inherited pixbuf loaders.
_FILE_DIALOG_OPTIONS = QFileDialog.Option.DontUseNativeDialog
_SHOW_HIDDEN_SETTINGS_KEY = "file_dialogs/show_hidden"


def _select_path(
    parent: QWidget,
    caption: str,
    directory: str = "",
    name_filter: str = "",
    *,
    file_mode: QFileDialog.FileMode = QFileDialog.FileMode.ExistingFile,
    accept_mode: QFileDialog.AcceptMode = QFileDialog.AcceptMode.AcceptOpen,
) -> str:
    """Select a file or directory with a remembered hidden-entry toggle."""
    dialog = QFileDialog(parent, caption, options=_FILE_DIALOG_OPTIONS)
    # Qt's stock arrow icon pixmaps stay black in dark themes. Native button
    # arrows follow the live palette, including disabled and high-DPI states.
    back_arrow = Qt.ArrowType.RightArrow if dialog.isRightToLeft() else Qt.ArrowType.LeftArrow
    forward_arrow = Qt.ArrowType.LeftArrow if dialog.isRightToLeft() else Qt.ArrowType.RightArrow
    for name, arrow in (
        ("backButton", back_arrow),
        ("forwardButton", forward_arrow),
        ("toParentButton", Qt.ArrowType.UpArrow),
    ):
        button = dialog.findChild(QToolButton, name)
        if button is not None:
            button.setIcon(QIcon())
            button.setArrowType(arrow)
    dialog.setFileMode(file_mode)
    dialog.setAcceptMode(accept_mode)
    if file_mode == QFileDialog.FileMode.Directory:
        dialog.setOption(QFileDialog.Option.ShowDirsOnly, True)
    if name_filter:
        dialog.setNameFilter(name_filter)
    if directory:
        initial_path = Path(directory).expanduser()
        if file_mode == QFileDialog.FileMode.Directory or initial_path.is_dir():
            dialog.setDirectory(str(initial_path))
        else:
            dialog.setDirectory(str(initial_path.parent))
            dialog.selectFile(initial_path.name)

    settings = QSettings("ModSim", "Studio")
    show_hidden = bool(settings.value(_SHOW_HIDDEN_SETTINGS_KEY, False, type=bool))

    def set_hidden_filter(visible: bool) -> None:
        # Use integer bits so clearing Hidden preserves AllDirs and NoDot flags.
        # AllDirs keeps folders visible under a URDF filename filter.
        filters = dialog.filter().value
        hidden = QDir.Filter.Hidden.value
        dialog.setFilter(QDir.Filter(filters | hidden if visible else filters & ~hidden))

    set_hidden_filter(show_hidden)
    checkbox = QCheckBox("Show hidden files and folders", dialog)
    checkbox.setObjectName("showHiddenFiles")
    checkbox.setChecked(show_hidden)

    def remember_hidden(visible: bool) -> None:
        settings.setValue(_SHOW_HIDDEN_SETTINGS_KEY, visible)
        settings.sync()

    def toggle_hidden(visible: bool) -> None:
        set_hidden_filter(visible)
        remember_hidden(visible)

    checkbox.toggled.connect(toggle_hidden)

    def sync_hidden(_checked: bool) -> None:
        visible = bool(dialog.filter() & QDir.Filter.Hidden)
        was_blocked = checkbox.blockSignals(True)
        try:
            checkbox.setChecked(visible)
        finally:
            checkbox.blockSignals(was_blocked)
        remember_hidden(visible)

    # Qt changes its filter on triggered. Reflect the result without feeding
    # another visibility change back into its context-menu handler.
    hidden_action = dialog.findChild(QAction, "qt_show_hidden_action")
    if hidden_action is not None:
        hidden_action.triggered.connect(sync_hidden)
    layout = dialog.layout()
    if isinstance(layout, QGridLayout):
        layout.addWidget(checkbox, layout.rowCount(), 0, 1, layout.columnCount())
    elif layout is not None:
        layout.addWidget(checkbox)

    window_sizing = manage_window(
        dialog, "file-browser", preferred=QSize(980, 680), settings=settings
    )
    try:
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return ""
        selected = dialog.selectedFiles()
        return selected[0] if selected else ""
    finally:
        # Keep the policy alive throughout the modal loop, including Hide on
        # accept/reject. The QObject parent handles its eventual destruction.
        del window_sizing
        dialog.deleteLater()


class _FormDialog(QDialog):
    """Scroll a creation form while keeping confirmation controls reachable."""

    def __init__(self, parent: QWidget, title: str, key: str, preferred: QSize) -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        self._sizing_key = key
        self._preferred_size = preferred
        self._window_sizing: WindowSizingPolicy | None = None
        layout = QVBoxLayout(self)
        content = QWidget()
        self.form = QFormLayout(content)
        self.form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(content)
        layout.addWidget(scroll, 1)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def exec(self) -> int:
        if self._window_sizing is None:
            self._window_sizing = manage_window(
                self, self._sizing_key, preferred=self._preferred_size
            )
        return super().exec()


class _LogEmitter(QObject):
    """Move Python log records safely onto the Qt GUI thread."""

    line = Signal(str)


class _QtLogHandler(logging.Handler):
    """Mirror formatted records into the Studio session-log panel."""

    def __init__(self, target: QPlainTextEdit) -> None:
        super().__init__(level=logging.DEBUG)
        self.emitter = _LogEmitter(target)
        self.emitter.line.connect(target.appendPlainText)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.emitter.line.emit(self.format(record))
        except Exception:
            self.handleError(record)


class _MetadataEditor(QWidget):
    """Edit a bounded mapping of custom metadata keys to JSON values."""

    def __init__(self, metadata: Mapping[str, JsonValue]) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Field", "JSON value"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.setWordWrap(False)
        self.table.setMaximumHeight(180)
        layout.addWidget(self.table)
        buttons = QHBoxLayout()
        add_button = QPushButton("Add field")
        remove_button = QPushButton("Remove selected")
        add_button.clicked.connect(self.add_field)
        remove_button.clicked.connect(self.remove_selected)
        buttons.addWidget(add_button)
        buttons.addWidget(remove_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        guidance = QLabel('Values use JSON syntax; quote strings, for example "front".')
        guidance.setWordWrap(True)
        layout.addWidget(guidance)
        for key, value in metadata.items():
            self._append_row(key, json.dumps(value, ensure_ascii=False, sort_keys=True))

    def add_field(self) -> None:
        """Append one editable metadata row."""
        self._append_row("custom_field", '""')

    def remove_selected(self) -> None:
        """Remove every selected row, or the current row when no cells are selected."""
        rows = {index.row() for index in self.table.selectedIndexes()}
        if not rows and self.table.currentRow() >= 0:
            rows.add(self.table.currentRow())
        for row in sorted(rows, reverse=True):
            self.table.removeRow(row)

    def value(self) -> dict[str, JsonValue]:
        """Return validated JSON-compatible metadata from the table."""
        result: dict[str, JsonValue] = {}
        for row in range(self.table.rowCount()):
            key_item = self.table.item(row, 0)
            value_item = self.table.item(row, 1)
            key = key_item.text().strip() if key_item is not None else ""
            if not key:
                raise ValueError(f"metadata row {row + 1} requires a field name")
            if key in result:
                raise ValueError(f"duplicate metadata field '{key}'")
            raw_value = value_item.text() if value_item is not None else ""
            try:
                parsed = json.loads(raw_value)
            except json.JSONDecodeError as error:
                raise ValueError(f"metadata field '{key}' has invalid JSON: {error.msg}") from error
            result[key] = cast(JsonValue, parsed)
        return result

    def _append_row(self, key: str, value: str) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(key))
        self.table.setItem(row, 1, QTableWidgetItem(value))


class _CubicLatticeFields(QWidget):
    """Author the built-in lattice's declared quantities without guessing JSON units."""

    def __init__(self, data: dict[str, JsonValue], preferences: UnitPreferences) -> None:
        super().__init__()
        self._original = dict(data)
        form = QFormLayout(self)
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.scalars: dict[str, OptionalQuantityEdit] = {}
        for label, name, kind in (
            ("Cell pitch", "pitch_m", QuantityKind.LENGTH),
            ("Position tolerance", "position_tolerance_m", QuantityKind.LENGTH),
            ("Orientation tolerance", "orientation_tolerance_rad", QuantityKind.ANGLE),
        ):
            value = data.get(name)
            if value is not None and (
                isinstance(value, bool) or not isinstance(value, int | float)
            ):
                raise ValueError(f"{name} needs numeric JSON editing")
            field = OptionalQuantityEdit(
                float(value) if value is not None else None,
                kind=kind,
                unit=preferences.unit_for(kind),
            )
            field.setObjectName(name)
            field.setAccessibleName(label)
            field.refresh_tooltip()
            form.addRow(label, field)
            self.scalars[name] = field
        origin = self._vector(data.get("origin_world_m", [0.0, 0.0, 0.0]), 3)
        quaternion = self._vector(data.get("orientation_world_wxyz", [1.0, 0.0, 0.0, 0.0]), 4)
        if not math.isclose(math.hypot(*quaternion), 1.0, rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError("lattice orientation requires a unit quaternion")
        self._quaternion = cast(tuple[float, float, float, float], quaternion)
        self._rpy = quat_to_rpy(self._quaternion)
        self.origin = QuantityVectorEdit(
            cast(tuple[float, float, float], origin),
            kind=QuantityKind.LENGTH,
            unit=preferences.unit_for(QuantityKind.LENGTH),
            name="Lattice origin",
            labels="XYZ",
        )
        self.origin.setObjectName("origin_world_m")
        self.orientation = QuantityVectorEdit(
            self._rpy,
            kind=QuantityKind.ANGLE,
            unit=preferences.unit_for(QuantityKind.ANGLE),
            name="Lattice orientation",
            labels="RPY",
        )
        self.orientation.setObjectName("orientation_world_rpy")
        form.addRow("World origin XYZ", self.origin)
        form.addRow("World orientation RPY", self.orientation)

    @staticmethod
    def _vector(value: JsonValue, size: int) -> tuple[float, ...]:
        if not isinstance(value, list) or len(value) != size:
            raise ValueError("configuration vector requires JSON numeric components")
        if any(isinstance(item, bool) or not isinstance(item, int | float) for item in value):
            raise ValueError("configuration vector requires JSON numeric components")
        vector = tuple(float(cast(int | float, item)) for item in value)
        if not all(math.isfinite(item) for item in vector):
            raise ValueError("configuration vector must be finite")
        return vector

    def value(self) -> dict[str, JsonValue]:
        scalars = {name: field.value_si() for name, field in self.scalars.items()}
        if any(value is None for value in scalars.values()):
            raise ValueError("Cell pitch and both lattice tolerances are required")
        pitch = cast(float, scalars["pitch_m"])
        position = cast(float, scalars["position_tolerance_m"])
        angle = cast(float, scalars["orientation_tolerance_rad"])
        if pitch <= 0 or not 0 <= position < pitch / 2 or not 0 <= angle <= math.pi / 4:
            raise ValueError(
                "Lattice pitch must be positive; position tolerance less than half a cell; "
                "orientation tolerance from 0 to 45 degrees."
            )
        result = dict(self._original)
        result.update(scalars)
        result["origin_world_m"] = list(self.origin.value_si())
        rpy = self.orientation.value_si()
        quaternion = self._quaternion if rpy == self._rpy else quat_from_rpy(rpy)
        result["orientation_world_wxyz"] = list(quaternion)
        return result


class _ModelViewConfigurationEditor(QWidget):
    """Offer typed lattice quantities and preserve arbitrary other builder configuration."""

    def __init__(
        self,
        data: dict[str, JsonValue],
        builder: str,
        preferences: Callable[[], UnitPreferences],
    ) -> None:
        super().__init__()
        self._preferences = preferences
        self._builder = ""
        self._data = dict(data)
        self._typed: _CubicLatticeFields | None = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.guidance = QLabel()
        self.guidance.setWordWrap(True)
        layout.addWidget(self.guidance)
        self.stack = QStackedWidget()
        self._raw = _MetadataEditor(data)
        self.stack.addWidget(self._raw)
        layout.addWidget(self.stack)
        self._mode_button = QPushButton("Edit raw builder configuration")
        self._mode_button.setToolTip(
            "Edit the builder's declared JSON fields directly. Physical values use canonical SI."
        )
        self._mode_button.clicked.connect(self._toggle_editor)
        layout.addWidget(self._mode_button)
        self.set_builder(builder)

    def _toggle_editor(self) -> None:
        if self._typed is not None and self.stack.currentWidget() is self._typed:
            self._show_raw()
            return
        try:
            data = self._raw.value()
            if self._typed is not None and data == self._data:
                self.stack.setCurrentWidget(self._typed)
            else:
                typed = _CubicLatticeFields(data, self._preferences())
                if self._typed is not None:
                    self.stack.removeWidget(self._typed)
                    self._typed.deleteLater()
                self._typed = typed
                self.stack.addWidget(typed)
                self.stack.setCurrentWidget(typed)
                self._data = dict(data)
        except ValueError as error:
            self.guidance.setText(f"Correct the raw configuration first: {error}")
            return
        self._mode_button.setText("Edit raw builder configuration")
        self.guidance.setText(
            "Lattice fields use your preferred units; runtime orientation is a quaternion."
        )

    def _show_raw(self) -> None:
        try:
            data = self.value()
        except ValueError:
            data = self._data
        self._data = dict(data)
        raw = _MetadataEditor(data)
        self.stack.removeWidget(self._raw)
        self._raw.deleteLater()
        self._raw = raw
        self.stack.addWidget(raw)
        self.stack.setCurrentWidget(raw)
        self._mode_button.setText("Use quantity-aware lattice fields")
        self.guidance.setText(
            "Raw configuration uses each builder's declared units. "
            "Unfinished quantity fields remain available in their previous editor."
        )

    def set_builder(self, builder: str) -> None:
        builder = builder.strip()
        if builder == self._builder:
            return
        try:
            data = self.value()
        except ValueError:
            self.guidance.setText("Finish the current configuration before changing its editor.")
            return
        self._builder = builder
        self._mode_button.setVisible(builder == "cubic_lattice")
        self._data = dict(data)
        if builder == "cubic_lattice":
            try:
                typed = _CubicLatticeFields(data, self._preferences())
            except ValueError:
                self.guidance.setText(
                    "This lattice configuration needs raw JSON editing to repair its values."
                )
            else:
                if self._typed is not None:
                    self.stack.removeWidget(self._typed)
                    self._typed.deleteLater()
                self._typed = typed
                self.stack.addWidget(typed)
                self.stack.setCurrentWidget(typed)
                self._mode_button.setText("Edit raw builder configuration")
                self.guidance.setText(
                    "Lattice positions and angles use your display units; "
                    "orientation is saved as a quaternion."
                )
                return
        raw = _MetadataEditor(data)
        self.stack.removeWidget(self._raw)
        self._raw.deleteLater()
        self._raw = raw
        self.stack.addWidget(raw)
        self.stack.setCurrentWidget(raw)
        self._mode_button.setText("Use quantity-aware lattice fields")
        if builder != "cubic_lattice":
            self.guidance.setText(
                "Builder-specific JSON retains its declared units. Custom values are not rescaled."
            )

    def value(self) -> dict[str, JsonValue]:
        if self._typed is not None and self.stack.currentWidget() is self._typed:
            return self._typed.value()
        return self._raw.value()


class MainWindow(QMainWindow):
    """Studio shell with Robot Pack authoring and live Runtime workspaces."""

    def __init__(
        self,
        *,
        initial_path: str | Path | None = None,
        home: str | Path | None = None,
        session_logger: logging.Logger | None = None,
        session_log_path: str | Path | None = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("ModSim Studio")
        self._appearance = theme_manager()
        self._units = units_manager(home)
        self.header = StudioHeader(self, "Robot Pack Builder")
        self.header.setObjectName("StudioHeader")
        self.addToolBar(self.header)
        self.addToolBarBreak()
        self.project: StudioProject | None = None
        self._current_selection: tuple[str, str, str] | None = None
        self._logger = session_logger or logging.getLogger("modsim_robotics")
        self._session_log_path = (
            Path(session_log_path).resolve() if session_log_path is not None else None
        )
        self._qt_log_handler: _QtLogHandler | None = None
        self._workspace_shown = False

        preview = QWidget()
        preview.setObjectName("Panel")
        preview_layout = QVBoxLayout(preview)
        preview_layout.setContentsMargins(0, 0, 0, 0)
        preview_layout.setSpacing(0)
        self.layer_toolbar = QToolBar("Viewport layers")
        self.layer_toolbar.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        self.layer_toolbar.setIconSize(QSize(16, 16))
        preview_title = QLabel("Module preview   ")
        preview_title.setObjectName("SectionTitle")
        self.layer_toolbar.addWidget(preview_title)
        preview_layout.addWidget(self.layer_toolbar)
        self.viewport = RobotViewport(self)
        self.preview_stack = QStackedWidget()
        welcome = LoadingPanel("Build your Robot Pack")
        welcome.setMinimumSize(0, 0)
        welcome.setObjectName("BrandWelcome")
        welcome.set_stage("Import a URDF, author your connectors, and prepare a simulation.")
        welcome.set_busy(False)
        welcome_buttons = QVBoxLayout()
        welcome_buttons.setObjectName("CoverActions")
        welcome_buttons.setSpacing(12)
        self._welcome_buttons: list[tuple[QPushButton, str, bool]] = []
        for title, callback in (
            ("Create from URDF", self._create_from_urdf_dialog),
            ("Open Robot Pack", self._open_dialog),
        ):
            button = QPushButton(title)
            primary = title == "Create from URDF"
            button.setProperty("primary", primary)
            self._welcome_buttons.append((button, "modules" if primary else "folder", primary))
            button.setMinimumHeight(40)
            button.clicked.connect(callback)
            welcome_buttons.addWidget(button)
        welcome_buttons.addStretch(1)
        welcome_layout = cast(QVBoxLayout, welcome.copy.layout())
        welcome_layout.insertLayout(welcome_layout.count() - 1, welcome_buttons)
        welcome_scroll = QScrollArea()
        welcome_scroll.setWidgetResizable(True)
        welcome_scroll.setFrameShape(QFrame.Shape.NoFrame)
        welcome_scroll.setWidget(welcome)
        self.preview_stack.addWidget(welcome_scroll)
        self.preview_stack.addWidget(self.viewport)
        preview_layout.addWidget(self.preview_stack, 1)
        hint = QLabel(
            "Zero joint pose  ·  Drag to orbit  ·  Wheel to zoom  ·  Click a link to select"
        )
        hint.setObjectName("Hint")
        hint.setWordWrap(True)
        hint.hide()
        self.preview_hint = hint
        preview_layout.addWidget(hint)
        self.setCentralWidget(preview)
        self.viewport.entity_selected.connect(self._select_from_viewport)

        self.project_tree = QTreeWidget()
        self.project_tree.setHeaderLabels(["Robot Pack", "Type"])
        self.project_tree.setHeaderHidden(True)
        self.project_tree.setColumnHidden(1, True)
        self.project_tree.setIndentation(17)
        self.project_tree.setUniformRowHeights(True)
        self.project_tree.setIconSize(QSize(18, 18))
        self.project_tree.currentItemChanged.connect(self._tree_selection_changed)
        project_panel = QWidget()
        project_panel.setObjectName("Panel")
        project_layout = QVBoxLayout(project_panel)
        project_layout.setContentsMargins(8, 8, 8, 8)
        self.project_search = QLineEdit()
        self.project_search.setPlaceholderText("Search project")
        self.project_search.setAccessibleName("Search project")
        self.project_search.setClearButtonEnabled(True)
        self.project_search.textChanged.connect(self._filter_project_tree)
        project_layout.addWidget(self.project_search)
        project_layout.addWidget(self.project_tree, 1)
        project_dock = self._add_dock(
            "Project", project_panel, Qt.DockWidgetArea.LeftDockWidgetArea
        )

        self.properties = QWidget()
        self.properties.setObjectName("InspectorContent")
        self.properties_layout = QVBoxLayout(self.properties)
        self.properties_layout.addWidget(QLabel("Open or create a Robot Pack."))
        self.properties_layout.addStretch(1)
        self.properties_scroll = QScrollArea()
        self.properties_scroll.setWidgetResizable(True)
        self.properties_scroll.setWidget(self.properties)
        properties_dock = self._add_dock(
            "Inspector",
            self.properties_scroll,
            Qt.DockWidgetArea.RightDockWidgetArea,
        )

        self.validation_table = QTableWidget(0, 4)
        self.validation_table.setHorizontalHeaderLabels(["Severity", "Code", "Location", "Message"])
        self.validation_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.validation_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.validation_table.setShowGrid(False)
        self.validation_table.verticalHeader().hide()
        self.validation_table.verticalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents
        )
        self.validation_table.horizontalHeader().setStretchLastSection(True)
        validation_panel = QWidget()
        validation_panel.setObjectName("Panel")
        validation_layout = QVBoxLayout(validation_panel)
        validation_layout.setContentsMargins(16, 12, 16, 12)
        validation_row = QFormLayout()
        validation_row.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        validation_row.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.validation_summary = QLabel("No Robot Pack open")
        self.validation_summary.setObjectName("ValidationSummary")
        self.validation_summary.setWordWrap(True)
        run_validation = QPushButton("Check simulation metadata")
        run_validation.clicked.connect(lambda: self._validate(ValidationProfile.SIMULATION))
        validation_row.addRow(self.validation_summary, run_validation)
        validation_layout.addLayout(validation_row)
        self.validation_detail = QLabel("Open a pack to check its authoring metadata.")
        self.validation_detail.setObjectName("Muted")
        self.validation_detail.setWordWrap(True)
        validation_layout.addWidget(self.validation_detail)
        validation_layout.addWidget(self.validation_table, 1)
        self.validation_table.hide()
        self.yaml_preview = QPlainTextEdit()
        self.yaml_preview.setReadOnly(True)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Studio operations and exception traces appear here.")
        mono = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
        self.yaml_preview.setFont(mono)
        self.log.setFont(mono)
        bottom_tabs = QTabWidget()
        validation_scroll = QScrollArea()
        validation_scroll.setWidgetResizable(True)
        validation_scroll.setFrameShape(QFrame.Shape.NoFrame)
        validation_scroll.setWidget(validation_panel)
        bottom_tabs.addTab(validation_scroll, "Validation")
        bottom_tabs.addTab(self.yaml_preview, "YAML Preview")
        bottom_tabs.addTab(self.log, "Session Log")
        bottom_dock = self._add_dock(
            "Document",
            bottom_tabs,
            Qt.DockWidgetArea.BottomDockWidgetArea,
        )
        bottom_dock.setMinimumHeight(180)
        self.setCorner(Qt.Corner.BottomLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)
        self.setCorner(Qt.Corner.BottomRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)
        self._attach_session_log()

        self._build_actions()
        self._authoring_docks = (project_dock, properties_dock, bottom_dock)
        self._authoring_dock_visibility: dict[QDockWidget, bool] = {}
        self.runtime_workspace = RuntimeWorkspace(
            lambda: self.project, self._save_for_runtime, self, units=self._units
        )
        self.runtime_workspace.project_changed.connect(self._set_project)
        self.runtime_workspace.execution_active_changed.connect(self._runtime_active_changed)
        self.workspaces = QTabWidget()
        self.workspaces.setObjectName("StudioWorkspaces")
        self.workspaces.setDocumentMode(True)
        # takeCentralWidget prevents replacing the authoring preview from
        # scheduling its destruction; both pages live for the Studio session.
        self.takeCentralWidget()
        self.workspaces.addTab(preview, "Robot Pack")
        self.workspaces.addTab(self.runtime_workspace, "Runtime")
        self.setCentralWidget(self.workspaces)
        self.workspaces.currentChanged.connect(self._switch_workspace)
        self._switch_workspace(0)
        self.save_state = QLabel("No pack open")
        self.save_state.setObjectName("SaveState")
        self.statusBar().addPermanentWidget(self.save_state)
        self.units_status = QLabel()
        self.units_status.setToolTip("Authoring and runtime display units; Robot Packs remain SI")
        self.statusBar().addPermanentWidget(self.units_status)
        self._appearance.changed.connect(self._refresh_chrome)
        self._units.changed.connect(self._refresh_units)
        self._refresh_units()
        self._refresh_chrome()
        self.statusBar().showMessage("Ready")
        self._logger.info("Studio window initialized")
        if initial_path is not None:
            self.open_project(initial_path)
        self._window_sizing = manage_window(self, "studio", preferred=QSize(1500, 920))
        if not self._window_sizing.has_saved_layout:
            preview_width = max(300, self.fontMetrics().horizontalAdvance("Create from URDF") + 96)
            side_width = max(0, self.width() - preview_width - 18)
            self.resizeDocks(
                [project_dock, properties_dock],
                [min(250, int(side_width * 0.4)), min(370, int(side_width * 0.6))],
                Qt.Orientation.Horizontal,
            )
            self.resizeDocks([bottom_dock], [180], Qt.Orientation.Vertical)

    def _build_actions(self) -> None:
        file_menu = self.menuBar().addMenu("&File")
        create_action = QAction("Create from &URDF", self)
        create_action.setShortcut("Ctrl+N")
        create_action.triggered.connect(self._create_from_urdf_dialog)
        self.create_action = create_action
        file_menu.addAction(create_action)

        open_action = QAction("&Open Robot Pack", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self._open_dialog)
        self.open_action = open_action
        file_menu.addAction(open_action)

        self.save_action = QAction("&Save", self)
        self.save_action.setShortcut("Ctrl+S")
        self.save_action.setEnabled(False)
        self.save_action.triggered.connect(self._save)
        file_menu.addAction(self.save_action)

        self.export_action = QAction("&Export As", self)
        self.export_action.setShortcut("Ctrl+Shift+S")
        self.export_action.setEnabled(False)
        self.export_action.triggered.connect(self._export_dialog)
        file_menu.addAction(self.export_action)
        file_menu.addSeparator()
        file_menu.addAction("E&xit", self.close)

        validate_menu = self.menuBar().addMenu("&Validate")
        authoring_action = QAction("&Authoring Profile", self)
        authoring_action.setShortcut("F6")
        authoring_action.triggered.connect(lambda: self._validate(ValidationProfile.AUTHORING))
        validate_menu.addAction(authoring_action)
        simulation_action = QAction("&Simulation Metadata (Structural)", self)
        simulation_action.setShortcut("F7")
        simulation_action.triggered.connect(lambda: self._validate(ValidationProfile.SIMULATION))
        validate_menu.addAction(simulation_action)
        runtime_action = QAction("&Runtime Launch", self)
        runtime_action.setShortcut("F8")
        runtime_action.triggered.connect(self._validate_runtime_workspace)
        validate_menu.addAction(runtime_action)

        view_menu = self.menuBar().addMenu("&View")
        for dock in self.findChildren(QDockWidget):
            view_menu.addAction(dock.toggleViewAction())
        view_menu.addSeparator()
        self.units_action = QAction("Display &units", self)
        self.units_action.setObjectName("displayUnitsAction")
        self.units_action.triggered.connect(self._display_units_dialog)
        view_menu.addAction(self.units_action)

        toolbar = QToolBar("Document actions", self)
        toolbar.setObjectName("DocumentToolbar")
        toolbar.setMovable(False)
        toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        toolbar.setIconSize(QSize(18, 18))
        self.addToolBar(toolbar)
        self.document_toolbar = toolbar
        open_action.setIconText("Open pack")
        create_action.setIconText("Import URDF")
        self.save_action.setIconText("Save")
        authoring_action.setIconText("Validate")
        self._icon_actions = [
            (open_action, "folder"),
            (create_action, "cube"),
            (self.save_action, "save"),
            (authoring_action, "check"),
        ]
        for action, _icon in self._icon_actions:
            toolbar.addAction(action)
        toolbar.addSeparator()
        self.breadcrumb = QLabel("No Robot Pack open")
        self.breadcrumb.setObjectName("Breadcrumb")
        self.breadcrumb.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.breadcrumb.setMinimumWidth(0)
        toolbar.addWidget(self.breadcrumb)

        for label, keyword, checked in (
            ("Visual", "visuals", True),
            ("Collision", "collisions", False),
            ("Frames", "frames", False),
            ("Joint axes", "joint_axes", False),
            ("Connectors", "connectors", True),
            ("Ground", "ground", True),
        ):
            action = QAction(label, self)
            action.setCheckable(True)
            action.setChecked(checked)

            def toggle_layer(value: bool, *, key: str = keyword) -> None:
                self.viewport.set_layer_visibility(**{key: value})

            action.toggled.connect(toggle_layer)
            self.layer_toolbar.addAction(action)
        self.layer_toolbar.addSeparator()
        fit_action = QAction("Fit", self)
        fit_action.setToolTip("Fit the module in the viewport")
        fit_action.triggered.connect(self.viewport.fit_module)
        self.layer_toolbar.addAction(fit_action)

    def _refresh_chrome(self) -> None:
        color = self._appearance.theme.accent
        for action, icon in self._icon_actions:
            action.setIcon(studio_icon(icon, color))
        for button, icon, primary in self._welcome_buttons:
            button.setIcon(
                studio_icon(icon, self._appearance.theme.on_accent if primary else color)
            )
        self._decorate_tree()

    def _refresh_units(self) -> None:
        preferences = self._units.preferences
        preserved_draft = False
        editors = (
            self.findChildren(QuantityVectorEdit)
            + self.findChildren(OptionalQuantityEdit)
            + self.findChildren(QuantityListEdit)
            + self.findChildren(QuantityEdit)
        )
        for editor in editors:
            if self.runtime_workspace.isAncestorOf(editor):
                continue
            if isinstance(editor.parentWidget(), QuantityVectorEdit):
                continue
            try:
                changed = editor.set_unit(preferences.unit_for(editor.kind))
            except UnitError:
                changed = False
            if not changed:
                preserved_draft = True
        self.units_status.setText(
            f"Units: {preferences.length_unit} · {preferences.angle_unit} · {preferences.time_unit}"
        )
        if preserved_draft:
            self.statusBar().showMessage(
                "Unfinished fields retain their previous units until corrected."
            )

    def _display_units_dialog(self) -> None:
        show_display_units_dialog(self, self._units)

    def _quantity(
        self, value_si: float | None, kind: QuantityKind, name: str
    ) -> OptionalQuantityEdit:
        field = OptionalQuantityEdit(
            value_si, kind=kind, unit=self._units.preferences.unit_for(kind)
        )
        field.setObjectName(name)
        field.setAccessibleName(name.replace("_", " "))
        field.refresh_tooltip()
        return field

    def _decorate_tree(self) -> None:
        icons = {
            kind: studio_icon(icon, self._appearance.theme.muted)
            for kind, icon in (
                ("pack", "cube"),
                ("module", "cube"),
                ("link", "link"),
                ("joint", "joint"),
                ("connector", "connector"),
                ("connector_type", "connector"),
                ("model_view", "graph"),
                ("asset", "file"),
            )
        }
        folder = studio_icon("folder", self._appearance.theme.muted)
        pending = [
            self.project_tree.topLevelItem(i) for i in range(self.project_tree.topLevelItemCount())
        ]
        while pending:
            item = pending.pop()
            if item is None:
                continue
            item.setIcon(0, icons.get(str(item.data(0, _KIND_ROLE)), folder))
            item.setToolTip(0, f"{item.text(0)} · {item.text(1)}")
            pending.extend(item.child(i) for i in range(item.childCount()))

    def _filter_project_tree(self, query: str) -> None:
        needle = query.strip().casefold()

        def visit(item: QTreeWidgetItem, ancestor_matches: bool = False) -> bool:
            matches = ancestor_matches or needle in f"{item.text(0)} {item.text(1)}".casefold()
            child_matches = [visit(item.child(i), matches) for i in range(item.childCount())]
            visible = matches or any(child_matches)
            item.setHidden(not visible)
            if needle and any(child_matches):
                item.setExpanded(True)
            return visible

        for index in range(self.project_tree.topLevelItemCount()):
            item = self.project_tree.topLevelItem(index)
            if item is not None:
                visit(item)

    def open_project(self, path: str | Path) -> None:
        """Open a Robot Pack and populate all editor panels."""
        if self._runtime_is_active():
            return
        self._logger.info("Opening Robot Pack: %s", Path(path))
        try:
            with BusyDialog("Opening RobotPack…", self, str(path)) as loading:
                project = loading.run(lambda: StudioProject.open(path))
                loading.set_stage("Preparing the module preview…", project.pack.manifest.name)
                self._set_project(project)
        except Exception as error:
            self._show_error("Could not open Robot Pack", error)
            return
        self._logger.info(
            "Opened Robot Pack '%s' from %s",
            project.pack.id,
            project.loaded.root,
        )

    def _set_project(self, project: StudioProject) -> None:
        self.project = project
        self._current_selection = None
        self.save_action.setEnabled(not self._runtime_is_active())
        self.export_action.setEnabled(not self._runtime_is_active())
        self._rebuild_tree()
        self._refresh_document_panels()
        self._render_first_module()
        self._select_tree_entity(("pack", project.pack.id, ""))
        self._update_title()
        self._logger.debug(
            "Document ready: modules=%d, connector_types=%d, model_views=%d, imported_assets=%d",
            len(project.pack.hardware_catalog.module_types),
            len(project.pack.hardware_catalog.connector_types),
            len(project.pack.manifest.model_views),
            len(project.imported_assets),
        )

    def _runtime_is_active(self) -> bool:
        workspace = getattr(self, "runtime_workspace", None)
        return workspace is not None and workspace.is_running

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        if not self._workspace_shown:
            self._workspace_shown = True
            # The sizing policy restores saved docks before the first Show.
            # Reapply the selected mode after that restore, including when a
            # client selected Runtime before showing Studio.
            self._switch_workspace(self.workspaces.currentIndex())

    def _switch_workspace(self, index: int) -> None:
        runtime = index == 1
        for page_index in range(self.workspaces.count()):
            page = self.workspaces.widget(page_index)
            if page is not None:
                policy = (
                    QSizePolicy.Policy.Expanding
                    if page_index == index
                    else QSizePolicy.Policy.Ignored
                )
                page.setSizePolicy(policy, policy)
        self.workspaces.updateGeometry()
        if runtime:
            self._authoring_dock_visibility = {
                dock: not dock.isHidden() for dock in self._authoring_docks
            }
            for dock in self._authoring_docks:
                dock.hide()
            self.runtime_workspace.refresh_project()
        else:
            for dock, visible in self._authoring_dock_visibility.items():
                dock.setVisible(visible)
        for dock in self._authoring_docks:
            dock.toggleViewAction().setEnabled(not runtime)
        self.document_toolbar.setVisible(not runtime)
        workspace_label = self.header.findChild(QLabel, "Workspace")
        if workspace_label is not None:
            workspace_label.setText("Runtime" if runtime else "Robot Pack Builder")

    def _validate_runtime_workspace(self) -> None:
        self.workspaces.setCurrentIndex(1)
        self.runtime_workspace.validate_runtime()

    def _runtime_active_changed(self, active: bool) -> None:
        # Edits can remain in memory while a saved snapshot runs. Persistence
        # and project replacement wait for the execution owner to shut down.
        self.create_action.setEnabled(not active)
        self.open_action.setEnabled(not active)
        self.save_action.setEnabled(not active and self.project is not None)
        self.export_action.setEnabled(not active and self.project is not None)
        self.statusBar().showMessage(
            "Simulation running from the saved pack; save and project changes resume after Stop."
            if active
            else "Runtime stopped; results remain available in the Runtime workspace."
        )

    def _save_for_runtime(self) -> bool:
        self._save()
        return self.project is not None and not self.project.dirty

    def _rebuild_tree(self) -> None:
        self.project_tree.clear()
        if self.project is None:
            return
        root = QTreeWidgetItem([self.project.pack.manifest.name, "Robot Pack"])
        self._set_item_data(root, "pack", self.project.pack.id, "")
        self.project_tree.addTopLevelItem(root)

        assets_item = QTreeWidgetItem(["Assets", "Catalog"])
        root.addChild(assets_item)
        for asset_id, asset in self.project.imported_assets.items():
            asset_item = QTreeWidgetItem([asset_id, "URDF"])
            self._set_item_data(asset_item, "asset", asset_id, "")
            assets_item.addChild(asset_item)
            for warning in asset.warnings:
                asset_item.addChild(QTreeWidgetItem([warning, "Warning"]))

        model_views_item = QTreeWidgetItem(["Model Views", "Catalog"])
        self._set_item_data(model_views_item, "model_views", "", "")
        root.addChild(model_views_item)
        for model_view in self.project.pack.manifest.model_views:
            label = model_view.name or model_view.id
            item = QTreeWidgetItem([label, model_view.builder])
            self._set_item_data(item, "model_view", model_view.id, "")
            model_views_item.addChild(item)

        connector_types_item = QTreeWidgetItem(["Connector Types", "Catalog"])
        self._set_item_data(connector_types_item, "connector_types", "", "")
        root.addChild(connector_types_item)
        for type_id, connector_type in self.project.pack.hardware_catalog.connector_types.items():
            item = QTreeWidgetItem([connector_type.name or type_id, connector_type.gender.value])
            self._set_item_data(item, "connector_type", type_id, "")
            connector_types_item.addChild(item)

        modules_item = QTreeWidgetItem(["Module Types", "Catalog"])
        root.addChild(modules_item)
        for module_id, module in self.project.pack.hardware_catalog.module_types.items():
            module_item = QTreeWidgetItem([module.name or module_id, "Module"])
            self._set_item_data(module_item, "module", module_id, module_id)
            modules_item.addChild(module_item)
            asset = self.project.imported_assets.get(module.asset_ref)

            links_item = QTreeWidgetItem(["Links", str(len(asset.links) if asset else 0)])
            module_item.addChild(links_item)
            if asset is not None:
                for link in asset.links:
                    item = QTreeWidgetItem([link.name, "Link"])
                    self._set_item_data(item, "link", link.name, module_id)
                    links_item.addChild(item)

            joints_item = QTreeWidgetItem(["Joints", str(len(module.joints))])
            module_item.addChild(joints_item)
            for joint in module.joints:
                item = QTreeWidgetItem([joint.id, joint.type.value])
                self._set_item_data(item, "joint", joint.id, module_id)
                joints_item.addChild(item)

            connectors_item = QTreeWidgetItem(["Connectors", str(len(module.connectors))])
            module_item.addChild(connectors_item)
            for connector in module.connectors:
                item = QTreeWidgetItem([connector.id, connector.connector_type])
                self._set_item_data(item, "connector", connector.id, module_id)
                connectors_item.addChild(item)
            module_item.setExpanded(True)
            links_item.setExpanded(True)
            joints_item.setExpanded(True)
            connectors_item.setExpanded(True)
        root.setExpanded(True)
        assets_item.setExpanded(True)
        model_views_item.setExpanded(True)
        connector_types_item.setExpanded(True)
        modules_item.setExpanded(True)
        self.project_tree.resizeColumnToContents(0)
        self._decorate_tree()
        self._filter_project_tree(self.project_search.text())

    def _refresh_document_panels(self) -> None:
        if self.project is None:
            return
        try:
            self.yaml_preview.setPlainText(self.project.yaml_preview())
        except Exception as error:
            self.yaml_preview.setPlainText(f"YAML preview failed: {error}")
            self._logger.error(
                "YAML preview failed",
                exc_info=(type(error), error, error.__traceback__),
            )
        self._validate(ValidationProfile.AUTHORING)
        if hasattr(self, "runtime_workspace"):
            self.runtime_workspace.refresh_project()

    def _render_first_module(self) -> None:
        if self.project is None:
            return
        for module in self.project.pack.hardware_catalog.module_types.values():
            asset = self.project.imported_assets.get(module.asset_ref)
            if asset is not None:
                self.viewport.render_module(asset, module)
                self.preview_stack.setCurrentWidget(self.viewport)
                self.preview_hint.show()
                return

    def _render_module(self, module_id: str) -> None:
        if self.project is None:
            return
        module = self.project.pack.hardware_catalog.module_types[module_id]
        asset = self.project.imported_assets.get(module.asset_ref)
        if asset is not None:
            self.viewport.render_module(asset, module)
            self._logger.debug("Rendered module '%s'", module_id)

    def _create_from_urdf_dialog(self) -> None:
        if self._runtime_is_active():
            return
        urdf_path = _select_path(
            self,
            "Select a URDF",
            name_filter="URDF files (*.urdf *.xml);;All files (*)",
        )
        if not urdf_path:
            return
        asset_roots: tuple[str | Path, ...] = ()
        try:
            with BusyDialog("Inspecting URDF…", self, urdf_path) as loading:
                preview = loading.run(lambda: URDFImporter().load(urdf_path))
        except Exception as error:
            self._show_error("Could not inspect URDF", error)
            return
        self._logger.info("Inspected URDF for import: %s", urdf_path)
        if any("mesh could not be resolved" in warning for warning in preview.warnings):
            asset_root = _select_path(
                self,
                "Select the package or mesh asset root",
                str(Path(urdf_path).parent),
                file_mode=QFileDialog.FileMode.Directory,
            )
            if not asset_root:
                return
            asset_roots = (asset_root,)
        destination = _select_path(
            self,
            "Choose a new Robot Pack folder",
            str(Path(urdf_path).with_suffix("")),
            "Robot Pack folder (*)",
            file_mode=QFileDialog.FileMode.AnyFile,
            accept_mode=QFileDialog.AcceptMode.AcceptSave,
        )
        if not destination:
            return
        try:
            with BusyDialog("Creating RobotPack…", self, destination) as loading:
                project = loading.run(
                    lambda: StudioProject.create_from_urdf(
                        urdf_path, destination, asset_roots=asset_roots
                    )
                )
                loading.set_stage("Preparing the module preview…", project.pack.manifest.name)
                self._set_project(project)
        except Exception as error:
            self._show_error("Could not create Robot Pack", error)
            return
        warnings = next(iter(project.imported_assets.values())).warnings
        self._logger.info("Created draft Robot Pack: %s", project.loaded.root)
        if warnings:
            for warning in warnings:
                self._logger.warning("URDF import: %s", warning)
        else:
            self._logger.info("URDF import completed without warnings")

    def _open_dialog(self) -> None:
        path = _select_path(
            self,
            "Open Robot Pack",
            file_mode=QFileDialog.FileMode.Directory,
        )
        if path:
            self.open_project(path)

    def _export_dialog(self) -> None:
        if self.project is None or self._runtime_is_active():
            return
        destination = _select_path(
            self,
            "Export to a new Robot Pack folder",
            str(self.project.loaded.root.with_name(self.project.loaded.root.name + "_edited")),
            "Robot Pack folder (*)",
            file_mode=QFileDialog.FileMode.AnyFile,
            accept_mode=QFileDialog.AcceptMode.AcceptSave,
        )
        if not destination:
            return
        current = self.project
        try:
            with BusyDialog("Exporting RobotPack…", self, destination) as loading:
                project = loading.run(lambda: current.export(destination))
                reopened = loading.run(lambda: StudioProject.open(project.loaded.root))
                loading.set_stage("Preparing the exported pack…", reopened.pack.manifest.name)
                self._set_project(reopened)
        except Exception as error:
            self._show_error("Could not export Robot Pack", error)
            return
        self._logger.info("Exported Robot Pack to %s", destination)
        self.statusBar().showMessage(f"Exported {destination}", 8000)

    def _save(self) -> None:
        if self.project is None or self._runtime_is_active():
            return
        current = self.project
        try:
            with BusyDialog("Saving RobotPack…", self, str(current.loaded.root)) as loading:
                self.project = loading.run(current.save)
        except Exception as error:
            self._show_error("Could not save Robot Pack", error)
            return
        self._refresh_document_panels()
        self._update_title()
        self._logger.info("Saved Robot Pack to %s", self.project.loaded.root)
        self.statusBar().showMessage(f"Saved {self.project.loaded.root}", 8000)

    def _validate(self, profile: ValidationProfile) -> None:
        if self.project is None:
            return
        report = self.project.validate(profile)
        profile_name = (
            "Authoring validation"
            if profile == ValidationProfile.AUTHORING
            else "Simulation metadata (structural)"
        )
        state = "error" if report.errors else "warning" if report.warnings else "success"
        self.validation_summary.setText(
            f"{profile_name} {'passed' if report.valid else 'needs attention'}"
        )
        self.validation_summary.setProperty("state", state)
        self.validation_summary.style().unpolish(self.validation_summary)
        self.validation_summary.style().polish(self.validation_summary)
        self.validation_detail.setText(
            f"{len(report.errors)} errors  ·  {len(report.warnings)} warnings"
        )
        self.validation_table.setVisible(bool(report.issues))
        self.validation_table.setRowCount(len(report.issues))
        for row, issue in enumerate(report.issues):
            for column, value in enumerate(
                (issue.severity.value, issue.code, issue.location, issue.message)
            ):
                self.validation_table.setItem(row, column, QTableWidgetItem(value))
        self.validation_table.resizeColumnsToContents()
        status = "valid" if report.valid else "invalid"
        self._logger.info(
            "Validation profile=%s result=%s errors=%d warnings=%d",
            profile.value,
            status,
            len(report.errors),
            len(report.warnings),
        )
        for issue in report.issues:
            level = logging.ERROR if issue.severity.value == "error" else logging.WARNING
            self._logger.log(
                level,
                "Validation issue profile=%s code=%s location=%s message=%s",
                profile.value,
                issue.code,
                issue.location,
                issue.message,
            )
        self.statusBar().showMessage(
            f"{profile.value}: {status} — {len(report.errors)} errors, "
            f"{len(report.warnings)} warnings"
        )

    def _tree_selection_changed(
        self,
        current: QTreeWidgetItem | None,
        _previous: QTreeWidgetItem | None,
    ) -> None:
        if current is None:
            self._current_selection = None
            self._replace_properties(QLabel("Select a Robot Pack entity to edit."))
            return
        kind = current.data(0, _KIND_ROLE)
        entity_id = current.data(0, _ID_ROLE)
        module_id = current.data(0, _MODULE_ROLE)
        if not all(isinstance(value, str) for value in (kind, entity_id, module_id)):
            self._current_selection = None
            self._replace_properties(QLabel("This category has no editable fields."))
            return
        self._current_selection = (kind, entity_id, module_id)
        if self.project is not None:
            self.breadcrumb.setText(
                f"{self.project.pack.id}  /  {kind.replace('_', ' ')}  /  {entity_id}"
            )
            self.breadcrumb.setToolTip(self.breadcrumb.text())
        self._logger.debug(
            "Selected entity kind=%s id=%s module=%s",
            kind,
            entity_id,
            module_id or "-",
        )
        if module_id:
            self._render_module(module_id)
        self.viewport.select_entity(kind, entity_id)
        self._show_properties(kind, entity_id, module_id)

    def _select_from_viewport(self, kind: str, entity_id: str) -> None:
        iterator = self.project_tree.findItems(
            entity_id,
            Qt.MatchFlag.MatchExactly | Qt.MatchFlag.MatchRecursive,
            0,
        )
        for item in iterator:
            if item.data(0, _KIND_ROLE) == kind:
                self.project_tree.setCurrentItem(item)
                return

    def _show_properties(self, kind: str, entity_id: str, module_id: str) -> None:
        if kind == "pack":
            self._show_pack_properties()
        elif kind == "link":
            self._show_link_properties(entity_id, module_id)
        elif kind == "joint":
            self._show_joint_properties(entity_id, module_id)
        elif kind == "connector":
            self._show_connector_properties(entity_id, module_id)
        elif kind == "connector_type":
            self._show_connector_type_properties(entity_id)
        elif kind == "connector_types":
            self._show_connector_types_properties()
        elif kind == "model_view":
            self._show_model_view_properties(entity_id)
        elif kind == "model_views":
            self._show_model_views_properties()
        elif kind == "module":
            self._show_module_properties(entity_id)
        else:
            self._replace_properties(QLabel(f"{kind}: {entity_id}"))

    def _show_pack_properties(self) -> None:
        if self.project is None:
            return
        manifest = self.project.pack.manifest
        panel = QWidget()
        form = QFormLayout(panel)
        form.addRow("Pack ID", QLabel(manifest.id))
        name = QLineEdit(manifest.name)
        version = QLineEdit(manifest.version)
        description = QPlainTextEdit(manifest.description or "")
        description.setMaximumHeight(110)
        metadata = _MetadataEditor(manifest.metadata)
        form.addRow("Name", name)
        form.addRow("Version", version)
        form.addRow("Description", description)
        form.addRow("Custom metadata", metadata)
        apply_button = QPushButton("Apply metadata")
        apply_button.clicked.connect(
            lambda: self._apply_project_change(
                lambda project: project.update_manifest(
                    name=name.text(),
                    version=version.text(),
                    description=description.toPlainText(),
                    metadata=metadata.value(),
                )
            )
        )
        form.addRow(apply_button)
        self._replace_properties(panel)

    def _show_module_properties(self, module_id: str) -> None:
        if self.project is None:
            return
        module = self.project.pack.hardware_catalog.module_types[module_id]
        asset = self.project.imported_assets.get(module.asset_ref)
        panel = QWidget()
        form = QFormLayout(panel)
        form.addRow("Module ID", QLabel(module.id))
        name = QLineEdit(module.name or "")
        form.addRow("Name", name)
        form.addRow("URDF asset", QLabel(module.asset_ref))
        root_link = QComboBox()
        link_names = (
            [link.name for link in asset.links] if asset is not None else [module.root_link]
        )
        root_link.addItems(link_names)
        root_link.setCurrentText(module.root_link)
        mass = self._quantity(module.mass_kg, QuantityKind.MASS, "mass_kg")
        form.addRow("Root link", root_link)
        form.addRow("Mass", mass)
        form.addRow(
            "Capabilities",
            QLabel(", ".join(module.capabilities) if module.capabilities else "none"),
        )

        apply_button = QPushButton("Apply module metadata")
        apply_button.clicked.connect(
            lambda: self._apply_project_change(
                lambda project: project.update_module_metadata(
                    module_id,
                    name.text().strip() or None,
                    root_link.currentText(),
                    mass.value_si(),
                )
            )
        )
        form.addRow(apply_button)

        guidance = QLabel(
            "The module name, root link, and mass are Robot Pack metadata. "
            "Geometry, mesh files, and kinematics come from the URDF; edit and "
            "reimport the URDF to change the rendered cube itself."
        )
        guidance.setWordWrap(True)
        form.addRow(guidance)
        self._replace_properties(panel)

    def _show_link_properties(self, link_name: str, module_id: str) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(QLabel(f"<b>Link</b><br>{link_name}"))
        add_button = QPushButton("Add connector to this link")
        add_button.clicked.connect(lambda: self._add_connector_dialog(module_id, link_name))
        layout.addWidget(add_button)
        layout.addStretch(1)
        self._replace_properties(panel)

    def _show_joint_properties(self, joint_id: str, module_id: str) -> None:
        if self.project is None:
            return
        module = self.project.pack.hardware_catalog.module_types[module_id]
        joint = next(item for item in module.joints if item.id == joint_id)
        panel = QWidget()
        form = QFormLayout(panel)
        form.addRow("Joint ID", QLabel(joint.id))
        form.addRow("Source joint", QLabel(joint.source_joint_name))
        form.addRow("Type", QLabel(joint.type.value))
        form.addRow("Parent → child", QLabel(f"{joint.parent_link} → {joint.child_link}"))
        modes = QLineEdit(", ".join(mode.value for mode in joint.control_modes))
        form.addRow("Control modes", modes)
        limits = joint.limits or JointLimits()
        fields: dict[str, OptionalQuantityEdit] = {}
        for label, field_name, kind in _joint_limit_fields(joint.type):
            field = self._quantity(getattr(limits, field_name), kind, field_name)
            fields[field_name] = field
            form.addRow(label, field)
        apply_button = QPushButton("Apply joint metadata")

        def update(project: StudioProject) -> StudioProject:
            parsed_modes = tuple(
                ControlMode(value.strip()) for value in modes.text().split(",") if value.strip()
            )
            limit_data = {field_name: field.value_si() for field_name, field in fields.items()}
            updated_limits = (
                JointLimits(**limit_data)
                if any(value is not None for value in limit_data.values())
                else None
            )
            updated = JointSpec.model_validate(
                {
                    **joint.model_dump(mode="python"),
                    "control_modes": parsed_modes,
                    "limits": updated_limits,
                }
            )
            return project.update_joint(module_id, updated)

        apply_button.clicked.connect(lambda: self._apply_project_change(update))
        form.addRow(apply_button)
        self._replace_properties(panel)

    def _show_connector_properties(self, connector_id: str, module_id: str) -> None:
        if self.project is None:
            return
        module = self.project.pack.hardware_catalog.module_types[module_id]
        connector = next(item for item in module.connectors if item.id == connector_id)
        asset = self.project.imported_assets.get(module.asset_ref)
        panel = QWidget()
        sections = QVBoxLayout(panel)
        sections.setContentsMargins(0, 0, 0, 0)
        form = property_group("Identity", sections)
        form.addRow("Connector ID", QLabel(connector.id))
        connector_type = QComboBox()
        connector_type.addItems(list(self.project.pack.hardware_catalog.connector_types))
        connector_type.setCurrentText(connector.connector_type)
        parent_link = QComboBox()
        parent_link.addItems(
            [link.name for link in asset.links] if asset is not None else [connector.parent_link]
        )
        parent_link.setCurrentText(connector.parent_link)
        form.addRow("Type", connector_type)
        form.addRow("Parent link", parent_link)
        form = property_group("Local pose", sections)
        frame = QLineEdit(connector.frame or "")
        use_local_pose = QCheckBox()
        use_local_pose.setChecked(connector.local_pose is not None)
        frame.setPlaceholderText("Optional")
        form.addRow("Named frame", frame)
        form.addRow("Numeric pose", use_local_pose)
        pose = connector.local_pose or PoseSpec()
        xyz = QuantityVectorEdit(
            pose.xyz_m,
            kind=QuantityKind.LENGTH,
            unit=self._units.preferences.length_unit,
            labels="XYZ",
            name="Position",
        )
        rpy = QuantityVectorEdit(
            pose.rpy_rad,
            kind=QuantityKind.ANGLE,
            unit=self._units.preferences.angle_unit,
            labels="RPY",
            name="Roll / Pitch / Yaw",
        )
        use_local_pose.toggled.connect(xyz.setEnabled)
        use_local_pose.toggled.connect(rpy.setEnabled)
        xyz.setEnabled(use_local_pose.isChecked())
        rpy.setEnabled(use_local_pose.isChecked())
        docking = QLineEdit(
            _vector_text(connector.docking_axis) if connector.docking_axis is not None else ""
        )
        approach = QLineEdit(
            _vector_text(connector.approach_axis) if connector.approach_axis is not None else ""
        )
        form.addRow("Position XYZ", xyz)
        form.addRow("Rotation RPY", rpy)
        form = property_group("Axes", sections)
        form.addRow("Docking axis", docking)
        form.addRow("Approach axis", approach)
        metadata = _MetadataEditor(connector.metadata)
        sections.addWidget(DetailsSection(f"Custom fields · {len(connector.metadata)}", metadata))

        apply_button = QPushButton("Apply connector")

        def update(project: StudioProject) -> StudioProject:
            updated = ConnectorSpec.model_validate(
                {
                    **connector.model_dump(mode="python"),
                    "connector_type": connector_type.currentText(),
                    "parent_link": parent_link.currentText(),
                    "frame": frame.text().strip() or None,
                    "local_pose": (
                        PoseSpec(
                            xyz_m=xyz.value_si(),
                            rpy_rad=rpy.value_si(),
                        )
                        if use_local_pose.isChecked()
                        else None
                    ),
                    "docking_axis": _optional_vector(docking.text()),
                    "approach_axis": _optional_vector(approach.text()),
                    "metadata": metadata.value(),
                }
            )
            return project.update_connector(module_id, updated)

        apply_button.clicked.connect(
            lambda: self._apply_project_change(
                update,
                selection_after=("connector", connector_id, module_id),
            )
        )
        sections.addWidget(apply_button)
        remove_button = QPushButton("Remove connector")
        remove_button.clicked.connect(
            lambda: self._apply_project_change(
                lambda project: project.remove_connector(module_id, connector_id),
                selection_after=("link", connector.parent_link, module_id),
            )
        )
        sections.addWidget(remove_button)
        self._replace_properties(panel)

    def _show_connector_types_properties(self) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(QLabel("Create reusable connector types before assigning connectors."))
        add_button = QPushButton("Add connector type")
        add_button.clicked.connect(self._add_connector_type_dialog)
        layout.addWidget(add_button)
        layout.addStretch(1)
        self._replace_properties(panel)

    def _show_model_views_properties(self) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        guidance = QLabel(
            "Model-view recipes select and configure registered builders. The generated "
            "view data comes from the Robot Pack or changing runtime state."
        )
        guidance.setWordWrap(True)
        layout.addWidget(guidance)
        add_button = QPushButton("Add model view")
        add_button.clicked.connect(self._add_model_view_dialog)
        layout.addWidget(add_button)
        layout.addStretch(1)
        self._replace_properties(panel)

    def _show_model_view_properties(self, model_view_id: str) -> None:
        if self.project is None:
            return
        model_view = next(
            item for item in self.project.pack.manifest.model_views if item.id == model_view_id
        )
        panel = QWidget()
        form = QFormLayout(panel)
        form.addRow("Model view ID", QLabel(model_view.id))
        name = QLineEdit(model_view.name or "")
        builder = QLineEdit(model_view.builder)
        authoring_mode = QCheckBox()
        authoring_mode.setChecked(ModelViewMode.AUTHORING in model_view.modes)
        runtime_mode = QCheckBox()
        runtime_mode.setChecked(ModelViewMode.RUNTIME in model_view.modes)
        default_view = QCheckBox()
        default_view.setChecked(model_view.default)
        configuration = _ModelViewConfigurationEditor(
            model_view.configuration, model_view.builder, lambda: self._units.preferences
        )
        builder.textChanged.connect(configuration.set_builder)
        form.addRow("Name", name)
        form.addRow("Builder", builder)
        form.addRow("Available while authoring", authoring_mode)
        form.addRow("Available at runtime", runtime_mode)
        form.addRow("Default view", default_view)
        form.addRow("Builder configuration", configuration)

        apply_button = QPushButton("Apply model view")

        def update(project: StudioProject) -> StudioProject:
            updated = ModelViewSpec(
                id=model_view.id,
                name=name.text().strip() or None,
                builder=validate_identifier(builder.text(), field_name="Builder ID"),
                modes=_model_view_modes(authoring_mode, runtime_mode),
                default=default_view.isChecked(),
                configuration=configuration.value(),
            )
            return project.update_model_view(updated)

        apply_button.clicked.connect(
            lambda: self._apply_project_change(
                update,
                selection_after=("model_view", model_view_id, ""),
            )
        )
        form.addRow(apply_button)
        remove_button = QPushButton("Remove model view")
        remove_button.clicked.connect(
            lambda: self._apply_project_change(
                lambda project: project.remove_model_view(model_view_id),
                selection_after=("model_views", "", ""),
            )
        )
        form.addRow(remove_button)
        self._replace_properties(panel)

    def _add_model_view_dialog(self) -> None:
        if self.project is None:
            return
        dialog = _FormDialog(self, "Add model view", "add-model-view", QSize(760, 700))
        layout = dialog.form
        model_view_id = QLineEdit()
        model_view_id.setPlaceholderText("for example: smores_topology")
        name = QLineEdit()
        builder = QLineEdit("module_topology_graph")
        authoring_mode = QCheckBox()
        runtime_mode = QCheckBox()
        runtime_mode.setChecked(True)
        default_view = QCheckBox()
        configuration = _ModelViewConfigurationEditor(
            {}, builder.text(), lambda: self._units.preferences
        )
        builder.textChanged.connect(configuration.set_builder)
        layout.addRow("Model view ID", model_view_id)
        id_help = QLabel("Lowercase YAML ID, such as topology or smores_topology.")
        id_help.setWordWrap(True)
        layout.addRow("", id_help)
        layout.addRow("Name", name)
        layout.addRow("Builder", builder)
        layout.addRow("Available while authoring", authoring_mode)
        layout.addRow("Available at runtime", runtime_mode)
        layout.addRow("Default view", default_view)
        layout.addRow("Builder configuration", configuration)
        model_view: ModelViewSpec | None = None
        while dialog.exec() == int(QDialog.DialogCode.Accepted):
            try:
                validated_model_view_id = validate_identifier(
                    model_view_id.text(),
                    field_name="Model view ID",
                )
                model_view = ModelViewSpec(
                    id=validated_model_view_id,
                    name=name.text().strip() or None,
                    builder=validate_identifier(builder.text(), field_name="Builder ID"),
                    modes=_model_view_modes(authoring_mode, runtime_mode),
                    default=default_view.isChecked(),
                    configuration=configuration.value(),
                )
            except Exception as error:
                self._show_error("Invalid model view", error)
                continue
            break
        if model_view is None:
            return
        self._apply_project_change(
            lambda project: project.add_model_view(model_view),
            selection_after=("model_view", model_view.id, ""),
        )

    def _show_connector_type_properties(self, type_id: str) -> None:
        if self.project is None:
            return
        connector_type = self.project.pack.hardware_catalog.connector_types[type_id]
        panel = QWidget()
        sections = QVBoxLayout(panel)
        sections.setContentsMargins(0, 0, 0, 0)
        form = property_group("Identity & compatibility", sections)
        form.addRow("Connector type ID", QLabel(connector_type.id))
        name = QLineEdit(connector_type.name or "")
        active = QCheckBox()
        active.setChecked(connector_type.active)
        gender = _enum_combo(ConnectorGender, connector_type.gender.value)
        compatible = QLineEdit(", ".join(connector_type.compatible_with))
        form.addRow("Name", name)
        form.addRow("Active connector", active)
        form.addRow("Gender", gender)
        form.addRow("Compatible with", compatible)

        form = property_group("Allowed orientations", sections)
        orientation_mode = QComboBox()
        orientation_mode.setObjectName("orientation_mode")
        orientation_mode.addItems(["", *[item.value for item in OrientationMode]])
        if connector_type.allowed_orientations is not None:
            orientation_mode.setCurrentText(connector_type.allowed_orientations.mode.value)
        orientation_values = QuantityListEdit(
            connector_type.allowed_orientations.values_rad
            if connector_type.allowed_orientations is not None
            else (),
            kind=QuantityKind.ANGLE,
            unit=self._units.preferences.unit_for(QuantityKind.ANGLE),
        )
        orientation_values.setObjectName("values_rad")
        orientation_values.setAccessibleName("Allowed orientation values")
        orientation_values.refresh_tooltip()
        form.addRow("Orientation mode", orientation_mode)
        form.addRow("Allowed values", orientation_values)

        form = property_group("Acceptance region", sections)
        acceptance = connector_type.acceptance_region
        acceptance_shape = _enum_combo(
            AcceptanceShape,
            acceptance.shape.value if acceptance is not None else AcceptanceShape.BOX.value,
        )
        position_tolerance = self._quantity(
            acceptance.position_tolerance_m if acceptance is not None else None,
            QuantityKind.LENGTH,
            "position_tolerance_m",
        )
        orientation_tolerance = self._quantity(
            acceptance.orientation_tolerance_rad if acceptance is not None else None,
            QuantityKind.ANGLE,
            "orientation_tolerance_rad",
        )
        relative_velocity = self._quantity(
            acceptance.max_relative_velocity_m_s if acceptance is not None else None,
            QuantityKind.LINEAR_VELOCITY,
            "max_relative_velocity_m_s",
        )
        form.addRow("Acceptance shape", acceptance_shape)
        form.addRow("Position tolerance", position_tolerance)
        form.addRow("Orientation tolerance", orientation_tolerance)
        form.addRow("Max relative velocity", relative_velocity)

        form = property_group("Physical connection", sections)
        physical = connector_type.physical_connection
        constraint = QComboBox()
        constraint.setObjectName("constraint")
        constraint.addItems(["", *[item.value for item in PhysicalConstraintType]])
        if physical is not None:
            constraint.setCurrentText(physical.constraint.value)
        translational_stiffness = self._quantity(
            physical.compliance.translational_stiffness_n_per_m
            if physical is not None and physical.compliance is not None
            else None,
            QuantityKind.LINEAR_STIFFNESS,
            "translational_stiffness_n_per_m",
        )
        rotational_stiffness = self._quantity(
            physical.compliance.rotational_stiffness_nm_per_rad
            if physical is not None and physical.compliance is not None
            else None,
            QuantityKind.ANGULAR_STIFFNESS,
            "rotational_stiffness_nm_per_rad",
        )
        hinge_axis = QLineEdit(
            _vector_text(physical.hinge.axis)
            if physical is not None and physical.hinge is not None
            else ""
        )
        hinge_axis.setObjectName("hinge_axis")
        hinge_anchor_separation = self._quantity(
            physical.hinge.anchor_separation_m
            if physical is not None and physical.hinge is not None
            else None,
            QuantityKind.LENGTH,
            "anchor_separation_m",
        )

        def set_constraint_fields_enabled(value: str) -> None:
            compliant = value == PhysicalConstraintType.COMPLIANT.value
            hinged = value == PhysicalConstraintType.HINGE.value
            translational_stiffness.setEnabled(compliant)
            rotational_stiffness.setEnabled(compliant)
            hinge_axis.setEnabled(hinged)
            hinge_anchor_separation.setEnabled(hinged)

        constraint.currentTextChanged.connect(set_constraint_fields_enabled)
        set_constraint_fields_enabled(constraint.currentText())
        form.addRow("Physical constraint", constraint)
        form.addRow("Translation stiffness", translational_stiffness)
        form.addRow("Rotation stiffness", rotational_stiffness)
        form.addRow("Hinge axis (x, y, z)", hinge_axis)
        form.addRow("Hinge anchor separation", hinge_anchor_separation)

        form = property_group("Load limits", sections)
        limits = connector_type.limits
        max_normal = self._quantity(
            limits.max_normal_force_n if limits is not None else None,
            QuantityKind.FORCE,
            "max_normal_force_n",
        )
        max_shear = self._quantity(
            limits.max_shear_force_n if limits is not None else None,
            QuantityKind.FORCE,
            "max_shear_force_n",
        )
        max_bending = self._quantity(
            limits.max_bending_moment_nm if limits is not None else None,
            QuantityKind.TORQUE,
            "max_bending_moment_nm",
        )
        supports_undocking = QCheckBox()
        supports_undocking.setChecked(connector_type.supports_undocking)
        form.addRow("Max normal force", max_normal)
        form.addRow("Max shear force", max_shear)
        form.addRow("Max bending moment", max_bending)
        form = property_group("Docking policy", sections)
        form.addRow("Supports undocking", supports_undocking)

        policy = connector_type.docking_policy
        declare_policy = QCheckBox()
        declare_policy.setObjectName("declare_policy")
        declare_policy.setChecked(policy is not None)
        auto_latch = QCheckBox()
        auto_latch.setChecked(policy.auto_latch if policy is not None else False)
        alignment = _enum_combo(
            AlignmentMode,
            policy.alignment.value if policy is not None else AlignmentMode.MEASURED.value,
        )
        redock_cooldown = self._quantity(
            policy.redock_cooldown_s if policy is not None else None,
            QuantityKind.TIME,
            "redock_cooldown_s",
        )
        break_force = self._quantity(
            policy.break_force_n if policy is not None else None,
            QuantityKind.FORCE,
            "break_force_n",
        )

        def set_policy_fields_enabled(enabled: bool) -> None:
            auto_latch.setEnabled(enabled)
            alignment.setEnabled(enabled)
            redock_cooldown.setEnabled(enabled)
            break_force.setEnabled(enabled)

        declare_policy.toggled.connect(set_policy_fields_enabled)
        set_policy_fields_enabled(declare_policy.isChecked())
        form.addRow("Declare docking policy", declare_policy)
        form.addRow("Automatic latching", auto_latch)
        form.addRow("Docking alignment", alignment)
        form.addRow("Redock cooldown", redock_cooldown)
        form.addRow("Break force", break_force)
        metadata = _MetadataEditor(connector_type.metadata)
        form = property_group("Custom metadata", sections)
        form.addRow(metadata)

        apply_button = QPushButton("Apply connector type")

        def update(project: StudioProject) -> StudioProject:
            mode_value = orientation_mode.currentText()
            allowed_orientations = (
                AllowedOrientations(
                    mode=OrientationMode(mode_value),
                    values_rad=(
                        orientation_values.value_si()
                        if mode_value == OrientationMode.DISCRETE.value
                        else ()
                    ),
                )
                if mode_value
                else None
            )
            acceptance_values = (
                position_tolerance.value_si(),
                orientation_tolerance.value_si(),
                relative_velocity.value_si(),
            )
            if any(value is not None for value in acceptance_values) and any(
                value is None for value in acceptance_values
            ):
                raise ValueError("all three acceptance-region values are required together")
            complete_acceptance = cast(
                tuple[float, float, float],
                acceptance_values,
            )
            acceptance_region = (
                AcceptanceRegion(
                    shape=AcceptanceShape(acceptance_shape.currentText()),
                    position_tolerance_m=complete_acceptance[0],
                    orientation_tolerance_rad=complete_acceptance[1],
                    max_relative_velocity_m_s=complete_acceptance[2],
                )
                if any(value is not None for value in acceptance_values)
                else None
            )
            constraint_value = constraint.currentText()
            compliance_values = (
                translational_stiffness.value_si(),
                rotational_stiffness.value_si(),
            )
            compliance = (
                ComplianceSpec(
                    translational_stiffness_n_per_m=compliance_values[0],
                    rotational_stiffness_nm_per_rad=compliance_values[1],
                )
                if constraint_value == PhysicalConstraintType.COMPLIANT.value
                else None
            )
            hinge = None
            if constraint_value == PhysicalConstraintType.HINGE.value:
                axis = _optional_vector(hinge_axis.text())
                anchor_separation_m = hinge_anchor_separation.value_si()
                if axis is None or anchor_separation_m is None:
                    raise ValueError(
                        "hinge constraints require a three-component axis and anchor separation"
                    )
                hinge = HingeConstraintSpec(
                    axis=axis,
                    anchor_separation_m=anchor_separation_m,
                )
            physical_connection = (
                PhysicalConnectionSpec(
                    constraint=PhysicalConstraintType(constraint_value),
                    compliance=compliance,
                    hinge=hinge,
                )
                if constraint_value
                else None
            )
            limit_values = (
                max_normal.value_si(),
                max_shear.value_si(),
                max_bending.value_si(),
            )
            connector_limits = (
                ConnectorLimits(
                    max_normal_force_n=limit_values[0],
                    max_shear_force_n=limit_values[1],
                    max_bending_moment_nm=limit_values[2],
                )
                if any(value is not None for value in limit_values)
                else None
            )
            docking_policy = (
                DockingPolicySpec(
                    auto_latch=auto_latch.isChecked(),
                    alignment=AlignmentMode(alignment.currentText()),
                    redock_cooldown_s=redock_cooldown.value_si(),
                    break_force_n=break_force.value_si(),
                )
                if declare_policy.isChecked()
                else None
            )
            updated = ConnectorTypeSpec.model_validate(
                {
                    **connector_type.model_dump(mode="python"),
                    "name": name.text() or None,
                    "active": active.isChecked(),
                    "gender": ConnectorGender(gender.currentText()),
                    "compatible_with": tuple(
                        value.strip() for value in compatible.text().split(",") if value.strip()
                    ),
                    "allowed_orientations": allowed_orientations,
                    "acceptance_region": acceptance_region,
                    "physical_connection": physical_connection,
                    "limits": connector_limits,
                    "supports_undocking": supports_undocking.isChecked(),
                    "docking_policy": docking_policy,
                    "metadata": metadata.value(),
                }
            )
            return project.update_connector_type(updated)

        apply_button.clicked.connect(
            lambda: self._apply_project_change(
                update,
                selection_after=("connector_type", type_id, ""),
            )
        )
        sections.addWidget(apply_button)
        remove_button = QPushButton("Remove connector type")
        remove_button.clicked.connect(
            lambda: self._apply_project_change(
                lambda project: project.remove_connector_type(type_id),
                selection_after=("connector_types", "", ""),
            )
        )
        sections.addWidget(remove_button)
        self._replace_properties(panel)

    def _add_connector_type_dialog(self) -> None:
        if self.project is None:
            return
        dialog = _FormDialog(self, "Add connector type", "add-connector-type", QSize(680, 620))
        layout = dialog.form
        type_id = QLineEdit()
        type_id.setPlaceholderText("for example: smores_ep")
        name = QLineEdit()
        active = QCheckBox()
        gender = _enum_combo(ConnectorGender, ConnectorGender.GENDERLESS.value)
        metadata = _MetadataEditor({})
        layout.addRow("Connector type ID", type_id)
        id_help = QLabel(
            "Lowercase YAML ID, such as ep or smores_ep. Use Name for display text such as EP."
        )
        id_help.setWordWrap(True)
        layout.addRow("", id_help)
        layout.addRow("Name", name)
        layout.addRow("Active connector", active)
        layout.addRow("Gender", gender)
        layout.addRow("Custom metadata", metadata)
        connector_type: ConnectorTypeSpec | None = None
        while dialog.exec() == int(QDialog.DialogCode.Accepted):
            try:
                connector_type_id = validate_identifier(
                    type_id.text(),
                    field_name="Connector type ID",
                )
                connector_type = ConnectorTypeSpec(
                    id=connector_type_id,
                    name=name.text().strip() or None,
                    active=active.isChecked(),
                    gender=ConnectorGender(gender.currentText()),
                    compatible_with=(connector_type_id,),
                    metadata=metadata.value(),
                )
            except Exception as error:
                self._show_error("Invalid connector type", error)
                continue
            break
        if connector_type is None:
            return
        self._apply_project_change(
            lambda project: project.add_connector_type(connector_type),
            selection_after=("connector_type", connector_type.id, ""),
        )

    def _add_connector_dialog(self, module_id: str, link_name: str) -> None:
        if self.project is None:
            return
        connector_type_ids = list(self.project.pack.hardware_catalog.connector_types)
        if not connector_type_ids:
            QMessageBox.information(
                self,
                "Create a connector type first",
                "Select Connector Types and create a reusable type before adding a connector.",
            )
            return
        dialog = _FormDialog(
            self, f"Add connector to {link_name}", "add-connector", QSize(720, 720)
        )
        layout = dialog.form
        connector_id = QLineEdit()
        connector_id.setPlaceholderText("for example: front_face")
        connector_type = QComboBox()
        connector_type.addItems(connector_type_ids)
        xyz = QuantityVectorEdit(
            (0.0, 0.0, 0.0),
            kind=QuantityKind.LENGTH,
            unit=self._units.preferences.length_unit,
            labels="XYZ",
            name="Position",
        )
        rpy = QuantityVectorEdit(
            (0.0, 0.0, 0.0),
            kind=QuantityKind.ANGLE,
            unit=self._units.preferences.angle_unit,
            labels="RPY",
            name="Roll / Pitch / Yaw",
        )
        docking = QLineEdit("1, 0, 0")
        approach = QLineEdit("1, 0, 0")
        metadata = _MetadataEditor({})
        layout.addRow("Connector ID", connector_id)
        id_help = QLabel("Lowercase YAML ID, such as front_face or dock_1.")
        id_help.setWordWrap(True)
        layout.addRow("", id_help)
        layout.addRow("Connector type", connector_type)
        layout.addRow("Position XYZ", xyz)
        layout.addRow("Rotation RPY", rpy)
        layout.addRow("Docking axis", docking)
        layout.addRow("Approach axis", approach)
        layout.addRow("Custom fields", metadata)
        connector: ConnectorSpec | None = None
        while dialog.exec() == int(QDialog.DialogCode.Accepted):
            try:
                validated_connector_id = validate_identifier(
                    connector_id.text(),
                    field_name="Connector ID",
                )
                connector = ConnectorSpec(
                    id=validated_connector_id,
                    connector_type=connector_type.currentText(),
                    parent_link=link_name,
                    local_pose=PoseSpec(
                        xyz_m=xyz.value_si(),
                        rpy_rad=rpy.value_si(),
                    ),
                    docking_axis=_parse_vector(docking.text()),
                    approach_axis=_parse_vector(approach.text()),
                    metadata=metadata.value(),
                )
            except Exception as error:
                self._show_error("Invalid connector", error)
                continue
            break
        if connector is None:
            return
        self._apply_project_change(
            lambda project: project.add_connector(module_id, connector),
            selection_after=("connector", connector.id, module_id),
        )

    def _apply_project_change(
        self,
        operation: Callable[[StudioProject], StudioProject],
        *,
        selection_after: tuple[str, str, str] | None = None,
    ) -> None:
        if self.project is None:
            return
        try:
            self.project = operation(self.project)
        except Exception as error:
            self._show_error("Could not apply edit", error)
            return
        selection = selection_after or self._current_selection
        self._rebuild_tree()
        self._refresh_document_panels()
        if (
            selection is not None
            and selection[2] in self.project.pack.hardware_catalog.module_types
        ):
            self._render_module(selection[2])
        else:
            self._render_first_module()
        self._update_title()
        self._logger.info(
            "Applied Robot Pack edit; selection=%s; document_dirty=%s",
            selection or "none",
            self.project.dirty,
        )
        if selection is None or not self._select_tree_entity(selection):
            self._current_selection = None
            self._replace_properties(QLabel("Select a Robot Pack entity to edit."))

    def _select_tree_entity(self, selection: tuple[str, str, str]) -> bool:
        kind, entity_id, module_id = selection
        pending: list[QTreeWidgetItem] = []
        for index in range(self.project_tree.topLevelItemCount()):
            item = self.project_tree.topLevelItem(index)
            if item is not None:
                pending.append(item)
        while pending:
            item = pending.pop()
            if (
                item.data(0, _KIND_ROLE) == kind
                and item.data(0, _ID_ROLE) == entity_id
                and item.data(0, _MODULE_ROLE) == module_id
            ):
                self.project_tree.setCurrentItem(item)
                return True
            for index in range(item.childCount()):
                pending.append(item.child(index))
        return False

    def _replace_properties(self, widget: QWidget) -> None:
        while self.properties_layout.count():
            item = self.properties_layout.takeAt(0)
            if item is None:
                continue
            old_widget = item.widget()
            if old_widget is not None:
                old_widget.hide()
                old_widget.deleteLater()
        if self._current_selection is not None:
            kind, entity_id, _module_id = self._current_selection
            heading = QLabel(kind.replace("_", " ").capitalize())
            heading.setObjectName("SectionTitle")
            self.properties_layout.addWidget(heading)
            identity = QLabel(entity_id)
            identity.setObjectName("Muted")
            identity.setTextFormat(Qt.TextFormat.PlainText)
            identity.setWordWrap(True)
            self.properties_layout.addWidget(identity)
        for form in widget.findChildren(QFormLayout):
            form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
            form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
            form.setVerticalSpacing(9)
        for label in widget.findChildren(QLabel):
            label.setWordWrap(True)
        for combo in widget.findChildren(QComboBox):
            combo.setSizeAdjustPolicy(
                QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
            )
            combo.setMinimumContentsLength(10)
        for button in widget.findChildren(QPushButton):
            if button.text().startswith("Apply"):
                button.setProperty("primary", True)
            elif button.text().startswith("Remove"):
                button.setProperty("danger", True)
        self.properties_layout.addWidget(widget)
        self.properties_layout.addStretch(1)
        self.properties_scroll.verticalScrollBar().setValue(0)

    def _add_dock(
        self,
        title: str,
        widget: QWidget,
        area: Qt.DockWidgetArea,
    ) -> QDockWidget:
        dock = QDockWidget(title, self)
        dock.setObjectName(title.replace(" ", "_"))
        dock.setWidget(widget)
        self.addDockWidget(area, dock)
        return dock

    def _attach_session_log(self) -> None:
        """Backfill and then mirror the process logger into the GUI."""
        if self._session_log_path is not None:
            self.log.setToolTip(str(self._session_log_path))
        if self._session_log_path is not None and self._session_log_path.is_file():
            try:
                self.log.setPlainText(self._session_log_path.read_text(encoding="utf-8"))
            except OSError as error:
                self.log.setPlainText(f"Could not read session log: {error}")
        handler = _QtLogHandler(self.log)
        handler.setFormatter(
            logging.Formatter(
                "%(asctime)s %(levelname)s %(name)s %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            )
        )
        self._logger.addHandler(handler)
        self._qt_log_handler = handler

    @staticmethod
    def _set_item_data(
        item: QTreeWidgetItem,
        kind: str,
        entity_id: str,
        module_id: str,
    ) -> None:
        item.setData(0, _KIND_ROLE, kind)
        item.setData(0, _ID_ROLE, entity_id)
        item.setData(0, _MODULE_ROLE, module_id)

    def _update_title(self) -> None:
        if self.project is None:
            self.setWindowTitle("ModSim Studio")
            return
        marker = " *" if self.project.dirty else ""
        self.setWindowTitle(f"{self.project.pack.manifest.name}{marker} — ModSim Studio")
        self.save_state.setText("●  Unsaved changes" if self.project.dirty else "✓  Saved")
        self.save_state.setToolTip(str(self.project.loaded.root))

    def _show_error(self, title: str, error: Exception) -> None:
        message = user_error_message(error)
        self._logger.error(
            "%s\n%s",
            title,
            error_log_details(error),
            exc_info=(type(error), error, error.__traceback__),
        )
        QMessageBox.critical(self, title, message)
        status_message = " ".join(message.splitlines())
        self.statusBar().showMessage(f"{title}: {status_message}", 10000)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.project is not None and self.project.dirty:
            answer = QMessageBox.question(
                self,
                "Unsaved Robot Pack edits",
                "Save changes before closing?",
                QMessageBox.StandardButton.Save
                | QMessageBox.StandardButton.Discard
                | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Save,
            )
            if answer == QMessageBox.StandardButton.Save:
                if not self.runtime_workspace.stop_and_wait():
                    event.ignore()
                    self.statusBar().showMessage(
                        "The runtime is still stopping. Try closing again."
                    )
                    return
                self._save()
                if self.project.dirty:
                    event.ignore()
                    return
            elif answer != QMessageBox.StandardButton.Discard:
                event.ignore()
                return
        if not self.runtime_workspace.stop_and_wait():
            event.ignore()
            self.statusBar().showMessage("The runtime is still stopping. Try closing again.")
            return
        # Persist the authoring dock layout, rather than the temporarily hidden
        # docks used while the Runtime page is visible.
        self.workspaces.setCurrentIndex(0)
        self._logger.info("Studio window closing")
        try:
            # Release the viewport's OpenGL/shadow-map resources while the GL
            # context is still current; otherwise VTK's shadow render passes are
            # destroyed without ReleaseGraphicsResources() and log errors.
            self.viewport.close()
        except Exception:  # never block window close on viewport teardown
            self._logger.exception("Viewport teardown failed")
        if self._qt_log_handler is not None:
            self._logger.removeHandler(self._qt_log_handler)
            self._qt_log_handler.close()
            self._qt_log_handler = None
        super().closeEvent(event)


def _parse_vector(value: str) -> tuple[float, float, float]:
    parts = value.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("expected exactly three numeric components")
    parsed = tuple(float(part) for part in parts)
    return parsed[0], parsed[1], parsed[2]


def _optional_vector(value: str) -> tuple[float, float, float] | None:
    return _parse_vector(value) if value.strip() else None


def _vector_text(value: tuple[float, float, float]) -> str:
    return ", ".join(f"{component:g}" for component in value)


def _enum_combo(enum_type: type[StrEnum], current: str) -> QComboBox:
    combo = QComboBox()
    combo.addItems([item.value for item in enum_type])
    combo.setCurrentText(current)
    return combo


def _model_view_modes(
    authoring: QCheckBox,
    runtime: QCheckBox,
) -> tuple[ModelViewMode, ...]:
    """Return the checked model-view modes for strict schema validation."""
    return tuple(
        mode
        for mode, checkbox in (
            (ModelViewMode.AUTHORING, authoring),
            (ModelViewMode.RUNTIME, runtime),
        )
        if checkbox.isChecked()
    )


def _joint_limit_fields(
    joint_type: JointType,
) -> tuple[tuple[str, str, QuantityKind], ...]:
    if joint_type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
        bounds = (
            ()
            if joint_type is JointType.CONTINUOUS
            else (
                ("Lower position", "lower_position_rad", QuantityKind.ANGLE),
                ("Upper position", "upper_position_rad", QuantityKind.ANGLE),
            )
        )
        return (
            *bounds,
            ("Max velocity", "max_velocity_rad_per_s", QuantityKind.ANGULAR_VELOCITY),
            ("Max effort", "max_effort_nm", QuantityKind.TORQUE),
        )
    if joint_type is JointType.PRISMATIC:
        return (
            ("Lower position", "lower_position_m", QuantityKind.LENGTH),
            ("Upper position", "upper_position_m", QuantityKind.LENGTH),
            ("Max velocity", "max_velocity_m_per_s", QuantityKind.LINEAR_VELOCITY),
            ("Max effort", "max_effort_n", QuantityKind.FORCE),
        )
    return ()
