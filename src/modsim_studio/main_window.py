# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
"""Main ModSim Studio Robot Pack Builder window."""

from __future__ import annotations

import json
import logging
from collections.abc import Callable, Mapping
from enum import StrEnum
from pathlib import Path
from typing import cast

from pydantic import JsonValue
from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QAction, QCloseEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDockWidget,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from modsim.importers import URDFImporter
from modsim.robot_packs import (
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
from modsim_studio.project import StudioProject
from modsim_studio.user_errors import (
    error_log_details,
    user_error_message,
    validate_identifier,
)
from modsim_studio.viewport import RobotViewport

_KIND_ROLE = int(Qt.ItemDataRole.UserRole)
_ID_ROLE = _KIND_ROLE + 1
_MODULE_ROLE = _KIND_ROLE + 2


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


class MainWindow(QMainWindow):
    """Dock-based Robot Pack authoring application."""

    def __init__(
        self,
        *,
        initial_path: str | Path | None = None,
        session_logger: logging.Logger | None = None,
        session_log_path: str | Path | None = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("ModSim Studio")
        self.resize(1500, 920)
        self.project: StudioProject | None = None
        self._current_selection: tuple[str, str, str] | None = None
        self._logger = session_logger or logging.getLogger("modsim")
        self._session_log_path = (
            Path(session_log_path).resolve() if session_log_path is not None else None
        )
        self._qt_log_handler: _QtLogHandler | None = None

        self.viewport = RobotViewport(self)
        self.setCentralWidget(self.viewport)
        self.viewport.entity_selected.connect(self._select_from_viewport)

        self.project_tree = QTreeWidget()
        self.project_tree.setHeaderLabels(["Robot Pack", "Type"])
        self.project_tree.currentItemChanged.connect(self._tree_selection_changed)
        self._add_dock("Project / URDF", self.project_tree, Qt.DockWidgetArea.LeftDockWidgetArea)

        self.properties = QWidget()
        self.properties_layout = QVBoxLayout(self.properties)
        self.properties_layout.addWidget(QLabel("Open or create a Robot Pack."))
        self.properties_layout.addStretch(1)
        self._add_dock(
            "Properties",
            self.properties,
            Qt.DockWidgetArea.RightDockWidgetArea,
        )

        self.validation_table = QTableWidget(0, 4)
        self.validation_table.setHorizontalHeaderLabels(["Severity", "Code", "Location", "Message"])
        self.validation_table.horizontalHeader().setStretchLastSection(True)
        self.yaml_preview = QPlainTextEdit()
        self.yaml_preview.setReadOnly(True)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Studio operations and exception traces appear here.")
        bottom_tabs = QTabWidget()
        bottom_tabs.addTab(self.validation_table, "Validation")
        bottom_tabs.addTab(self.yaml_preview, "YAML Preview")
        bottom_tabs.addTab(self.log, "Session Log")
        bottom_dock = self._add_dock(
            "Document",
            bottom_tabs,
            Qt.DockWidgetArea.BottomDockWidgetArea,
        )
        bottom_dock.setMinimumHeight(220)
        self._attach_session_log()

        self._build_actions()
        self.statusBar().showMessage("Ready")
        self._logger.info("Studio window initialized")
        if initial_path is not None:
            self.open_project(initial_path)

    def _build_actions(self) -> None:
        file_menu = self.menuBar().addMenu("&File")
        create_action = QAction("Create from &URDF…", self)
        create_action.setShortcut("Ctrl+N")
        create_action.triggered.connect(self._create_from_urdf_dialog)
        file_menu.addAction(create_action)

        open_action = QAction("&Open Robot Pack…", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self._open_dialog)
        file_menu.addAction(open_action)

        self.save_action = QAction("&Save", self)
        self.save_action.setShortcut("Ctrl+S")
        self.save_action.setEnabled(False)
        self.save_action.triggered.connect(self._save)
        file_menu.addAction(self.save_action)

        self.export_action = QAction("&Export As…", self)
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
        simulation_action = QAction("&Simulation Readiness", self)
        simulation_action.setShortcut("F7")
        simulation_action.triggered.connect(lambda: self._validate(ValidationProfile.SIMULATION))
        validate_menu.addAction(simulation_action)

        view_toolbar = self.addToolBar("Viewport layers")
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
            view_toolbar.addAction(action)

    def open_project(self, path: str | Path) -> None:
        """Open a Robot Pack and populate all editor panels."""
        self._logger.info("Opening Robot Pack: %s", Path(path))
        try:
            project = StudioProject.open(path)
        except Exception as error:
            self._show_error("Could not open Robot Pack", error)
            return
        self._set_project(project)
        self._logger.info(
            "Opened Robot Pack '%s' from %s",
            project.pack.id,
            project.loaded.root,
        )

    def _set_project(self, project: StudioProject) -> None:
        self.project = project
        self._current_selection = None
        self.save_action.setEnabled(True)
        self.export_action.setEnabled(True)
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

    def _render_first_module(self) -> None:
        if self.project is None:
            return
        for module in self.project.pack.hardware_catalog.module_types.values():
            asset = self.project.imported_assets.get(module.asset_ref)
            if asset is not None:
                self.viewport.render_module(asset, module)
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
        urdf_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select a URDF",
            "",
            "URDF files (*.urdf *.xml);;All files (*)",
        )
        if not urdf_path:
            return
        asset_roots: tuple[str | Path, ...] = ()
        try:
            preview = URDFImporter().load(urdf_path)
        except Exception as error:
            self._show_error("Could not inspect URDF", error)
            return
        self._logger.info("Inspected URDF for import: %s", urdf_path)
        if any("mesh could not be resolved" in warning for warning in preview.warnings):
            asset_root = QFileDialog.getExistingDirectory(
                self,
                "Select the package or mesh asset root",
                str(Path(urdf_path).parent),
            )
            if not asset_root:
                return
            asset_roots = (asset_root,)
        destination, _ = QFileDialog.getSaveFileName(
            self,
            "Choose a new Robot Pack folder",
            str(Path(urdf_path).with_suffix("")),
            "Robot Pack folder (*)",
        )
        if not destination:
            return
        try:
            project = StudioProject.create_from_urdf(
                urdf_path,
                destination,
                asset_roots=asset_roots,
            )
        except Exception as error:
            self._show_error("Could not create Robot Pack", error)
            return
        self._set_project(project)
        warnings = next(iter(project.imported_assets.values())).warnings
        self._logger.info("Created draft Robot Pack: %s", project.loaded.root)
        if warnings:
            for warning in warnings:
                self._logger.warning("URDF import: %s", warning)
        else:
            self._logger.info("URDF import completed without warnings")

    def _open_dialog(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Open Robot Pack")
        if path:
            self.open_project(path)

    def _export_dialog(self) -> None:
        if self.project is None:
            return
        destination, _ = QFileDialog.getSaveFileName(
            self,
            "Export to a new Robot Pack folder",
            str(self.project.loaded.root.with_name(self.project.loaded.root.name + "_edited")),
            "Robot Pack folder (*)",
        )
        if not destination:
            return
        try:
            project = self.project.export(destination)
        except Exception as error:
            self._show_error("Could not export Robot Pack", error)
            return
        self._set_project(StudioProject.open(project.loaded.root))
        self._logger.info("Exported Robot Pack to %s", destination)
        self.statusBar().showMessage(f"Exported {destination}", 8000)

    def _save(self) -> None:
        if self.project is None:
            return
        try:
            self.project = self.project.save()
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
        mass = QLineEdit(_optional_number(module.mass_kg))
        form.addRow("Root link", root_link)
        form.addRow("Mass (kg)", mass)
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
                    _optional_float(mass.text()),
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
        add_button = QPushButton("Add connector to this link…")
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
        fields: dict[str, QLineEdit] = {}
        for label, field_name in _joint_limit_fields(joint.type):
            field = QLineEdit(_optional_number(getattr(limits, field_name)))
            fields[field_name] = field
            form.addRow(label, field)
        apply_button = QPushButton("Apply joint metadata")

        def update(project: StudioProject) -> StudioProject:
            parsed_modes = tuple(
                ControlMode(value.strip()) for value in modes.text().split(",") if value.strip()
            )
            limit_data = {
                field_name: _optional_float(field.text()) for field_name, field in fields.items()
            }
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
        form = QFormLayout(panel)
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
        form.addRow("URDF body / link", parent_link)
        frame = QLineEdit(connector.frame or "")
        use_local_pose = QCheckBox()
        use_local_pose.setChecked(connector.local_pose is not None)
        form.addRow("Named frame (optional)", frame)
        form.addRow("Use numeric local pose", use_local_pose)
        pose = connector.local_pose or PoseSpec()
        xyz = QLineEdit(_vector_text(pose.xyz_m))
        rpy = QLineEdit(_vector_text(pose.rpy_rad))
        docking = QLineEdit(
            _vector_text(connector.docking_axis) if connector.docking_axis is not None else ""
        )
        approach = QLineEdit(
            _vector_text(connector.approach_axis) if connector.approach_axis is not None else ""
        )
        form.addRow("Position xyz (m)", xyz)
        form.addRow("Rotation rpy (rad)", rpy)
        form.addRow("Docking axis", docking)
        form.addRow("Approach axis", approach)
        metadata = _MetadataEditor(connector.metadata)
        form.addRow("Custom fields", metadata)

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
                            xyz_m=_parse_vector(xyz.text()),
                            rpy_rad=_parse_vector(rpy.text()),
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
        form.addRow(apply_button)
        remove_button = QPushButton("Remove connector")
        remove_button.clicked.connect(
            lambda: self._apply_project_change(
                lambda project: project.remove_connector(module_id, connector_id),
                selection_after=("link", connector.parent_link, module_id),
            )
        )
        form.addRow(remove_button)
        self._replace_properties(panel)

    def _show_connector_types_properties(self) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(QLabel("Create reusable connector types before assigning connectors."))
        add_button = QPushButton("Add connector type…")
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
        add_button = QPushButton("Add model view…")
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
        configuration = _MetadataEditor(model_view.configuration)
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
        dialog = QDialog(self)
        dialog.setWindowTitle("Add model view")
        layout = QFormLayout(dialog)
        model_view_id = QLineEdit()
        model_view_id.setPlaceholderText("for example: smores_topology")
        name = QLineEdit()
        builder = QLineEdit("module_topology_graph")
        authoring_mode = QCheckBox()
        runtime_mode = QCheckBox()
        runtime_mode.setChecked(True)
        default_view = QCheckBox()
        configuration = _MetadataEditor({})
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
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
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
        form = QFormLayout(panel)
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

        orientation_mode = QComboBox()
        orientation_mode.addItems(["", *[item.value for item in OrientationMode]])
        if connector_type.allowed_orientations is not None:
            orientation_mode.setCurrentText(connector_type.allowed_orientations.mode.value)
        orientation_values = QLineEdit(
            ", ".join(
                f"{value:g}"
                for value in (
                    connector_type.allowed_orientations.values_rad
                    if connector_type.allowed_orientations is not None
                    else ()
                )
            )
        )
        form.addRow("Orientation mode", orientation_mode)
        form.addRow("Allowed values (rad)", orientation_values)

        acceptance = connector_type.acceptance_region
        acceptance_shape = _enum_combo(
            AcceptanceShape,
            acceptance.shape.value if acceptance is not None else AcceptanceShape.BOX.value,
        )
        position_tolerance = QLineEdit(
            _optional_number(acceptance.position_tolerance_m if acceptance is not None else None)
        )
        orientation_tolerance = QLineEdit(
            _optional_number(
                acceptance.orientation_tolerance_rad if acceptance is not None else None
            )
        )
        relative_velocity = QLineEdit(
            _optional_number(
                acceptance.max_relative_velocity_m_s if acceptance is not None else None
            )
        )
        form.addRow("Acceptance shape", acceptance_shape)
        form.addRow("Position tolerance (m)", position_tolerance)
        form.addRow("Orientation tolerance (rad)", orientation_tolerance)
        form.addRow("Max relative velocity (m/s)", relative_velocity)

        physical = connector_type.physical_connection
        constraint = QComboBox()
        constraint.addItems(["", *[item.value for item in PhysicalConstraintType]])
        if physical is not None:
            constraint.setCurrentText(physical.constraint.value)
        translational_stiffness = QLineEdit(
            _optional_number(
                physical.compliance.translational_stiffness_n_per_m
                if physical is not None and physical.compliance is not None
                else None
            )
        )
        rotational_stiffness = QLineEdit(
            _optional_number(
                physical.compliance.rotational_stiffness_nm_per_rad
                if physical is not None and physical.compliance is not None
                else None
            )
        )
        hinge_axis = QLineEdit(
            _vector_text(physical.hinge.axis)
            if physical is not None and physical.hinge is not None
            else ""
        )
        hinge_anchor_separation = QLineEdit(
            _optional_number(
                physical.hinge.anchor_separation_m
                if physical is not None and physical.hinge is not None
                else None
            )
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
        form.addRow("Translation stiffness (N/m)", translational_stiffness)
        form.addRow("Rotation stiffness (Nm/rad)", rotational_stiffness)
        form.addRow("Hinge axis (x, y, z)", hinge_axis)
        form.addRow("Hinge anchor separation (m)", hinge_anchor_separation)

        limits = connector_type.limits
        max_normal = QLineEdit(
            _optional_number(limits.max_normal_force_n if limits is not None else None)
        )
        max_shear = QLineEdit(
            _optional_number(limits.max_shear_force_n if limits is not None else None)
        )
        max_bending = QLineEdit(
            _optional_number(limits.max_bending_moment_nm if limits is not None else None)
        )
        supports_undocking = QCheckBox()
        supports_undocking.setChecked(connector_type.supports_undocking)
        form.addRow("Max normal force (N)", max_normal)
        form.addRow("Max shear force (N)", max_shear)
        form.addRow("Max bending moment (Nm)", max_bending)
        form.addRow("Supports undocking", supports_undocking)

        policy = connector_type.docking_policy
        declare_policy = QCheckBox()
        declare_policy.setChecked(policy is not None)
        auto_latch = QCheckBox()
        auto_latch.setChecked(policy.auto_latch if policy is not None else False)
        alignment = _enum_combo(
            AlignmentMode,
            policy.alignment.value if policy is not None else AlignmentMode.MEASURED.value,
        )
        redock_cooldown = QLineEdit(
            _optional_number(policy.redock_cooldown_s if policy is not None else None)
        )
        break_force = QLineEdit(
            _optional_number(policy.break_force_n if policy is not None else None)
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
        form.addRow("Redock cooldown (s)", redock_cooldown)
        form.addRow("Break force (N)", break_force)
        metadata = _MetadataEditor(connector_type.metadata)
        form.addRow("Custom metadata", metadata)

        apply_button = QPushButton("Apply connector type")

        def update(project: StudioProject) -> StudioProject:
            mode_value = orientation_mode.currentText()
            allowed_orientations = (
                AllowedOrientations(
                    mode=OrientationMode(mode_value),
                    values_rad=(
                        _parse_float_list(orientation_values.text())
                        if mode_value == OrientationMode.DISCRETE.value
                        else ()
                    ),
                )
                if mode_value
                else None
            )
            acceptance_values = (
                _optional_float(position_tolerance.text()),
                _optional_float(orientation_tolerance.text()),
                _optional_float(relative_velocity.text()),
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
                _optional_float(translational_stiffness.text()),
                _optional_float(rotational_stiffness.text()),
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
                anchor_separation_m = _optional_float(hinge_anchor_separation.text())
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
                _optional_float(max_normal.text()),
                _optional_float(max_shear.text()),
                _optional_float(max_bending.text()),
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
                    redock_cooldown_s=_optional_float(redock_cooldown.text()),
                    break_force_n=_optional_float(break_force.text()),
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
        form.addRow(apply_button)
        remove_button = QPushButton("Remove connector type")
        remove_button.clicked.connect(
            lambda: self._apply_project_change(
                lambda project: project.remove_connector_type(type_id),
                selection_after=("connector_types", "", ""),
            )
        )
        form.addRow(remove_button)
        self._replace_properties(panel)

    def _add_connector_type_dialog(self) -> None:
        if self.project is None:
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("Add connector type")
        layout = QFormLayout(dialog)
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
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
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
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Add connector to {link_name}")
        layout = QFormLayout(dialog)
        connector_id = QLineEdit()
        connector_id.setPlaceholderText("for example: front_face")
        connector_type = QComboBox()
        connector_type.addItems(connector_type_ids)
        xyz = QLineEdit("0, 0, 0")
        rpy = QLineEdit("0, 0, 0")
        docking = QLineEdit("1, 0, 0")
        approach = QLineEdit("1, 0, 0")
        metadata = _MetadataEditor({})
        layout.addRow("Connector ID", connector_id)
        id_help = QLabel("Lowercase YAML ID, such as front_face or dock_1.")
        id_help.setWordWrap(True)
        layout.addRow("", id_help)
        layout.addRow("Connector type", connector_type)
        layout.addRow("Position xyz (m)", xyz)
        layout.addRow("Rotation rpy (rad)", rpy)
        layout.addRow("Docking axis", docking)
        layout.addRow("Approach axis", approach)
        layout.addRow("Custom fields", metadata)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
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
                        xyz_m=_parse_vector(xyz.text()),
                        rpy_rad=_parse_vector(rpy.text()),
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
                old_widget.deleteLater()
        self.properties_layout.addWidget(widget)
        self.properties_layout.addStretch(1)

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
                self._save()
                if self.project.dirty:
                    event.ignore()
                    return
            elif answer != QMessageBox.StandardButton.Discard:
                event.ignore()
                return
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


def _optional_float(value: str) -> float | None:
    stripped = value.strip()
    return float(stripped) if stripped else None


def _optional_number(value: float | None) -> str:
    return "" if value is None else f"{value:g}"


def _parse_float_list(value: str) -> tuple[float, ...]:
    return tuple(float(part) for part in value.replace(",", " ").split())


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


def _joint_limit_fields(joint_type: JointType) -> tuple[tuple[str, str], ...]:
    if joint_type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
        bounds = (
            ()
            if joint_type is JointType.CONTINUOUS
            else (
                ("Lower position (rad)", "lower_position_rad"),
                ("Upper position (rad)", "upper_position_rad"),
            )
        )
        return (
            *bounds,
            ("Max velocity (rad/s)", "max_velocity_rad_per_s"),
            ("Max effort (Nm)", "max_effort_nm"),
        )
    if joint_type is JointType.PRISMATIC:
        return (
            ("Lower position (m)", "lower_position_m"),
            ("Upper position (m)", "upper_position_m"),
            ("Max velocity (m/s)", "max_velocity_m_per_s"),
            ("Max effort (N)", "max_effort_n"),
        )
    return ()
