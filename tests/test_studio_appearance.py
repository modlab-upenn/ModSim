"""Appearance preferences and Builder interactions without a GPU context."""

from __future__ import annotations

import os
from collections.abc import Iterator
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyvistaqt")

from PySide6.QtCore import QCoreApplication, QEvent, QSettings, Qt, QTimer, Signal
from PySide6.QtWidgets import QApplication, QFileDialog, QPushButton, QWidget

from modsim_studio import main_window
from modsim_studio.appearance import DEFAULT_THEME, THEMES, ThemeManager, theme_manager
from modsim_studio.chrome import ThemePicker, VectorEdit


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.fixture
def appearance(
    application: QApplication, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> Iterator[ThemeManager]:
    manager = theme_manager()
    original = manager.theme.id
    monkeypatch.setattr(
        manager, "_settings", QSettings(str(tmp_path / "theme.ini"), QSettings.Format.IniFormat)
    )
    yield manager
    manager.set_theme(original, persist=False)
    manager._apply()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)


def test_theme_pickers_sync_and_preference_survives_new_settings_instance(
    application: QApplication, appearance: ThemeManager, tmp_path: Path
) -> None:
    first = ThemePicker(appearance)
    second = ThemePicker(appearance)
    try:
        for identifier in THEMES:
            first.setCurrentIndex(first.findData(identifier))
            assert appearance.theme.id == identifier
            assert second.currentData() == identifier
        settings = QSettings(str(tmp_path / "theme.ini"), QSettings.Format.IniFormat)
        restored = ThemeManager(application, settings)
        assert restored.theme.id == appearance.theme.id
        restored.deleteLater()
    finally:
        first.deleteLater()
        second.deleteLater()


def test_unknown_saved_theme_falls_back_without_rewriting_settings(
    application: QApplication, appearance: ThemeManager, tmp_path: Path
) -> None:
    settings = QSettings(str(tmp_path / "old-theme.ini"), QSettings.Format.IniFormat)
    settings.setValue("appearance/theme", "removed-theme")
    restored = ThemeManager(application, settings)
    assert restored.theme.id == DEFAULT_THEME
    assert settings.value("appearance/theme") == "removed-theme"
    restored.deleteLater()


class _ViewportStub(QWidget):
    """Keep these tests about UI state; real VTK is exercised by the launch smoke test."""

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


@pytest.fixture
def builder(
    application: QApplication, appearance: ThemeManager, monkeypatch: pytest.MonkeyPatch
) -> Iterator[main_window.MainWindow]:
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)
    window = main_window.MainWindow(initial_path="examples/robot_packs/generic_cube")
    window.show()
    application.processEvents()
    yield window
    window.project = None  # Edits deliberately remain in memory; no save dialog in teardown.
    window.close()
    window.deleteLater()


def test_builder_theme_switch_keeps_pending_edits_selection_and_document(
    application: QApplication, appearance: ThemeManager, builder: main_window.MainWindow
) -> None:
    project = builder.project
    assert project is not None
    module = next(iter(project.pack.hardware_catalog.module_types.values()))
    connector = module.connectors[0]
    selection = ("connector", connector.id, module.id)
    assert builder._select_tree_entity(selection)
    position = builder.properties.findChildren(VectorEdit)[0]
    position.fields[0].setText("0.037")
    rendered = builder.viewport.render_count
    for identifier in THEMES:
        builder.header.theme_picker.setCurrentIndex(
            builder.header.theme_picker.findData(identifier)
        )
        application.processEvents()
        assert builder.project is project
        assert builder._current_selection == selection
        assert position.fields[0].text() == "0.037"
        assert builder.viewport.render_count == rendered
    apply_button = next(
        button
        for button in builder.properties.findChildren(QPushButton)
        if button.text() == "Apply connector" and button.isVisible()
    )
    apply_button.click()
    edited = builder.project
    assert edited is not None and edited.dirty
    updated = next(
        item
        for item in edited.pack.hardware_catalog.module_types[module.id].connectors
        if item.id == connector.id
    )
    assert updated.local_pose is not None
    assert updated.local_pose.xyz_m[0] == pytest.approx(0.037)


def test_project_search_keeps_matching_ancestors_and_clears_without_changing_document(
    builder: main_window.MainWindow,
) -> None:
    project = builder.project
    assert project is not None
    module = next(iter(project.pack.hardware_catalog.module_types.values()))
    connector = module.connectors[0]
    assert builder._select_tree_entity(("connector", connector.id, module.id))
    item = builder.project_tree.currentItem()
    assert item is not None
    builder.project_search.setText(connector.id.upper())
    ancestor = item
    while ancestor is not None:
        assert not ancestor.isHidden()
        ancestor = ancestor.parent()
    builder.project_search.setText("no entity has this name")
    root = builder.project_tree.topLevelItem(0)
    assert root is not None and root.isHidden()
    builder.project_search.clear()
    assert not root.isHidden() and not item.isHidden()
    assert builder.project is project and not project.dirty


@pytest.mark.parametrize("accept", [False, True])
def test_open_pack_uses_qt_picker_and_handles_accept_or_cancel(
    builder: main_window.MainWindow, accept: bool
) -> None:
    # Without the application flag, Qt can construct a GTK helper (and abort
    # while loading its icons) before the per-dialog option takes effect.
    assert QApplication.testAttribute(Qt.ApplicationAttribute.AA_DontUseNativeDialogs)
    original = builder.project
    target = Path("examples/robot_packs/mblocks_3d").resolve()
    observed: list[tuple[bool, QFileDialog.FileMode]] = []

    def finish_dialog() -> None:
        dialog = QApplication.activeModalWidget()
        if isinstance(dialog, QFileDialog):
            observed.append(
                (dialog.testOption(QFileDialog.Option.DontUseNativeDialog), dialog.fileMode())
            )
            if accept:
                dialog.setDirectory(str(target))
                dialog.accept()
            else:
                dialog.reject()

    timer = QTimer(builder)
    timer.setInterval(50)
    timer.timeout.connect(finish_dialog)
    timer.start()
    try:
        builder._open_dialog()
    finally:
        timer.stop()
        timer.deleteLater()
    assert observed == [(True, QFileDialog.FileMode.Directory)]
    if accept:
        assert builder.project is not None
        assert builder.project.pack.id == "mblocks_3d"
        assert builder.project.loaded.root == target
    else:
        assert builder.project is original


@pytest.mark.parametrize("action", ["_create_from_urdf_dialog", "_export_dialog"])
def test_import_and_export_use_qt_picker_and_cancel_without_document_changes(
    builder: main_window.MainWindow, action: str
) -> None:
    original = builder.project
    observed: list[bool] = []

    def cancel_dialog() -> None:
        dialog = QApplication.activeModalWidget()
        if isinstance(dialog, QFileDialog):
            observed.append(dialog.testOption(QFileDialog.Option.DontUseNativeDialog))
            dialog.reject()

    timer = QTimer(builder)
    timer.setInterval(50)
    timer.timeout.connect(cancel_dialog)
    timer.start()
    try:
        getattr(builder, action)()
    finally:
        timer.stop()
        timer.deleteLater()
    assert observed == [True]
    assert builder.project is original
