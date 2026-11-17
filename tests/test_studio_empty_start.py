"""The blank Builder can cancel or import a first document without a GPU."""

from __future__ import annotations

import os
from collections.abc import Callable, Iterator
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyvistaqt")

from PySide6.QtCore import Signal
from PySide6.QtGui import QAction, QFont
from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QLineEdit,
    QPushButton,
    QStyle,
    QStyleOptionFrame,
    QWidget,
)

from modsim_robotics.importers import ImportedRobotAsset
from modsim_robotics.robot_packs import ModuleType, RobotPackLoader, ValidationProfile
from modsim_robotics_studio import main_window
from modsim_robotics_studio.appearance import theme_manager


class _ViewportStub(QWidget):
    entity_selected = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.rendered: tuple[ImportedRobotAsset, ModuleType] | None = None

    def render_module(self, asset: ImportedRobotAsset, module: ModuleType) -> None:
        self.rendered = asset, module

    def set_layer_visibility(self, **_kwargs: bool) -> None:
        pass

    def select_entity(self, *_args: str) -> None:
        pass

    def fit_module(self) -> None:
        pass


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.fixture
def builder(
    application: QApplication, monkeypatch: pytest.MonkeyPatch
) -> Iterator[main_window.MainWindow]:
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)
    window = main_window.MainWindow(initial_path=None)
    window.show()
    application.processEvents()
    try:
        yield window
    finally:
        window.project = None
        window.close()
        window.deleteLater()
        application.processEvents()


@pytest.fixture
def urdf(tmp_path: Path) -> Path:
    path = tmp_path / "practice_cube.urdf"
    path.write_text(
        '<robot name="practice_cube"><link name="base_link">'
        '<inertial><mass value="1"/></inertial>'
        '<visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>'
        "</link></robot>",
        encoding="utf-8",
    )
    return path


def _button(parent: QWidget, text: str) -> QPushButton:
    return next(button for button in parent.findChildren(QPushButton) if button.text() == text)


def _file_selection(*paths: str) -> Callable[..., str]:
    choices = iter(paths)

    def select(*_args: object, **_kwargs: object) -> str:
        return next(choices)

    return select


@pytest.mark.parametrize("point_size", [9, 14, 18])
def test_builder_fits_a_small_screen_without_clipping_action_labels(
    application: QApplication, monkeypatch: pytest.MonkeyPatch, point_size: int
) -> None:
    original = QFont(application.font())
    manager = theme_manager()
    enlarged = QFont(original)
    enlarged.setPointSize(point_size)
    application.setFont(enlarged)
    manager._apply()
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)
    window = main_window.MainWindow()
    try:
        window.resize(800, 600)
        window.show()
        application.processEvents()
        assert window.width() <= 800 and window.height() <= 600
        welcome = window.preview_stack.currentWidget()
        assert welcome is not None
        for title in ("Open Robot Pack", "Create from URDF"):
            button = _button(welcome, title)
            assert button.width() >= button.minimumSizeHint().width()
            assert button.height() >= button.minimumSizeHint().height()
        for action in window.findChildren(QAction):
            assert not action.text().endswith(("…", "...")), action.text()
    finally:
        window.close()
        window.deleteLater()
        application.setFont(original)
        manager._apply()
        application.processEvents()


def test_metadata_rows_and_editors_fit_an_enlarged_font(application: QApplication) -> None:
    editor = main_window._MetadataEditor({"label": "front"})
    font = QFont(editor.font())
    font.setPointSize(18)
    editor.setFont(font)
    editor.resize(600, 300)
    try:
        editor.show()
        application.processEvents()
        table = editor.table
        item = table.item(0, 1)
        assert item is not None
        table.editItem(item)
        application.processEvents()
        cell = table.findChild(QLineEdit)
        assert cell is not None
        option = QStyleOptionFrame()
        cell.initStyleOption(option)
        content = cell.style().subElementRect(QStyle.SubElement.SE_LineEditContents, option, cell)
        assert table.rowHeight(0) >= cell.fontMetrics().height()
        assert content.height() >= cell.fontMetrics().height()
    finally:
        editor.close()
        editor.deleteLater()
        application.processEvents()


@pytest.mark.parametrize("cancel_at", ("urdf", "destination"))
def test_blank_welcome_survives_cancelled_first_import(
    builder: main_window.MainWindow,
    urdf: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    cancel_at: str,
) -> None:
    welcome = builder.preview_stack.currentWidget()
    assert welcome is not None and welcome is not builder.viewport
    assert any(label.text() == "Build your Robot Pack" for label in welcome.findChildren(QLabel))
    assert _button(welcome, "Open Robot Pack").isEnabled()
    create = _button(welcome, "Create from URDF")
    assert create.isEnabled()
    assert not builder.save_action.isEnabled() and not builder.export_action.isEnabled()
    original_files = set(tmp_path.iterdir())
    monkeypatch.setattr(
        main_window,
        "_select_path",
        _file_selection("" if cancel_at == "urdf" else str(urdf), ""),
    )

    create.click()

    assert builder.project is None
    assert builder.preview_stack.currentWidget() is welcome
    assert builder.project_tree.topLevelItemCount() == 0
    assert not builder.save_action.isEnabled() and not builder.export_action.isEnabled()
    assert set(tmp_path.iterdir()) == original_files
    assert isinstance(builder.viewport, _ViewportStub) and builder.viewport.rendered is None


def test_first_urdf_import_becomes_an_editable_saved_pack(
    builder: main_window.MainWindow,
    urdf: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    destination = tmp_path / "new_pack"
    monkeypatch.setattr(main_window, "_select_path", _file_selection(str(urdf), str(destination)))
    welcome = builder.preview_stack.currentWidget()
    assert welcome is not None
    _button(welcome, "Create from URDF").click()

    project = builder.project
    assert project is not None and project.loaded.root == destination
    assert project.validate(ValidationProfile.AUTHORING).valid
    assert builder.preview_stack.currentWidget() is builder.viewport
    assert builder.save_action.isEnabled() and builder.export_action.isEnabled()
    assert builder.project_tree.topLevelItemCount() == 1
    assert isinstance(builder.viewport, _ViewportStub)
    assert builder.viewport.rendered is not None
    asset, module = builder.viewport.rendered
    assert asset.root_links == ("base_link",) and module.root_link == "base_link"

    # Edit the newly imported document through its real Inspector controls,
    # then exercise Save and reload the on-disk pack independently.
    name = builder.properties.findChildren(QLineEdit)[0]
    name.setText("My first RobotPack")
    _button(builder.properties, "Apply metadata").click()
    assert builder.project is not None and builder.project.dirty
    builder.save_action.trigger()

    assert builder.project is not None and not builder.project.dirty
    reloaded = RobotPackLoader().load(destination)
    assert reloaded.pack.manifest.name == "My first RobotPack"
    assert reloaded.pack == builder.project.pack
