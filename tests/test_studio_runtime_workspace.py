# pyright: reportPrivateUsage=false
"""Studio's Runtime workspace launches saved packs without leaving the editor."""

from __future__ import annotations

import os
import shutil
import time
from collections.abc import Callable, Iterator
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyvistaqt")
pytest.importorskip("pyqtgraph")

from PySide6.QtCore import QObject, QPoint, QSettings, Signal
from PySide6.QtGui import QFont
from PySide6.QtTest import QTest
from PySide6.QtWidgets import (
    QApplication,
    QMessageBox,
    QScrollArea,
    QStyle,
    QStyleOptionButton,
    QWidget,
)

from modsim_robotics.importers import ImportedRobotAsset
from modsim_robotics.robot_packs import ModuleType, RobotPackLoader
from modsim_robotics.runtime import RuntimeInspectorConfig
from modsim_robotics_studio import _window_sizing, main_window, runtime_window
from modsim_robotics_studio.appearance import theme_manager
from modsim_robotics_studio.project import StudioProject

_EXAMPLE = Path(__file__).resolve().parents[1] / "examples/robot_packs/generic_cube"


class _ViewportStub(QWidget):
    entity_selected = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.on_close: Callable[[], None] | None = None

    def render_module(self, _asset: ImportedRobotAsset, _module: ModuleType) -> None:
        pass

    def set_layer_visibility(self, **_kwargs: bool) -> None:
        pass

    def select_entity(self, *_args: str) -> None:
        pass

    def fit_module(self) -> None:
        pass

    def close(self) -> bool:
        if self.on_close is not None:
            self.on_close()
        return super().close()


class _ControllerStub(QObject):
    frame_ready = Signal(object)
    status_changed = Signal(str)
    playback_changed = Signal(bool)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, _config: RuntimeInspectorConfig, parent: QObject) -> None:
        super().__init__(parent)
        self.running = False
        self.interrupted = False
        self.shutdown_count = 0

    def start(self) -> None:
        self.running = True
        self.playback_changed.emit(False)

    def is_running(self) -> bool:
        return self.running

    def request_interruption(self) -> None:
        self.interrupted = True

    def set_paused(self, paused: bool) -> None:
        self.playback_changed.emit(paused)

    def finish(self) -> None:
        self.running = False
        self.finished.emit()

    def stop_and_wait(self, _timeout_ms: int = 10_000) -> bool:
        self.shutdown_count += 1
        self.finish()
        return True


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.fixture
def pack_path(tmp_path: Path) -> Path:
    return Path(shutil.copytree(_EXAMPLE, tmp_path / "generic_cube"))


@pytest.fixture
def builder(
    application: QApplication, monkeypatch: pytest.MonkeyPatch
) -> Iterator[main_window.MainWindow]:
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)

    def discard_changes(*_args: object) -> QMessageBox.StandardButton:
        return QMessageBox.StandardButton.Discard

    monkeypatch.setattr(QMessageBox, "question", discard_changes)
    window = main_window.MainWindow()
    window.show()
    application.processEvents()
    try:
        yield window
    finally:
        window.runtime_workspace.stop_and_wait()
        window.project = None
        window.close()
        window.deleteLater()
        application.processEvents()


def _open(builder: main_window.MainWindow, path: Path) -> None:
    builder._set_project(StudioProject.open(path))
    builder.workspaces.setCurrentIndex(1)
    workspace = builder.runtime_workspace
    workspace.backend.setCurrentIndex(workspace.backend.findData("mock"))


def _wait_until(condition: Callable[[], bool], timeout_ms: int = 5_000) -> None:
    deadline = time.monotonic() + timeout_ms / 1_000
    while not condition() and time.monotonic() < deadline:
        QTest.qWait(10)
    assert condition(), "Runtime did not reach the expected state before the timeout"


def test_blank_workspace_disables_launch_until_a_pack_is_open(
    builder: main_window.MainWindow,
) -> None:
    builder.workspaces.setCurrentIndex(1)
    workspace = builder.runtime_workspace
    assert not workspace.settings.isEnabled()
    assert not workspace.validate_button.isEnabled()
    assert not workspace.start_button.isEnabled()
    assert workspace.validate_runtime() is None
    workspace.start_simulation()
    assert workspace.inspector is None
    assert not workspace.is_running


def test_backend_comparison_preserves_scenario_settings(
    builder: main_window.MainWindow, pack_path: Path
) -> None:
    _open(builder, pack_path)
    workspace = builder.runtime_workspace
    workspace.duration.setValue(42)
    workspace.speed.setValue(3)
    workspace.timestep.setValue(0.001)
    workspace.gap.setText("0.007")
    workspace.fixed_connector.setCurrentIndex(workspace.fixed_connector.findData("front"))
    workspace.backend.setCurrentIndex(workspace.backend.findData("mujoco"))
    assert workspace.duration.value() == 42
    assert workspace.speed.value() == 3
    assert workspace.timestep.value() == pytest.approx(0.001)
    assert workspace.gap.text() == "0.007"
    assert workspace.fixed_connector.currentData() == "front"
    assert workspace.viewer.isEnabled() and workspace.viewer.isChecked()
    workspace.backend.setCurrentIndex(workspace.backend.findData("mock"))
    assert not workspace.viewer.isEnabled() and not workspace.viewer.isChecked()
    assert workspace.gap.text() == "0.007"


@pytest.mark.parametrize("point_size", [9, 14, 18])
def test_setup_and_embedded_runtime_fit_small_windows_with_large_fonts(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    pack_path: Path,
    point_size: int,
) -> None:
    original_font = QFont(application.font())
    font = QFont(original_font)
    font.setPointSize(point_size)
    application.setFont(font)
    manager = theme_manager()
    manager._apply()
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)
    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", _ControllerStub)
    window = main_window.MainWindow()
    try:
        _open(window, pack_path)
        window.resize(800, 600)
        window.show()
        application.processEvents()
        workspace = window.runtime_workspace
        assert window.width() <= 800 and window.height() <= 600
        assert all(dock.isHidden() for dock in window._authoring_docks)
        assert window.document_toolbar.isHidden()
        workspace.advanced.toggle.setChecked(True)
        application.processEvents()
        assert window.width() <= 800 and window.height() <= 600
        scroll = next(
            item
            for item in workspace.setup_page.findChildren(QScrollArea)
            if item.widget() is workspace.settings
        )
        assert scroll.verticalScrollBar().maximum() > 0
        for control in (workspace.validate_button, workspace.start_button):
            assert control.isVisible()
            assert control.width() >= control.minimumSizeHint().width()
            option = QStyleOptionButton()
            option.initFrom(control)
            option.text = control.text()
            contents = control.style().subElementRect(
                QStyle.SubElement.SE_PushButtonContents, option, control
            )
            assert contents.height() >= control.fontMetrics().height()
            assert contents.width() >= control.fontMetrics().horizontalAdvance(control.text())
            assert window.rect().contains(control.mapTo(window, QPoint(0, 0)))
            assert window.rect().contains(control.mapTo(window, control.rect().bottomRight()))

        workspace.start_button.click()
        application.processEvents()
        assert workspace.is_running
        assert workspace.inspector is not None
        assert window.width() <= 800 and window.height() <= 600
        assert workspace.pages.currentWidget() is workspace.live_page
        assert workspace.inspector.width() <= window.width()
        assert workspace.inspector.height() <= window.height()
    finally:
        window.runtime_workspace.stop_and_wait()
        window.project = None
        window.close()
        window.deleteLater()
        application.processEvents()
        application.setFont(original_font)
        manager._apply()


def test_workspace_switch_preserves_dirty_edits_and_individual_dock_visibility(
    builder: main_window.MainWindow, pack_path: Path, application: QApplication
) -> None:
    _open(builder, pack_path)
    builder.workspaces.setCurrentIndex(0)
    assert builder.project is not None
    changed = builder.project.update_manifest(
        name="Edited in memory", version="0.1.0", description="Unsaved authoring notes"
    )
    builder._set_project(changed)
    docks = builder._authoring_docks
    docks[0].show()
    docks[1].hide()
    docks[2].show()
    before = tuple(not dock.isHidden() for dock in docks)
    builder.workspaces.setCurrentIndex(1)
    application.processEvents()
    assert all(dock.isHidden() for dock in docks)
    assert all(not dock.toggleViewAction().isEnabled() for dock in docks)
    assert builder.document_toolbar.isHidden()
    assert builder.project is changed
    assert builder.project.dirty
    builder.workspaces.setCurrentIndex(0)
    application.processEvents()
    assert tuple(not dock.isHidden() for dock in docks) == before
    assert all(dock.toggleViewAction().isEnabled() for dock in docks)
    assert not builder.document_toolbar.isHidden()
    assert builder.project is changed
    assert RobotPackLoader().load(pack_path).pack.manifest.name != "Edited in memory"


def test_runtime_selected_before_first_show_survives_saved_dock_restoration(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    pack_path: Path,
    tmp_path: Path,
) -> None:
    settings = QSettings(str(tmp_path / "window-layout.ini"), QSettings.Format.IniFormat)

    def local_settings(*_args: object, **_kwargs: object) -> QSettings:
        return settings

    monkeypatch.setattr(_window_sizing, "QSettings", local_settings)
    monkeypatch.setattr(main_window, "RobotViewport", _ViewportStub)
    seed = main_window.MainWindow()
    window: main_window.MainWindow | None = None
    try:
        seed.show()
        application.processEvents()
        for dock in seed._authoring_docks:
            dock.show()
        application.processEvents()
        seed.close()
        assert settings.contains("windows/studio/dock_state")

        window = main_window.MainWindow()
        _open(window, pack_path)
        assert all(dock.isHidden() for dock in window._authoring_docks)
        window.show()
        QTest.qWait(30)
        assert window.workspaces.currentIndex() == 1
        assert all(dock.isHidden() for dock in window._authoring_docks)
        assert window.document_toolbar.isHidden()
        window.workspaces.setCurrentIndex(0)
        application.processEvents()
        assert all(not dock.isHidden() for dock in window._authoring_docks)
        assert not window.document_toolbar.isHidden()
    finally:
        seed.close()
        seed.deleteLater()
        if window is not None:
            window.project = None
            window.close()
            window.deleteLater()
        application.processEvents()


def test_missing_view_is_actionable_and_save_cancel_preserves_unsaved_recipe(
    builder: main_window.MainWindow, pack_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    project = StudioProject.open(pack_path)
    for recipe in project.pack.manifest.model_views:
        project = project.remove_model_view(recipe.id)
    project = project.save()
    builder._set_project(project)
    builder.workspaces.setCurrentIndex(1)
    workspace = builder.runtime_workspace
    workspace.backend.setCurrentIndex(workspace.backend.findData("mock"))
    report = workspace.validate_runtime()
    assert report is not None and not report.valid
    assert any(issue.code == "runtime.model_view" for issue in report.errors)
    workspace.start_simulation()
    assert workspace.inspector is None

    workspace.add_view_button.click()
    assert builder.project is not None and builder.project.dirty
    assert workspace.model_view.currentData() is not None
    assert not RobotPackLoader().load(pack_path).pack.manifest.model_views
    report = workspace.validate_runtime()
    assert report is not None and report.valid
    assert "Unsaved" in workspace.readiness.toPlainText()

    monkeypatch.setattr(workspace, "_confirm_save", lambda: False)
    workspace.start_simulation()
    assert builder.project.dirty
    assert workspace.inspector is None
    assert not RobotPackLoader().load(pack_path).pack.manifest.model_views

    monkeypatch.setattr(workspace, "_confirm_save", lambda: True)
    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", _ControllerStub)
    workspace.start_simulation()
    assert builder.project is not None and not builder.project.dirty
    assert workspace.is_running
    assert workspace.inspector is not None
    persisted = RobotPackLoader().load(pack_path).pack.manifest.model_views
    assert persisted and persisted[-1].id == workspace.model_view.currentData()
    workspace.stop_and_wait()


def test_invalid_form_settings_block_start_before_creating_an_inspector(
    builder: main_window.MainWindow, pack_path: Path
) -> None:
    _open(builder, pack_path)
    workspace = builder.runtime_workspace
    workspace.gap.setText("not a number")
    workspace.start_simulation()
    assert workspace.inspector is None
    assert not workspace.is_running
    assert "Connector gap must be a number" in workspace.readiness.toPlainText()
    workspace.gap.setText("0.02")
    workspace.viewer.setChecked(True)
    report = workspace.validate_runtime()
    assert report is not None and not report.valid
    workspace.start_simulation()
    assert workspace.inspector is None
    assert not workspace.is_running
    assert "runtime.viewer" in workspace.readiness.toPlainText()


def test_active_execution_blocks_persistence_even_after_authoring_edits(
    builder: main_window.MainWindow, pack_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _open(builder, pack_path)
    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", _ControllerStub)
    workspace = builder.runtime_workspace
    workspace.start_simulation()
    assert workspace.is_running
    inspector = workspace.inspector
    assert inspector is not None
    workspace.start_simulation()
    assert workspace.inspector is inspector
    controller = inspector.findChild(_ControllerStub)
    assert controller is not None
    for action in (
        builder.create_action,
        builder.open_action,
        builder.save_action,
        builder.export_action,
    ):
        assert not action.isEnabled()
    assert not workspace.back_button.isEnabled()

    builder.workspaces.setCurrentIndex(0)
    assert builder.project is not None
    edited = builder.project.update_manifest(
        name="Next launch", version="0.1.0", description="Changed during simulation"
    )
    builder._set_project(edited)
    assert builder.project is edited and edited.dirty
    assert workspace.is_running
    assert not builder.save_action.isEnabled()
    assert not builder.export_action.isEnabled()
    builder._save()
    assert builder.project is edited and edited.dirty
    assert RobotPackLoader().load(pack_path).pack.manifest.name != "Next launch"

    controller.finish()
    assert not workspace.is_running
    for action in (
        builder.create_action,
        builder.open_action,
        builder.save_action,
        builder.export_action,
    ):
        assert action.isEnabled()
    assert workspace.back_button.isEnabled()
    assert workspace.inspector is inspector


def test_start_rechecks_saved_pack_after_an_external_disk_change(
    builder: main_window.MainWindow, pack_path: Path
) -> None:
    _open(builder, pack_path)
    workspace = builder.runtime_workspace
    report = workspace.validate_runtime()
    assert report is not None and report.valid
    external = StudioProject.open(pack_path)
    for recipe in external.pack.manifest.model_views:
        external = external.remove_model_view(recipe.id)
    external.save()
    workspace.start_simulation()
    assert workspace.inspector is None
    assert not workspace.is_running
    assert "runtime.model_view" in workspace.readiness.toPlainText()


def test_main_window_close_waits_if_execution_cannot_shut_down_yet(
    builder: main_window.MainWindow, pack_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _open(builder, pack_path)
    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", _ControllerStub)
    workspace = builder.runtime_workspace
    workspace.start_simulation()
    assert workspace.inspector is not None
    controller = workspace.inspector.findChild(_ControllerStub)
    assert controller is not None and controller.running
    closed: list[bool] = []
    assert isinstance(builder.viewport, _ViewportStub)
    builder.viewport.on_close = lambda: closed.append(controller.running)
    with monkeypatch.context() as blocked_shutdown:
        blocked_shutdown.setattr(controller, "stop_and_wait", lambda _timeout_ms=10_000: False)
        assert not builder.close()
        assert builder.isVisible()
        assert workspace.is_running
        assert controller.running
        assert not closed
    assert builder.close()
    assert closed == [False]


def test_main_window_close_reaps_execution_before_tearing_down_the_viewport(
    builder: main_window.MainWindow, pack_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _open(builder, pack_path)
    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", _ControllerStub)
    workspace = builder.runtime_workspace
    workspace.start_simulation()
    assert workspace.inspector is not None
    controller = workspace.inspector.findChild(_ControllerStub)
    assert controller is not None and controller.running
    closed: list[bool] = []
    assert isinstance(builder.viewport, _ViewportStub)
    builder.viewport.on_close = lambda: closed.append(controller.running)
    assert builder.close()
    assert controller.shutdown_count >= 1
    assert not controller.running
    assert closed == [False]
    assert not workspace.is_running


def test_real_mock_run_publishes_events_supports_pause_stop_and_relaunch(
    builder: main_window.MainWindow, pack_path: Path
) -> None:
    _open(builder, pack_path)
    workspace = builder.runtime_workspace
    workspace.gap.setText("0")
    workspace.duration.setValue(6)
    workspace.start_button.click()
    first = workspace.inspector
    assert first is not None
    assert not first.isWindow()
    _wait_until(lambda: bool(first.events.model.events))
    assert workspace.is_running
    assert first.last_error is None
    assert first._latest_frame is not None
    assert first._latest_frame.metrics.module_count_total == 2
    _wait_until(lambda: first.pause_button.isEnabled())
    first.pause_button.click()
    _wait_until(lambda: first.pause_button.text() == "Resume")
    # The acknowledged pause can precede presentation of the last frame that
    # was already queued by the worker. Drain that bounded UI frame batch.
    QTest.qWait(50)
    assert first._latest_frame is not None
    paused_time = first._latest_frame.metrics.time_s
    QTest.qWait(80)
    assert first._latest_frame.metrics.time_s == pytest.approx(paused_time)
    first.pause_button.click()
    _wait_until(lambda: first.pause_button.text() == "Pause")
    first.stop_button.click()
    _wait_until(lambda: not workspace.is_running)
    assert not first.is_running
    assert first.events.model.events
    assert workspace.inspector is first
    workspace.back_button.click()
    assert workspace.pages.currentWidget() is workspace.setup_page

    workspace.duration.setValue(0.05)
    workspace.speed.setValue(10)
    workspace.start_button.click()
    second = workspace.inspector
    assert second is not None and second is not first
    _wait_until(lambda: not workspace.is_running)
    assert second.last_error is None
    assert second._latest_frame is not None
    assert second.events.model.events
    assert builder.save_action.isEnabled()
    assert workspace.back_button.isEnabled()
