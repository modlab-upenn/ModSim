"""Studio loading feedback remains responsive and cleans up failed work."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from threading import Event, get_ident

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QElapsedTimer, Qt, QTimer
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QWidget

from modsim_robotics_studio._loading import BrandLogo, BusyDialog, LoadingPanel, StartupSplash
from modsim_robotics_studio.chrome import StudioHeader


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


def test_startup_intro_stays_visible_for_three_seconds_and_services_events(
    application: QApplication,
) -> None:
    splash = StartupSplash("Robot Pack Builder")
    window = QWidget()
    elapsed = QElapsedTimer()
    heartbeat: list[int] = []
    timer = QTimer()
    timer.setInterval(25)
    timer.timeout.connect(lambda: heartbeat.append(elapsed.elapsed()))
    try:
        elapsed.start()
        splash.show()
        timer.start()
        QTest.mouseClick(splash, Qt.MouseButton.LeftButton)
        assert splash.isVisible()
        assert splash.play_intro()
        assert elapsed.elapsed() >= 1_100
        assert not window.isVisible()
        assert splash.present(window)
        assert elapsed.elapsed() >= 3_000
        assert window.isVisible() and not splash.isVisible()
        assert len(heartbeat) > 10
        animation = splash.findChild(QTimer, "modsimLogoAnimation")
        assert animation is not None and not animation.isActive()
    finally:
        timer.stop()
        window.close()
        splash.close()
        window.deleteLater()
        splash.deleteLater()
        application.processEvents()


def test_completed_startup_does_not_add_another_three_second_wait(
    application: QApplication, monkeypatch: pytest.MonkeyPatch
) -> None:
    splash = StartupSplash("Runtime Inspector")

    class CompletedClock:
        def isValid(self) -> bool:
            return True

        def elapsed(self) -> int:
            return 3_001

    try:
        splash.show()
        monkeypatch.setattr(splash, "_shown_clock", CompletedClock())
        assert splash._wait_until(3_000)
        assert splash._waiting_loop is None
    finally:
        splash.close()
        splash.deleteLater()
        application.processEvents()


def test_closing_startup_cancels_reveal_and_stops_animation(application: QApplication) -> None:
    splash = StartupSplash("Runtime Inspector")
    window = QWidget()
    try:
        splash.show()
        QTimer.singleShot(40, splash.close)
        assert not splash.present(window)
        assert not window.isVisible() and not splash.isVisible()
        animation = splash.findChild(QTimer, "modsimLogoAnimation")
        assert animation is not None and not animation.isActive()
        assert not splash.play_intro()
    finally:
        splash.close()
        window.close()
        splash.deleteLater()
        window.deleteLater()
        application.processEvents()


def test_header_keeps_workspace_beside_wordmark(application: QApplication) -> None:
    window = QMainWindow()
    window.resize(1000, 600)
    header = StudioHeader(window, "Robot Pack Builder")
    window.addToolBar(header)
    try:
        window.show()
        application.processEvents()
        logo = header.findChild(BrandLogo)
        workspace = header.findChild(QLabel, "Workspace")
        assert logo is not None and workspace is not None
        assert logo.geometry().right() < workspace.geometry().left()
    finally:
        window.close()
        window.deleteLater()
        application.processEvents()


def test_small_loading_panel_reserves_wrapped_title_height(application: QApplication) -> None:
    panel = LoadingPanel("Robot Pack Builder")
    panel.resize(680, 382)
    try:
        panel.show()
        application.processEvents()
        title = panel.findChild(QLabel, "BrandTitle")
        assert title is not None
        assert title.height() >= title.heightForWidth(title.width())
        assert title.geometry().bottom() < panel.stage.geometry().top()
        assert panel.copy.geometry().bottom() < panel.height()
    finally:
        panel.close()
        panel.deleteLater()
        application.processEvents()


def test_document_work_runs_off_the_gui_thread_while_events_remain_responsive(
    application: QApplication,
) -> None:
    parent = QWidget()
    parent.show()
    released = Event()
    heartbeat: list[bool] = []
    gui_thread = get_ident()

    def release() -> None:
        heartbeat.append(True)
        released.set()

    def operation() -> int:
        assert released.wait(2), "The GUI timer was blocked by document work"
        return get_ident()

    try:
        with BusyDialog("Importing URDF…", parent) as loading:
            assert loading.panel.progress.maximum() == 0
            QTimer.singleShot(40, release)
            worker_thread = loading.run(operation)
            assert heartbeat == [True]
            assert worker_thread != gui_thread
            assert loading.isVisible()
        assert not loading.isVisible()
    finally:
        parent.close()
        parent.deleteLater()
        application.processEvents()


def test_failed_document_work_closes_loading_feedback(application: QApplication) -> None:
    parent = QWidget()
    parent.show()

    def fail() -> None:
        raise ValueError("Invalid test URDF")

    try:
        with (
            pytest.raises(ValueError, match="Invalid test URDF"),
            BusyDialog("Importing URDF…", parent) as loading,
        ):
            loading.run(fail)
        assert not loading.isVisible()
        assert loading.panel.progress.maximum() == 1
    finally:
        parent.close()
        parent.deleteLater()
        application.processEvents()


def test_studio_entry_module_defers_plotting_and_builder_imports() -> None:
    probe = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import modsim_robotics_studio.app; "
            "assert 'modsim_robotics_studio.main_window' not in sys.modules; "
            "assert 'pyvista' not in sys.modules; assert 'pyvistaqt' not in sys.modules; "
            "assert 'pyqtgraph' not in sys.modules",
        ],
        text=True,
        capture_output=True,
        timeout=20,
    )
    assert probe.returncode == 0, probe.stderr


def test_failed_startup_closes_splash_and_restores_exception_hook(
    application: QApplication, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from modsim_robotics_studio import app

    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "desktop-data"))
    monkeypatch.delenv("MODSIM_STUDIO_LOG", raising=False)
    hook = sys.excepthook

    def fail(**_kwargs: object) -> QWidget:
        splash = next(
            widget for widget in application.topLevelWidgets() if isinstance(widget, StartupSplash)
        )
        assert splash.isVisible()
        assert splash.panel.stage.text() == "Loading Studio…"
        raise RuntimeError("Builder initialization failed")

    monkeypatch.setattr(app, "_create_window", fail)
    with pytest.raises(RuntimeError, match="Builder initialization failed"):
        app.main()
    assert sys.excepthook is hook
    assert not any(
        widget.isVisible()
        for widget in application.topLevelWidgets()
        if isinstance(widget, StartupSplash)
    )
