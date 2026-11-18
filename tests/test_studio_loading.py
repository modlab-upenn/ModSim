"""Studio loading feedback remains responsive and cleans up failed work."""

from __future__ import annotations

import os
import subprocess
import sys
from itertools import pairwise
from pathlib import Path
from threading import Event, get_ident

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QElapsedTimer, QPoint, QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPixmap
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QPushButton, QVBoxLayout, QWidget

from modsim_robotics_studio._loading import BrandLogo, BusyDialog, LoadingPanel, StartupSplash
from modsim_robotics_studio._splash_animation import AnimatedBrandLogo
from modsim_robotics_studio.appearance import theme_manager
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
        assert splash.findChild(BrandLogo) is None
        assert splash.findChild(AnimatedBrandLogo) is None
        assert splash.findChild(QTimer, "modsimLogoAnimation") is None
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


def test_closing_startup_cancels_reveal_without_an_overlay_logo(application: QApplication) -> None:
    splash = StartupSplash("Runtime Inspector")
    window = QWidget()
    try:
        splash.show()
        QTimer.singleShot(40, splash.close)
        assert not splash.present(window)
        assert not window.isVisible() and not splash.isVisible()
        assert splash.findChild(BrandLogo) is None
        assert splash.findChild(AnimatedBrandLogo) is None
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


@pytest.mark.parametrize("size", [(460, 400), (720, 405), (1000, 650)])
@pytest.mark.parametrize("point_size", [10.0, 18.0])
@pytest.mark.parametrize("content_kind", ["runtime", "welcome"])
def test_cover_fills_panel_and_controls_stay_in_the_faded_overlay(
    application: QApplication,
    size: tuple[int, int],
    point_size: float,
    content_kind: str,
) -> None:
    original = QFont(application.font())
    font = QFont(original)
    font.setPointSizeF(point_size)
    application.setFont(font)
    manager = theme_manager()
    manager._apply()
    panel = LoadingPanel(
        "Build your Robot Pack" if content_kind == "welcome" else "Runtime Inspector"
    )
    buttons: list[QPushButton] = []
    if content_kind == "runtime":
        panel.set_steps(
            (
                "Prepare runtime execution",
                "Load and validate Robot Pack",
                "Initialize simulation backend",
                "Receive first simulation state",
            )
        )
        panel.set_stage(
            "Initializing the backend", "Loading module geometry and connector metadata."
        )
    else:
        panel.set_busy(False)
        panel.set_stage("Import a URDF or open an existing Robot Pack to begin authoring.")
        actions = QVBoxLayout()
        for text in ("Create from URDF", "Open Robot Pack"):
            button = QPushButton(text)
            actions.addWidget(button)
            buttons.append(button)
        panel._content.insertLayout(panel._content.count() - 1, actions)
    panel.resize(*size)
    try:
        panel.show()
        application.processEvents()
        panel.grab()  # Exercise the exact artwork rectangle used by painting.
        application.processEvents()
        cover = panel._cover_rect
        controls = QRectF(panel.copy.geometry())
        assert cover.width() > 0 and cover.height() > 0
        assert cover.width() / cover.height() == pytest.approx(16 / 9, rel=0.01)
        assert cover.contains(QRectF(panel.rect()))
        assert panel._fade_rect.contains(controls)
        assert panel.rect().contains(panel.copy.geometry())
        assert panel.findChild(BrandLogo) is None
        assert panel.findChild(AnimatedBrandLogo) is None
        for widget in [
            *panel.copy.findChildren(QLabel),
            *buttons,
            panel.progress,
        ]:
            if widget.isHidden():
                continue
            assert widget.isVisible()
            if widget is not panel.progress:
                assert widget.height() >= widget.minimumSizeHint().height()
            rectangle = QRectF(widget.mapTo(panel, QPoint()), widget.size())
            assert controls.contains(rectangle)
            assert panel._fade_rect.contains(rectangle)
        for button in buttons:
            assert button.width() >= button.minimumSizeHint().width()
    finally:
        panel.close()
        panel.deleteLater()
        application.setFont(original)
        manager._apply()
        application.processEvents()


def test_cover_fade_blends_smoothly_without_an_opaque_panel_edge(application: QApplication) -> None:
    manager = theme_manager()
    previous_theme = manager.theme.id
    manager.set_theme("midnight", persist=False)
    panel = LoadingPanel("Robot Pack Builder")
    source = QColor("#808080")
    backdrop = QPixmap(1920, 1080)
    backdrop.fill(source)
    panel._background = backdrop
    panel._scaled_background = None
    panel._background_scale = None
    panel.resize(900, 506)
    try:
        panel.show()
        application.processEvents()
        image = panel.grab().toImage()
        ratio = image.devicePixelRatio()
        fade = panel._fade_rect
        assert fade.top() > 20
        assert fade.bottom() >= panel.height() - 1

        def pixel(y: int) -> QColor:
            # The controls have a 24px margin; this column sees only artwork
            # and the full-width fade, never labels or progress animation.
            return image.pixelColor(round(8 * ratio), round(y * ratio))

        assert pixel(20) == source
        assert pixel(int(fade.top()) - 8) == source
        colors = [pixel(y) for y in range(int(fade.top()) + 4, panel.height() - 20)]
        differences = [
            sum(abs(channel - 128) for channel in color.getRgb()[:3]) for color in colors
        ]
        assert differences[-1] > 100
        # A fade should change progressively, rather than introduce the hard
        # color jump that made the previous floating rectangle conspicuous.
        assert max(abs(after - before) for before, after in pairwise(differences)) < 20
        assert all(after >= before - 3 for before, after in pairwise(differences))
        blank_y = panel.copy.geometry().bottom() - 2
        blank_x = panel.copy.geometry().right() - 8
        assert image.pixelColor(round(blank_x * ratio), round(blank_y * ratio)) == pixel(blank_y)
    finally:
        panel.close()
        panel.deleteLater()
        manager.set_theme(previous_theme, persist=False)
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
