"""Runtime startup stays responsive until a valid immutable frame is presented."""

from __future__ import annotations

import os
import subprocess
import sys
from collections.abc import Iterator
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtCore import QObject, Signal
from PySide6.QtTest import QSignalSpy
from PySide6.QtWidgets import QApplication, QLabel, QMessageBox, QProgressBar

from modsim_robotics.model_views import ModelViewSourceStamp, ModuleTopologyGraphView
from modsim_robotics.runtime import RuntimeInspectorConfig, RuntimeInspectorFrame
from modsim_robotics.runtime.inspection import RuntimeEventRow
from modsim_robotics.runtime.metrics import DockingMetrics
from modsim_robotics_studio import runtime_app, runtime_window
from modsim_robotics_studio._loading import LoadingPanel
from modsim_robotics_studio.runtime_window import RuntimeInspectorWindow


class StubController(QObject):
    """Publish real controller signals without starting a backend or process."""

    frame_ready = Signal(object)
    status_changed = Signal(str)
    playback_changed = Signal(bool)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, _config: RuntimeInspectorConfig, parent: QObject) -> None:
        super().__init__(parent)
        self.running = False
        self.start_count = 0
        self.stop_requested = False

    def start(self) -> None:
        self.start_count += 1
        self.running = True

    def is_running(self) -> bool:
        return self.running

    def request_interruption(self) -> None:
        self.stop_requested = True

    def set_paused(self, paused: bool) -> None:
        self.playback_changed.emit(paused)

    def stop_and_wait(self, _timeout_ms: int = 10_000) -> bool:
        self.running = False
        return True


@pytest.fixture(scope="module")
def application() -> Iterator[QApplication]:
    existing = QApplication.instance()
    app = existing if isinstance(existing, QApplication) else QApplication([])
    yield app
    app.processEvents()


def test_runtime_waits_for_the_inspector_to_be_shown(
    application: QApplication, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", StubController)
    window = RuntimeInspectorWindow(RuntimeInspectorConfig(pack_path=tmp_path, backend="mock"))
    worker = window.findChild(StubController)
    assert worker is not None
    try:
        application.processEvents()
        assert not worker.running and worker.start_count == 0
        window.show()
        application.processEvents()
        assert worker.running and worker.start_count == 1
        window.hide()
        window.show()
        application.processEvents()
        assert worker.start_count == 1
    finally:
        window.stop_and_wait()
        window.close()


def _frame(sequence: int) -> RuntimeInspectorFrame:
    return RuntimeInspectorFrame(
        backend_name="mock",
        view=ModuleTopologyGraphView(
            id="module_topology",
            name="Module topology",
            builder="module_topology_graph",
            source=ModelViewSourceStamp(
                pack_id="generic_cube",
                pack_version="0.1.0",
                world_time_s=float(sequence),
                sample_sequence=sequence,
            ),
            nodes=(),
            edges=(),
        ),
        metrics=DockingMetrics(
            time_s=float(sequence),
            module_count_total=0,
            module_count_connected=0,
            module_count_free=0,
            assembly_count=0,
            largest_assembly_size=0,
            connection_count_active=0,
            connector_count_total=0,
            connector_count_free=0,
            connector_count_candidate=0,
            connector_count_docked=0,
            docking_success_count=0,
            docking_failure_count=0,
            docking_attempt_count=0,
            undocking_success_count=0,
            undocking_failure_count=0,
            assembly_merge_count=0,
            assembly_split_count=0,
            connector_overload_count=0,
            event_count_total=sequence + 1,
        ),
        events=(RuntimeEventRow(sequence=sequence, time_s=float(sequence), kind="Ready"),),
        event_start_sequence=sequence,
        next_event_sequence=sequence + 1,
    )


def _step_states(window: RuntimeInspectorWindow) -> tuple[str, ...]:
    return tuple(
        label.accessibleName()
        for label in window.loading_panel.findChildren(QLabel)
        if label.objectName().startswith("StartupStep")
    )


@pytest.mark.parametrize("viewer_enabled", [False, True], ids=["thread", "process"])
def test_loading_reveals_workspace_after_first_presented_frame(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    viewer_enabled: bool,
) -> None:
    controllers: list[StubController] = []

    def controller(config: RuntimeInspectorConfig, parent: QObject) -> StubController:
        value = StubController(config, parent)
        controllers.append(value)
        return value

    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", controller)
    monkeypatch.setattr(runtime_window, "RuntimeInspectorProcessController", controller)
    window = RuntimeInspectorWindow(
        RuntimeInspectorConfig(
            pack_path=tmp_path,
            backend="mujoco" if viewer_enabled else "mock",
            viewer_enabled=viewer_enabled,
        )
    )
    worker = controllers[0]
    try:
        window.show()
        application.processEvents()
        assert worker.running
        assert window.stop_button.isEnabled()
        assert window.workspace_stack.currentWidget() is window.loading_panel
        progress = window.loading_panel.findChild(QProgressBar)
        assert progress is not None
        assert progress.minimum() == progress.maximum() == 0
        assert _step_states(window) == (
            "Prepare runtime execution: current",
            "Load and validate Robot Pack: waiting",
            "Initialize simulation backend: waiting",
            "Receive first simulation state: waiting",
        )

        worker.status_changed.emit("Loading and validating Robot Pack…")
        assert window.status_label.text() == "Loading and validating Robot Pack…"
        assert any(
            label.text() == "Loading and validating Robot Pack…"
            for label in window.loading_panel.findChildren(QLabel)
        )
        assert _step_states(window)[0].endswith(": complete")
        assert _step_states(window)[1].endswith(": current")
        worker.status_changed.emit(f"Starting {'mujoco' if viewer_enabled else 'mock'} backend…")
        assert _step_states(window)[1].endswith(": complete")
        assert _step_states(window)[2].endswith(": current")
        worker.status_changed.emit("Running generic_cube: front ↔ front")
        assert _step_states(window)[2].endswith(": complete")
        assert _step_states(window)[3].endswith(": current")

        workspace_ready = QSignalSpy(window.workspace_stack.currentChanged)
        worker.frame_ready.emit(_frame(0))
        worker.frame_ready.emit(_frame(1))
        # Queuing a frame alone must not expose an uninitialized workspace.
        assert window.workspace_stack.currentWidget() is window.loading_panel
        assert workspace_ready.wait(1000)

        assert window.workspace_stack.currentWidget() is window.workspace_tabs
        assert progress.maximum() > progress.minimum()
        assert progress.isHidden()
        assert all(state.endswith(": complete") for state in _step_states(window))
        assert tuple(row.sequence for row in window.events.model.events) == (0, 1)
        assert window.source_label.text() == "generic_cube@0.1.0 | t=1.000 s | sample=1"
        worker.running = False
        worker.finished.emit()
        assert window.workspace_stack.currentWidget() is window.workspace_tabs
        assert tuple(row.sequence for row in window.events.model.events) == (0, 1)
        assert not window.stop_button.isEnabled()
    finally:
        worker.running = False
        window.stop_and_wait()
        window.close()


@pytest.mark.parametrize("failed", [False, True], ids=["stop", "failure"])
def test_loading_can_stop_or_fail_before_first_frame(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    failed: bool,
) -> None:
    monkeypatch.setattr(runtime_window, "RuntimeInspectorThreadController", StubController)

    def dismiss_error(*_args: object) -> QMessageBox.StandardButton:
        return QMessageBox.StandardButton.Ok

    monkeypatch.setattr(QMessageBox, "critical", dismiss_error)
    window = RuntimeInspectorWindow(RuntimeInspectorConfig(pack_path=tmp_path, backend="mock"))
    worker = window.findChild(StubController)
    assert worker is not None
    try:
        window.show()
        application.processEvents()
        worker.status_changed.emit("Loading and validating Robot Pack…")
        if failed:
            worker.failed.emit("Robot Pack could not be loaded")
        else:
            window.stop_button.click()
            assert worker.stop_requested
        worker.running = False
        worker.finished.emit()

        assert window.workspace_stack.currentWidget() is window.loading_panel
        progress = window.loading_panel.findChild(QProgressBar)
        assert progress is not None
        assert progress.maximum() > progress.minimum()
        assert progress.isHidden()
        assert _step_states(window)[1].endswith(": failed" if failed else ": stopped")
        assert _step_states(window)[2].endswith(": waiting")
        assert not window.stop_button.isEnabled()
        assert not window.pause_button.isEnabled()
        assert window.exit_code == int(failed)
        assert (window.last_error is not None) == failed
    finally:
        worker.running = False
        window.stop_and_wait()
        window.close()


def test_loading_background_cache_tracks_device_pixel_ratio(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = LoadingPanel("Runtime Inspector")
    panel.resize(640, 360)
    try:
        panel.show()
        application.processEvents()
        # Moving between screens can change DPR without changing logical dimensions.
        monkeypatch.setattr(panel, "devicePixelRatioF", lambda: 2.0)
        panel.repaint()
        image = panel._scaled_background  # pyright: ignore[reportPrivateUsage]
        assert image is not None
        assert image.devicePixelRatio() == 2.0
        assert image.width() >= 1280 and image.height() >= 720

        monkeypatch.setattr(panel, "devicePixelRatioF", lambda: 1.0)
        panel.repaint()
        image = panel._scaled_background  # pyright: ignore[reportPrivateUsage]
        assert image is not None
        assert image.devicePixelRatio() == 1.0
        assert image.width() >= 640 and image.height() >= 360
    finally:
        panel.close()


def test_startup_splash_canvas_uses_screen_pixel_ratio() -> None:
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    environment["QT_SCALE_FACTOR"] = "2"
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "from PySide6.QtWidgets import QApplication; "
            "from modsim_robotics_studio._loading import StartupSplash; "
            "app = QApplication([]); splash = StartupSplash('Runtime Inspector'); "
            "splash.show(); app.processEvents(); "
            "ratio = app.primaryScreen().devicePixelRatio(); "
            "assert ratio >= 2; assert splash.pixmap().devicePixelRatio() == ratio; "
            "assert splash.pixmap().width() >= splash.width() * ratio; "
            "assert splash.rect().contains(splash.panel.geometry()); "
            "assert splash.panel.rect().contains(splash.panel.copy.geometry()); splash.close()",
        ],
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr


def test_runtime_entry_module_defers_plotting_imports() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import modsim_robotics_studio.runtime_app; "
            "assert 'modsim_robotics_studio.runtime_window' not in sys.modules; "
            "assert 'pyqtgraph' not in sys.modules",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr


def test_startup_failure_closes_visible_splash_and_restores_exception_hook(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    def fail_window(_config: RuntimeInspectorConfig) -> None:
        assert any(
            widget.objectName() == "ModSimStartupSplash" and widget.isVisible()
            for widget in application.topLevelWidgets()
        )
        raise ValueError("workspace initialization failed")

    monkeypatch.setenv("MODSIM_STUDIO_LOG", str(tmp_path / "runtime.log"))
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "desktop-data"))
    monkeypatch.setattr(runtime_window, "RuntimeInspectorWindow", fail_window)
    previous_hook = sys.excepthook

    with pytest.raises(ValueError, match="workspace initialization failed"):
        runtime_app.main(RuntimeInspectorConfig(pack_path=tmp_path, backend="mock"))

    assert sys.excepthook is previous_hook
    assert not any(
        widget.objectName() == "ModSimStartupSplash" and widget.isVisible()
        for widget in application.topLevelWidgets()
    )
    assert "Runtime Inspector session ended" in (tmp_path / "runtime.log").read_text(
        encoding="utf-8"
    )
