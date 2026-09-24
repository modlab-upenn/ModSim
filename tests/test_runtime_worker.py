"""Thread-boundary smoke test for the standalone Runtime Inspector worker."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QEventLoop, QThread, QTimer
from PySide6.QtWidgets import QApplication

from modsim.runtime import RuntimeInspectorFrame
from modsim.runtime.inspector_runner import RuntimeInspectorConfig as CoreRuntimeInspectorConfig
from modsim_studio.runtime_worker import RuntimeInspectorConfig, RuntimeInspectorWorker


def test_worker_preserves_public_config_import() -> None:
    assert RuntimeInspectorConfig is CoreRuntimeInspectorConfig
    assert not RuntimeInspectorConfig(pack_path=Path("pack")).viewer_enabled


def test_worker_publishes_a_docked_graph_and_lossless_events(
    example_pack_dir: Path,
) -> None:
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        application = QApplication([])

    config = RuntimeInspectorConfig(
        pack_path=example_pack_dir,
        backend="mock",
        connector_gap_m=0.005,
        approach_m_s=0.03,
        duration_s=0.05,
        dt_s=0.01,
        publish_hz=100.0,
    )
    worker = RuntimeInspectorWorker(config)
    thread = QThread()
    worker.moveToThread(thread)
    frames: list[RuntimeInspectorFrame] = []
    failures: list[str] = []
    timed_out = False
    loop = QEventLoop()

    def receive_frame(value: object) -> None:
        assert isinstance(value, RuntimeInspectorFrame)
        frames.append(value)

    def timeout() -> None:
        nonlocal timed_out
        timed_out = True
        worker.request_interruption()
        thread.requestInterruption()
        loop.quit()

    worker.frame_ready.connect(receive_frame)
    worker.failed.connect(failures.append)
    worker.finished.connect(thread.quit)
    thread.finished.connect(loop.quit)
    thread.started.connect(worker.run)
    timer = QTimer()
    timer.setSingleShot(True)
    timer.timeout.connect(timeout)
    timer.start(3_000)
    thread.start()
    loop.exec()
    timer.stop()
    if thread.isRunning():
        worker.request_interruption()
        thread.requestInterruption()
        thread.quit()
        thread.wait(3_000)

    assert not timed_out
    assert failures == []
    assert not thread.isRunning()
    assert frames
    assert frames[0].view.edges == ()
    assert len(frames[-1].view.edges) == 1
    assert [row.kind for frame in frames for row in frame.events] == [
        "DockCandidateDetected",
        "DockCommitted",
        "AssemblyMerged",
    ]


def test_spatial_worker_publishes_timeout_instead_of_completion(smores_pack_dir: Path) -> None:
    pytest.importorskip("mujoco")
    from modsim.runtime import ReconfigurationPhase, RuntimeDemo

    application = QApplication.instance()
    if not isinstance(application, QApplication):
        application = QApplication([])
    worker = RuntimeInspectorWorker(
        RuntimeInspectorConfig(
            pack_path=smores_pack_dir,
            demo=RuntimeDemo.SMORES_SPATIAL_HANDOFF,
            duration_s=0.004,
            dt_s=0.002,
        )
    )
    frames: list[RuntimeInspectorFrame] = []
    statuses: list[str] = []
    failures: list[str] = []

    def receive_frame(value: object) -> None:
        assert isinstance(value, RuntimeInspectorFrame)
        frames.append(value)

    worker.frame_ready.connect(receive_frame)
    worker.status_changed.connect(statuses.append)
    worker.failed.connect(failures.append)
    worker.run()
    assert failures == []
    assert frames[-1].scenario is not None
    assert frames[-1].scenario.phase is ReconfigurationPhase.TIMED_OUT
    assert "Time limit reached" in statuses[-1]


@pytest.mark.parametrize("resume", [True, False])
def test_worker_pause_can_resume_or_stop_without_advancing_while_paused(
    example_pack_dir: Path,
    resume: bool,
) -> None:
    application = QApplication.instance()
    if not isinstance(application, QApplication):
        application = QApplication([])
    worker = RuntimeInspectorWorker(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            backend="mock",
            duration_s=0.004,
            dt_s=0.002,
        )
    )
    frames: list[RuntimeInspectorFrame] = []
    playback: list[bool] = []
    statuses: list[str] = []

    def receive_frame(value: object) -> None:
        assert isinstance(value, RuntimeInspectorFrame)
        frames.append(value)

    def playback_changed(paused: bool) -> None:
        playback.append(paused)
        if paused:
            assert len(frames) == 1
            assert frames[0].metrics.time_s == 0
            if resume:
                worker.set_paused(False)
            else:
                worker.request_interruption()

    worker.frame_ready.connect(receive_frame)
    worker.playback_changed.connect(playback_changed)
    worker.status_changed.connect(statuses.append)
    worker.set_paused(True)
    worker.run()
    assert playback == ([False, True, False] if resume else [False, True])
    assert frames[-1].metrics.time_s == pytest.approx(0.004 if resume else 0)
    assert statuses[-1] == ("Run complete" if resume else "Run stopped")
