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
from modsim_studio.runtime_worker import RuntimeInspectorConfig, RuntimeInspectorWorker


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
