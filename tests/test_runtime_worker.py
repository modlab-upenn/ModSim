"""Thread-boundary smoke test for the standalone Runtime Inspector worker."""

from __future__ import annotations

import os
import time
from pathlib import Path
from threading import Event as ThreadEvent
from threading import Thread

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QEventLoop, QThread, QTimer
from PySide6.QtWidgets import QApplication

from modsim.runtime import RuntimeInspectorFrame
from modsim.runtime.inspector_runner import (
    RuntimeInspectorConfig as CoreRuntimeInspectorConfig,
)
from modsim.runtime.inspector_runner import RuntimeInspectorRunner
from modsim_studio import runtime_worker as runtime_worker_module
from modsim_studio.runtime_worker import RuntimeInspectorConfig, RuntimeInspectorWorker


def test_worker_preserves_public_config_import() -> None:
    assert RuntimeInspectorConfig is CoreRuntimeInspectorConfig
    config = RuntimeInspectorConfig(pack_path=Path("pack"))
    assert not config.viewer_enabled
    assert config.real_time_factor == 1.0


def test_worker_stops_on_terminal_result_and_publishes_the_exact_final_frame(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = RuntimeInspectorConfig(
        pack_path=example_pack_dir, backend="mock", dt_s=0.01, duration_s=2.0
    )
    runner = RuntimeInspectorRunner.create(config)
    worker = RuntimeInspectorWorker(config)
    frames: list[RuntimeInspectorFrame] = []
    worker.frame_ready.connect(frames.append)
    monkeypatch.setattr(
        RuntimeInspectorRunner,
        "execution_finished",
        property(lambda owner: owner.session.world.time_s >= 0.02),
    )
    monkeypatch.setattr(RuntimeInspectorWorker, "_wait_until", lambda *_args: True)
    try:
        assert not worker._run_scenario(runner)
        assert runner.session.world.time_s == pytest.approx(0.02)
        assert frames[-1].metrics.time_s == pytest.approx(0.02)
    finally:
        runner.shutdown()


def test_worker_applies_real_time_factor_only_to_pacing_deadlines(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = RuntimeInspectorConfig(
        pack_path=example_pack_dir,
        backend="mock",
        connector_gap_m=0.005,
        approach_m_s=0.03,
        duration_s=0.05,
        dt_s=0.01,
        publish_hz=100.0,
        real_time_factor=4.0,
    )
    runner = RuntimeInspectorRunner.create(config)
    worker = RuntimeInspectorWorker(config)
    pacing_calls: list[tuple[float, float, float]] = []

    def record_deadline(
        wall_started_s: float,
        simulated_elapsed_s: float,
        real_time_factor: float,
    ) -> float:
        pacing_calls.append((wall_started_s, simulated_elapsed_s, real_time_factor))
        # The helper's arithmetic has dependency-free unit coverage. Returning
        # an already-reached deadline keeps this execution-lane test clock-free.
        return wall_started_s

    monkeypatch.setattr(runtime_worker_module, "wall_clock_deadline_s", record_deadline)
    monkeypatch.setattr(RuntimeInspectorWorker, "_wait_until", lambda _self, _target: True)
    try:
        interrupted = worker._run_scenario(runner)
    finally:
        runner.shutdown()

    assert not interrupted
    assert [call[1] for call in pacing_calls] == pytest.approx([0.01, 0.02, 0.03, 0.04, 0.05])
    assert {call[2] for call in pacing_calls} == {4.0}
    assert runner.session.world.time_s == pytest.approx(config.duration_s)


def test_worker_pause_freezes_world_time_and_resets_playback_state(
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
    runner = RuntimeInspectorRunner.create(config)
    worker = RuntimeInspectorWorker(config)
    playback: list[bool] = []
    paused_acknowledged = ThreadEvent()
    resumed_acknowledged = ThreadEvent()

    def receive_playback(paused: bool) -> None:
        playback.append(paused)
        if paused:
            paused_acknowledged.set()
        elif paused_acknowledged.is_set():
            resumed_acknowledged.set()

    worker.playback_changed.connect(receive_playback)
    worker.set_paused(True)
    execution = Thread(target=worker._run_scenario, args=(runner,), daemon=True)
    execution.start()
    try:
        deadline_s = time.monotonic() + 1.0
        while not paused_acknowledged.is_set() and time.monotonic() < deadline_s:
            application.processEvents()
            time.sleep(0.001)
        assert paused_acknowledged.is_set()
        paused_time_s = runner.session.world.time_s
        time.sleep(0.03)
        assert runner.session.world.time_s == pytest.approx(paused_time_s)

        worker.set_paused(False)
        deadline_s = time.monotonic() + 1.0
        while not resumed_acknowledged.is_set() and time.monotonic() < deadline_s:
            application.processEvents()
            time.sleep(0.001)
        assert resumed_acknowledged.is_set()
        execution.join(timeout=2.0)

        assert not execution.is_alive()
        assert playback == [False, True, False]
        assert runner.session.world.time_s == pytest.approx(config.duration_s)
    finally:
        worker.request_interruption()
        execution.join(timeout=1.0)
        runner.shutdown()


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
            assert all(frame.metrics.time_s == 0 for frame in frames)
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
    assert statuses[-1] == ("Time limit reached" if resume else "Run stopped")
