"""Companion-process controller tests for dual-window runtime inspection."""

from __future__ import annotations

import os
import sys
from collections.abc import Iterator
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QEventLoop, QTimer
from PySide6.QtWidgets import QApplication

from modsim.runtime import RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    RuntimeHello,
    RuntimePlaybackState,
    RuntimeSetPaused,
    encode_runtime_message,
)
from modsim_studio.runtime_process import (
    RuntimeInspectorProcessController,
    RuntimeInspectorThreadController,
    RuntimeViewerLaunchError,
    resolve_runtime_viewer_python,
)


@pytest.fixture(scope="module")
def application() -> Iterator[QApplication]:
    existing = QApplication.instance()
    app = existing if isinstance(existing, QApplication) else QApplication([])
    yield app
    app.processEvents()


def test_process_controller_forwards_fragmented_immutable_frame(
    application: QApplication,
    example_pack_dir: Path,
) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(pack_path=example_pack_dir, backend="mock")
    )
    controller = RuntimeInspectorProcessController(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            viewer_enabled=True,
        )
    )
    received: list[object] = []
    statuses: list[str] = []
    playback: list[bool] = []
    controller.frame_ready.connect(received.append)
    controller.status_changed.connect(statuses.append)
    controller.playback_changed.connect(playback.append)
    try:
        frame = runner.frame()
        wire = b"".join(
            (
                encode_runtime_message(RuntimeHello()),
                encode_runtime_message(RuntimeFrame(frame=frame)),
                encode_runtime_message(RuntimePlaybackState(paused=True)),
                encode_runtime_message(RuntimePlaybackState(paused=False)),
            )
        )
        split = len(wire) // 3
        controller.consume_protocol_output(wire[:split])
        controller.consume_protocol_output(wire[split : split * 2])
        controller.consume_protocol_output(wire[split * 2 :])
        application.processEvents()

        assert received == [frame]
        assert statuses == ["Native MuJoCo viewer connected…"]
        assert playback == [True, False]
    finally:
        runner.shutdown()


def test_thread_controller_forwards_and_requests_playback_state(
    application: QApplication,
    example_pack_dir: Path,
) -> None:
    controller = RuntimeInspectorThreadController(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            backend="mock",
            duration_s=0.5,
            dt_s=0.01,
            publish_hz=100.0,
        )
    )
    observed: list[bool] = []
    failures: list[str] = []
    timed_out = False
    pause_requested = False
    loop = QEventLoop()

    def receive_status(_message: str) -> None:
        nonlocal pause_requested
        if not pause_requested:
            pause_requested = True
            controller.set_paused(True)

    def receive_playback(paused: bool) -> None:
        observed.append(paused)
        if paused:
            controller.set_paused(False)
        elif True in observed:
            controller.request_interruption()

    def timeout() -> None:
        nonlocal timed_out
        timed_out = True
        controller.request_interruption()
        loop.quit()

    controller.status_changed.connect(receive_status)
    controller.playback_changed.connect(receive_playback)
    controller.failed.connect(failures.append)
    controller.finished.connect(loop.quit)
    timer = QTimer()
    timer.setSingleShot(True)
    timer.timeout.connect(timeout)
    timer.start(5_000)
    controller.start()
    loop.exec()
    timer.stop()
    if controller.is_running():
        controller.stop_and_wait(3_000)

    assert not timed_out
    assert failures == []
    assert observed[:3] == [False, True, False]


def test_process_controller_rejects_parent_bound_pause_command_from_child(
    application: QApplication,
    example_pack_dir: Path,
) -> None:
    controller = RuntimeInspectorProcessController(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            viewer_enabled=True,
        )
    )
    failures: list[str] = []
    controller.failed.connect(failures.append)

    controller.consume_protocol_output(
        encode_runtime_message(RuntimeHello())
        + encode_runtime_message(RuntimeSetPaused(paused=True))
    )
    application.processEvents()

    assert len(failures) == 1
    assert "invalid parent-bound message 'set_paused'" in failures[0]


def test_process_controller_launches_without_shell_and_stops_cooperatively(
    application: QApplication,
    example_pack_dir: Path,
) -> None:
    child_code = "\n".join(
        (
            "import sys",
            "from modsim.runtime.inspection_protocol import (",
            "    RuntimeFinished, RuntimeFinishedReason, RuntimeHello, RuntimeInitialize,",
            "    RuntimePlaybackState, RuntimeSetPaused, RuntimeStatus, RuntimeStop,",
            "    decode_runtime_message, encode_runtime_message,",
            ")",
            "def send(message):",
            "    sys.stdout.buffer.write(encode_runtime_message(message))",
            "    sys.stdout.buffer.flush()",
            "send(RuntimeHello())",
            "assert isinstance(decode_runtime_message("
            "sys.stdin.buffer.readline()), RuntimeInitialize)",
            "send(RuntimeStatus(message='fake viewer waiting'))",
            "pause = decode_runtime_message(sys.stdin.buffer.readline())",
            "assert isinstance(pause, RuntimeSetPaused) and pause.paused",
            "send(RuntimePlaybackState(paused=True))",
            "resume = decode_runtime_message(sys.stdin.buffer.readline())",
            "assert isinstance(resume, RuntimeSetPaused) and not resume.paused",
            "send(RuntimePlaybackState(paused=False))",
            "assert isinstance(decode_runtime_message(sys.stdin.buffer.readline()), RuntimeStop)",
            "send(RuntimeFinished(reason=RuntimeFinishedReason.STOPPED))",
        )
    )
    controller = RuntimeInspectorProcessController(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            viewer_enabled=True,
        ),
        command=(sys.executable, "-c", child_code),
    )
    statuses: list[str] = []
    playback: list[bool] = []
    failures: list[str] = []
    timed_out = False
    loop = QEventLoop()

    def receive_status(message: str) -> None:
        statuses.append(message)
        if message == "fake viewer waiting":
            controller.set_paused(True)

    def receive_playback(paused: bool) -> None:
        playback.append(paused)
        if paused:
            controller.set_paused(False)
        else:
            controller.request_interruption()

    def timeout() -> None:
        nonlocal timed_out
        timed_out = True
        controller.request_interruption()
        loop.quit()

    controller.status_changed.connect(receive_status)
    controller.playback_changed.connect(receive_playback)
    controller.failed.connect(failures.append)
    controller.finished.connect(loop.quit)
    timer = QTimer()
    timer.setSingleShot(True)
    timer.timeout.connect(timeout)
    timer.start(5_000)
    controller.start()
    loop.exec()
    timer.stop()
    if controller.is_running():
        controller.stop_and_wait(3_000)

    assert not timed_out
    assert failures == []
    assert playback == [True, False]
    assert not controller.is_running()
    assert "Native MuJoCo viewer connected…" in statuses
    assert "fake viewer waiting" in statuses
    assert statuses[-1] == "Run stopped"


def test_runtime_viewer_python_uses_current_interpreter_off_macos(tmp_path: Path) -> None:
    interpreter = tmp_path / "python"
    assert resolve_runtime_viewer_python(
        platform="linux",
        python_executable=interpreter,
    ) == str(interpreter)


def test_runtime_viewer_python_requires_environment_sibling_on_macos(
    tmp_path: Path,
) -> None:
    interpreter = tmp_path / "python"
    interpreter.write_text("", encoding="utf-8")
    mjpython = tmp_path / "mjpython"
    mjpython.write_text("#!/bin/sh\n", encoding="utf-8")
    mjpython.chmod(0o755)

    assert resolve_runtime_viewer_python(
        platform="darwin",
        python_executable=interpreter,
    ) == str(mjpython)

    mjpython.unlink()
    with pytest.raises(RuntimeViewerLaunchError, match="not found next to"):
        resolve_runtime_viewer_python(
            platform="darwin",
            python_executable=interpreter,
        )
