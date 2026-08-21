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
    encode_runtime_message,
)
from modsim_studio.runtime_process import (
    RuntimeInspectorProcessController,
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
    controller.frame_ready.connect(received.append)
    controller.status_changed.connect(statuses.append)
    try:
        frame = runner.frame()
        wire = encode_runtime_message(RuntimeHello()) + encode_runtime_message(
            RuntimeFrame(frame=frame)
        )
        split = len(wire) // 3
        controller.consume_protocol_output(wire[:split])
        controller.consume_protocol_output(wire[split : split * 2])
        controller.consume_protocol_output(wire[split * 2 :])
        application.processEvents()

        assert received == [frame]
        assert statuses == ["Native MuJoCo viewer connected…"]
    finally:
        runner.shutdown()


def test_process_controller_launches_without_shell_and_stops_cooperatively(
    application: QApplication,
    example_pack_dir: Path,
) -> None:
    child_code = "\n".join(
        (
            "import sys",
            "from modsim.runtime.inspection_protocol import (",
            "    RuntimeFinished, RuntimeFinishedReason, RuntimeHello, RuntimeInitialize,",
            "    RuntimeStatus, RuntimeStop, decode_runtime_message, encode_runtime_message,",
            ")",
            "def send(message):",
            "    sys.stdout.buffer.write(encode_runtime_message(message))",
            "    sys.stdout.buffer.flush()",
            "send(RuntimeHello())",
            "assert isinstance(decode_runtime_message("
            "sys.stdin.buffer.readline()), RuntimeInitialize)",
            "send(RuntimeStatus(message='fake viewer waiting'))",
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
    failures: list[str] = []
    timed_out = False
    loop = QEventLoop()

    def receive_status(message: str) -> None:
        statuses.append(message)
        if message == "fake viewer waiting":
            controller.request_interruption()

    def timeout() -> None:
        nonlocal timed_out
        timed_out = True
        controller.request_interruption()
        loop.quit()

    controller.status_changed.connect(receive_status)
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
