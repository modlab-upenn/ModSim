# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
"""Execution controllers for the standalone Runtime Inspector.

The ordinary controller keeps the existing headless runtime in a ``QThread``.
The companion-process controller launches a separate Python interpreter for
MuJoCo's native viewer.  That child is the sole owner of the simulation and
publishes the same immutable frames over a strict local pipe protocol.
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, QThread, QTimer, Signal, Slot

from modsim.runtime import RuntimeInspectorConfig
from modsim.runtime.inspection_protocol import (
    RuntimeError as RuntimeErrorMessage,
)
from modsim.runtime.inspection_protocol import (
    RuntimeFinished,
    RuntimeFinishedReason,
    RuntimeFrame,
    RuntimeHello,
    RuntimeInitialize,
    RuntimePlaybackState,
    RuntimeProtocolError,
    RuntimeProtocolFramer,
    RuntimeSetPaused,
    RuntimeStatus,
    RuntimeStop,
    encode_runtime_message,
)
from modsim_studio.runtime_worker import RuntimeInspectorWorker

_LOGGER = logging.getLogger("modsim.runtime_inspector")
_GRACEFUL_STOP_MS = 5_000
_TERMINATE_GRACE_MS = 2_000


class RuntimeViewerLaunchError(RuntimeError):
    """Raised when the dedicated native-viewer interpreter is unavailable."""


class RuntimeInspectorThreadController(QObject):
    """Adapt the existing worker thread to the common execution-source API."""

    frame_ready = Signal(object)
    status_changed = Signal(str)
    playback_changed = Signal(bool)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, config: RuntimeInspectorConfig, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._started = False
        self._finished_emitted = False
        self._thread = QThread(self)
        self._thread.setObjectName("ModSimRuntimeInspector")
        self._worker = RuntimeInspectorWorker(config)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.frame_ready.connect(self.frame_ready.emit)
        self._worker.status_changed.connect(self.status_changed.emit)
        self._worker.playback_changed.connect(self.playback_changed.emit)
        self._worker.failed.connect(self.failed.emit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._worker.finished.connect(self._thread.quit)
        self._thread.finished.connect(self._emit_finished)

    @Slot()
    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    def request_interruption(self) -> None:
        if not self._thread.isRunning():
            return
        self._worker.request_interruption()
        self._thread.requestInterruption()

    def set_paused(self, paused: bool) -> None:
        """Request a thread-safe worker playback-state change."""
        if not self.is_running():
            return
        self._worker.set_paused(paused)

    def is_running(self) -> bool:
        return self._thread.isRunning()

    def stop_and_wait(self, timeout_ms: int = 10_000) -> bool:
        if not self._thread.isRunning():
            return True
        self.request_interruption()
        self._thread.quit()
        return self._thread.wait(timeout_ms)

    @Slot()
    def _emit_finished(self) -> None:
        if self._finished_emitted:
            return
        self._finished_emitted = True
        self.finished.emit()


class RuntimeInspectorProcessController(QObject):
    """Supervise the one-session MuJoCo viewer host through ``QProcess``."""

    frame_ready = Signal(object)
    status_changed = Signal(str)
    playback_changed = Signal(bool)
    failed = Signal(str)
    finished = Signal()

    def __init__(
        self,
        config: RuntimeInspectorConfig,
        parent: QObject | None = None,
        *,
        command: tuple[str, ...] | None = None,
    ) -> None:
        super().__init__(parent)
        self._config = config
        self._command = command
        self._started = False
        self._hello_received = False
        self._terminal_received = False
        self._failure_emitted = False
        self._finished_emitted = False
        self._stop_requested = False
        self._framer = RuntimeProtocolFramer()

        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert("PYTHONUNBUFFERED", "1")
        self._process.setProcessEnvironment(environment)
        self._process.started.connect(self._process_started)
        self._process.readyReadStandardOutput.connect(self._read_stdout)
        self._process.readyReadStandardError.connect(self._read_stderr)
        self._process.errorOccurred.connect(self._process_error)
        self._process.finished.connect(self._process_finished)

        self._terminate_timer = QTimer(self)
        self._terminate_timer.setSingleShot(True)
        self._terminate_timer.timeout.connect(self._terminate_if_running)
        self._kill_timer = QTimer(self)
        self._kill_timer.setSingleShot(True)
        self._kill_timer.timeout.connect(self._kill_if_running)

    @Slot()
    def start(self) -> None:
        """Launch the native-viewer host once, without invoking a shell."""
        if self._started:
            return
        self._started = True
        if not self._config.viewer_enabled:
            self._fail("The native-viewer process was selected with viewer_enabled false.")
            self._emit_finished()
            return
        if self._config.backend != "mujoco":
            self._fail("The native Runtime Inspector viewer requires the MuJoCo backend.")
            self._emit_finished()
            return
        try:
            command = self._command or runtime_viewer_host_command()
        except RuntimeViewerLaunchError as error:
            self._fail(str(error))
            self._emit_finished()
            return
        self.status_changed.emit("Starting native MuJoCo viewer process…")
        self._process.setProgram(command[0])
        self._process.setArguments(list(command[1:]))
        self._process.start()

    def request_interruption(self) -> None:
        """Request cooperative child shutdown, then arm bounded escalation."""
        if self._stop_requested or not self.is_running():
            return
        self._stop_requested = True
        if self._process.state() == QProcess.ProcessState.Running:
            self._write_message(RuntimeStop())
        self._terminate_timer.start(_GRACEFUL_STOP_MS)

    def set_paused(self, paused: bool) -> None:
        """Request an acknowledged playback-state change from the child."""
        if self._stop_requested or self._process.state() != QProcess.ProcessState.Running:
            return
        self._write_message(RuntimeSetPaused(paused=paused))

    def is_running(self) -> bool:
        return self._process.state() != QProcess.ProcessState.NotRunning

    def stop_and_wait(self, timeout_ms: int = 10_000) -> bool:
        """Stop the host and synchronously reap it during application teardown."""
        if not self.is_running():
            return True
        self.request_interruption()
        if self._process.waitForFinished(timeout_ms):
            return True
        self._process.terminate()
        if self._process.waitForFinished(_TERMINATE_GRACE_MS):
            return True
        self._process.kill()
        return self._process.waitForFinished(_TERMINATE_GRACE_MS)

    @Slot()
    def _process_started(self) -> None:
        self._write_message(RuntimeInitialize(config=self._config))
        if self._stop_requested:
            self._write_message(RuntimeStop())

    @Slot()
    def _read_stdout(self) -> None:
        self.consume_protocol_output(self._process.readAllStandardOutput().data())

    def consume_protocol_output(self, chunk: bytes | bytearray | memoryview) -> None:
        """Validate and dispatch a child-output chunk in protocol order.

        ``QProcess`` calls this through its standard-output handler. Keeping the
        byte-stream boundary explicit also makes fragmentation behavior
        independently testable without starting a native viewer.
        """
        try:
            messages = self._framer.feed(chunk)
        except RuntimeProtocolError as error:
            self._protocol_failure(error)
            return
        for message in messages:
            if isinstance(message, RuntimeHello):
                if self._hello_received:
                    self._protocol_failure(RuntimeProtocolError("duplicate runtime hello"))
                    return
                self._hello_received = True
                self.status_changed.emit("Native MuJoCo viewer connected…")
            elif isinstance(message, RuntimeStatus):
                if not self._require_hello(message.kind):
                    return
                self.status_changed.emit(message.message)
            elif isinstance(message, RuntimeFrame):
                if not self._require_hello(message.kind):
                    return
                self.frame_ready.emit(message.frame)
            elif isinstance(message, RuntimePlaybackState):
                if not self._require_hello(message.kind):
                    return
                self.playback_changed.emit(message.paused)
            elif isinstance(message, RuntimeErrorMessage):
                if not self._require_hello(message.kind):
                    return
                self._fail(message.message)
            elif isinstance(message, RuntimeFinished):
                if not self._require_hello(message.kind):
                    return
                self._terminal_received = True
                # The child has already left the viewer and shut the backend
                # down. Closing our control channel now gives its dedicated
                # stdin reader a portable EOF so it can be joined before the
                # interpreter finalizes buffered streams.
                self._process.closeWriteChannel()
                self._apply_finished_reason(message.reason)
            else:
                self._protocol_failure(
                    RuntimeProtocolError(
                        f"viewer host emitted invalid parent-bound message '{message.kind}'"
                    )
                )
                return

    @Slot()
    def _read_stderr(self) -> None:
        raw_output = self._process.readAllStandardError().data()
        output = bytes(raw_output).decode("utf-8", errors="replace")
        if output:
            _LOGGER.warning("MuJoCo viewer host: %s", output.rstrip())

    @Slot(QProcess.ProcessError)
    def _process_error(self, error: QProcess.ProcessError) -> None:
        if error == QProcess.ProcessError.FailedToStart:
            self._fail(
                f"Native MuJoCo viewer process could not start: {self._process.errorString()}"
            )
            QTimer.singleShot(0, self._emit_finished)
        elif error not in (
            QProcess.ProcessError.Crashed,
            QProcess.ProcessError.Timedout,
        ):
            _LOGGER.warning("MuJoCo viewer process error: %s", self._process.errorString())

    @Slot(int, QProcess.ExitStatus)
    def _process_finished(self, exit_code: int, exit_status: QProcess.ExitStatus) -> None:
        self._terminate_timer.stop()
        self._kill_timer.stop()
        self._read_stdout()
        self._read_stderr()
        try:
            self._framer.finish()
        except RuntimeProtocolError as error:
            self._fail(f"Native MuJoCo viewer protocol ended unexpectedly: {error}")
        if exit_status != QProcess.ExitStatus.NormalExit or exit_code != 0:
            self._fail(
                "Native MuJoCo viewer process exited unexpectedly "
                f"(exit code {exit_code}). See the session log for details."
            )
        elif not self._terminal_received:
            self._fail(
                "Native MuJoCo viewer process exited without a completion message. "
                "See the session log for details."
            )
        self._emit_finished()

    def _write_message(
        self,
        message: RuntimeInitialize | RuntimeSetPaused | RuntimeStop,
    ) -> None:
        try:
            encoded = encode_runtime_message(message)
        except RuntimeProtocolError as error:  # pragma: no cover - validated config
            self._protocol_failure(error)
            return
        written = self._process.write(encoded)
        if written < 0:
            self._fail("Could not send a control message to the native MuJoCo viewer process.")
            self._terminate_if_running()

    def _require_hello(self, kind: str) -> bool:
        if self._hello_received:
            return True
        self._protocol_failure(
            RuntimeProtocolError(f"viewer host sent '{kind}' before its hello message")
        )
        return False

    def _protocol_failure(self, error: Exception) -> None:
        self._fail(f"Native MuJoCo viewer protocol failed: {error}")
        self.request_interruption()

    def _fail(self, message: str) -> None:
        if self._failure_emitted:
            return
        self._failure_emitted = True
        _LOGGER.error("%s", message)
        self.failed.emit(message)

    def _apply_finished_reason(self, reason: RuntimeFinishedReason) -> None:
        messages = {
            RuntimeFinishedReason.COMPLETED: "Run complete; final state remains available",
            RuntimeFinishedReason.STOPPED: "Run stopped",
            RuntimeFinishedReason.VIEWER_CLOSED: (
                "MuJoCo viewer closed; final state remains available"
            ),
            RuntimeFinishedReason.FAILED: "Native MuJoCo viewer run failed",
        }
        if reason == RuntimeFinishedReason.FAILED and not self._failure_emitted:
            self._fail(messages[reason])
        else:
            self.status_changed.emit(messages[reason])

    @Slot()
    def _terminate_if_running(self) -> None:
        if not self.is_running():
            return
        _LOGGER.warning("Native MuJoCo viewer did not stop cooperatively; terminating it")
        self._process.terminate()
        self._kill_timer.start(_TERMINATE_GRACE_MS)

    @Slot()
    def _kill_if_running(self) -> None:
        if not self.is_running():
            return
        _LOGGER.error("Native MuJoCo viewer did not terminate; killing it")
        self._process.kill()

    @Slot()
    def _emit_finished(self) -> None:
        if self._finished_emitted:
            return
        self._finished_emitted = True
        self.finished.emit()


def resolve_runtime_viewer_python(
    *,
    platform: str | None = None,
    python_executable: str | Path | None = None,
) -> str:
    """Resolve the interpreter that can own a passive viewer main loop."""
    selected_platform = sys.platform if platform is None else platform
    selected_python = Path(sys.executable if python_executable is None else python_executable)
    if selected_platform != "darwin":
        return str(selected_python)

    sibling = selected_python.with_name("mjpython")
    if sibling.is_file() and os.access(sibling, os.X_OK):
        return str(sibling)
    raise RuntimeViewerLaunchError(
        "Native MuJoCo viewer requires mjpython, but it was not found next to this "
        "environment's Python. Install the MuJoCo extra in this environment with "
        "pip install -e '.[studio,mujoco]'."
    )


def runtime_viewer_host_command() -> tuple[str, ...]:
    """Return the shell-free command used for the dedicated viewer host."""
    return (
        resolve_runtime_viewer_python(),
        "-m",
        "modsim_backend_mujoco.runtime_inspector_process",
    )


__all__ = [
    "RuntimeInspectorProcessController",
    "RuntimeInspectorThreadController",
    "RuntimeViewerLaunchError",
    "resolve_runtime_viewer_python",
    "runtime_viewer_host_command",
]
