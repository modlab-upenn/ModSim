"""No-Qt MuJoCo runtime host for the dual-window Runtime Inspector.

The ordinary Python parent process owns Qt.  This child is launched with
``mjpython`` on macOS so the native MuJoCo viewer can own that process's Cocoa
main thread.  It is also the sole owner of the mutable runtime, scenario,
model-view factory, ``MjModel``, and ``MjData``.  The parent receives only the
versioned immutable protocol values written to stdout.

Stdout is reserved for protocol records.  Full diagnostics go to stderr, and
the child never configures or truncates the Studio session log.
"""

from __future__ import annotations

import os
import queue
import sys
import time
import traceback
from contextlib import suppress
from dataclasses import dataclass
from threading import Event as ThreadEvent
from threading import Thread
from typing import BinaryIO, TextIO

from modsim.runtime.inspection_protocol import (
    MAX_RUNTIME_MESSAGE_BYTES,
    RuntimeError,
    RuntimeFinished,
    RuntimeFinishedReason,
    RuntimeFrame,
    RuntimeHello,
    RuntimeInitialize,
    RuntimePlaybackState,
    RuntimeProtocolError,
    RuntimeProtocolMessage,
    RuntimeSetPaused,
    RuntimeStatus,
    RuntimeStop,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim.runtime.inspector_runner import (
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
    RuntimeInspectorSetupError,
)
from modsim.runtime.spatial import SpatialReconfigurationScenario
from modsim_backend_mujoco.spatial_experiment import SpatialExperiment
from modsim_backend_mujoco.spatial_viewer import run_spatial_viewer
from modsim_backend_mujoco.viewer import run_with_viewer

_FAILED_MESSAGE_PREFIX = "MuJoCo Runtime Inspector failed: "
_CONTROL_CLOSE_GRACE_S = 0.5
_CONTROL_FORCE_GRACE_S = 0.5


class _ProtocolOutputClosed(ConnectionError):
    """Raised internally when the parent no longer consumes child output."""


@dataclass(slots=True)
class _ProtocolWriter:
    stream: BinaryIO
    connected: bool = True

    def send(self, message: RuntimeProtocolMessage) -> None:
        """Write and flush one complete record without polluting stdout."""
        if not self.connected:
            raise _ProtocolOutputClosed("runtime protocol output is closed")
        encoded = encode_runtime_message(message)
        try:
            self.stream.write(encoded)
            self.stream.flush()
        except (BrokenPipeError, OSError, ValueError) as error:
            self.connected = False
            raise _ProtocolOutputClosed("Runtime Inspector parent disconnected") from error


@dataclass(slots=True)
class _FramePublisher:
    """Publish throttled live frames and explicit lifecycle frames."""

    runner: RuntimeInspectorRunner
    writer: _ProtocolWriter
    stop_requested: ThreadEvent
    next_publish_at_s: float

    @classmethod
    def create(
        cls,
        runner: RuntimeInspectorRunner,
        writer: _ProtocolWriter,
        stop_requested: ThreadEvent,
    ) -> _FramePublisher:
        return cls(
            runner=runner,
            writer=writer,
            stop_requested=stop_requested,
            next_publish_at_s=time.monotonic() + 1.0 / runner.config.publish_hz,
        )

    def publish_initial(self) -> None:
        self.writer.send(RuntimeFrame(frame=self.runner.frame()))

    def viewer_started(self) -> None:
        self.writer.send(RuntimeStatus(message="MuJoCo viewer opened; runtime is running"))

    def after_step(self) -> None:
        now = time.monotonic()
        if now < self.next_publish_at_s:
            return
        self.writer.send(RuntimeFrame(frame=self.runner.frame()))
        interval_s = 1.0 / self.runner.config.publish_hz
        while self.next_publish_at_s <= now:
            self.next_publish_at_s += interval_s

    def scenario_complete(self) -> None:
        # This explicit frame cannot be skipped by refresh throttling and keeps
        # all event deltas lossless before the viewer enters its final hold.
        self.writer.send(RuntimeFrame(frame=self.runner.frame()))
        self.writer.send(
            RuntimeStatus(
                message=(
                    f"{self.runner.completion_message}; "
                    "the MuJoCo viewer is holding the final state. "
                    "Close either window when finished."
                )
            )
        )

    def viewer_stopped(self) -> None:
        # ``on_stopped`` runs from the viewer's finally block. Do not replace an
        # in-flight broken-pipe exception with a second write failure.
        if not self.writer.connected:
            self.stop_requested.set()
            return
        try:
            self.writer.send(RuntimeFrame(frame=self.runner.frame()))
        except _ProtocolOutputClosed:
            self.stop_requested.set()


def run_runtime_inspector_process(
    input_stream: BinaryIO,
    output_stream: BinaryIO,
    diagnostic_stream: TextIO,
) -> int:
    """Run one protocol-controlled MuJoCo viewer child and return an exit code."""
    writer = _ProtocolWriter(output_stream)
    stop_requested = ThreadEvent()
    paused = ThreadEvent()
    control_shutdown = ThreadEvent()
    control_errors: queue.SimpleQueue[Exception] = queue.SimpleQueue()
    control_thread: Thread | None = None
    runner: RuntimeInspectorRunner | None = None
    failure: Exception | None = None
    reason = RuntimeFinishedReason.FAILED

    try:
        writer.send(RuntimeHello())
        config = _read_initialization(input_stream)
        _validate_child_config(config)
        control_thread = Thread(
            target=_watch_control_stream,
            args=(input_stream, stop_requested, control_shutdown, control_errors, paused),
            name="ModSimRuntimeControl",
            daemon=True,
        )
        control_thread.start()

        def report_status(message: str) -> None:
            try:
                writer.send(RuntimeStatus(message=message))
            except _ProtocolOutputClosed:
                # Let construction finish so the returned runner can follow
                # the normal, explicit shutdown path below.
                stop_requested.set()

        runner = RuntimeInspectorRunner.create(
            config,
            status_callback=report_status,
        )
        publisher = _FramePublisher.create(runner, writer, stop_requested)
        publisher.publish_initial()

        if stop_requested.is_set():
            runner.stop()
            publisher.viewer_stopped()
            reason = RuntimeFinishedReason.STOPPED
        else:
            if isinstance(runner.scenario, SpatialReconfigurationScenario):
                motion = runner.scenario.motion
                if not isinstance(motion, SpatialExperiment):
                    raise TypeError("spatial viewer requires MuJoCo mechanical services")
                run_spatial_viewer(
                    motion,
                    pause_requested=paused.is_set,
                    set_paused=lambda value: paused.set() if value else paused.clear(),
                    on_playback_changed=lambda value: writer.send(
                        RuntimePlaybackState(paused=value)
                    ),
                    speed=1.0,
                    hold=True,
                    stop_requested=stop_requested.is_set,
                    on_started=publisher.viewer_started,
                    after_step=publisher.after_step,
                    on_scenario_complete=publisher.scenario_complete,
                    on_stopped=publisher.viewer_stopped,
                )
                reason = {
                    "complete": RuntimeFinishedReason.COMPLETED,
                    "stopped": RuntimeFinishedReason.STOPPED,
                    "failed": RuntimeFinishedReason.FAILED,
                    "timeout": RuntimeFinishedReason.FAILED,
                }[runner.scenario.phase]
            else:
                run_with_viewer(
                    runner.session,
                    duration_s=config.duration_s,
                    step_once=runner.step,
                    hold=True,
                    stop_requested=stop_requested.is_set,
                    on_started=publisher.viewer_started,
                    after_step=publisher.after_step,
                    on_scenario_complete=publisher.scenario_complete,
                    on_stopped=publisher.viewer_stopped,
                )
                reason = (
                    RuntimeFinishedReason.STOPPED
                    if stop_requested.is_set()
                    else RuntimeFinishedReason.VIEWER_CLOSED
                )

        control_error = _next_control_error(control_errors)
        if control_error is not None:
            raise control_error
    except _ProtocolOutputClosed:
        # A closed parent pipe is itself a cooperative stop signal. There is no
        # recipient for another protocol record, but backend cleanup still runs.
        stop_requested.set()
        reason = RuntimeFinishedReason.STOPPED
    except Exception as error:
        failure = error
        reason = RuntimeFinishedReason.FAILED
        _write_diagnostic(error, diagnostic_stream)
    finally:
        if runner is not None:
            try:
                runner.shutdown()
            except Exception as error:
                _write_diagnostic(error, diagnostic_stream)
                if failure is None:
                    failure = error
                    reason = RuntimeFinishedReason.FAILED

    if writer.connected:
        try:
            if failure is not None:
                writer.send(RuntimeError(message=_failure_message(failure)))
            writer.send(RuntimeFinished(reason=reason))
        except _ProtocolOutputClosed:
            stop_requested.set()
    _shutdown_control_reader(
        input_stream,
        control_thread,
        control_shutdown,
        diagnostic_stream,
    )
    return 1 if reason is RuntimeFinishedReason.FAILED else 0


def _read_initialization(stream: BinaryIO) -> RuntimeInspectorConfig:
    line = stream.readline(MAX_RUNTIME_MESSAGE_BYTES + 1)
    if not line:
        raise RuntimeProtocolError("runtime protocol input ended before the initialize message")
    message = decode_runtime_message(line)
    if not isinstance(message, RuntimeInitialize):
        raise RuntimeProtocolError("the first parent message must be a runtime initialize message")
    return message.config


def _validate_child_config(config: RuntimeInspectorConfig) -> None:
    if config.backend != "mujoco":
        raise RuntimeInspectorSetupError("the native viewer process requires the MuJoCo backend")
    if not config.viewer_enabled:
        raise RuntimeInspectorSetupError("the native viewer process requires viewer_enabled=true")


def _watch_control_stream(
    stream: BinaryIO,
    stop_requested: ThreadEvent,
    control_shutdown: ThreadEvent,
    errors: queue.SimpleQueue[Exception],
    paused: ThreadEvent | None = None,
) -> None:
    """Read parent commands without ever touching runtime or MuJoCo state."""
    while not stop_requested.is_set():
        try:
            line = stream.readline(MAX_RUNTIME_MESSAGE_BYTES + 1)
            if not line:
                if not control_shutdown.is_set():
                    stop_requested.set()
                return
            message = decode_runtime_message(line)
            if isinstance(message, RuntimeSetPaused) and paused is not None:
                if message.paused:
                    paused.set()
                else:
                    paused.clear()
                continue
            if not isinstance(message, RuntimeStop):
                raise RuntimeProtocolError(
                    "the runtime host accepts only stop or playback commands after initialization"
                )
        except Exception as error:
            if not control_shutdown.is_set():
                errors.put(error)
                stop_requested.set()
            return
        stop_requested.set()
        return


def _shutdown_control_reader(
    stream: BinaryIO,
    thread: Thread | None,
    control_shutdown: ThreadEvent,
    diagnostic_stream: TextIO,
) -> None:
    """Unblock and join the stdin reader before CPython finalizes streams.

    A daemon blocked inside ``sys.stdin.buffer.readline`` can make CPython
    abort while finalizing the buffered stream. The conforming parent closes
    its write channel when it receives ``RuntimeFinished``, so EOF is the
    portable primary wake-up. Raw-descriptor closure is only a bounded fallback
    for a disconnected or nonconforming parent.
    """
    if thread is None or not thread.is_alive():
        return
    control_shutdown.set()
    thread.join(timeout=_CONTROL_CLOSE_GRACE_S)
    if not thread.is_alive():
        return
    try:
        descriptor = stream.fileno()
    except (AttributeError, OSError, ValueError):
        _write_control_shutdown_warning(diagnostic_stream)
        return
    with suppress(OSError):
        os.close(descriptor)
    thread.join(timeout=_CONTROL_FORCE_GRACE_S)
    if not thread.is_alive():
        with suppress(OSError, ValueError):
            stream.close()
        return
    _write_control_shutdown_warning(diagnostic_stream)


def _write_control_shutdown_warning(diagnostic_stream: TextIO) -> None:
    try:
        print(
            "MuJoCo Runtime Inspector control reader did not stop cleanly",
            file=diagnostic_stream,
            flush=True,
        )
    except Exception:
        return


def _next_control_error(
    errors: queue.SimpleQueue[Exception],
) -> Exception | None:
    try:
        return errors.get_nowait()
    except queue.Empty:
        return None


def _failure_message(error: Exception) -> str:
    detail = str(error).strip() or type(error).__name__
    limit = 8192 - len(_FAILED_MESSAGE_PREFIX)
    return _FAILED_MESSAGE_PREFIX + detail[:limit]


def _write_diagnostic(error: Exception, stream: TextIO) -> None:
    try:
        traceback.print_exception(type(error), error, error.__traceback__, file=stream)
        stream.flush()
    except Exception:
        # Diagnostics must never prevent viewer/backend cleanup.
        return


def main() -> None:
    """Module entry point used by the Qt process controller."""
    raise SystemExit(
        run_runtime_inspector_process(
            sys.stdin.buffer,
            sys.stdout.buffer,
            sys.stderr,
        )
    )


if __name__ == "__main__":
    main()


__all__ = ["main", "run_runtime_inspector_process"]
