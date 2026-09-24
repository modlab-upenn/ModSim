# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
"""Background execution for the standalone Studio Runtime Inspector.

For headless execution, the worker thread exclusively owns a Qt-free
:class:`~modsim.runtime.RuntimeInspectorRunner`. The Qt thread boundary carries
frozen :class:`~modsim.runtime.RuntimeInspectorFrame` objects, never a backend
adapter or mutable ``WorldState``.
"""

from __future__ import annotations

import logging
import time
from threading import Event as ThreadEvent

from PySide6.QtCore import QObject, QThread, Signal, Slot

from modsim.runtime.demos import RuntimeDemo
from modsim.runtime.inspector_runner import (
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
    RuntimeInspectorSetupError,
    runtime_frame_signature,
)

_LOGGER = logging.getLogger("modsim.runtime_inspector")
_INTERRUPTION_POLL_S = 0.01


class RuntimeInspectorWorker(QObject):
    """Load and execute one demonstration on its owning Qt thread."""

    playback_changed = Signal(bool)
    frame_ready = Signal(object)
    status_changed = Signal(str)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, config: RuntimeInspectorConfig) -> None:
        super().__init__()
        self._config = config
        self._interruption = ThreadEvent()
        self._paused = ThreadEvent()

    def set_paused(self, paused: bool) -> None:
        if paused:
            self._paused.set()
        else:
            self._paused.clear()

    def request_interruption(self) -> None:
        """Request a prompt, cooperative stop from any thread."""
        self._interruption.set()

    @Slot()
    def run(self) -> None:
        """Run the configured scenario, reporting failures instead of escaping."""
        runner: RuntimeInspectorRunner | None = None
        failure: Exception | None = None
        try:
            runner = RuntimeInspectorRunner.create(
                self._config,
                status_callback=self.status_changed.emit,
            )
            if self._interrupted():
                runner.stop()
                self.frame_ready.emit(runner.frame())
                self.status_changed.emit("Run stopped")
                return
            interrupted = self._run_scenario(runner)
            self.status_changed.emit("Run stopped" if interrupted else runner.completion_message)
        except Exception as error:
            failure = error
            _LOGGER.exception("Runtime Inspector worker failed")
            self.failed.emit(_failure_message(error))
        finally:
            if runner is not None:
                try:
                    runner.shutdown()
                except Exception as error:
                    _LOGGER.exception("Runtime Inspector backend shutdown failed")
                    if failure is None:
                        self.failed.emit(
                            "The scenario ended, but the backend could not shut down cleanly: "
                            f"{error}"
                        )
            self.finished.emit()

    def _run_scenario(
        self,
        runner: RuntimeInspectorRunner,
    ) -> bool:
        last_signature: object | None = None

        frame = runner.frame()
        self.frame_ready.emit(frame)
        last_signature = runtime_frame_signature(frame)

        wall_started = time.monotonic()
        simulated_started = runner.session.world.time_s
        publish_interval_s = 1.0 / self._config.publish_hz
        next_publish_at = wall_started + publish_interval_s

        self.playback_changed.emit(False)
        for _ in range(runner.step_count):
            if self._paused.is_set():
                pause_started = time.monotonic()
                self.playback_changed.emit(True)
                while self._paused.is_set() and not self._interrupted():
                    self._interruption.wait(_INTERRUPTION_POLL_S)
                wall_started += time.monotonic() - pause_started
                if not self._interrupted():
                    self.playback_changed.emit(False)
            if self._interrupted():
                break
            runner.step()
            if runner.finished:
                break

            simulated_elapsed = runner.session.world.time_s - simulated_started
            target_wall_time = wall_started + min(simulated_elapsed, self._config.duration_s)
            if not self._wait_until(target_wall_time):
                break

            now = time.monotonic()
            if now >= next_publish_at:
                frame = runner.frame()
                self.frame_ready.emit(frame)
                last_signature = runtime_frame_signature(frame)
                while next_publish_at <= now:
                    next_publish_at += publish_interval_s

        if self._interrupted():
            runner.stop()
        elif not runner.finished and self._config.demo is RuntimeDemo.SMORES_SPATIAL_HANDOFF:
            # Allow the feedback controller to mark a reached time budget.
            runner.step()
        final_frame = runner.frame()
        if runtime_frame_signature(final_frame) != last_signature:
            self.frame_ready.emit(final_frame)
        return self._interrupted()

    def _wait_until(self, target_s: float) -> bool:
        while not self._interrupted():
            remaining_s = target_s - time.monotonic()
            if remaining_s <= 0.0:
                return True
            self._interruption.wait(min(remaining_s, _INTERRUPTION_POLL_S))
        return False

    def _interrupted(self) -> bool:
        thread = QThread.currentThread()
        return self._interruption.is_set() or thread.isInterruptionRequested()


def _failure_message(error: Exception) -> str:
    message = str(error).strip() or type(error).__name__
    return f"Runtime Inspector could not start or continue: {message}"


__all__ = [
    "RuntimeInspectorConfig",
    "RuntimeInspectorSetupError",
    "RuntimeInspectorWorker",
]
