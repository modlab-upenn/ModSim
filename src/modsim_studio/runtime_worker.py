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

from modsim.runtime.inspector_runner import (
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
    RuntimeInspectorSetupError,
    runtime_frame_signature,
)
from modsim.runtime.pacing import wall_clock_deadline_s

_LOGGER = logging.getLogger("modsim.runtime_inspector")
_INTERRUPTION_POLL_S = 0.01


class RuntimeInspectorWorker(QObject):
    """Load and execute one demonstration on its owning Qt thread."""

    frame_ready = Signal(object)
    status_changed = Signal(str)
    playback_changed = Signal(bool)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, config: RuntimeInspectorConfig) -> None:
        super().__init__()
        self._config = config
        self._interruption = ThreadEvent()
        self._pause_requested = ThreadEvent()

    def request_interruption(self) -> None:
        """Request a prompt, cooperative stop from any thread."""
        self._interruption.set()

    def set_paused(self, paused: bool) -> None:
        """Request a pause state from another thread at the next step boundary."""
        if paused:
            self._pause_requested.set()
        else:
            self._pause_requested.clear()

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
                self.status_changed.emit("Run stopped")
                return
            interrupted = self._run_scenario(runner)
            phase = runner.scenario.status.phase.value
            self.status_changed.emit(
                "Run stopped"
                if interrupted
                else "Simulation complete"
                if phase == "complete"
                else "Simulation failed"
                if phase == "failed"
                else "Time limit reached"
            )
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
        self.playback_changed.emit(False)

        publish_interval_s = 1.0 / self._config.publish_hz
        pacing_wall_started = time.monotonic()
        pacing_simulated_started = runner.session.world.time_s
        next_publish_at = pacing_wall_started + publish_interval_s
        completed_steps = 0
        was_paused = False

        while completed_steps < runner.step_count and not runner.execution_finished:
            if self._interrupted():
                break
            if self._pause_requested.is_set():
                if not was_paused:
                    paused_frame = runner.frame()
                    self.frame_ready.emit(paused_frame)
                    last_signature = runtime_frame_signature(paused_frame)
                    self.playback_changed.emit(True)
                    was_paused = True
                self._interruption.wait(_INTERRUPTION_POLL_S)
                continue
            if was_paused:
                self.playback_changed.emit(False)
                was_paused = False
                pacing_wall_started = time.monotonic()
                pacing_simulated_started = runner.session.world.time_s
                next_publish_at = pacing_wall_started + publish_interval_s

            runner.step()
            completed_steps += 1

            simulated_elapsed = runner.session.world.time_s - pacing_simulated_started
            target_wall_time = wall_clock_deadline_s(
                pacing_wall_started,
                min(simulated_elapsed, self._config.duration_s),
                self._config.real_time_factor,
            )
            if not self._wait_until(target_wall_time):
                if self._interrupted():
                    break
                # A pause can arrive while pacing the step that just
                # completed. Return to the top of the loop so that pause is
                # acknowledged without advancing the runtime again.
                continue

            now = time.monotonic()
            if now >= next_publish_at:
                frame = runner.frame()
                self.frame_ready.emit(frame)
                last_signature = runtime_frame_signature(frame)
                while next_publish_at <= now:
                    next_publish_at += publish_interval_s

        if self._interrupted():
            runner.stop()
        final_frame = runner.frame()
        if runtime_frame_signature(final_frame) != last_signature:
            self.frame_ready.emit(final_frame)
        return self._interrupted()

    def _wait_until(self, target_s: float) -> bool:
        while not self._interrupted():
            if self._pause_requested.is_set():
                return False
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
