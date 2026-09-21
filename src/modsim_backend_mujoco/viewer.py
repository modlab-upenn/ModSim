"""Real-time visualisation of a running session.

The viewer lives in the backend package rather than in the CLI because it is a
MuJoCo facility: it needs the compiled model and live data directly. Keeping it
here is what lets ``modsim.cli`` stay free of engine imports.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from threading import enumerate as enumerate_threads
from typing import Any

import mujoco.viewer

from modsim.backends.base import BackendError
from modsim.core.events import Event
from modsim.core.validation import require_finite_positive
from modsim.runtime.pacing import wall_clock_deadline_s
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
from modsim_backend_mujoco.scene import URDF_COLLISION_GEOM_GROUP

Stepper = Callable[[], tuple[Event, ...]]
ViewerCallback = Callable[[], None]
StopPredicate = Callable[[], bool]
PausePredicate = Callable[[], bool]
PauseCallback = Callable[[bool], None]
KeyCallback = Callable[[int], None]


IDLE_REFRESH_S = 1.0 / 60.0
VIEWER_REFRESH_S = 1.0 / 30.0
_VIEWER_CLOSE_TIMEOUT_S = 3.0

MACOS_HINT = (
    "MuJoCo's passive viewer must own the main thread on macOS, so the script has to "
    "run under 'mjpython' rather than 'python'. The MuJoCo wheel installs it alongside "
    "python in your environment:\n"
    "    mjpython -m modsim run ... --view"
)


def run_with_viewer(
    session: RuntimeSession,
    *,
    duration_s: float,
    step_once: Stepper,
    real_time_factor: float = 1.0,
    hold: bool = True,
    stop_requested: StopPredicate | None = None,
    pause_requested: PausePredicate | None = None,
    execution_finished: StopPredicate | None = None,
    key_callback: KeyCallback | None = None,
    on_started: ViewerCallback | None = None,
    after_step: ViewerCallback | None = None,
    on_scenario_complete: ViewerCallback | None = None,
    on_pause_changed: PauseCallback | None = None,
    on_stopped: ViewerCallback | None = None,
) -> tuple[Event, ...]:
    """Run a session to ``duration_s`` with the passive viewer open.

    ``step_once`` performs one scripted step and returns the events it produced,
    so the scenario logic stays with its caller and this function only handles
    rendering and wall-clock pacing. ``real_time_factor`` changes only that
    pacing: a factor of two requests two simulated seconds per wall-clock
    second without changing the physics step, controller timing, or actuator
    limits.

    When ``hold`` is set the window stays open after the scenario finishes,
    because the final configuration is usually the thing worth looking at and a
    window that vanishes on the last step gives you no chance to.

    The optional lifecycle hooks support clients that mirror the same runtime
    into another process. ``stop_requested`` is polled while stepping, pacing,
    and holding the final state. ``pause_requested`` is polled by the same
    authoritative loop; while true the viewer remains responsive but neither
    scenario logic nor physics advances. A caller can update that state from
    the public passive-viewer ``key_callback`` without touching MuJoCo data.
    ``on_started`` runs after the passive viewer is configured, ``after_step``
    runs after each viewer synchronization, and ``on_scenario_complete`` runs
    once when the requested simulated duration is reached or the optional
    ``execution_finished`` predicate reports a terminal scenario result. In
    either case physics freezes while the final-view hold remains interactive.
    ``on_pause_changed`` reports actual playback transitions and ``on_stopped``
    runs exactly once immediately before the viewer context is closed. Existing
    callers need none of these hooks.
    """
    require_finite_positive(real_time_factor, "real_time_factor")
    adapter = session.adapter
    if not isinstance(adapter, MuJoCoBackendAdapter):
        raise BackendError("the viewer requires the MuJoCo backend")

    collected: list[Event] = []
    should_stop = stop_requested if stop_requested is not None else _never_stop
    should_pause = pause_requested if pause_requested is not None else _never_pause
    is_finished = execution_finished if execution_finished is not None else _never_stop
    playback_controls_enabled = (
        pause_requested is not None or key_callback is not None or on_pause_changed is not None
    )
    with _managed_passive_viewer(adapter, key_callback) as viewer:
        try:
            # URDF collision proxies stay active in physics but start hidden so
            # detailed visual meshes are not covered by opaque boxes. The native
            # viewer's geom-group controls can re-enable group 3 for debugging.
            with viewer.lock():
                viewer.opt.geomgroup[URDF_COLLISION_GEOM_GROUP] = 0
            if playback_controls_enabled:
                _set_playback_overlay(
                    viewer,
                    paused=False,
                    space_action=key_callback is not None,
                )
            if on_started is not None:
                on_started()
            if on_pause_changed is not None:
                # Report the initial authoritative playback state as an
                # acknowledgement, even before any pause request arrives.
                on_pause_changed(False)

            pacing_wall_start = time.perf_counter()
            simulated_start = session.world.time_s
            pacing_simulated_start = simulated_start
            next_viewer_sync_s = pacing_wall_start
            last_step_was_synced = False
            paused = False
            while (
                viewer.is_running()
                and not should_stop()
                and session.world.time_s - simulated_start < duration_s
                and not is_finished()
            ):
                requested_pause = should_pause()
                if requested_pause:
                    if not paused:
                        paused = True
                        _set_playback_overlay(
                            viewer,
                            paused=True,
                            space_action=key_callback is not None,
                        )
                        if on_pause_changed is not None:
                            on_pause_changed(True)
                    # ``sync`` keeps camera interaction, window close, and the
                    # public key callback live even though ModSim owns and has
                    # deliberately suspended every physics/runtime step.
                    viewer.sync()
                    last_step_was_synced = True
                    _wait_until(
                        time.perf_counter() + IDLE_REFRESH_S,
                        lambda: should_stop() or not should_pause() or not viewer.is_running(),
                    )
                    continue

                if paused:
                    paused = False
                    # A pause consumes wall time but no simulated time. Start a
                    # fresh pacing epoch so resume does not race to catch up to
                    # deadlines that elapsed while playback was suspended.
                    pacing_wall_start = time.perf_counter()
                    pacing_simulated_start = session.world.time_s
                    next_viewer_sync_s = pacing_wall_start
                    _set_playback_overlay(
                        viewer,
                        paused=False,
                        space_action=key_callback is not None,
                    )
                    if on_pause_changed is not None:
                        on_pause_changed(False)

                collected.extend(step_once())
                now = time.perf_counter()
                last_step_was_synced = now >= next_viewer_sync_s
                if last_step_was_synced:
                    viewer.sync()
                    if after_step is not None:
                        after_step()
                    # Schedule from completion, not from the pre-sync time.
                    # A costly render must not leave the deadline overdue and
                    # cause another sync after the very next physics step.
                    next_viewer_sync_s = time.perf_counter() + VIEWER_REFRESH_S
                # Pace to wall clock so the run is watchable rather than a flash.
                simulated_elapsed = session.world.time_s - pacing_simulated_start
                duration_remaining_at_epoch = max(
                    0.0,
                    duration_s - (pacing_simulated_start - simulated_start),
                )
                target_wall_time = wall_clock_deadline_s(
                    pacing_wall_start,
                    min(simulated_elapsed, duration_remaining_at_epoch),
                    real_time_factor,
                )
                if not _wait_until(
                    target_wall_time,
                    lambda: should_stop() or should_pause() or not viewer.is_running(),
                ):
                    if should_stop() or not viewer.is_running():
                        break
                    # A pause arrived during pacing. Re-enter the loop so it is
                    # acknowledged without advancing another runtime step.
                    continue

            # A short or faster-than-real-time run can finish between display
            # refreshes. Push its final state before entering the hold loop or
            # closing the viewer without coupling every physics step to rendering.
            finished = session.world.time_s - simulated_start >= duration_s or is_finished()
            if finished and not last_step_was_synced:
                viewer.sync()
                if after_step is not None:
                    after_step()

            if finished:
                if hold and playback_controls_enabled:
                    _set_finished_overlay(viewer)
                if on_scenario_complete is not None:
                    on_scenario_complete()

            while hold and viewer.is_running() and not should_stop():
                viewer.sync()
                if not _wait_until(time.perf_counter() + IDLE_REFRESH_S, should_stop):
                    break
        finally:
            if on_stopped is not None:
                on_stopped()
    return tuple(collected)


@contextmanager
def _managed_passive_viewer(
    adapter: MuJoCoBackendAdapter, key_callback: KeyCallback | None
) -> Iterator[Any]:
    """Close and join the passive viewer before backend/interpreter teardown.

    MuJoCo's handle.close() only requests exit. On Linux/Windows the render
    thread is a daemon; allowing Python's glfw.terminate atexit hook to race
    that thread can segfault even after a successful Stop. The public handle
    exposes no join operation, so retain the Python threads created during
    this launch, while excluding pre-existing runtime/control threads. ModSim
    launches its one viewer from the sole simulation owner. On macOS mjpython
    owns the existing UI thread, which must not be joined here.
    """
    existing_threads = set(enumerate_threads())
    try:
        if key_callback is None:
            handle = mujoco.viewer.launch_passive(adapter.model, adapter.data)
        else:
            handle = mujoco.viewer.launch_passive(
                adapter.model, adapter.data, key_callback=key_callback
            )
    except RuntimeError as error:
        if "mjpython" in str(error):
            raise BackendError(MACOS_HINT) from error
        raise
    viewer_threads = tuple(
        thread for thread in enumerate_threads() if thread not in existing_threads
    )
    try:
        with handle as viewer:
            yield viewer
    finally:
        deadline = time.monotonic() + _VIEWER_CLOSE_TIMEOUT_S
        for thread in viewer_threads:
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if any(thread.is_alive() for thread in viewer_threads):
            raise BackendError("MuJoCo viewer did not finish shutting down within 3 seconds")


def _wait_until(target_s: float, stop_requested: StopPredicate) -> bool:
    """Wait until ``target_s`` while keeping cooperative stop latency bounded."""
    while not stop_requested():
        remaining_s = target_s - time.perf_counter()
        if remaining_s <= 0.0:
            return True
        time.sleep(min(remaining_s, IDLE_REFRESH_S))
    return False


def _never_stop() -> bool:
    return False


def _never_pause() -> bool:
    return False


def _set_playback_overlay(
    viewer: Any,
    *,
    paused: bool,
    space_action: bool,
) -> None:
    """Show playback ownership without depending on private viewer state."""
    set_texts = getattr(viewer, "set_texts", None)
    if not callable(set_texts):
        return

    state = "PAUSED" if paused else "RUNNING"
    action = f"\nSpace: {'Resume' if paused else 'Pause'}" if space_action else ""
    # Handle.set_texts coordinates with the render thread through its own
    # request queue. Holding the general viewer lock here can block that queue
    # while replacing an overlay that has not yet been consumed.
    set_texts(
        (
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOPRIGHT,
            "ModSim playback",
            f"{state}{action}",
        )
    )


def _set_finished_overlay(viewer: Any) -> None:
    set_texts = getattr(viewer, "set_texts", None)
    if callable(set_texts):
        set_texts(
            (
                mujoco.mjtFontScale.mjFONTSCALE_150,
                mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                "ModSim playback",
                "FINISHED · final state frozen\nSee Inspector for the result",
            )
        )
