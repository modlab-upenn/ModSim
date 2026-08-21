"""Real-time visualisation of a running session.

The viewer lives in the backend package rather than in the CLI because it is a
MuJoCo facility: it needs the compiled model and live data directly. Keeping it
here is what lets ``modsim.cli`` stay free of engine imports.
"""

from __future__ import annotations

import time
from collections.abc import Callable

import mujoco.viewer

from modsim.backends.base import BackendError
from modsim.core.events import Event
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
from modsim_backend_mujoco.scene import URDF_COLLISION_GEOM_GROUP

Stepper = Callable[[], tuple[Event, ...]]
ViewerCallback = Callable[[], None]
StopPredicate = Callable[[], bool]


IDLE_REFRESH_S = 1.0 / 60.0

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
    hold: bool = True,
    stop_requested: StopPredicate | None = None,
    on_started: ViewerCallback | None = None,
    after_step: ViewerCallback | None = None,
    on_scenario_complete: ViewerCallback | None = None,
    on_stopped: ViewerCallback | None = None,
) -> tuple[Event, ...]:
    """Run a session to ``duration_s`` with the passive viewer open.

    ``step_once`` performs one scripted step and returns the events it produced,
    so the scenario logic stays with its caller and this function only handles
    rendering and wall-clock pacing.

    When ``hold`` is set the window stays open after the scenario finishes,
    because the final configuration is usually the thing worth looking at and a
    window that vanishes on the last step gives you no chance to.

    The optional lifecycle hooks support clients that mirror the same runtime
    into another process. ``stop_requested`` is polled while stepping, pacing,
    and holding the final state. ``on_started`` runs after the passive viewer is
    configured, ``after_step`` runs after each viewer synchronization, and
    ``on_scenario_complete`` runs once when the requested simulated duration is
    reached. ``on_stopped`` runs exactly once immediately before the viewer
    context is closed. Existing callers need none of these hooks.
    """
    adapter = session.adapter
    if not isinstance(adapter, MuJoCoBackendAdapter):
        raise BackendError("the viewer requires the MuJoCo backend")

    collected: list[Event] = []
    try:
        handle = mujoco.viewer.launch_passive(adapter.model, adapter.data)
    except RuntimeError as error:
        if "mjpython" in str(error):
            raise BackendError(MACOS_HINT) from error
        raise

    should_stop = stop_requested if stop_requested is not None else _never_stop
    with handle as viewer:
        try:
            # URDF collision proxies stay active in physics but start hidden so
            # detailed visual meshes are not covered by opaque boxes. The native
            # viewer's geom-group controls can re-enable group 3 for debugging.
            with viewer.lock():
                viewer.opt.geomgroup[URDF_COLLISION_GEOM_GROUP] = 0
            if on_started is not None:
                on_started()

            wall_start = time.perf_counter()
            simulated_start = session.world.time_s
            while (
                viewer.is_running()
                and not should_stop()
                and session.world.time_s - simulated_start < duration_s
            ):
                collected.extend(step_once())
                viewer.sync()
                if after_step is not None:
                    after_step()
                # Pace to wall clock so the run is watchable rather than a flash.
                simulated_elapsed = session.world.time_s - simulated_start
                target_wall_time = wall_start + min(simulated_elapsed, duration_s)
                if not _wait_until(target_wall_time, should_stop):
                    break

            if (
                session.world.time_s - simulated_start >= duration_s
                and on_scenario_complete is not None
            ):
                on_scenario_complete()

            while hold and viewer.is_running() and not should_stop():
                viewer.sync()
                if not _wait_until(time.perf_counter() + IDLE_REFRESH_S, should_stop):
                    break
        finally:
            if on_stopped is not None:
                on_stopped()
    return tuple(collected)


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
