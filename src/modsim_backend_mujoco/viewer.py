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
) -> tuple[Event, ...]:
    """Run a session to ``duration_s`` with the passive viewer open.

    ``step_once`` performs one scripted step and returns the events it produced,
    so the scenario logic stays with its caller and this function only handles
    rendering and wall-clock pacing.

    When ``hold`` is set the window stays open after the scenario finishes,
    because the final configuration is usually the thing worth looking at and a
    window that vanishes on the last step gives you no chance to.
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

    with handle as viewer:
        # URDF collision proxies stay active in physics but start hidden so
        # detailed visual meshes are not covered by opaque boxes. The native
        # viewer's geom-group controls can re-enable group 3 for debugging.
        with viewer.lock():
            viewer.opt.geomgroup[URDF_COLLISION_GEOM_GROUP] = 0
        wall_start = time.perf_counter()
        while viewer.is_running() and session.world.time_s < duration_s:
            collected.extend(step_once())
            viewer.sync()
            # Pace to wall clock so the run is watchable rather than a flash.
            ahead = session.world.time_s - (time.perf_counter() - wall_start)
            if ahead > 0.0:
                time.sleep(ahead)

        while hold and viewer.is_running():
            viewer.sync()
            time.sleep(IDLE_REFRESH_S)
    return tuple(collected)
