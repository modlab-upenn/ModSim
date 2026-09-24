"""Native visualization for the fixture-supported spatial planning experiment."""

from __future__ import annotations

import math
import time
from collections.abc import Callable

import mujoco
import mujoco.viewer
import numpy as np

from modsim.runtime.spatial import SpatialReconfigurationScenario
from modsim_backend_mujoco.spatial_experiment import ANCHORS, MODULES, SpatialExperiment


def configure_lighting(experiment: SpatialExperiment) -> None:
    """Light the imported CAD while preserving its original materials."""
    model = experiment.model
    model.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
    model.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]


def decorate_scene(
    scene: mujoco.MjvScene, experiment: SpatialExperiment, *, reset: bool = True
) -> None:
    """Draw measured bonds, fixture markers, and the approach path.

    Target geometry lives in the Runtime Inspector, not as extra native bodies.
    """
    if reset:
        scene.ngeom = 0

    def line(a: object, b: object, color: tuple[float, ...], width: float = 0.0015) -> None:
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).ravel(),
            np.array(color),
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.array(a), np.array(b))
        scene.ngeom += 1

    current = {m: experiment.data.body(f"{m}/base_link").xpos.copy() for m in MODULES}
    for connection in experiment.session.world.connections.values():
        a = str(connection.connector_a).split("/")[0]
        b = str(connection.connector_b).split("/")[0]
        line(current[a], current[b], (0.0, 0.9, 0.95, 1.0), 0.002)
    for a, b in zip(experiment.planned_trace, experiment.planned_trace[1:], strict=False):
        if np.linalg.norm(np.array(a) - np.array(b)) > 1e-8:
            line(a, b, (1.0, 1.0, 1.0, 1.0), 0.0007)
    for anchor in ANCHORS:
        center = np.array(experiment.fixture_poses[anchor].translation)
        center[2] = 0.007
        corners = [
            center + np.array([x, y, 0])
            for x, y in ((-0.045, -0.045), (0.045, -0.045), (0.045, 0.045), (-0.045, 0.045))
        ]
        for i in range(4):
            line(corners[i], corners[(i + 1) % 4], (1.0, 0.8, 0.05, 1.0), 0.003)
        line(center, current[anchor], (1.0, 0.8, 0.05, 1.0), 0.002)


def run_spatial_viewer(
    experiment: SpatialExperiment,
    *,
    speed: float = 0.5,
    hold: bool = True,
    stop_requested: Callable[[], bool] | None = None,
    on_started: Callable[[], None] | None = None,
    after_step: Callable[[], None] | None = None,
    on_scenario_complete: Callable[[], None] | None = None,
    on_stopped: Callable[[], None] | None = None,
    pause_requested: Callable[[], bool] | None = None,
    set_paused: Callable[[bool], None] | None = None,
    on_playback_changed: Callable[[bool], None] | None = None,
) -> None:
    """Run at bounded playback speed; retain terminal state until window closes."""
    if not math.isfinite(speed) or not 0 < speed <= 10:
        raise ValueError("playback speed must be finite, positive, and at most 10")
    paused = False
    key_stop = False
    completion_sent = False
    step_credit = 0.0
    published_pause: bool | None = None

    def keypress(key: int) -> None:
        nonlocal paused, key_stop
        if key == 32:
            paused = not paused
            if set_paused is not None:
                set_paused(paused)
        elif key in (ord("S"), ord("s")):
            key_stop = True

    try:
        configure_lighting(experiment)
        with mujoco.viewer.launch_passive(
            experiment.model, experiment.data, key_callback=keypress
        ) as viewer:
            with viewer.lock():
                positions = np.array(
                    [experiment.data.body(f"{m}/base_link").xpos.copy() for m in MODULES]
                    + list(experiment.target_positions.values())
                )
                viewer.cam.lookat[:] = (positions.min(axis=0) + positions.max(axis=0)) / 2
                viewer.cam.distance = 0.95
                viewer.cam.azimuth = 125
                viewer.cam.elevation = -28
                viewer.opt.geomgroup[3] = 0
            if on_started is not None:
                on_started()
            while viewer.is_running():
                frame_start = time.monotonic()
                if stop_requested is not None and stop_requested():
                    break
                if pause_requested is not None:
                    paused = pause_requested()
                if paused != published_pause:
                    published_pause = paused
                    if on_playback_changed is not None:
                        on_playback_changed(paused)
                if key_stop:
                    experiment.stop()
                if not paused and not experiment.terminal:
                    step_credit += speed / 60
                    steps = int(step_credit / experiment.dt_s)
                    step_credit -= steps * experiment.dt_s
                    for _ in range(steps):
                        experiment.step()
                        if after_step is not None:
                            after_step()
                        if experiment.terminal:
                            break
                with viewer.lock():
                    if viewer.user_scn is not None:
                        decorate_scene(viewer.user_scn, experiment)
                if hasattr(viewer, "set_texts"):
                    status = (
                        "PAUSED" if paused and not experiment.terminal else experiment.phase.upper()
                    )
                    motion = f"waypoint {experiment.waypoint}/{len(experiment.path.points) - 1}"
                    if experiment.phase in {"retreat", "verify", "complete"}:
                        motion = (
                            f"withdrawal {experiment.return_waypoint}/"
                            f"{len(experiment.return_path.points) - 1}"
                        )
                    effort = (
                        f"{experiment.peak_effort_nm:.2f} / {experiment.torque_limit_nm:.2f} Nm"
                    )
                    viewer.set_texts(
                        [
                            (
                                mujoco.mjtFontScale.mjFONTSCALE_100,
                                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                "SMORES vertical chain\nStatus\nTime / motion\n"
                                "Final-position error\n"
                                "Peak effort\nAssumptions\nControls\nLegend\n\n",
                                "4-module tower + separate helper\n"
                                f"{status}\n{experiment.data.time:.1f} s / {motion}\n"
                                f"{1000 * experiment.target_error_m():.1f} mm\n"
                                f"{effort}\n"
                                "2 fixed bases; gravity; ideal measured welds\n"
                                "Space: pause/resume | S: stop | close: exit\n"
                                "Cyan: bonds / Gold pads: fixtures / White: planned path\n"
                                "Target and planner: Runtime Inspector\n"
                                f"{experiment.detail}",
                            )
                        ]
                    )
                viewer.sync()
                if experiment.terminal and not completion_sent:
                    completion_sent = True
                    if on_scenario_complete is not None:
                        on_scenario_complete()
                if experiment.terminal and not hold:
                    break
                time.sleep(max(0.0, 1 / 60 - (time.monotonic() - frame_start)))
            experiment.stop()
    finally:
        experiment.stop()
        if on_stopped is not None:
            on_stopped()


def view_spatial_scenario(scenario: SpatialReconfigurationScenario) -> None:
    """Display a ModSim-owned scenario with its matching mechanical services."""
    if not isinstance(scenario.motion, SpatialExperiment):
        raise TypeError("the native spatial viewer requires MuJoCo mechanical services")
    run_spatial_viewer(scenario.motion)
