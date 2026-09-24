"""Physical handoff, effort bounds, and explicit experiment limitations."""

from __future__ import annotations

import json
from contextlib import nullcontext
from pathlib import Path
from threading import Event as ThreadEvent
from typing import Any

import pytest

pytest.importorskip("mujoco", reason="requires the MuJoCo extra")
pytest.importorskip("numpy")

import mujoco
import numpy as np
from typer.testing import CliRunner

from modsim.cli import app
from modsim.planning.spatial import Bond, SpatialPlanningError
from modsim.runtime import RuntimeDemo, RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim.runtime.reconfiguration import ReconfigurationPhase
from modsim.runtime.spatial import SpatialReconfigurationScenario
from modsim_backend_mujoco.scene import MuJoCoSceneError, PositionServo
from modsim_backend_mujoco.spatial_experiment import CAPTURE, GOAL, SpatialExperiment
from modsim_backend_mujoco.spatial_viewer import decorate_scene, run_spatial_viewer

pytestmark = pytest.mark.mujoco
PACK = Path(__file__).parents[1] / "examples/robot_packs/smores_ep"


def test_real_handoff_never_uses_root_control(monkeypatch: pytest.MonkeyPatch) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=PACK,
            demo=RuntimeDemo.SMORES_SPATIAL_HANDOFF,
            duration_s=45,
        )
    )
    assert isinstance(runner.scenario, SpatialReconfigurationScenario)
    experiment = runner.scenario.motion
    assert isinstance(experiment, SpatialExperiment)
    try:
        initial = runner.frame()
        assert len(initial.view.nodes) == 5
        assert len(initial.view.edges) == 3

        def forbidden(*args: object, **kwargs: object) -> None:
            raise AssertionError("runtime must use bounded joint actuation only")

        for name in (
            "set_module_pose",
            "set_module_twist",
            "apply_module_wrench",
            "_snap_to_nominal",
        ):
            monkeypatch.setattr(type(experiment.adapter), name, forbidden)
        monkeypatch.setattr(experiment, "_place", forbidden)
        while not runner.finished:
            runner.step()
        assert experiment.phase == "complete", experiment.report()
        assert experiment.peak_effort_nm <= experiment.torque_limit_nm + 1e-9
        assert experiment.target_error_m() < 0.004
        assert experiment.peak_penetration_m < 0.004
        assert np.count_nonzero(experiment.data.qfrc_applied) == 0
        assert np.count_nonzero(experiment.data.xfrc_applied) == 0
        bonds = frozenset(
            Bond(str(c.connector_a), str(c.connector_b))
            for c in experiment.session.world.connections.values()
        )
        assert bonds == GOAL
        assert CAPTURE in bonds
        assert [h["phase"] for h in experiment.history] == [
            "moving",
            "transfer",
            "retreat",
            "verify",
            "complete",
        ]
        assert [h["connections"] for h in experiment.history] == [3, 4, 3, 3, 3]
        # CAD root origins are offset from the module centers. The midpoint of
        # the two wheel connector sites identifies the physical stacking axis.
        points = np.array(
            [
                (
                    experiment.data.site(f"{m}/connector/left").xpos
                    + experiment.data.site(f"{m}/connector/right").xpos
                )
                / 2
                for m in ("receiver", "arm", "upper", "payload")
            ]
        )
        assert np.ptp(points[:, :2], axis=0).max() < 0.004
        assert np.all(np.diff(points[:, 2]) > 0.08)
        assert points[-1, 2] > 0.32
        assert experiment.data.site("upper/connector/pan").xpos[2] > 0.28
        time = experiment.data.time
        experiment.step()
        assert experiment.data.time == time
        final = runner.frame()
        assert final.planning is not None
        assert all(a.phase == "complete" for a in final.planning.actions)
        assert len(final.planning.target_bonds) == 3
        assert len(final.planning.targets) == 5
        assert len(final.planning.decisions) >= 26
        assert final.scenario is not None
        assert final.scenario.phase is ReconfigurationPhase.COMPLETE
        assert len(final.view.edges) == 3
        assert [row.sequence for frame in (initial, final) for row in frame.events] == list(
            range(len(experiment.session.world.event_log))
        )
        assert [row.kind for row in final.events if row.kind != "DockCandidateDetected"] == [
            "DockCommitted",
            "AssemblyMerged",
            "UndockCommitted",
            "AssemblySplit",
        ]
        message = RuntimeFrame(frame=final)
        assert decode_runtime_message(encode_runtime_message(message)) == message
    finally:
        runner.shutdown()


def test_modsim_cli_reports_timeout_as_failure() -> None:
    result = CliRunner().invoke(
        app,
        [
            "run",
            str(PACK),
            "--demo",
            "smores_spatial_handoff",
            "--duration",
            "0.01",
            "--output",
            "json",
        ],
    )
    assert result.exit_code == 1, result.output
    report = json.loads(result.output)
    assert report["phase"] == "timeout"
    assert report["time_s"] == pytest.approx(0.01)
    assert [action["operation"] for action in report["topology_plan"]] == ["dock", "release"]


def test_planning_does_not_write_live_state_and_stop_freezes() -> None:
    experiment = SpatialExperiment(PACK)
    try:
        initial = experiment.data.qpos.copy()
        experiment._feasible((3.0, 6.0, 0.0))
        assert np.array_equal(initial, experiment.data.qpos)
        assert experiment.path.rejected > 0
        # Withdrawal must move the receiving arm before lowering the helper.
        assert experiment.return_path.points[1] == (6.0, 5.0, 6.0)
        assert experiment.return_path.points[-1] == (0.0, 0.0, 6.0)
        assert experiment.target_positions["payload"][2] > 0.32
        experiment.stop()
        experiment.step()
        assert experiment.phase == "stopped"
        assert experiment.data.time == 0
    finally:
        experiment.close()


def test_insufficient_torque_is_refused_before_execution() -> None:
    with pytest.raises(SpatialPlanningError, match="feasibility"):
        SpatialExperiment(PACK, torque_limit_nm=0.1)


def test_viewer_overlay_and_pause_stop_lifecycle(monkeypatch: pytest.MonkeyPatch) -> None:
    experiment = SpatialExperiment(PACK)
    try:
        scene = mujoco.MjvScene(experiment.model, maxgeom=128)
        decorate_scene(scene, experiment)
        # Three measured bonds, ten fixture strokes, and nonzero approach edges.
        # No extra target boxes or duplicated target bonds in the native scene.
        trace_segments = sum(
            np.linalg.norm(np.array(a) - np.array(b)) > 1e-8
            for a, b in zip(experiment.planned_trace, experiment.planned_trace[1:], strict=False)
        )
        assert scene.ngeom == 3 + 10 + trace_segments
        assert all(np.all(np.isfinite(g.pos)) for g in scene.geoms[: scene.ngeom])
        original_colors = experiment.model.geom_rgba.copy()
        original_materials = experiment.model.geom_matid.copy()
        samples: list[float] = []
        texts: list[object] = []

        class Viewer:
            cam = mujoco.MjvCamera()
            opt = mujoco.MjvOption()
            user_scn = scene

            def __enter__(self) -> Viewer:
                return self

            def __exit__(self, *args: object) -> None:
                return None

            def lock(self) -> Any:
                return nullcontext()

            def is_running(self) -> bool:
                return len(samples) < 3

            def set_texts(self, value: object) -> None:
                texts.append(value)

            def sync(self) -> None:
                samples.append(float(experiment.data.time))
                callback(32 if len(samples) == 1 else ord("S"))

        callback: Any = None

        def launch(*args: object, **kwargs: Any) -> Viewer:
            nonlocal callback
            callback = kwargs["key_callback"]
            return Viewer()

        monkeypatch.setattr("mujoco.viewer.launch_passive", launch)
        monkeypatch.setattr("modsim_backend_mujoco.spatial_viewer.time.sleep", lambda _: None)
        callbacks: list[str] = []
        paused = ThreadEvent()
        playback: list[bool] = []
        run_spatial_viewer(
            experiment,
            pause_requested=paused.is_set,
            set_paused=lambda value: paused.set() if value else paused.clear(),
            on_playback_changed=playback.append,
            on_started=lambda: callbacks.append("started"),
            on_scenario_complete=lambda: callbacks.append(experiment.phase),
            on_stopped=lambda: callbacks.append("closed"),
        )
        assert len(samples) == 3
        assert samples[0] > 0 and samples[0] == samples[1] == samples[2]
        assert experiment.phase == "stopped"
        assert "STOPPED" in str(texts[-1])
        assert callbacks == ["started", "stopped", "closed"]
        assert playback == [False, True]
        assert paused.is_set()
        assert np.array_equal(experiment.model.geom_rgba, original_colors)
        assert np.array_equal(experiment.model.geom_matid, original_materials)
        assert "TARGET wireframe" not in str(texts[-1])
        samples.clear()
        callbacks.clear()
        run_spatial_viewer(
            experiment,
            stop_requested=lambda: True,
            on_stopped=lambda: callbacks.append(experiment.phase),
        )
        assert samples == []
        assert callbacks == ["stopped"]
    finally:
        experiment.close()


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_servo_parameters_are_finite_positive(value: float) -> None:
    with pytest.raises(MuJoCoSceneError):
        PositionServo(45, 0.2, value)
