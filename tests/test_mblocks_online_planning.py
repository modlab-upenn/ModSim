# pyright: reportPrivateUsage=false
"""Measured online execution, including a ban on runtime root control."""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim.backends.base import BackendError
from modsim.model_views import CubicLatticeView
from modsim.planning.mblocks import apply_pivot
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.demos import RuntimeDemo
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim.runtime.inspector_runner import RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim.runtime.mblocks_lattice import face_pairs, observe_lattice, tabletop_scene
from modsim.runtime.mblocks_online_planning import OnlineLatticeScenario
from modsim.runtime.reconfiguration import ReconfigurationPhase
from modsim.runtime.session import RuntimeSession

_PACK = Path(__file__).resolve().parents[1] / "examples/robot_packs/mblocks_3d"


def _runner(demo: RuntimeDemo = RuntimeDemo.MBLOCKS_ONLINE_LATTICE) -> RuntimeInspectorRunner:
    return RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=_PACK,
            demo=demo,
            dt_s=0.0005,
            duration_s=60.0,
            gravity=True,
            ground=True,
            height_m=0.025,
        )
    )


@pytest.mark.mujoco
@pytest.mark.parametrize(
    ("demo", "modules", "actions"),
    (
        (RuntimeDemo.MBLOCKS_ONLINE_LATTICE, 4, 8),
        (RuntimeDemo.MBLOCKS_ONLINE_LATTICE_LARGE, 6, 21),
    ),
)
def test_online_line_reaches_measured_goal_without_root_control(
    monkeypatch: pytest.MonkeyPatch,
    demo: RuntimeDemo,
    modules: int,
    actions: int,
) -> None:
    pytest.importorskip("mujoco")
    runner = _runner(demo)
    try:
        scenario = runner.scenario
        assert isinstance(scenario, OnlineLatticeScenario)
        assert len(runner.session.world.modules) == modules
        assert len(scenario.plan.actions) == actions
        assert len({a.moving for a in scenario.plan.actions}) == modules - 1
        adapter_type = type(runner.session.adapter)

        def reject_root_control(*_args: object, **_kwargs: object) -> None:
            raise AssertionError("online planner wrote a root pose, twist, or wrench")

        for method in ("set_module_pose", "set_module_twist", "apply_module_wrench"):
            monkeypatch.setattr(adapter_type, method, reject_root_control)
        peak_landing_effort = 0.0
        for tick in range(120000):
            scenario.step()
            peak_landing_effort = max(peak_landing_effort, abs(scenario._landing_effort_nm))
            if tick % 500 == 0:
                frame = runner.frame()
                assert isinstance(frame.view, CubicLatticeView)
                assert frame.lattice_planning is not None and frame.planning is None
                assert decode_runtime_message(
                    encode_runtime_message(RuntimeFrame(frame=frame))
                ) == RuntimeFrame(frame=frame)
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break
        assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
        assert scenario.planning_snapshot.completed_actions == actions
        assert 0 < peak_landing_effort <= 0.03
        observed, residual = observe_lattice(runner.session, scenario.frame_transform, 0.05)
        assert observed.cells == frozenset(scenario.plan.goal.cells)
        assert residual <= 0.003
        assert set(runner.session.world.connections) == {
            p.connection_id for p in face_pairs(observed, scenario.frame_transform, 0.05)
        }
        assert len(runner.session.world.connections) == modules - 1  # Face tree, no hinge.
        assert scenario._landing_effort_nm == 0.0
    finally:
        runner.shutdown()


@pytest.mark.mujoco
def test_failed_landing_is_terminal_and_keeps_diagnostics(monkeypatch: pytest.MonkeyPatch) -> None:
    pytest.importorskip("mujoco")

    def reject_capture(_self: OnlineLatticeScenario) -> bool:
        return False

    monkeypatch.setattr(OnlineLatticeScenario, "_capture_ready", reject_capture)
    runner = _runner()
    try:
        scenario = runner.scenario
        assert isinstance(scenario, OnlineLatticeScenario)
        for _ in range(12000):
            scenario.step()
            if scenario.status.phase is ReconfigurationPhase.FAILED:
                break
        assert scenario.status.phase is ReconfigurationPhase.FAILED
        snapshot = scenario.planning_snapshot
        assert snapshot.completed_actions == 0
        assert snapshot.active is not None
        assert snapshot.landing_effort_nm == 0.0
        assert "timed out" in snapshot.detail
        assert snapshot.decisions[-1].kind == "failed"
        assert sum(record.phase == "failed" for record in snapshot.history) == 1
        scenario.step()
        assert scenario.status.phase is ReconfigurationPhase.FAILED
        assert scenario.planning_snapshot.decisions == snapshot.decisions
    finally:
        runner.shutdown()


@pytest.mark.mujoco
def test_constraint_solver_option_rejects_invalid_values() -> None:
    pytest.importorskip("mujoco")
    from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter

    for value in (0.0, -1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="constraint_time_constant"):
            MuJoCoBackendAdapter(constraint_time_constant_s=value)
    with pytest.raises(BackendError, match="twice the timestep"):
        RuntimeSession.create(
            RobotPackLoader().load(_PACK),
            tabletop_scene(),
            "mujoco",
            timestep_s=0.0005,
            constraint_time_constant_s=0.0009,
        )


@pytest.mark.mujoco
def test_recovery_search_yields_physics_without_committing_planned_topology() -> None:
    pytest.importorskip("mujoco")
    runner = _runner()
    try:
        scenario = runner.scenario
        assert isinstance(scenario, OnlineLatticeScenario)
        baseline = set(runner.session.world.connections)
        scenario._start_recovery(scenario.plan.initial)
        for _ in range(210):
            scenario.step()
            assert set(runner.session.world.connections) == baseline
            if scenario._search is None:
                break
        assert scenario._search is None
        assert scenario.status.phase is not ReconfigurationPhase.FAILED, scenario.status.detail
        assert scenario.planning_snapshot.replans == 1
        assert runner.session.world.time_s > 0.0
        state = scenario.plan.initial
        for action in scenario.planning_snapshot.remaining:
            state = apply_pivot(state, action)
        assert state.cells == frozenset(scenario.plan.goal.cells)
        assert scenario.planning_snapshot.decisions[-1].kind == "replanned"
    finally:
        runner.shutdown()
