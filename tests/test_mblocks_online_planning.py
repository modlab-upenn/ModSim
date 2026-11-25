# pyright: reportPrivateUsage=false
"""Measured online execution, including a ban on runtime root control."""

from __future__ import annotations

import math
import re
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
from modsim.runtime.mblocks_online_planning import (
    CAPTURE_YAW_GATE_RAD,
    FINE_RECOVERY_RPM,
    STALL_ERROR_RAD,
    OnlineLatticeScenario,
)
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
        # Landing feedback is bounded by the 0.03 N m motor cap; a stalled-landing
        # recovery additionally brakes the flywheel, bounded by the 2.6 N m brake.
        cap = 0.03 if scenario.recovery_impulses == 0 else 2.6
        assert 0 < peak_landing_effort <= cap
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


# The placement jitter numpy.random.default_rng(2).normal(0.0, 2e-6, 2) draws for the six
# cubes, in placement order. Frozen here so the core CI job, which has no numpy, can import
# this module; the values reproduce the seeded rebound exactly.
_SEED_2_JITTER_M = (
    (3.781067635870661e-07, -1.0454968829614947e-06),
    (-8.261270867837868e-07, -4.882934765279711e-06),
    (3.599414765441804e-06, 2.2883317440744573e-06),
    (-6.508456737356487e-07, 1.5476131734553228e-06),
    (5.624213395952985e-07, -1.1076456728481047e-06),
    (1.9551349022520714e-06, -6.211130933183051e-07),
)


def _run_shifted_six_cube(jitter: bool) -> tuple[OnlineLatticeScenario, list[str]]:
    """Run the six-cube line moved by (0.123, 0.456) m; return the scenario and decisions."""
    pytest.importorskip("mujoco")
    from modsim.core.scene import ModulePlacement, SceneSpec
    from modsim.core.transforms import Transform
    from modsim.runtime.mblocks_lattice import larger_state, line_goal

    initial = larger_state()
    goal = line_goal(initial)
    shift = Transform.from_translation((0.123, 0.456, 0.0))
    offsets = iter(_SEED_2_JITTER_M)

    def offset() -> Transform:
        if not jitter:
            return Transform()
        dx, dy = next(offsets)
        return Transform.from_translation((dx, dy, 0.0))

    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=p.instance_id,
            module_type_id=p.module_type_id,
            pose=offset().compose(shift.compose(p.pose)),
        )
        for p in tabletop_scene(initial, pitch_m=goal.pitch_m).placements
    )
    session = RuntimeSession.create(
        RobotPackLoader().load(_PACK),
        scene,
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        timestep_s=0.0005,
        weld_pool_size=18,
        hinge_pool_size=2,
        constraint_time_constant_s=0.002,
    )
    try:
        scenario = OnlineLatticeScenario.create(session, goal, initial=initial, dt_s=0.0005)
        while session.world.time_s < 60.0 and scenario.status.phase not in (
            ReconfigurationPhase.COMPLETE,
            ReconfigurationPhase.FAILED,
        ):
            scenario.step()
        details = [d.detail for d in scenario.planning_snapshot.decisions if d.kind == "recovery"]
        return scenario, details
    finally:
        session.shutdown()


def test_stalled_landing_recovers_instead_of_winding_up_the_flywheel() -> None:
    """Regression: moving the six-cube start by (0.123, 0.456) m made pivot 19 rebound
    off its target face and stall about 12 degrees short. Landing feedback then wound
    the flywheel up to the 20,000 RPM ceiling. A recovery impulse must finish it.

    Which scene stalls depends on step timing and floating-point details. Under
    split stepping with a one-step connection delay, the plain shifted scene no
    longer stalls this way; a 2 micrometre jitter of each placement (seed 2)
    reproduces a rebound that a recovery impulse resolves.
    """
    scenario, recoveries = _run_shifted_six_cube(jitter=True)
    assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
    assert scenario.planning_snapshot.completed_actions == 21
    # The stall itself depends on floating-point details of the platform; when it
    # occurs it must be visible in the decision log and resolved by an impulse.
    assert len(recoveries) == scenario.recovery_impulses


def test_cube_resting_just_outside_the_capture_gate_is_nudged_in() -> None:
    """Regression: under split stepping the plain shifted scene left a slab cube at
    rest 0.76 degrees from its target yaw, just outside the 0.75 degree capture
    gate. The landing law's torque there is below breakaway friction and the
    coarse 2.5 degree stall threshold never tripped, so the pivot timed out. A
    fine impulse must bring such a cube into the gate.
    """
    scenario, recoveries = _run_shifted_six_cube(jitter=False)
    assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
    assert scenario.planning_snapshot.completed_actions == 21
    assert len(recoveries) == scenario.recovery_impulses
    for detail in recoveries:
        match = re.search(r"stalled ([+-]?[\d.]+) degrees", detail)
        assert match is not None, detail
        degrees = abs(float(match.group(1)))
        # Fine nudges cover the band between the capture gate and the coarse threshold.
        assert degrees > math.degrees(CAPTURE_YAW_GATE_RAD) - 0.05
        if degrees <= math.degrees(STALL_ERROR_RAD):
            assert f"{FINE_RECOVERY_RPM:.0f} RPM" in detail
