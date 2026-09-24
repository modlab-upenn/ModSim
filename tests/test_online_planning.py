"""Physical online execution and immutable planner transport regression tests."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

pytest.importorskip("mujoco")

from modsim.backends.base import BackendAdapter, ConnectionOutcome, ConnectionRequest
from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, quat_from_rpy
from modsim.model_views import ModelViewFactory
from modsim.planning import AssemblyGoal, GoalEdge
from modsim.robot_packs import RobotPackLoader
from modsim.runtime import ReconfigurationPhase, RuntimeSession, inspector_runner
from modsim.runtime.demos import RuntimeDemo
from modsim.runtime.inspection import build_runtime_inspector_frame
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim.runtime.inspector_runner import (
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
    resolve_runtime_recipe,
)
from modsim.runtime.online_planning import OnlineAssemblyScenario

PACK = Path(__file__).resolve().parents[1] / "examples/robot_packs/smores_ep"


@pytest.mark.mujoco
@pytest.mark.parametrize(
    ("parallel", "refuse_first"), ((False, False), (True, False), (False, True))
)
def test_online_routes_commit_real_docks_without_root_control(
    monkeypatch: pytest.MonkeyPatch, parallel: bool, refuse_first: bool
) -> None:
    loaded = RobotPackLoader().load(PACK)
    placements = [("module_a", 0.0, 0.0, 0.0)]
    if parallel:
        placements += [
            ("module_b", 0.034458, 0.3, math.pi / 2),
            ("module_c", 0.034458, -0.3, -math.pi / 2),
        ]
        edges = (
            GoalEdge(a="a", face_a="left", b="b", face_b="bottom"),
            GoalEdge(a="a", face_a="right", b="c", face_b="bottom"),
        )
    else:
        placements += [("module_b", -0.3, 0.0, 0.0)]
        edges = (GoalEdge(a="a", face_a="bottom", b="b", face_b="pan"),)
    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=ModuleInstanceId(m),
            module_type_id="smores_ep",
            pose=Transform(translation=(x, y, 0.05), rotation=quat_from_rpy((0.0, 0.0, yaw))),
        )
        for m, x, y, yaw in placements
    )
    session = RuntimeSession.create(loaded, scene, "mujoco", gravity=(0.0, 0.0, -9.81), ground=True)
    try:
        config = inspector_runner._load_smores_physical_reconfiguration_config(  # pyright: ignore[reportPrivateUsage]
            RuntimeInspectorConfig(pack_path=PACK, dt_s=0.002)
        )
        goal = AssemblyGoal(
            id="online_test", nodes=("a", "b", "c") if parallel else ("a", "b"), edges=edges
        )
        scenario = OnlineAssemblyScenario.create(session, goal, config)
        if refuse_first:
            original_create = session.adapter.create_physical_connection
            refused = False

            def refuse_once(
                _adapter: BackendAdapter, request: ConnectionRequest
            ) -> ConnectionOutcome:
                nonlocal refused
                if not refused:
                    refused = True
                    return ConnectionOutcome.refused("Injected first-capture rejection")
                return original_create(request)

            monkeypatch.setattr(type(session.adapter), "create_physical_connection", refuse_once)

        def forbidden(*args: object, **kwargs: object) -> None:
            raise AssertionError("online execution changed root state")

        for method in ("set_module_pose", "set_module_twist", "apply_module_wrench"):
            monkeypatch.setattr(type(session.adapter), method, forbidden)
        peak_active = 0
        for _ in range(30000):
            before = session.world.time_s
            scenario.step()
            assert session.world.time_s - before == pytest.approx(0.002)
            peak_active = max(
                peak_active,
                sum(
                    e.phase in {"navigating", "aligning", "approaching"}
                    for e in scenario.executions
                ),
            )
            assert scenario.status.phase is not ReconfigurationPhase.FAILED, scenario.status.detail
            if scenario.status.phase is ReconfigurationPhase.COMPLETE:
                break
        assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.planning_snapshot
        assert len(session.world.connections) == len(edges)
        assert peak_active == (2 if parallel else 1)
        snapshot = scenario.planning_snapshot
        frame = build_runtime_inspector_frame(
            session,
            resolve_runtime_recipe(loaded.pack, None),
            ModelViewFactory(),
            event_cursor=0,
            scenario_status=scenario.status,
            planning=snapshot,
        )
        assert decode_runtime_message(
            encode_runtime_message(RuntimeFrame(frame=frame))
        ) == RuntimeFrame(frame=frame)
        assert snapshot.sample_sequence == frame.view.source.sample_sequence
        with pytest.raises(ValueError, match="share a world sample"):
            frame.model_validate(
                {
                    **dict(frame),
                    "planning": snapshot.model_copy(
                        update={"sample_sequence": snapshot.sample_sequence + 1}
                    ),
                }
            )
        assert any(d.kind == "route" for d in snapshot.decisions)
        assert any(d.kind == "dock" for d in snapshot.decisions)
        assert all(a.intervals for a in snapshot.actions)
        if refuse_first:
            assert snapshot.replans >= 1
            assert any(i.phase == "retreating" for a in snapshot.actions for i in a.intervals)
    finally:
        session.shutdown()


@pytest.mark.mujoco
def test_online_reconfiguration_stops_when_a_preserved_bond_is_lost() -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=PACK,
            demo=RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE,
            backend="mujoco",
            gravity=True,
            ground=True,
            height_m=0.05,
            dt_s=0.002,
        )
    )
    try:
        assert isinstance(runner.scenario, OnlineAssemblyScenario)
        assert len(runner.scenario.plan.releases) == 4
        connection = next(
            c
            for c in runner.session.world.connections.values()
            if {str(c.connector_a), str(c.connector_b)} == {"module_2/bottom", "module_3/pan"}
        )
        runner.session.request_undock(connection.id)
        runner.session.process_docking()
        runner.step()
        assert runner.scenario.status.phase is ReconfigurationPhase.FAILED
        assert "preserved was lost" in runner.scenario.status.detail
        assert runner.session.world.time_s == pytest.approx(0.002)
    finally:
        runner.shutdown()
