"""Backend-neutral kinematic pivot scenario tests."""

from __future__ import annotations

import math
from dataclasses import replace
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest

from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import DockCommitted, Event, UndockCommitted
from modsim.core.ids import ModuleInstanceId, connection_id, connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.kinematic_pivot import (
    KinematicPivotConfig,
    KinematicPivotRoute,
    KinematicPivotScenario,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationAction,
    ReconfigurationPhase,
    ReconfigurationPlan,
    ReconfigurationScenarioError,
    stage_docking_assembly_pair,
)
from modsim.runtime.session import RuntimeSession

_ROOT = Path(__file__).resolve().parents[1]
_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"
_SCENARIO_PATH = _ROOT / "examples" / "scenarios" / "mblocks_five_module_pivot.py"


def _example() -> tuple[ReconfigurationPlan, tuple[KinematicPivotRoute, ...]]:
    spec = spec_from_file_location("test_mblocks_five_module_pivot", _SCENARIO_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    plan = module.build_plan()
    routes = module.build_routes()
    assert isinstance(plan, ReconfigurationPlan)
    assert all(isinstance(route, KinematicPivotRoute) for route in routes)
    return plan, routes


def _session(plan: ReconfigurationPlan) -> tuple[RuntimeSession, MockBackendAdapter]:
    loaded = RobotPackLoader().load(_PACK_PATH)
    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=module_id,
            module_type_id="mblocks_3d",
            pose=Transform.from_translation((index * 0.12, 0.0, 0.0)),
        )
        for index, module_id in enumerate(plan.module_ids)
    )
    adapter = MockBackendAdapter()
    return RuntimeSession.create(loaded, scene, adapter), adapter


def _pair(
    fixed_module: str,
    fixed_face: str,
    moving_module: str,
    moving_face: str,
) -> ConnectorPairRef:
    return ConnectorPairRef(
        connector_instance_id(ModuleInstanceId(fixed_module), fixed_face),
        connector_instance_id(ModuleInstanceId(moving_module), moving_face),
    )


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"pivot_axis": (0.0, 0.0, 0.0)}, "pivot_axis must not be"),
        ({"angle_rad": 0.0}, "angle_rad must be non-zero"),
        ({"duration_s": 0.0}, "duration_s must be greater than zero"),
        ({"pivot_point_m": (math.nan, 0.0, 0.0)}, r"pivot_point_m\[0\] must"),
    ),
)
def test_kinematic_pivot_route_rejects_invalid_geometry(
    overrides: dict[str, object],
    message: str,
) -> None:
    values: dict[str, object] = {
        "moving_module": ModuleInstanceId("moving"),
        "reference_module": ModuleInstanceId("fixed"),
        "pivot_point_m": (0.025, 0.0, 0.025),
        "pivot_axis": (0.0, 1.0, 0.0),
        "angle_rad": math.pi / 2.0,
        "duration_s": 1.0,
    }

    with pytest.raises(ValueError, match=message):
        KinematicPivotRoute(**(values | overrides))  # type: ignore[arg-type]


def test_pivot_validation_precedes_initial_topology_staging() -> None:
    plan, routes = _example()
    session, adapter = _session(plan)

    with pytest.raises(ReconfigurationScenarioError, match=r"3 action.*2 route"):
        KinematicPivotScenario.create(session, plan, routes[:-1])

    assert session.world.connections == {}
    assert adapter.welds == ()
    assert session.world.time_s == 0.0


def test_session_step_can_defer_passive_capture_during_authored_transit() -> None:
    plan, _ = _example()
    session, _ = _session(plan)
    fixed = connector_instance_id(ModuleInstanceId("block_1"), "pos_x")
    moving = connector_instance_id(ModuleInstanceId("block_2"), "neg_x")
    stage_docking_assembly_pair(session, fixed, moving, gap_m=0.0)

    transit_events = session.step(0.01, process_connectors=False)

    assert transit_events == ()
    assert session.world.connections == {}
    capture_events = session.step(0.01)
    assert any(isinstance(event, DockCommitted) for event in capture_events)
    assert len(session.world.connections) == 1


def test_five_module_pivot_uses_rigid_arcs_and_canonical_lifecycle() -> None:
    plan, routes = _example()
    quick_routes = tuple(replace(route, duration_s=0.08) for route in routes)
    session, adapter = _session(plan)
    scenario = KinematicPivotScenario.create(
        session,
        plan,
        quick_routes,
        KinematicPivotConfig(
            dt_s=0.01,
            initial_hold_s=0.0,
            connected_hold_s=0.01,
        ),
    )

    assert scenario.status.phase is ReconfigurationPhase.UNDOCKING
    assert len(session.world.connections) == 4
    assert session.world.assemblies.count == 1
    assert all(
        session.world.modules[ModuleInstanceId(f"block_{index}")].pose.rotation
        == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-12)
        for index in range(1, 6)
    )

    release_events = scenario.step()
    assert any(isinstance(event, UndockCommitted) for event in release_events)
    assert scenario.status.phase is ReconfigurationPhase.PIVOTING
    assert len(session.world.connections) == 3
    assert session.world.assemblies.count == 2

    route = quick_routes[0]
    assert route.reference_module is not None
    reference = session.world.modules[route.reference_module].pose
    start_relative = session.world.modules[route.moving_module].pose.relative_to(reference)
    expected = reference.compose(
        Transform.from_translation(route.pivot_point_m)
        .compose(
            Transform(
                rotation=(
                    math.cos(math.pi / 32.0),
                    0.0,
                    math.sin(math.pi / 32.0),
                    0.0,
                )
            )
        )
        .compose(
            Transform.from_translation(
                (
                    -route.pivot_point_m[0],
                    -route.pivot_point_m[1],
                    -route.pivot_point_m[2],
                )
            )
        )
        .compose(start_relative)
    )
    scenario.step()
    assert session.world.modules[route.moving_module].pose.is_close(
        expected,
        position_tolerance_m=1e-10,
        orientation_tolerance_rad=1e-10,
    )

    phases = {scenario.status.phase}
    runtime_events: list[Event] = list(release_events)
    for _ in range(200):
        runtime_events.extend(scenario.step())
        phases.add(scenario.status.phase)
        if scenario.status.phase in (
            ReconfigurationPhase.COMPLETE,
            ReconfigurationPhase.FAILED,
        ):
            break

    assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
    assert ReconfigurationPhase.PIVOTING in phases
    assert len([event for event in runtime_events if isinstance(event, UndockCommitted)]) == 3
    assert len([event for event in runtime_events if isinstance(event, DockCommitted)]) == 3
    assert len(session.world.connections) == 4
    assert session.world.assemblies.count == 1
    assert len(adapter.welds) == 4
    assert (
        connection_id(
            connector_instance_id(ModuleInstanceId("block_4"), "pos_z"),
            connector_instance_id(ModuleInstanceId("block_5"), "neg_x"),
        )
        in session.world.connections
    )
    assert session.metrics().docking_success_count == 7
    assert session.metrics().undocking_success_count == 3
    assert session.world.modules[ModuleInstanceId("block_5")].pose.translation == pytest.approx(
        (0.15, 0.0, 0.05), abs=1e-12
    )


def test_pivot_moves_a_detached_multi_module_component_rigidly() -> None:
    plan = ReconfigurationPlan(
        id="rigid_component_pivot",
        name="Rigid component pivot",
        module_ids=tuple(ModuleInstanceId(f"block_{index}") for index in range(1, 4)),
        initial_connections=(
            _pair("block_1", "pos_z", "block_2", "neg_z"),
            _pair("block_2", "pos_y", "block_3", "neg_y"),
        ),
        actions=(
            ReconfigurationAction(
                label="Pivot two-module branch",
                undock=_pair("block_1", "pos_z", "block_2", "neg_z"),
                dock=_pair("block_1", "pos_x", "block_2", "neg_z"),
            ),
        ),
    )
    route = KinematicPivotRoute(
        moving_module=ModuleInstanceId("block_2"),
        reference_module=ModuleInstanceId("block_1"),
        pivot_point_m=(0.025, 0.0, 0.025),
        pivot_axis=(0.0, 1.0, 0.0),
        angle_rad=math.pi / 2.0,
        duration_s=0.08,
    )
    session, _ = _session(plan)
    scenario = KinematicPivotScenario.create(
        session,
        plan,
        (route,),
        KinematicPivotConfig(dt_s=0.01, initial_hold_s=0.0, connected_hold_s=0.0),
    )

    scenario.step()
    assert scenario.status.phase is ReconfigurationPhase.PIVOTING
    before = session.world.modules[ModuleInstanceId("block_3")].pose.relative_to(
        session.world.modules[ModuleInstanceId("block_2")].pose
    )
    scenario.step()
    after = session.world.modules[ModuleInstanceId("block_3")].pose.relative_to(
        session.world.modules[ModuleInstanceId("block_2")].pose
    )

    assert after.is_close(
        before,
        position_tolerance_m=1e-10,
        orientation_tolerance_rad=1e-10,
    )
