"""Regression coverage for the twelve-module M-Blocks benchmark plan."""

from __future__ import annotations

from dataclasses import replace
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import cast

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
from modsim.runtime.reconfiguration import ReconfigurationPhase, ReconfigurationPlan
from modsim.runtime.session import RuntimeSession

_ROOT = Path(__file__).resolve().parents[1]
_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"
_SCENARIO_PATH = _ROOT / "examples" / "scenarios" / "mblocks_twelve_module_line.py"


def _example() -> tuple[ReconfigurationPlan, tuple[KinematicPivotRoute, ...]]:
    spec = spec_from_file_location("test_mblocks_twelve_module_line", _SCENARIO_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    plan = module.build_plan()
    routes = module.build_routes()
    assert isinstance(plan, ReconfigurationPlan)
    assert isinstance(routes, tuple)
    route_values = cast(tuple[object, ...], routes)
    assert all(isinstance(route, KinematicPivotRoute) for route in route_values)
    return plan, cast(tuple[KinematicPivotRoute, ...], route_values)


def test_twelve_module_plan_has_connected_structure_and_line_goal() -> None:
    plan, routes = _example()

    assert len(plan.module_ids) == 12
    assert len(plan.initial_connections) == 11
    assert len(plan.actions) == 11
    assert len(routes) == 11
    assert all(route.angle_rad == pytest.approx(1.5707963267948966) for route in routes[:-1])
    assert routes[-1].angle_rad == pytest.approx(3.141592653589793)
    final = plan.actions[-1].dock
    assert final is not None
    assert final.connection_id == connection_id(
        connector_instance_id(ModuleInstanceId("block_11"), "pos_x"),
        connector_instance_id(ModuleInstanceId("block_12"), "neg_x"),
    )


def test_twelve_module_reference_plan_reaches_one_lattice_line() -> None:
    plan, routes = _example()
    loaded = RobotPackLoader().load(_PACK_PATH)
    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=module_id,
            module_type_id="mblocks_3d",
            pose=Transform.from_translation((index * 0.12, 0.0, 0.0)),
        )
        for index, module_id in enumerate(plan.module_ids)
    )
    session = RuntimeSession.create(loaded, scene, MockBackendAdapter())
    scenario = KinematicPivotScenario.create(
        session,
        plan,
        # Four quarter-turns return to the same moving face. Keep that cycle
        # longer than the pack's one-second redock cooldown while still making
        # the regression much faster than the presentation demo.
        tuple(replace(route, duration_s=0.26) for route in routes),
        KinematicPivotConfig(dt_s=0.01, initial_hold_s=0.0, connected_hold_s=0.0),
    )

    events: list[Event] = []
    for _ in range(500):
        events.extend(scenario.step())
        if scenario.status.phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            break

    assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
    assert len([event for event in events if isinstance(event, UndockCommitted)]) == 11
    assert len([event for event in events if isinstance(event, DockCommitted)]) == 11
    assert len(session.world.connections) == 11
    assert session.world.assemblies.count == 1
    cells = {
        tuple(round(component / 0.05) for component in instance.pose.translation)
        for instance in session.world.modules.values()
    }
    assert cells == {(index, 0, 0) for index in range(12)}
