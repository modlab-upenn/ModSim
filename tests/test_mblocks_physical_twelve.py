"""Physical regression coverage for the twelve-module M-Blocks sequence."""

from __future__ import annotations

import math
from dataclasses import replace
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import Any

import pytest

from modsim.core.events import DockCommitted, UndockCommitted
from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.model_views import CubicLatticeView, ModelViewContext, ModelViewFactory
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.momentum_sequence import (
    MomentumPivotSequencePlan,
    MomentumPivotSequenceScenario,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationPhase,
    ReconfigurationPlan,
)
from modsim.runtime.session import RuntimeSession

_ROOT = Path(__file__).resolve().parents[1]
_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"
_SCENARIO_PATH = _ROOT / "examples" / "scenarios" / "mblocks_twelve_module_physics.py"


def _example_module() -> Any:
    spec = spec_from_file_location("test_mblocks_twelve_module_physics", _SCENARIO_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _three_module_first_traverse(
    full_plan: MomentumPivotSequencePlan,
    full_scene: SceneSpec,
) -> tuple[MomentumPivotSequencePlan, SceneSpec]:
    selected = {
        ModuleInstanceId("block_1"),
        ModuleInstanceId("block_2"),
        ModuleInstanceId("block_12"),
    }
    reconfiguration = ReconfigurationPlan(
        id=full_plan.reconfiguration.id,
        name=full_plan.reconfiguration.name,
        module_ids=tuple(
            module for module in full_plan.reconfiguration.module_ids if module in selected
        ),
        initial_connections=(
            full_plan.reconfiguration.initial_connections[0],
            full_plan.reconfiguration.initial_connections[-1],
        ),
        actions=(full_plan.reconfiguration.actions[0],),
        source_url=full_plan.reconfiguration.source_url,
    )
    scene = SceneSpec.of(
        placement for placement in full_scene.placements if placement.instance_id in selected
    )
    return MomentumPivotSequencePlan(reconfiguration, (full_plan.pivots[0],)), scene


def test_physical_plan_encodes_ten_quarter_turns_and_one_half_turn() -> None:
    module = _example_module()
    plan = module.build_plan()

    assert isinstance(plan, MomentumPivotSequencePlan)
    assert len(plan.reconfiguration.module_ids) == 12
    assert len(plan.reconfiguration.initial_connections) == 11
    assert len(plan.pivots) == 11
    assert [pivot.initial_face.moving_connector.rsplit("/", 1)[-1] for pivot in plan.pivots] == [
        "neg_z",
        "pos_x",
        "pos_z",
        "neg_x",
        "neg_z",
        "pos_x",
        "pos_z",
        "neg_x",
        "neg_z",
        "pos_x",
        "pos_z",
    ]
    assert [pivot.edge_hinge.moving_connector.rsplit("/", 1)[-1] for pivot in plan.pivots] == [
        "edge_pos_x_neg_z",
        "edge_pos_x_pos_z_from_pos_x",
        "edge_neg_x_pos_z",
        "edge_neg_x_neg_z_from_neg_x",
        "edge_pos_x_neg_z",
        "edge_pos_x_pos_z_from_pos_x",
        "edge_neg_x_pos_z",
        "edge_neg_x_neg_z_from_neg_x",
        "edge_pos_x_neg_z",
        "edge_pos_x_pos_z_from_pos_x",
        "edge_neg_x_pos_z",
    ]
    assert all(
        pivot.target_pivot_angle_rad == pytest.approx(math.pi / 2.0) for pivot in plan.pivots[:-1]
    )
    assert plan.pivots[-1].target_pivot_angle_rad == pytest.approx(math.pi)


def test_physical_sequence_rejects_a_reversed_directed_action_pair() -> None:
    module = _example_module()
    plan = module.build_plan()
    first_action = plan.reconfiguration.actions[0]
    assert first_action.undock is not None
    reversed_undock = ConnectorPairRef(
        fixed_connector=first_action.undock.moving_connector,
        moving_connector=first_action.undock.fixed_connector,
    )
    actions = (
        replace(first_action, undock=reversed_undock),
        *plan.reconfiguration.actions[1:],
    )

    with pytest.raises(ValueError, match="directed undock"):
        MomentumPivotSequencePlan(
            replace(plan.reconfiguration, actions=actions),
            plan.pivots,
        )


def test_physical_sequence_rejects_pairs_whose_modules_are_absent() -> None:
    module = _example_module()
    plan = module.build_plan()

    with pytest.raises(ValueError, match="absent from the plan"):
        MomentumPivotSequencePlan(
            replace(
                plan.reconfiguration,
                module_ids=plan.reconfiguration.module_ids[:-1],
            ),
            plan.pivots,
        )


@pytest.mark.mujoco
def test_real_mujoco_completes_first_three_module_surface_traverse(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    module = _example_module()
    full_plan = module.build_plan(dt_s=0.00025)
    plan, scene = _three_module_first_traverse(full_plan, module.build_scene())
    loaded = RobotPackLoader().load(_PACK_PATH)
    runtime = RuntimeSession.create(
        loaded,
        scene,
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        ground_height_m=0.0,
        timestep_s=0.00025,
        hinge_pool_size=2,
        weld_pool_size=4,
    )
    try:
        scenario = MomentumPivotSequenceScenario.create(runtime, plan)
        adapter_type = type(runtime.adapter)

        def reject_runtime_root_control(*_args: object, **_kwargs: object) -> None:
            raise AssertionError("physical sequence wrote a root pose, twist, or wrench")

        monkeypatch.setattr(adapter_type, "set_module_pose", reject_runtime_root_control)
        monkeypatch.setattr(adapter_type, "set_module_twist", reject_runtime_root_control)
        monkeypatch.setattr(adapter_type, "apply_module_wrench", reject_runtime_root_control)

        for _ in range(math.ceil(5.0 / 0.00025)):
            scenario.step()
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break

        assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
        first_dock = plan.reconfiguration.actions[0].dock
        assert first_dock is not None
        expected = {
            plan.reconfiguration.initial_connections[0].connection_id,
            first_dock.connection_id,
        }
        assert set(runtime.world.connections) == expected
        assert runtime.world.assemblies.count == 1
        commits = [event for event in runtime.world.event_log if isinstance(event, DockCommitted)]
        releases = [
            event for event in runtime.world.event_log if isinstance(event, UndockCommitted)
        ]
        assert len(commits) == 4
        assert len(releases) == 2

        recipe = next(
            item
            for item in loaded.pack.manifest.model_views
            if item.id == "mblocks_physics_lattice"
        )
        view = ModelViewFactory().build(
            recipe,
            ModelViewContext(loaded.pack, runtime.world),
        )
        assert isinstance(view, CubicLatticeView)
        assert view.position_tolerance_m == pytest.approx(0.003)
        nodes = {node.id: node for node in view.nodes}
        assert nodes["block_1"].cell == (0, 0, 0)
        assert nodes["block_2"].cell == (1, 0, 0)
        assert nodes["block_12"].cell == (1, 0, 1)
        assert all(not node.off_lattice for node in nodes.values()), [
            (
                node.id,
                node.pose_residual.position_m,
                node.pose_residual.orientation_rad,
            )
            for node in nodes.values()
        ]
    finally:
        runtime.shutdown()


@pytest.mark.mujoco
def test_real_mujoco_completes_all_eleven_physical_pivots(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    module = _example_module()
    plan = module.build_plan(dt_s=0.0005)
    loaded = RobotPackLoader().load(_PACK_PATH)
    runtime = RuntimeSession.create(
        loaded,
        module.build_scene(),
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        ground_height_m=0.0,
        timestep_s=0.0005,
        hinge_pool_size=2,
        weld_pool_size=12,
    )
    try:
        scenario = MomentumPivotSequenceScenario.create(runtime, plan)
        adapter_type = type(runtime.adapter)

        def reject_runtime_root_control(*_args: object, **_kwargs: object) -> None:
            raise AssertionError("physical sequence wrote a root pose, twist, or wrench")

        monkeypatch.setattr(adapter_type, "set_module_pose", reject_runtime_root_control)
        monkeypatch.setattr(adapter_type, "set_module_twist", reject_runtime_root_control)
        monkeypatch.setattr(adapter_type, "apply_module_wrench", reject_runtime_root_control)

        for _ in range(math.ceil(12.0 / 0.0005)):
            scenario.step()
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break

        assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
        expected = {pair.connection_id for pair in plan.reconfiguration.initial_connections[:-1]}
        final_dock = plan.reconfiguration.actions[-1].dock
        assert final_dock is not None
        expected.add(final_dock.connection_id)
        assert set(runtime.world.connections) == expected
        assert runtime.world.assemblies.count == 1
        assert len(scenario.completed_telemetry) == 11
        for pivot, telemetry in zip(
            plan.pivots,
            scenario.completed_telemetry,
            strict=True,
        ):
            assert telemetry.capture_time_s is not None
            assert abs(abs(telemetry.pivot_angle_rad) - pivot.target_pivot_angle_rad) <= (
                pivot.capture_angle_tolerance_rad
            )
            if pivot.target_pivot_angle_rad < math.pi:
                assert telemetry.pivot_angle_rad > 0.0
        commits = [event for event in runtime.world.event_log if isinstance(event, DockCommitted)]
        releases = [
            event for event in runtime.world.event_log if isinstance(event, UndockCommitted)
        ]
        assert len(commits) == 33
        assert len(releases) == 22

        recipe = next(
            item
            for item in loaded.pack.manifest.model_views
            if item.id == "mblocks_physics_lattice"
        )
        view = ModelViewFactory().build(
            recipe,
            ModelViewContext(loaded.pack, runtime.world),
        )
        assert isinstance(view, CubicLatticeView)
        nodes = {node.id: node for node in view.nodes}
        assert {module_id: node.cell for module_id, node in nodes.items()} == {
            f"block_{index}": (index - 1, 0, 0) for index in range(1, 13)
        }
        assert all(not node.off_lattice for node in nodes.values()), [
            (
                node.id,
                node.pose_residual.position_m,
                node.pose_residual.orientation_rad,
            )
            for node in nodes.values()
        ]
    finally:
        runtime.shutdown()
