"""Reference and MuJoCo regressions for the M-Blocks staircase demo."""

from __future__ import annotations

import math
import sys
from dataclasses import replace
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
from typer.testing import CliRunner

from modsim.backends.mock import MockBackendAdapter
from modsim.cli import app
from modsim.core.events import AssemblyMerged, AssemblySplit, DockCommitted, UndockCommitted
from modsim.model_views import CubicLatticeView, ModelViewContext, ModelViewFactory
from modsim.robot_packs import RobotPackLoader
from modsim.runtime import (
    CoordinatedKinematicPivotScenario,
    CoordinatedMomentumPivotScenario,
    CoordinatedPivotPlan,
    ReconfigurationPhase,
    RuntimeDemo,
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
    RuntimeInspectorSetupError,
)
from modsim.runtime.inspector_runner import validate_runtime_inspector_config
from modsim.runtime.momentum_pivot import MAX_MOMENTUM_TIMESTEP_S
from modsim.runtime.session import RuntimeSession

_ROOT = Path(__file__).resolve().parents[1]
_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"
_SCENARIO_PATH = _ROOT / "examples" / "scenarios" / "mblocks_twelve_module_staircase.py"


def _example_module() -> Any:
    spec = spec_from_file_location("test_mblocks_twelve_module_staircase", _SCENARIO_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _lattice_view(loaded: Any, runtime: RuntimeSession) -> CubicLatticeView:
    recipe = next(
        item for item in loaded.pack.manifest.model_views if item.id == "mblocks_physics_lattice"
    )
    view = ModelViewFactory().build(recipe, ModelViewContext(loaded.pack, runtime.world))
    assert isinstance(view, CubicLatticeView)
    return view


def test_plan_encodes_cyclic_mat_and_three_step_staircase() -> None:
    module = _example_module()
    plan = module.build_plan()

    assert isinstance(plan, CoordinatedPivotPlan)
    assert plan.id == "mblocks_twelve_module_staircase"
    assert len(plan.module_ids) == 12
    assert len(plan.initial_connections) == 16
    assert len(plan.actions) == 11
    assert len(plan.final_connections) == 18
    assert [action.moving_modules for action in plan.actions] == [
        module.SLABS["A"],
        module.SLABS["A"],
        module.SLABS["A"],
        module.SLABS["A"],
        module.SLABS["A"],
        module.SLABS["B"],
        module.SLABS["B"],
        module.SLABS["B"],
        module.SLABS["C"],
        module.SLABS["C"],
        module.SLABS["C"],
    ]
    assert [len(action.release_faces) for action in plan.actions] == [
        2,
        2,
        2,
        2,
        2,
        2,
        2,
        2,
        2,
        4,
        2,
    ]
    assert [len(action.target_faces) for action in plan.actions] == [
        2,
        2,
        2,
        2,
        2,
        2,
        2,
        4,
        4,
        2,
        2,
    ]
    assert all(action.route.angle_rad > 0.0 for action in plan.actions)
    assert all(
        action.route.reference_module == action.momentum.fixed_module for action in plan.actions
    )


def test_plan_rejects_a_cut_that_does_not_detach_the_declared_slab() -> None:
    module = _example_module()
    plan = module.build_plan()
    first = plan.actions[0]

    with pytest.raises(ValueError, match="detach exactly moving_modules"):
        replace(
            plan,
            actions=(replace(first, release_faces=first.release_faces[:1]), *plan.actions[1:]),
        )


def test_mock_reference_executes_exact_mat_to_staircase_topology() -> None:
    module = _example_module()
    loaded = RobotPackLoader().load(_PACK_PATH)
    runtime = RuntimeSession.create(loaded, module.build_scene(), MockBackendAdapter())
    scenario = module.build_kinematic_scenario(runtime, dt_s=0.01)

    assert isinstance(scenario, CoordinatedKinematicPivotScenario)
    for _ in range(math.ceil(28.0 / 0.01)):
        scenario.step()
        if scenario.status.phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            break

    assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
    assert set(runtime.world.connections) == {
        pair.connection_id for pair in scenario.plan.final_connections
    }
    assert runtime.world.assemblies.count == 1
    assert {node.id: node.cell for node in _lattice_view(loaded, runtime).nodes} == {
        str(module_id): cell for module_id, cell in module.FINAL_CELLS.items()
    }
    assert sum(isinstance(event, DockCommitted) for event in runtime.world.event_log) == 42
    assert sum(isinstance(event, UndockCommitted) for event in runtime.world.event_log) == 24
    assert sum(isinstance(event, AssemblySplit) for event in runtime.world.event_log) == 11
    # Eleven merges assemble the initial mat; eleven more land the detached slabs.
    assert sum(isinstance(event, AssemblyMerged) for event in runtime.world.event_log) == 22


def test_runtime_runner_loads_the_kinematic_staircase_and_lattice_view() -> None:
    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=_PACK_PATH,
            demo=RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
            backend="mock",
            height_m=0.025,
            duration_s=28.0,
            dt_s=0.01,
        ),
        status_callback=statuses.append,
    )
    try:
        assert isinstance(runner.scenario, CoordinatedKinematicPivotScenario)
        assert runner.recipe.id == "mblocks_physics_lattice"
        assert len(runner.session.world.connections) == 16
        frame = runner.frame()
        assert isinstance(frame.view, CubicLatticeView)
        assert frame.scenario is not None
        assert frame.scenario.plan_id == "mblocks_twelve_module_staircase"
        assert statuses[-1] == "Running M-Blocks Twelve-Module Mat-to-Staircase with 12 modules"
    finally:
        runner.shutdown()


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"backend": "mock"}, "requires the MuJoCo backend"),
        ({"gravity": False}, "requires gravity and the ground plane"),
        ({"ground": False}, "requires gravity and the ground plane"),
        ({"height_m": 0.024}, "requires height_m >= 0.025"),
        ({"dt_s": MAX_MOMENTUM_TIMESTEP_S * 2.0}, "requires dt_s <= 0.0005"),
        ({"connector_gap_m": 0.01}, "connector_gap_m unspecified"),
    ),
)
def test_physical_staircase_rejects_unsupported_runtime_options(
    overrides: dict[str, Any],
    message: str,
) -> None:
    config = RuntimeInspectorConfig(
        pack_path=_PACK_PATH,
        demo=RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
        backend="mujoco",
        gravity=True,
        ground=True,
        height_m=0.025,
        dt_s=0.0005,
    )

    with pytest.raises(RuntimeInspectorSetupError, match=message):
        validate_runtime_inspector_config(replace(config, **overrides))


@pytest.mark.parametrize(
    ("demo", "duration_s", "dt_s", "gravity", "height_m"),
    (
        (RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE, 28.0, 0.002, False, 0.025),
        (
            RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
            12.0,
            0.0005,
            True,
            0.025,
        ),
    ),
)
def test_cli_supplies_staircase_defaults(
    monkeypatch: pytest.MonkeyPatch,
    demo: RuntimeDemo,
    duration_s: float,
    dt_s: float,
    gravity: bool,
    height_m: float,
) -> None:
    captured: dict[str, object] = {}
    runtime_app = ModuleType("modsim_studio.runtime_app")

    def fake_main(config: object) -> int:
        captured["config"] = config
        return 0

    runtime_app.main = fake_main  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "modsim_studio.runtime_app", runtime_app)
    result = CliRunner().invoke(
        app,
        ["runtime", str(_PACK_PATH), "--demo", demo.value, "--no-viewer"],
    )

    assert result.exit_code == 0, result.output
    config = captured["config"]
    assert isinstance(config, RuntimeInspectorConfig)
    assert config.demo is demo
    assert config.duration_s == pytest.approx(duration_s)
    assert config.dt_s == pytest.approx(dt_s)
    assert config.gravity is gravity
    assert config.ground is gravity
    assert config.height_m == pytest.approx(height_m)


@pytest.mark.mujoco
def test_real_mujoco_completes_full_physical_staircase_without_root_control(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    module = _example_module()
    loaded = RobotPackLoader().load(_PACK_PATH)
    runtime = RuntimeSession.create(
        loaded,
        module.build_scene(),
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        ground_height_m=0.0,
        timestep_s=0.0005,
        weld_pool_size=24,
        hinge_pool_size=2,
    )
    try:
        scenario = module.build_physical_scenario(runtime, dt_s=0.0005)
        assert isinstance(scenario, CoordinatedMomentumPivotScenario)
        adapter_type = type(runtime.adapter)

        def reject_runtime_root_control(*_args: object, **_kwargs: object) -> None:
            raise AssertionError("physical staircase wrote a root pose, twist, or wrench")

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
        assert scenario.status.time_s < 12.0
        assert set(runtime.world.connections) == {
            pair.connection_id for pair in scenario.plan.final_connections
        }
        assert len(runtime.world.connections) == 18
        assert runtime.world.assemblies.count == 1
        assert len(scenario.completed_telemetry) == 11
        assert all(len(item.flywheel_speeds_rad_s) == 2 for item in scenario.completed_telemetry)
        assert all(item.capture_time_s is not None for item in scenario.completed_telemetry)
        assert {node.id: node.cell for node in _lattice_view(loaded, runtime).nodes} == {
            str(module_id): cell for module_id, cell in module.FINAL_CELLS.items()
        }
        assert sum(isinstance(event, DockCommitted) for event in runtime.world.event_log) == 53
        assert sum(isinstance(event, UndockCommitted) for event in runtime.world.event_log) == 35
    finally:
        runtime.shutdown()
