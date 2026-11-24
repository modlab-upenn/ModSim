"""World settings, connection budgets and capability gating are backend-neutral.

Gravity, ground and timestep travel in ``SceneSpec.world``; reserved
connection storage is sized from ``SceneSpec.connection_budget``; demos are
gated on declared capabilities rather than on a backend's name.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim.backends.base import BackendCapabilities, BackendError
from modsim.backends.registry import (
    BackendEntry,
    backend_capabilities,
    backend_entry,
)
from modsim.core.scene import ConnectionBudget, SceneError, SceneSpec, WorldSettings
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.robot_packs.schema import ControlMode, PhysicalConstraintType
from modsim.runtime.demos import RuntimeDemo
from modsim.runtime.requirements import (
    DEMO_REQUIREMENTS,
    demo_backend_problems,
    engine_tuning,
)
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "generic_cube"


@pytest.fixture
def loaded(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


def test_world_settings_default_to_backend_defaults() -> None:
    world = SceneSpec().world
    assert world.gravity_m_s2 is None
    assert not world.ground
    assert world.timestep_s is None
    assert not world.requests_gravity
    assert not WorldSettings(gravity_m_s2=(0.0, 0.0, 0.0)).requests_gravity


@pytest.mark.parametrize(
    "kwargs",
    [
        {"gravity_m_s2": (0.0, float("nan"), 0.0)},
        {"timestep_s": 0.0},
        {"timestep_s": float("inf")},
        {"ground_height_m": float("nan")},
    ],
)
def test_world_settings_reject_invalid_values(kwargs: dict[str, object]) -> None:
    with pytest.raises(SceneError):
        WorldSettings(**kwargs)  # type: ignore[arg-type]


def test_connection_budget_is_keyed_by_constraint_type() -> None:
    budget = ConnectionBudget(fixed=12, hinge=2)
    assert budget.limit(PhysicalConstraintType.FIXED) == 12
    assert budget.limit(PhysicalConstraintType.HINGE) == 2
    assert budget.limit(PhysicalConstraintType.BALL) is None
    with pytest.raises(SceneError):
        ConnectionBudget(fixed=-1)


def test_scene_helpers_replace_only_their_field() -> None:
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1)
    world = WorldSettings(ground=True)
    budget = ConnectionBudget(fixed=3)
    updated = scene.with_world(world).with_connection_budget(budget)
    assert updated.placements == scene.placements
    assert updated.world is world
    assert updated.connection_budget is budget


def test_mock_refuses_a_scene_that_needs_gravity(loaded: LoadedRobotPack) -> None:
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1).with_world(
        WorldSettings(gravity_m_s2=(0.0, 0.0, -9.81), ground=True)
    )
    with pytest.raises(BackendError, match=r"gravity.*ground plane"):
        RuntimeSession.create(loaded, scene, "mock")


def test_mock_accepts_zero_gravity_and_a_timestep_hint(loaded: LoadedRobotPack) -> None:
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1).with_world(
        WorldSettings(gravity_m_s2=(0.0, 0.0, 0.0), timestep_s=0.001)
    )
    session = RuntimeSession.create(loaded, scene, "mock")
    session.shutdown()


def test_capabilities_default_runtime_constraints_to_fixed() -> None:
    caps = BackendCapabilities(name="x", supports_runtime_constraints=True)
    assert caps.supported_constraint_types == frozenset({PhysicalConstraintType.FIXED})
    assert BackendCapabilities(name="y").supported_constraint_types == frozenset()
    with pytest.raises(ValueError):
        BackendCapabilities(name="z", supports_constraint_torques=True)
    with pytest.raises(ValueError):
        BackendCapabilities(name="w", connection_change_delay_substeps=-1)


def test_physical_demos_are_refused_on_the_kinematic_mock() -> None:
    problems = demo_backend_problems(RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT, "mock")
    assert "gravity" in problems
    assert "contact dynamics" in problems
    assert "hinge connections" in problems
    assert "effort joint control" in problems
    assert demo_backend_problems(RuntimeDemo.DOCK, "mock") == ()


def test_any_backend_with_the_capabilities_may_run_a_physical_demo(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Gating is by capability: a new engine needs no change to the demo code."""

    class _Capable:
        def capabilities(self) -> BackendCapabilities:
            return BackendCapabilities(
                name="capable_test_engine",
                supports_runtime_constraints=True,
                supports_constraint_removal=True,
                supports_gravity=True,
                supports_ground_plane=True,
                supports_contact_dynamics=True,
                supported_constraint_types=frozenset(
                    {PhysicalConstraintType.FIXED, PhysicalConstraintType.HINGE}
                ),
                supported_joint_control_modes=frozenset({ControlMode.EFFORT}),
            )

    import modsim.backends.registry as registry

    # Register only for this test, so other tests see the normal registry.
    monkeypatch.setitem(
        registry._REGISTRY,  # pyright: ignore[reportPrivateUsage]
        "capable_test_engine",
        BackendEntry(
            name="capable_test_engine",
            summary="test",
            # A capabilities-only stand-in: never instantiated as a real adapter here.
            loader=lambda: _Capable,  # pyright: ignore[reportArgumentType]
        ),
    )
    for demo in DEMO_REQUIREMENTS:
        assert demo_backend_problems(demo, "capable_test_engine") == ()


def test_unknown_backend_capabilities_raise_a_backend_error() -> None:
    with pytest.raises(BackendError):
        backend_capabilities("not_a_backend")


def test_viewer_comes_from_the_registry_entry() -> None:
    with pytest.raises(BackendError, match="no native viewer"):
        backend_entry("mock").viewer()


def test_engine_tuning_is_only_given_to_its_engine() -> None:
    assert engine_tuning(RuntimeDemo.MBLOCKS_ONLINE_LATTICE, "mujoco") == {
        "constraint_time_constant_s": 0.002
    }
    assert engine_tuning(RuntimeDemo.MBLOCKS_ONLINE_LATTICE, "isaac") == {}
    assert engine_tuning(RuntimeDemo.DOCK, "mujoco") == {}


@pytest.mark.mujoco
def test_mujoco_reads_world_settings_and_budget_from_the_scene(loaded: LoadedRobotPack) -> None:
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
    from modsim_backend_mujoco.scene import GROUND_GEOM

    scene = (
        SceneSpec.grid(MODULE_TYPE, 3, spacing_m=0.1, origin=(0.0, 0.0, 0.2))
        .with_world(
            WorldSettings(
                gravity_m_s2=(0.0, 0.0, -3.0), ground=True, ground_height_m=0.01, timestep_s=0.001
            )
        )
        .with_connection_budget(ConnectionBudget(fixed=1, hinge=0))
    )
    session = RuntimeSession.create(loaded, scene, "mujoco")
    try:
        adapter = session.adapter
        assert isinstance(adapter, MuJoCoBackendAdapter)
        compiled = adapter._require_compiled()
        assert tuple(compiled.model.opt.gravity) == (0.0, 0.0, -3.0)
        assert compiled.model.opt.timestep == 0.001
        assert len(compiled.weld_pool) == 1
        assert len(compiled.hinge_pool) == 0
        assert compiled.model.geom(GROUND_GEOM).id >= 0
    finally:
        session.shutdown()


@pytest.mark.mujoco
def test_mujoco_constructor_options_override_the_scene(loaded: LoadedRobotPack) -> None:
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter

    scene = (
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1)
        .with_world(WorldSettings(gravity_m_s2=(0.0, 0.0, -3.0), timestep_s=0.001))
        .with_connection_budget(ConnectionBudget(fixed=1))
    )
    adapter = MuJoCoBackendAdapter(gravity=(0.0, 0.0, 0.0), timestep_s=0.002, weld_pool_size=4)
    session = RuntimeSession.create(loaded, scene, adapter)
    try:
        compiled = adapter._require_compiled()
        assert tuple(compiled.model.opt.gravity) == (0.0, 0.0, 0.0)
        assert compiled.model.opt.timestep == 0.002
        assert len(compiled.weld_pool) == 4
    finally:
        session.shutdown()
