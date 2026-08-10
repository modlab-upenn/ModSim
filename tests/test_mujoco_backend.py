"""MuJoCo backend tests.

Skipped entirely when the optional MuJoCo dependencies are not installed, so a
core-only environment still runs a green suite.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.backends.base import BackendAdapter, BackendError, SupportsModuleKinematics
from modsim.core.entities import ConnectorLifecycleState
from modsim.core.events import DockCommitted, DockFailed
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, quat_from_axis_angle, vec_norm, vec_sub
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
from modsim_backend_mujoco.scene import (
    MuJoCoSceneError,
    body_name,
    build_scene,
    site_name,
)

pytestmark = pytest.mark.mujoco

MODULE_TYPE = "generic_cube"
SPACING_M = 0.1
CUBE_0 = ModuleInstanceId("generic_cube_0")
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")


@pytest.fixture
def loaded_pack(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


def weightless_session(loaded: LoadedRobotPack, count: int = 2) -> RuntimeSession:
    """Return a MuJoCo session with gravity disabled.

    Gravity off is what makes a MuJoCo run comparable to the kinematic mock, so
    a difference in behaviour is attributable to the adapter rather than to the
    modules falling.
    """
    return RuntimeSession.create(
        loaded,
        SceneSpec.grid(MODULE_TYPE, count, spacing_m=SPACING_M),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )


# ----------------------------------------------------------------------
# scene compilation
# ----------------------------------------------------------------------


def test_each_placement_becomes_its_own_body(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 3, spacing_m=SPACING_M),
        loaded_pack.root,
    )

    # Three independent copies, not one body with stacked name prefixes.
    assert compiled.model.nbody == 4  # world plus three modules
    for index in range(3):
        module = ModuleInstanceId(f"{MODULE_TYPE}_{index}")
        assert (module, "base_link") in compiled.body_ids
        assert compiled.handles.bodies[(module, "base_link")] == body_name(module, "base_link")


def test_every_connector_becomes_a_site(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=SPACING_M),
        loaded_pack.root,
    )

    assert compiled.model.nsite == 4
    assert FRONT_0 in compiled.site_ids
    assert compiled.handles.connector_frames[FRONT_0] == site_name(CUBE_0, "front")


def test_modules_get_six_degrees_of_freedom(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=SPACING_M),
        loaded_pack.root,
    )

    assert compiled.model.nq == 14  # two free joints, seven coordinates each


def test_a_weld_pool_is_reserved_and_inactive(loaded_pack: LoadedRobotPack) -> None:
    compiled = build_scene(
        loaded_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=SPACING_M),
        loaded_pack.root,
        weld_pool_size=5,
    )

    assert len(compiled.weld_pool) == 5
    assert not compiled.model.eq_active0.any()


def test_a_missing_asset_is_reported_clearly(loaded_pack: LoadedRobotPack, tmp_path: Path) -> None:
    with pytest.raises(MuJoCoSceneError, match="missing on disk"):
        build_scene(
            loaded_pack.pack,
            SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
            tmp_path,
        )


def test_a_non_positive_timestep_is_rejected(loaded_pack: LoadedRobotPack) -> None:
    with pytest.raises(MuJoCoSceneError, match="timestep must be positive"):
        build_scene(
            loaded_pack.pack,
            SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M),
            loaded_pack.root,
            timestep_s=0.0,
        )


# ----------------------------------------------------------------------
# adapter contract
# ----------------------------------------------------------------------


def test_the_adapter_satisfies_the_backend_protocol() -> None:
    assert isinstance(MuJoCoBackendAdapter(), BackendAdapter)


def test_loading_without_a_pack_root_is_refused(loaded_pack: LoadedRobotPack) -> None:
    adapter = MuJoCoBackendAdapter()
    with pytest.raises(BackendError, match="needs the Robot Pack directory"):
        adapter.load(loaded_pack.pack, SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M))


def test_using_the_adapter_before_loading_is_refused() -> None:
    adapter = MuJoCoBackendAdapter()
    with pytest.raises(BackendError, match="no scene loaded"):
        adapter.snapshot()


def test_stepping_backwards_is_refused(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    with pytest.raises(BackendError, match="step backwards"):
        session.adapter.step(-1.0)


# ----------------------------------------------------------------------
# physics
# ----------------------------------------------------------------------


def test_modules_fall_under_gravity(loaded_pack: LoadedRobotPack) -> None:
    session = RuntimeSession.create(
        loaded_pack, SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M), "mujoco"
    )
    for _ in range(20):
        session.step(0.01)

    height = session.world.modules[ModuleInstanceId(f"{MODULE_TYPE}_0")].pose.translation[2]
    assert height < -0.01


def test_gravity_can_be_disabled_for_comparison_with_the_mock(
    loaded_pack: LoadedRobotPack,
) -> None:
    session = weightless_session(loaded_pack, count=1)
    for _ in range(20):
        session.step(0.01)

    module = session.world.modules[ModuleInstanceId(f"{MODULE_TYPE}_0")]
    assert vec_norm(module.pose.translation) == pytest.approx(0.0, abs=1e-9)


def test_a_ground_plane_stops_modules_falling(loaded_pack: LoadedRobotPack) -> None:
    scene = SceneSpec(
        placements=(
            ModulePlacement(
                instance_id=CUBE_0,
                module_type_id=MODULE_TYPE,
                pose=Transform.from_translation((0.0, 0.0, 0.4)),
            ),
        )
    )
    session = RuntimeSession.create(loaded_pack, scene, "mujoco", ground=True)
    for _ in range(1500):
        session.step(0.002)

    # A 0.1 m cube resting on a plane at z = 0 sits with its centre at 0.05.
    assert session.world.modules[CUBE_0].pose.translation[2] == pytest.approx(0.05, abs=1e-3)


def test_without_a_ground_plane_modules_keep_falling(loaded_pack: LoadedRobotPack) -> None:
    session = RuntimeSession.create(
        loaded_pack, SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M), "mujoco"
    )
    for _ in range(500):
        session.step(0.002)

    assert session.world.modules[CUBE_0].pose.translation[2] < -0.1


def test_module_twist_is_expressed_in_world_coordinates(loaded_pack: LoadedRobotPack) -> None:
    """A free joint stores angular velocity in the body frame; the API does not.

    The module is spawned rotated so that a body-frame leak would show up as a
    velocity about the wrong world axis.
    """
    rotated = SceneSpec(
        placements=(
            ModulePlacement(
                instance_id=CUBE_0,
                module_type_id=MODULE_TYPE,
                pose=Transform(rotation=quat_from_axis_angle((1.0, 0.0, 0.0), math.pi / 2)),
            ),
        )
    )
    session = RuntimeSession.create(loaded_pack, rotated, "mujoco", gravity=(0.0, 0.0, 0.0))
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    assert isinstance(adapter, SupportsModuleKinematics)

    adapter.set_module_twist(CUBE_0, linear_m_s=(0.1, 0.0, 0.0), angular_rad_s=(0.0, 0.0, 2.0))
    body = adapter.snapshot().link_states[CUBE_0]["base_link"]

    assert body.linear_velocity_m_s == pytest.approx((0.1, 0.0, 0.0), abs=1e-9)
    assert body.angular_velocity_rad_s == pytest.approx((0.0, 0.0, 2.0), abs=1e-9)


def test_a_driven_module_travels_at_the_commanded_speed(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.set_module_twist(CUBE_0, linear_m_s=(0.1, 0.0, 0.0))
    start = session.world.modules[CUBE_0].pose.translation

    for _ in range(500):
        session.step(0.002)

    travelled = vec_norm(vec_sub(session.world.modules[CUBE_0].pose.translation, start))
    assert travelled == pytest.approx(0.1, abs=1e-3)


def test_an_applied_wrench_accelerates_a_module(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.apply_module_wrench(CUBE_0, force_n=(1.0, 0.0, 0.0))

    for _ in range(500):
        session.step(0.002)
    speed = vec_norm(session.world.modules[CUBE_0].linear_velocity_m_s)

    # 1 N on 1 kg for 1 s.
    assert speed == pytest.approx(1.0, rel=0.05)

    adapter.clear_wrenches()
    for _ in range(500):
        session.step(0.002)
    assert vec_norm(session.world.modules[CUBE_0].linear_velocity_m_s) == pytest.approx(
        speed, rel=0.05
    )


def test_driving_an_unknown_module_is_refused(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    with pytest.raises(BackendError, match="no free joint"):
        adapter.set_module_twist(ModuleInstanceId("nobody"))


def test_simulation_time_advances(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    for _ in range(10):
        session.step(0.01)

    assert session.world.time_s == pytest.approx(0.1, abs=1e-6)


def test_connector_frames_come_from_measured_sites(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    snapshot = session.adapter.snapshot()

    # The adapter populates the optional measured-frame channel, which
    # WorldState prefers over recomposing link pose with the authored offset.
    assert snapshot.connector_frames[CUBE_0]["front"].translation == pytest.approx(
        (0.05, 0.0, 0.0), abs=1e-9
    )


def test_a_moved_module_carries_its_connectors(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.set_module_pose(CUBE_0, Transform.from_translation((0.0, 0.3, 0.0)))
    session.step(0.0)

    front = session.world.connector(FRONT_0)
    assert front.world_pose.translation == pytest.approx((0.05, 0.3, 0.0), abs=1e-6)


def test_a_spinning_module_reports_connector_velocity(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack, count=1)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    body = adapter.model.body_jntadr[1]
    adapter.data.qvel[adapter.model.jnt_dofadr[body] + 5] = 4.0
    session.step(0.01)

    front = session.world.connector(FRONT_0)
    # The connector sits away from the spin axis, so the lever-arm term shows up.
    assert vec_norm(front.world_velocity_m_s) > 0.05


# ----------------------------------------------------------------------
# docking is not wired yet, and says so
# ----------------------------------------------------------------------


def test_a_commanded_dock_commits_under_physics(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    assert session.adapter.capabilities().supports_runtime_constraints

    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    assert any(isinstance(event, DockCommitted) for event in events)
    assert not [event for event in events if isinstance(event, DockFailed)]
    assert len(session.world.connections) == 1
    assert session.world.assemblies.count == 1
    assert session.world.connector(FRONT_0).lifecycle_state is ConnectorLifecycleState.DOCKED


def test_detection_runs_before_any_command(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    proposals = session.proposals()

    assert proposals
    assert proposals[0].compatibility.compatible
    assert proposals[0].acceptance.satisfied
    # Nothing latches without a command, because the example pack's connector
    # type is not auto-latching.
    assert session.world.connector(FRONT_0).lifecycle_state is not ConnectorLifecycleState.DOCKED


def test_connector_geometry_matches_the_authored_layout(loaded_pack: LoadedRobotPack) -> None:
    session = weightless_session(loaded_pack)
    front = session.world.connector(FRONT_0)
    rear = session.world.connector(REAR_1)

    assert vec_norm(vec_sub(front.world_pose.translation, rear.world_pose.translation)) == (
        pytest.approx(0.0, abs=1e-9)
    )
    assert front.world_docking_axis == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    assert rear.world_docking_axis == pytest.approx((-1.0, 0.0, 0.0), abs=1e-9)
