"""Lazy snapshots and connector frames must equal the former eager values exactly.

The M-Blocks demos amplify differences near 1e-9 m, so these checks use exact
equality, not tolerances.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import pytest

from modsim.core.entities import JointCommand
from modsim.core.ids import JointInstanceId, ModuleInstanceId, connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, FrozenMapping
from modsim.core.transforms import Transform, quat_rotate

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

from modsim.robot_packs import RobotPackLoader  # noqa: E402
from modsim.robot_packs.schema import ControlMode  # noqa: E402
from modsim.runtime.mblocks_lattice import larger_state, tabletop_scene  # noqa: E402
from modsim.runtime.session import RuntimeSession  # noqa: E402

PACK = Path(__file__).parents[1] / "examples/robot_packs/mblocks_3d"
SMORES_PACK = Path(__file__).parents[1] / "examples/robot_packs/smores_ep"


@pytest.fixture
def session() -> Iterator[RuntimeSession]:
    created = RuntimeSession.create(
        RobotPackLoader().load(PACK),
        tabletop_scene(larger_state()),
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        timestep_s=0.0005,
    )
    # Give the flywheels and bodies non-trivial motion before sampling.
    created.set_joint_commands(
        (JointCommand(JointInstanceId("block_1/flywheel_spin"), ControlMode.EFFORT, 0.01),)
    )
    for _ in range(40):
        created.step(0.0005, process_connectors=False)
    yield created
    created.shutdown()


@pytest.fixture(params=["mblocks", "smores"])
def any_session(request: pytest.FixtureRequest) -> Iterator[RuntimeSession]:
    """Both packs: SMORES bodies have inertial frames offset from their origins."""
    if request.param == "mblocks":
        loaded = RobotPackLoader().load(PACK)
        scene = tabletop_scene(larger_state())
        command = JointCommand(JointInstanceId("block_1/flywheel_spin"), ControlMode.EFFORT, 0.01)
        dt = 0.0005
    else:
        loaded = RobotPackLoader().load(SMORES_PACK)
        module_type = next(iter(loaded.pack.hardware_catalog.module_types))
        scene = SceneSpec.of(
            ModulePlacement(
                instance_id=ModuleInstanceId(f"smores_{index}"),
                module_type_id=module_type,
                pose=Transform.from_translation((0.2 * index, 0.0, 0.05)),
            )
            for index in range(2)
        )
        command = None
        dt = 0.002
    created = RuntimeSession.create(
        loaded, scene, "mujoco", gravity=(0.0, 0.0, -9.81), ground=True, timestep_s=dt
    )
    if command is not None:
        created.set_joint_commands((command,))
    for _ in range(50):  # falling and settling gives every body a non-trivial twist
        created.step(dt, process_connectors=False)
    yield created
    created.shutdown()


def _bits(values: tuple[float, ...]) -> tuple[str, ...]:
    """Exact representation, including the sign of zero."""
    return tuple(float(v).hex() for v in values)


def _eager_reference(adapter: object) -> dict[str, dict]:
    """Recompute the snapshot exactly the way the pre-lazy adapter did."""
    compiled = adapter._compiled  # type: ignore[attr-defined]
    model, data = compiled.model, adapter._data  # type: ignore[attr-defined]
    links: dict = {}
    velocity = np.zeros(6)
    for (module_id, link), body_id in compiled.body_ids.items():
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, velocity, 0)
        links[(module_id, link)] = (
            tuple(float(v) for v in data.xpos[body_id]),
            tuple(float(v) for v in data.xquat[body_id]),
            tuple(float(v) for v in velocity[0:3]),
            tuple(float(v) for v in velocity[3:6]),
        )
    sites: dict = {}
    for instance, site_id in compiled.site_ids.items():
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.asarray(data.site_xmat[site_id], dtype=float).reshape(9))
        sites[instance] = (
            tuple(float(v) for v in data.site_xpos[site_id]),
            tuple(float(v) for v in quat),
        )
    joints: dict = {}
    for (module_id, joint_id), native in compiled.joint_ids.items():
        actuator = compiled.actuator_ids.get((module_id, joint_id))
        joints[(module_id, joint_id)] = (
            float(data.qpos[model.jnt_qposadr[native]]),
            float(data.qvel[model.jnt_dofadr[native]]),
            0.0 if actuator is None else float(data.actuator_force[actuator]),
        )
    return {"links": links, "sites": sites, "joints": joints}


def test_lazy_snapshot_is_bit_identical_to_eager_values(any_session: RuntimeSession) -> None:
    session = any_session
    expected = _eager_reference(session.adapter)
    snapshot = session.adapter.snapshot()
    assert isinstance(snapshot.link_states, FrozenMapping)

    for (module_id, link), (pos, quat, ang, lin) in expected["links"].items():
        body = snapshot.link_states[module_id][link]
        assert _bits(body.pose.translation) == _bits(pos)
        assert _bits(body.pose.rotation) == _bits(quat)
        assert _bits(body.angular_velocity_rad_s) == _bits(ang)
        assert _bits(body.linear_velocity_m_s) == _bits(lin)
        assert snapshot.body(module_id, link) is body  # cached per snapshot
    for instance, (pos, quat) in expected["sites"].items():
        module_id, connector_id = instance.split("/", 1)
        frame = snapshot.connector_frames[ModuleInstanceId(module_id)][connector_id]
        assert _bits(frame.translation) == _bits(pos)
        assert _bits(frame.rotation) == _bits(quat)
    for (module_id, joint_id), (position, velocity, effort) in expected["joints"].items():
        state = snapshot.joint_states[module_id][joint_id]
        assert (state.position, state.velocity, state.effort) == (position, velocity, effort)


def test_snapshot_is_frozen_against_later_steps(any_session: RuntimeSession) -> None:
    session = any_session
    expected = _eager_reference(session.adapter)
    snapshot = session.adapter.snapshot()
    # Step before the first lookup: values must still describe the sampled time.
    session.step(0.05, process_connectors=False)
    for (module_id, link), (pos, _, ang, lin) in expected["links"].items():
        body = snapshot.link_states[module_id][link]
        assert body.pose.translation == pos
        assert (body.angular_velocity_rad_s, body.linear_velocity_m_s) == (ang, lin)
    assert session.adapter.snapshot().time_s > snapshot.time_s
    # Unknown keys behave like an ordinary mapping.
    assert snapshot.link_states.get(ModuleInstanceId("missing")) is None
    assert "missing" not in snapshot.connector_frames


def test_lazy_connector_frames_match_an_eager_refresh(session: RuntimeSession) -> None:
    world = session.world
    snapshot = session.adapter.snapshot()
    world.ingest(snapshot)
    pack = world.pack
    for connector in world.connectors.values():
        module = world.modules[connector.module_id]
        module_type = pack.hardware_catalog.module_types[module.module_type_id]
        spec = next(s for s in module_type.connectors if s.id == connector.connector_id)
        link_pose = module.link_poses[connector.parent_link]
        measured = snapshot.connector_frames[connector.module_id][connector.connector_id]
        body = snapshot.body(connector.module_id, connector.parent_link)
        assert body is not None
        assert connector.resolved
        assert connector.world_pose == measured
        assert spec.docking_axis is not None
        assert connector.world_docking_axis == quat_rotate(link_pose.rotation, spec.docking_axis)
        approach = spec.approach_axis or spec.docking_axis
        assert connector.world_approach_axis == quat_rotate(link_pose.rotation, approach)
        assert connector.world_velocity_m_s == body.velocity_at(measured.translation)


def test_connector_frame_tracks_new_samples_and_keeps_overrides(session: RuntimeSession) -> None:
    world = session.world
    connector = world.connector(connector_instance_id(ModuleInstanceId("block_1"), "pos_x"))
    first = connector.world_pose
    session.step(0.05, process_connectors=False)
    second = connector.world_pose
    assert second == world.connector(connector.id).world_pose
    # An assignment overrides only that field, and only until the next sample.
    connector.world_velocity_m_s = (9.0, 9.0, 9.0)
    assert connector.world_velocity_m_s == (9.0, 9.0, 9.0)
    assert connector.world_pose == second
    world.ingest(session.adapter.snapshot())
    assert connector.world_velocity_m_s != (9.0, 9.0, 9.0)
    del first


def test_plain_dictionary_snapshots_are_still_copied(session: RuntimeSession) -> None:
    world = session.world
    module_id = ModuleInstanceId("block_1")
    joint = JointInstanceId(f"{module_id}/flywheel_spin")
    joints = {"flywheel_spin": world.joint_state(joint)}
    world.ingest(BackendStateSnapshot(time_s=world.time_s, joint_states={module_id: joints}))
    joints.clear()
    assert world.joint_state(joint) is not None


def test_grouped_constraint_forces_match_a_per_connection_scan(session: RuntimeSession) -> None:
    from modsim_backend_mujoco.adapter import (
        _EqualityForceRows,  # pyright: ignore[reportPrivateUsage]
    )

    world = session.world
    pairs = [
        (
            connector_instance_id(ModuleInstanceId("block_1"), "pos_x"),
            connector_instance_id(ModuleInstanceId("block_2"), "neg_x"),
        ),
        (
            connector_instance_id(ModuleInstanceId("block_3"), "pos_x"),
            connector_instance_id(ModuleInstanceId("block_4"), "neg_x"),
        ),
    ]
    for pair in pairs:
        session.request_dock(*pair)
    session.process_docking()
    for _ in range(20):
        session.step(0.0005, process_connectors=False)
    data = session.adapter._data  # type: ignore[attr-defined]  # pyright: ignore[reportPrivateUsage]
    grouped = _EqualityForceRows(data)
    count = data.nefc
    kinds = np.asarray(data.efc_type[:count])
    identifiers = np.asarray(data.efc_id[:count])
    equality = int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    checked = 0
    for equality_id in set(identifiers[kinds == equality].tolist()):
        expected = np.asarray(data.efc_force[:count])[
            (kinds == equality) & (identifiers == equality_id)
        ]
        assert np.array_equal(grouped(equality_id), expected)
        checked += 1
    assert checked >= 1
    assert len(grouped(10_000)) == 0
    assert world.connections


def test_module_root_state_is_lazy_and_exact(session: RuntimeSession) -> None:
    world = session.world
    module_id = ModuleInstanceId("block_1")
    expected = _eager_reference(session.adapter)["links"][(module_id, "base_link")]
    world.ingest(session.adapter.snapshot())
    module = world.modules[module_id]
    assert _bits(module.pose.translation) == _bits(expected[0])
    assert _bits(module.angular_velocity_rad_s) == _bits(expected[2])
    assert _bits(module.linear_velocity_m_s) == _bits(expected[3])


def test_sample_without_a_module_keeps_its_last_root_state(session: RuntimeSession) -> None:
    world = session.world
    module_id = ModuleInstanceId("block_1")
    world.ingest(session.adapter.snapshot())  # not read before the next sample
    expected = _eager_reference(session.adapter)["links"][(module_id, "base_link")]
    session.adapter.step(0.01)
    world.ingest(BackendStateSnapshot(time_s=world.time_s + 0.01))  # omits every module
    module = world.modules[module_id]
    assert _bits(module.pose.translation) == _bits(expected[0])
    assert _bits(module.linear_velocity_m_s) == _bits(expected[3])
