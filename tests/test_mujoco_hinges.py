"""MuJoCo hinge constraints and equality-force reporting."""

# ruff: noqa: E402 -- optional MuJoCo must be skipped before backend imports

from __future__ import annotations

from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

import numpy as np

from modsim.core.events import DockCommitted, DockFailed, UndockCommitted
from modsim.core.ids import ConnectorInstanceId, ConstraintHandle, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.transforms import (
    Transform,
    angle_between,
    quat_angle,
    quat_conjugate,
    quat_multiply,
    vec_norm,
    vec_scale,
    vec_sub,
)
from modsim.robot_packs import LoadedRobotPack, RobotPack, RobotPackLoader
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
from modsim_backend_mujoco.hinges import (
    HingePool,
    HingePoolExhaustedError,
    hinge_anchor_pairs,
)
from modsim_backend_mujoco.scene import MuJoCoSceneError, build_scene

pytestmark = pytest.mark.mujoco

MODULE_TYPE = "generic_cube"
CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")
HINGE_AXIS = (0.0, 1.0, 0.0)
STEP_S = 0.0005


@pytest.fixture
def fixed_pack(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


@pytest.fixture
def hinge_pack(fixed_pack: LoadedRobotPack) -> LoadedRobotPack:
    """Put generic-cube connectors on a top edge and request a y-axis hinge."""
    data = fixed_pack.pack.model_dump(mode="python")
    connector_type = data["hardware_catalog"]["connector_types"]["fixed_face"]
    connector_type["physical_connection"] = {
        "constraint": "hinge",
        "hinge": {
            "axis": HINGE_AXIS,
            "anchor_separation_m": 0.08,
        },
    }
    connectors = data["hardware_catalog"]["module_types"][MODULE_TYPE]["connectors"]
    for connector in connectors:
        x = connector["local_pose"]["xyz_m"][0]
        connector["local_pose"]["xyz_m"] = (x, 0.0, 0.05)
    return fixed_pack.with_pack(RobotPack.model_validate(data))


def session_for(
    loaded: LoadedRobotPack,
    *,
    count: int = 2,
    hinge_pool_size: int = 1,
    weld_pool_size: int = 1,
) -> RuntimeSession:
    return RuntimeSession.create(
        loaded,
        SceneSpec.grid(MODULE_TYPE, count, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
        timestep_s=STEP_S,
        hinge_pool_size=hinge_pool_size,
        weld_pool_size=weld_pool_size,
    )


def dock_first_pair(session: RuntimeSession) -> DockCommitted:
    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.0)
    committed = [event for event in events if isinstance(event, DockCommitted)]
    assert len(committed) == 1
    return committed[0]


def test_anchor_pairs_follow_oppositely_oriented_connector_axes() -> None:
    body_a = Transform.identity()
    body_b = Transform.from_translation((0.1, 0.0, 0.0))
    connector_a = Transform.from_translation((0.05, 0.0, 0.05))
    connector_b = Transform(
        translation=(0.05, 0.0, 0.05),
        rotation=(0.0, 0.0, 0.0, 1.0),
    )

    anchors = hinge_anchor_pairs(
        body_a,
        connector_a,
        body_b,
        connector_b,
        HINGE_AXIS,
        0.08,
    )

    assert [anchor.body_a for anchor in anchors] == pytest.approx(
        [(0.05, -0.04, 0.05), (0.05, 0.04, 0.05)]
    )
    assert [anchor.body_b for anchor in anchors] == pytest.approx(
        [(-0.05, -0.04, 0.05), (-0.05, 0.04, 0.05)]
    )


def test_scene_reserves_two_inactive_connect_equalities_per_hinge(
    hinge_pack: LoadedRobotPack,
) -> None:
    compiled = build_scene(
        hinge_pack.pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1),
        hinge_pack.root,
        weld_pool_size=0,
        hinge_pool_size=3,
    )

    assert len(compiled.hinge_pool) == 3
    assert len({equality for pair in compiled.hinge_pool for equality in pair}) == 6
    for pair in compiled.hinge_pool:
        for equality in pair:
            assert compiled.model.eq_type[equality] == mujoco.mjtEq.mjEQ_CONNECT
            assert not compiled.model.eq_active0[equality]


def test_negative_hinge_pool_size_is_rejected(hinge_pack: LoadedRobotPack) -> None:
    with pytest.raises(MuJoCoSceneError, match="hinge pool size must not be negative"):
        build_scene(
            hinge_pack.pack,
            SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1),
            hinge_pack.root,
            hinge_pool_size=-1,
        )


def test_a_hinge_allows_rotation_only_about_its_line(hinge_pack: LoadedRobotPack) -> None:
    session = session_for(hinge_pack, weld_pool_size=0)
    try:
        committed = dock_first_pair(session)
        adapter = session.adapter
        assert isinstance(adapter, MuJoCoBackendAdapter)
        assert adapter.hinge_pool.in_use == 1
        # Hinges deliberately retain body contact; unlike a weld, they never
        # claim one of the runtime collision exclusions.
        assert not np.asarray(adapter.model.exclude_signature).any()

        adapter.apply_module_wrench(CUBE_1, torque_nm=(0.0, -0.1, 0.0))
        for _ in range(400):
            session.step(STEP_S, process_connectors=False)

        pose_a = session.world.modules[CUBE_0].pose
        pose_b = session.world.modules[CUBE_1].pose
        relative_rotation = quat_multiply(quat_conjugate(pose_a.rotation), pose_b.rotation)
        assert quat_angle(relative_rotation) > 0.1

        connector_a = session.world.connector(committed.connector_a).world_pose
        connector_b = session.world.connector(committed.connector_b).world_pose
        axis_a = connector_a.apply_direction(HINGE_AXIS)
        axis_b = connector_b.apply_direction(HINGE_AXIS)
        assert min(
            angle_between(axis_a, axis_b), angle_between(axis_a, vec_scale(axis_b, -1.0))
        ) < (1e-3)
        assert vec_norm(vec_sub(connector_a.translation, connector_b.translation)) < 1e-3
    finally:
        session.shutdown()


def test_hinge_release_deactivates_both_equalities_and_returns_slot(
    hinge_pack: LoadedRobotPack,
) -> None:
    session = session_for(hinge_pack, weld_pool_size=0)
    try:
        dock_first_pair(session)
        adapter = session.adapter
        assert isinstance(adapter, MuJoCoBackendAdapter)
        connection = next(iter(session.world.connections.values()))
        slot = adapter.hinge_pool.slot_for(connection.constraint_handle)
        assert slot is not None
        assert all(adapter.data.eq_active[equality] for equality in slot.equality_ids)

        session.request_undock(connection.id)
        events = session.step(0.0)

        assert any(isinstance(event, UndockCommitted) for event in events)
        assert adapter.hinge_pool.in_use == 0
        assert adapter.hinge_pool.available == adapter.hinge_pool.capacity
        assert not any(adapter.data.eq_active[equality] for equality in slot.equality_ids)
        assert connection.constraint_handle not in adapter.snapshot().constraint_forces_n
        assert not adapter.remove_physical_connection(connection.constraint_handle)
    finally:
        session.shutdown()


def test_exhausted_hinge_pool_refuses_without_creating_a_logical_connection(
    hinge_pack: LoadedRobotPack,
) -> None:
    session = session_for(hinge_pack, count=4, hinge_pool_size=1, weld_pool_size=0)
    try:
        dock_first_pair(session)
        session.request_dock(
            ConnectorInstanceId("generic_cube_2/front"),
            ConnectorInstanceId("generic_cube_3/rear"),
        )
        events = session.step(0.0)

        failures = [event for event in events if isinstance(event, DockFailed)]
        assert len(failures) == 1
        assert "hinge slots are in use" in failures[0].detail
        assert len(session.world.connections) == 1
    finally:
        session.shutdown()


def test_snapshot_reports_force_for_active_hinge(hinge_pack: LoadedRobotPack) -> None:
    session = session_for(hinge_pack, weld_pool_size=0)
    try:
        dock_first_pair(session)
        adapter = session.adapter
        assert isinstance(adapter, MuJoCoBackendAdapter)
        connection = next(iter(session.world.connections.values()))

        adapter.apply_module_wrench(CUBE_1, force_n=(4.0, 0.0, 0.0))
        session.step(0.01, process_connectors=False)
        forces = adapter.snapshot().constraint_forces_n

        assert set(forces) == {connection.constraint_handle}
        assert forces[connection.constraint_handle] > 0.0
    finally:
        session.shutdown()


def test_snapshot_reports_translational_force_for_active_weld(
    fixed_pack: LoadedRobotPack,
) -> None:
    session = session_for(fixed_pack, hinge_pool_size=0)
    try:
        dock_first_pair(session)
        adapter = session.adapter
        assert isinstance(adapter, MuJoCoBackendAdapter)
        connection = next(iter(session.world.connections.values()))

        adapter.apply_module_wrench(CUBE_1, force_n=(0.0, 4.0, 0.0))
        session.step(0.01, process_connectors=False)
        forces = adapter.snapshot().constraint_forces_n

        assert adapter.capabilities().supports_constraint_forces
        assert set(forces) == {connection.constraint_handle}
        assert forces[connection.constraint_handle] > 0.0
    finally:
        session.shutdown()


def test_hinge_pool_bookkeeping_reports_exhaustion() -> None:
    pool = HingePool.over(((1, 2),))
    first = ConstraintHandle("hinge:first")
    second = ConstraintHandle("hinge:second")
    pool.claim(first)

    with pytest.raises(HingePoolExhaustedError, match="raise hinge_pool_size"):
        pool.claim(second)

    assert pool.release(first) is not None
