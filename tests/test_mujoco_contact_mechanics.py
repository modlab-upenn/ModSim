"""Focused checks for composed ground contact and runtime weld exclusions."""

# ruff: noqa: E402 -- optional MuJoCo must be skipped before backend imports

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, quat_from_axis_angle
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
from modsim_backend_mujoco.scene import GROUND_GEOM, body_name, build_scene
from modsim_backend_mujoco.welds import collision_signature

pytestmark = pytest.mark.mujoco

SMORES_TYPE = "smores_ep"
CUBE_TYPE = "generic_cube"


def smores_scene(count: int, *, yaw_rad: float = 0.0) -> SceneSpec:
    return SceneSpec(
        placements=tuple(
            ModulePlacement(
                instance_id=ModuleInstanceId(f"smores_ep_{index}"),
                module_type_id=SMORES_TYPE,
                pose=Transform(
                    translation=(0.3 * index, 0.0, 0.05),
                    rotation=quat_from_axis_angle((0.0, 0.0, 1.0), yaw_rad),
                ),
            )
            for index in range(count)
        )
    )


def test_smores_instances_get_disk_faces_and_authored_ground_pairs(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    compiled = build_scene(
        smores_loaded_pack.pack,
        smores_scene(2),
        smores_loaded_pack.root,
        ground=True,
    )
    model = compiled.model
    ground = model.geom(GROUND_GEOM).id
    pairs = {
        frozenset((int(model.pair_geom1[index]), int(model.pair_geom2[index]))): index
        for index in range(model.npair)
    }

    assert model.npair == 6
    assert len(compiled.anisotropic_ground_geoms) == 4
    for module_index in range(2):
        prefix = f"smores_ep_{module_index}/"
        for face in ("left_face_proxy", "right_face_proxy", "pan_face_proxy"):
            geom = model.geom(f"{prefix}{face}").id
            assert int(model.geom_type[geom]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER)

        for tire_name in ("left_tire_contact", "right_tire_contact"):
            tire = model.geom(f"{prefix}{tire_name}").id
            pair = pairs[frozenset((ground, tire))]
            assert int(model.pair_dim[pair]) == 4
            assert model.pair_friction[pair] == pytest.approx((0.05, 1.0, 0.01, 0.0001, 0.0001))

        skid = model.geom(f"{prefix}rear_skid").id
        pair = pairs[frozenset((ground, skid))]
        assert int(model.pair_dim[pair]) == 3
        assert model.pair_friction[pair] == pytest.approx((0.08, 0.08, 0.005, 0.0001, 0.0001))


def test_tire_friction_frame_follows_a_yawed_wheel_axle(
    smores_loaded_pack: LoadedRobotPack,
) -> None:
    session = RuntimeSession.create(
        smores_loaded_pack,
        smores_scene(1, yaw_rad=math.pi / 2.0),
        "mujoco",
        ground=True,
        timestep_s=0.002,
    )
    try:
        adapter = session.adapter
        assert isinstance(adapter, MuJoCoBackendAdapter)
        session.step(1.0)

        model = adapter.model
        data = adapter.data
        ground = model.geom(GROUND_GEOM).id
        tire = model.geom("smores_ep_0/left_tire_contact").id
        contacts = [
            contact
            for contact in data.contact
            if {int(contact.geom[0]), int(contact.geom[1])} == {ground, tire}
        ]
        assert contacts
        frame = np.asarray(contacts[0].frame).reshape(3, 3)
        axle = data.geom_xmat[tire].reshape(3, 3)[:, 2]
        assert abs(float(np.dot(frame[1], axle))) > 0.999
    finally:
        session.shutdown()


def test_weld_claim_and_release_atomically_publish_contact_exclusion(
    example_pack_dir: Path,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    first = ModuleInstanceId("generic_cube_0")
    second = ModuleInstanceId("generic_cube_1")
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid(CUBE_TYPE, 2, spacing_m=0.1),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    try:
        adapter = session.adapter
        assert isinstance(adapter, MuJoCoBackendAdapter)
        assert not adapter.model.exclude_signature.any()

        session.request_dock(
            ConnectorInstanceId("generic_cube_0/front"),
            ConnectorInstanceId("generic_cube_1/rear"),
        )
        session.step(0.01)
        assert len(session.world.connections) == 1

        body_a = adapter.model.body(body_name(first, "base_link")).id
        body_b = adapter.model.body(body_name(second, "base_link")).id
        expected = collision_signature(body_a, body_b)
        assert np.count_nonzero(adapter.model.exclude_signature) == 1
        assert expected in adapter.model.exclude_signature

        session.request_undock(next(iter(session.world.connections)))
        session.step(0.01)
        assert not session.world.connections
        assert not adapter.model.exclude_signature.any()
        assert adapter.weld_pool.in_use == 0
    finally:
        session.shutdown()
