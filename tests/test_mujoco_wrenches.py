"""MuJoCo connection wrenches: exactness, frame convention and change delay.

The adapter turns MuJoCo's equality solver rows into a world-frame wrench on
module A about connector A's origin. These tests check that conversion against
MuJoCo itself: the generalized force the solver applied through the
connection's rows (``J.T @ f``) must equal the generalized force of the
reported wrench applied at connector A (``Jp.T @ F + Jr.T @ M``) on module A's
own degrees of freedom.
"""

# ruff: noqa: E402 -- optional MuJoCo must be skipped before backend imports

from __future__ import annotations

from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

import numpy as np

from modsim.core.events import DockCommitted
from modsim.core.ids import ConnectorInstanceId, ConstraintHandle, ModuleInstanceId
from modsim.core.scene import SceneSpec, WorldSettings
from modsim.core.snapshot import ConstraintWrench
from modsim.core.transforms import Transform, quat_conjugate, quat_from_axis_angle
from modsim.robot_packs import LoadedRobotPack, RobotPack, RobotPackLoader
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter

pytestmark = pytest.mark.mujoco

MODULE_TYPE = "generic_cube"
CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")
STEP_S = 0.0005


@pytest.fixture
def fixed_pack(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


@pytest.fixture
def hinge_pack(fixed_pack: LoadedRobotPack) -> LoadedRobotPack:
    data = fixed_pack.pack.model_dump(mode="python")
    connector_type = data["hardware_catalog"]["connector_types"]["fixed_face"]
    connector_type["physical_connection"] = {
        "constraint": "hinge",
        "hinge": {"axis": (0.0, 1.0, 0.0), "anchor_separation_m": 0.08},
    }
    for connector in data["hardware_catalog"]["module_types"][MODULE_TYPE]["connectors"]:
        x = connector["local_pose"]["xyz_m"][0]
        connector["local_pose"]["xyz_m"] = (x, 0.0, 0.05)
    return fixed_pack.with_pack(RobotPack.model_validate(data))


def _docked_session(loaded: LoadedRobotPack) -> tuple[RuntimeSession, MuJoCoBackendAdapter]:
    # A slightly rotated second cube makes every frame non-trivial.
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1).with_world(
        WorldSettings(gravity_m_s2=(0.0, 0.0, 0.0), timestep_s=STEP_S)
    )
    session = RuntimeSession.create(loaded, scene, "mujoco")
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    # Dense Jacobians let the test read efc_J directly. This changes storage,
    # not physics.
    adapter._require_compiled().model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.0)
    assert any(isinstance(event, DockCommitted) for event in events)
    return session, adapter


def _load_both(adapter: MuJoCoBackendAdapter) -> None:
    adapter.apply_module_wrench(CUBE_1, force_n=(0.7, 3.0, -1.2), torque_nm=(0.04, -0.03, 0.05))
    adapter.apply_module_wrench(CUBE_0, force_n=(-0.2, 0.5, 0.4), torque_nm=(-0.02, 0.01, 0.0))


def _solver_generalized_force(
    adapter: MuJoCoBackendAdapter, handle: ConstraintHandle
) -> np.ndarray:
    """Return ``J.T @ f`` over the equality rows that realise ``handle``."""
    model = adapter._require_compiled().model
    _, data = adapter._require_loaded()
    welds = adapter._welds.slot_for(handle)
    hinges = adapter._hinges.slot_for(handle)
    if welds is not None:
        equalities = {welds.equality_id}
    else:
        assert hinges is not None, f"no weld or hinge slot realises {handle}"
        equalities = set(hinges.equality_ids)
    nefc, nv = data.nefc, model.nv
    jacobian = np.asarray(data.efc_J[: nefc * nv]).reshape(nefc, nv)
    rows = [
        row
        for row in range(nefc)
        if data.efc_type[row] == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        and int(data.efc_id[row]) in equalities
    ]
    assert rows
    return jacobian[rows].T @ data.efc_force[rows]


def _wrench_generalized_force(
    adapter: MuJoCoBackendAdapter, wrench: ConstraintWrench
) -> np.ndarray:
    """Return ``Jp.T @ F + Jr.T @ M`` for the wrench applied to cube 0 at connector A."""
    model = adapter._require_compiled().model
    _, data = adapter._require_loaded()
    body_a, _, _ = adapter._free_joint(CUBE_0)
    point = adapter._connector_world_transform(FRONT_0).translation
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, np.asarray(point), body_a)
    assert wrench.torque_nm is not None
    return jacp.T @ np.asarray(wrench.force_n) + jacr.T @ np.asarray(wrench.torque_nm)


def _module_a_dofs(adapter: MuJoCoBackendAdapter) -> slice:
    _, _, dof = adapter._free_joint(CUBE_0)
    return slice(dof, dof + 6)


@pytest.mark.parametrize("pack_name", ["fixed_pack", "hinge_pack"])
def test_reported_wrench_reproduces_the_solver_force_on_module_a(
    pack_name: str, request: pytest.FixtureRequest
) -> None:
    session, adapter = _docked_session(request.getfixturevalue(pack_name))
    try:
        # Turn the pair so that no frame is world-aligned.
        adapter.set_module_twist(CUBE_1, linear_m_s=(0.0, 0.0, 0.0), angular_rad_s=(0.3, 0.2, 0.4))
        _load_both(adapter)
        session.step(0.05, process_connectors=False)
        model = adapter._require_compiled().model
        _, data = adapter._require_loaded()
        # Rebuild rows and forces for the current state, then convert them with
        # the adapter's own routine, so rows, anchors and frames all agree.
        mujoco.mj_forward(model, data)
        connection = next(iter(session.world.connections.values()))
        handle = connection.constraint_handle
        wrench = adapter._constraint_forces(data)[handle]

        solver = _solver_generalized_force(adapter, handle)
        reported = _wrench_generalized_force(adapter, wrench)
        dofs = _module_a_dofs(adapter)

        assert wrench.force_magnitude_n > 0.1
        assert wrench.torque_magnitude_nm is not None and wrench.torque_magnitude_nm > 1e-3
        np.testing.assert_allclose(reported[dofs], solver[dofs], rtol=1e-9, atol=1e-10)
    finally:
        session.shutdown()


def test_hinge_moment_has_no_component_along_the_hinge_axis(hinge_pack: LoadedRobotPack) -> None:
    session, adapter = _docked_session(hinge_pack)
    try:
        _load_both(adapter)
        session.step(0.05, process_connectors=False)
        connection = next(iter(session.world.connections.values()))
        wrench = adapter.snapshot().constraint_wrenches[connection.constraint_handle]
        frame = adapter._connector_world_transform(FRONT_0)
        local = wrench.resolved_in(Transform(rotation=frame.rotation))
        assert local.torque_nm is not None
        # The connector's y axis is the hinge axis: a hinge carries (almost) no
        # torque about it. What remains is MuJoCo's soft-constraint compliance.
        total = wrench.torque_magnitude_nm or 0.0
        assert total > 1e-3
        assert abs(local.torque_nm[1]) < 1e-2 * total
    finally:
        session.shutdown()


def test_snapshot_force_magnitude_matches_the_wrench(fixed_pack: LoadedRobotPack) -> None:
    session, adapter = _docked_session(fixed_pack)
    try:
        _load_both(adapter)
        session.step(0.01, process_connectors=False)
        snapshot = adapter.snapshot()
        for handle, wrench in snapshot.constraint_wrenches.items():
            assert snapshot.constraint_forces_n[handle] == wrench.force_magnitude_n
        assert adapter.capabilities().supports_constraint_torques
    finally:
        session.shutdown()


@pytest.mark.parametrize("immediate", [True, False])
def test_connection_change_takes_effect_after_the_declared_delay(
    fixed_pack: LoadedRobotPack, immediate: bool
) -> None:
    """A new weld is first solved ``connection_change_delay_substeps`` into the next step."""
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1).with_world(
        WorldSettings(gravity_m_s2=(0.0, 0.0, 0.0), timestep_s=STEP_S)
    )
    adapter = MuJoCoBackendAdapter(immediate_connection_changes=immediate)
    session = RuntimeSession.create(fixed_pack, scene, adapter)
    try:
        delay = adapter.capabilities().connection_change_delay_substeps
        assert delay == (0 if immediate else 1)
        session.step(STEP_S, process_connectors=False)
        session.request_dock(FRONT_0, REAR_1)
        # Committed after this step's physics, i.e. between steps.
        assert any(isinstance(event, DockCommitted) for event in session.step(STEP_S))
        handle = next(iter(session.world.connections.values())).constraint_handle
        _load_both(adapter)
        for _ in range(delay):
            adapter.step(STEP_S)
            assert adapter.snapshot().constraint_wrenches[handle].force_magnitude_n == 0.0
        adapter.step(STEP_S)
        assert adapter.snapshot().constraint_wrenches[handle].force_magnitude_n > 0.0
    finally:
        session.shutdown()


def test_resolved_in_rotates_axes_but_keeps_magnitudes() -> None:
    wrench = ConstraintWrench(force_n=(1.0, 2.0, 3.0), torque_nm=(-0.5, 0.25, 0.75))
    frame = Transform(rotation=quat_from_axis_angle((0.3, -0.4, 0.866), 1.1))
    local = wrench.resolved_in(frame)
    assert local.force_magnitude_n == pytest.approx(wrench.force_magnitude_n)
    assert local.torque_magnitude_nm == pytest.approx(wrench.torque_magnitude_nm)
    back = local.resolved_in(Transform(rotation=quat_conjugate(frame.rotation)))
    assert back.force_n == pytest.approx(wrench.force_n)
    assert back.torque_nm == pytest.approx(wrench.torque_nm)
