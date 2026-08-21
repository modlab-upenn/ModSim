"""MuJoCo backend adapter.

The adapter loads a scene, steps real physics, reports observed state, and
realises fixed runtime connections with reserved MuJoCo weld constraints.
Unsupported physical intents are refused explicitly so ModSim never records a
connection backed by different physics than the Robot Pack requested.
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from modsim.backends.base import (
    BackendCapabilities,
    BackendError,
    BackendHandleRegistry,
    ConnectionOutcome,
    ConnectionRequest,
)
from modsim.core.ids import (
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
    split_connector_instance_id,
)
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, BodyState
from modsim.core.transforms import (
    ZERO_VEC3,
    Quat,
    Transform,
    Vec3,
    quat_conjugate,
    quat_rotate,
)
from modsim.robot_packs.schema import PhysicalConstraintType, RobotPack
from modsim_backend_mujoco.scene import DEFAULT_GRAVITY, CompiledScene, build_scene
from modsim_backend_mujoco.welds import (
    ANCHOR,
    RELPOSE_POSITION,
    RELPOSE_ROTATION,
    RIGID_TORQUE_SCALE,
    TORQUE_SCALE,
    WeldPool,
    WeldPoolExhaustedError,
    body_relative_transform,
)

MUJOCO_BACKEND_NAME = "mujoco"


class MuJoCoBackendAdapter:
    """Rigid-body physics backend built on MuJoCo."""

    __slots__ = (
        "_compiled",
        "_data",
        "_gravity",
        "_ground",
        "_ground_height_m",
        "_scene",
        "_timestep_s",
        "_weld_pool_size",
        "_welds",
    )

    def __init__(
        self,
        *,
        gravity: Vec3 = DEFAULT_GRAVITY,
        timestep_s: float | None = None,
        weld_pool_size: int | None = None,
        ground: bool = False,
        ground_height_m: float = 0.0,
    ) -> None:
        self._gravity = gravity
        self._timestep_s = timestep_s
        self._weld_pool_size = weld_pool_size
        self._ground = ground
        self._ground_height_m = ground_height_m
        self._compiled: CompiledScene | None = None
        self._data: mujoco.MjData | None = None
        self._scene: SceneSpec | None = None
        self._welds = WeldPool()

    # ------------------------------------------------------------------
    # BackendAdapter protocol
    # ------------------------------------------------------------------

    def capabilities(self) -> BackendCapabilities:
        """Report what this adapter can currently do.

        ``supports_constraint_forces`` is still false: the weld holds, but the
        adapter does not yet read equality forces out of the solver, so
        break-force release and connector-load metrics stay dormant rather than
        reporting zeros that look like real measurements.
        """
        return BackendCapabilities(
            name=MUJOCO_BACKEND_NAME,
            supports_runtime_constraints=True,
            supports_constraint_removal=True,
            supports_constraint_forces=False,
            supports_contact_forces=True,
            supports_module_pose_write=True,
            # The imported model may contain joints, but ModSim does not yet
            # expose a joint-command API or populate actuator handles.
            supports_joint_commands=False,
            supports_external_viewer=True,
        )

    def load(
        self,
        pack: RobotPack,
        scene: SceneSpec,
        *,
        root: Path | None = None,
    ) -> BackendHandleRegistry:
        """Compile every placement into one MuJoCo model."""
        if root is None:
            raise BackendError(
                "the MuJoCo backend reads mechanical assets from disk and needs the "
                "Robot Pack directory; pass a LoadedRobotPack or an explicit root"
            )
        compiled = build_scene(
            pack,
            scene,
            Path(root),
            gravity=self._gravity,
            timestep_s=self._timestep_s,
            weld_pool_size=self._weld_pool_size,
            ground=self._ground,
            ground_height_m=self._ground_height_m,
        )
        self._compiled = compiled
        self._scene = scene
        self._data = mujoco.MjData(compiled.model)
        self._welds = WeldPool.over(compiled.weld_pool)
        mujoco.mj_forward(compiled.model, self._data)
        return compiled.handles

    def step(self, dt_s: float) -> None:
        """Advance physics by approximately ``dt_s`` seconds.

        MuJoCo's integrator step is a model property, so the requested interval
        is covered by whole solver steps rather than by changing the timestep
        mid-run. ``snapshot().time_s`` reports the time actually reached.
        """
        if dt_s < 0.0:
            raise BackendError("cannot step backwards")
        model, data = self._require_loaded()
        if dt_s == 0.0:
            mujoco.mj_forward(model, data)
            return
        steps = max(1, math.floor(dt_s / model.opt.timestep + 0.5))
        for _ in range(steps):
            mujoco.mj_step(model, data)

    def snapshot(self) -> BackendStateSnapshot:
        """Read link poses, world twists, and connector site frames."""
        model, data = self._require_loaded()
        compiled = self._require_compiled()

        link_states: dict[ModuleInstanceId, dict[str, BodyState]] = {}
        velocity = np.zeros(6)
        for (module_id, link), body_id in compiled.body_ids.items():
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, velocity, 0)
            link_states.setdefault(module_id, {})[link] = BodyState(
                pose=Transform(
                    translation=_vec3(data.xpos[body_id]),
                    rotation=_quat(data.xquat[body_id]),
                ),
                # mj_objectVelocity with flg_local=0 reports world-frame motion
                # at the body's own origin, which is what BodyState.velocity_at
                # expects before it adds the connector lever arm.
                angular_velocity_rad_s=_vec3(velocity[0:3]),
                linear_velocity_m_s=_vec3(velocity[3:6]),
            )

        connector_frames: dict[ModuleInstanceId, dict[str, Transform]] = {}
        for instance, site_id in compiled.site_ids.items():
            module_id, connector_id = _split(instance)
            connector_frames.setdefault(module_id, {})[connector_id] = Transform(
                translation=_vec3(data.site_xpos[site_id]),
                rotation=_mat_to_quat(data.site_xmat[site_id]),
            )

        return BackendStateSnapshot(
            time_s=float(data.time),
            link_states=link_states,
            connector_frames=connector_frames,
        )

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Claim a reserved weld slot and activate it for this connector pair.

        The pose ModSim commits is between connector frames; a weld constrains
        bodies. The conversion is done in :mod:`modsim_backend_mujoco.welds` so
        it can be checked without a simulation.
        """
        if request.physical_connection.constraint is not PhysicalConstraintType.FIXED:
            return ConnectionOutcome.refused(
                "the MuJoCo backend currently supports only fixed physical "
                f"connections; received '{request.physical_connection.constraint.value}'"
            )

        model, data = self._require_loaded()
        try:
            body_a = self._body_id(request.module_a, request.link_a)
            body_b = self._body_id(request.module_b, request.link_b)
        except BackendError as error:
            return ConnectionOutcome.refused(str(error))
        if body_a == body_b:
            return ConnectionOutcome.refused(
                "MuJoCo rejects a weld whose two operands are the same body"
            )

        handle = ConstraintHandle(f"weld:{request.connection_id}")
        try:
            slot = self._welds.claim(handle)
        except WeldPoolExhaustedError as error:
            return ConnectionOutcome.refused(str(error))

        if request.snap_to_nominal:
            self._snap_to_nominal(request, body_a, body_b)

        relative = body_relative_transform(
            request.connector_a_local,
            request.connector_b_local,
            request.relative_transform,
        )
        equality = slot.equality_id
        model.eq_type[equality] = mujoco.mjtEq.mjEQ_WELD
        model.eq_objtype[equality] = mujoco.mjtObj.mjOBJ_BODY
        model.eq_obj1id[equality] = body_a
        model.eq_obj2id[equality] = body_b
        model.eq_data[equality, ANCHOR] = 0.0
        model.eq_data[equality, RELPOSE_POSITION] = relative.translation
        model.eq_data[equality, RELPOSE_ROTATION] = relative.rotation
        model.eq_data[equality, TORQUE_SCALE] = RIGID_TORQUE_SCALE
        data.eq_active[equality] = 1
        mujoco.mj_forward(model, data)
        return ConnectionOutcome.accepted(handle)

    def remove_physical_connection(self, handle: ConstraintHandle) -> bool:
        """Deactivate the weld and return its slot to the pool."""
        model, data = self._require_loaded()
        slot = self._welds.release(handle)
        if slot is None:
            return False
        data.eq_active[slot.equality_id] = 0
        mujoco.mj_forward(model, data)
        return True

    def shutdown(self) -> None:
        """Release the compiled model and its data."""
        self._welds.clear()
        self._compiled = None
        self._data = None
        self._scene = None

    # ------------------------------------------------------------------
    # engine access
    # ------------------------------------------------------------------

    @property
    def model(self) -> mujoco.MjModel:
        """Return the compiled MuJoCo model."""
        return self._require_compiled().model

    @property
    def data(self) -> mujoco.MjData:
        """Return the live MuJoCo data."""
        return self._require_loaded()[1]

    @property
    def weld_pool(self) -> WeldPool:
        """Return the allocator over the reserved weld constraint slots."""
        return self._welds

    @property
    def time_s(self) -> float:
        """Return the current simulation time."""
        return float(self._require_loaded()[1].time)

    def set_module_pose(self, module_id: ModuleInstanceId, pose: Transform) -> None:
        """Teleport one module's root body and clear its velocity.

        Velocities are zeroed because writing a pose mid-simulation otherwise
        injects energy that the solver then has to absorb.
        """
        model, data = self._require_loaded()
        _, qpos_address, _ = self._free_joint(module_id)
        data.qpos[qpos_address : qpos_address + 3] = pose.translation
        data.qpos[qpos_address + 3 : qpos_address + 7] = pose.rotation
        self._clear_module_velocities(module_id, model, data)
        mujoco.mj_forward(model, data)

    def set_module_twist(
        self,
        module_id: ModuleInstanceId,
        *,
        linear_m_s: Vec3 = ZERO_VEC3,
        angular_rad_s: Vec3 = ZERO_VEC3,
    ) -> None:
        """Set one module's world-frame linear and angular velocity.

        A free joint stores linear velocity in the world frame but angular
        velocity in the *body* frame, so the angular argument is rotated into
        the body frame here. Callers work in world coordinates throughout
        ModSim and should not have to know that.
        """
        model, data = self._require_loaded()
        body_id, _, dof_address = self._free_joint(module_id)
        rotation = _quat(data.xquat[body_id])
        data.qvel[dof_address : dof_address + 3] = linear_m_s
        data.qvel[dof_address + 3 : dof_address + 6] = quat_rotate(
            quat_conjugate(rotation), angular_rad_s
        )
        mujoco.mj_forward(model, data)

    def apply_module_wrench(
        self,
        module_id: ModuleInstanceId,
        *,
        force_n: Vec3 = ZERO_VEC3,
        torque_nm: Vec3 = ZERO_VEC3,
    ) -> None:
        """Apply a persistent world-frame wrench to one module's root body.

        Unlike a twist, a wrench survives contact and gravity, which is what a
        scenario needs to drive a module against resistance. It persists until
        changed or cleared.
        """
        _, data = self._require_loaded()
        body_id, _, _ = self._free_joint(module_id)
        data.xfrc_applied[body_id, 0:3] = force_n
        data.xfrc_applied[body_id, 3:6] = torque_nm

    def clear_wrenches(self) -> None:
        """Remove every applied wrench."""
        _, data = self._require_loaded()
        data.xfrc_applied[:] = 0.0

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------

    def _require_compiled(self) -> CompiledScene:
        if self._compiled is None:
            raise BackendError("the MuJoCo backend has no scene loaded")
        return self._compiled

    def _require_loaded(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        compiled = self._require_compiled()
        if self._data is None:
            raise BackendError("the MuJoCo backend has no simulation data")
        return compiled.model, self._data

    def _body_id(self, module_id: ModuleInstanceId, link: str) -> int:
        compiled = self._require_compiled()
        try:
            return compiled.body_ids[(module_id, link)]
        except KeyError as error:
            raise BackendError(f"unknown body '{module_id}' / '{link}'") from error

    def _snap_to_nominal(
        self,
        request: ConnectionRequest,
        body_a: int,
        body_b: int,
    ) -> None:
        """Move module B so the requested connector poses coincide exactly.

        Velocity is cleared along with the pose. Writing a pose mid-simulation
        without doing so leaves the old velocity attached to a body that has
        just teleported, which injects energy the solver then has to absorb.
        """
        model, data = self._require_loaded()
        pose_a = Transform(translation=_vec3(data.xpos[body_a]), rotation=_quat(data.xquat[body_a]))
        pose_b = Transform(translation=_vec3(data.xpos[body_b]), rotation=_quat(data.xquat[body_b]))
        root_body_b, _, _ = self._free_joint(request.module_b)
        root_pose_b = Transform(
            translation=_vec3(data.xpos[root_body_b]),
            rotation=_quat(data.xquat[root_body_b]),
        )
        root_to_body_b = pose_b.relative_to(root_pose_b)
        target_body_b = pose_a.compose(
            body_relative_transform(
                request.connector_a_local,
                request.connector_b_local,
                request.relative_transform,
            )
        )
        # The constrained connector may live on an articulated child body.
        # Preserve that body's measured zero/current-configuration transform
        # relative to the module root when repositioning the whole module.
        target_root_b = target_body_b.compose(root_to_body_b.inverse())
        self.set_module_pose(request.module_b, target_root_b)
        mujoco.mj_forward(model, data)

    def _free_joint(self, module_id: ModuleInstanceId) -> tuple[int, int, int]:
        """Return ``(body id, qpos address, dof address)`` for a module's free joint."""
        model, _ = self._require_loaded()
        compiled = self._require_compiled()
        for (candidate, _), body_id in compiled.body_ids.items():
            if candidate != module_id:
                continue
            joint_address = int(model.body_jntadr[body_id])
            if joint_address < 0:
                continue
            if model.jnt_type[joint_address] != mujoco.mjtJoint.mjJNT_FREE:
                continue
            return (
                body_id,
                int(model.jnt_qposadr[joint_address]),
                int(model.jnt_dofadr[joint_address]),
            )
        raise BackendError(f"module '{module_id}' has no free joint to drive")

    def _clear_module_velocities(
        self,
        module_id: ModuleInstanceId,
        model: mujoco.MjModel,
        data: mujoco.MjData,
    ) -> None:
        """Zero root and articulated-joint velocities for one module instance."""
        compiled = self._require_compiled()
        dof_counts = {
            mujoco.mjtJoint.mjJNT_FREE: 6,
            mujoco.mjtJoint.mjJNT_BALL: 3,
            mujoco.mjtJoint.mjJNT_SLIDE: 1,
            mujoco.mjtJoint.mjJNT_HINGE: 1,
        }
        for (candidate, _), body_id in compiled.body_ids.items():
            if candidate != module_id:
                continue
            joint_start = int(model.body_jntadr[body_id])
            joint_count = int(model.body_jntnum[body_id])
            for joint_id in range(joint_start, joint_start + joint_count):
                joint_type = mujoco.mjtJoint(model.jnt_type[joint_id])
                dof_start = int(model.jnt_dofadr[joint_id])
                dof_count = dof_counts[joint_type]
                data.qvel[dof_start : dof_start + dof_count] = 0.0


def _vec3(values: object) -> Vec3:
    array = np.asarray(values, dtype=float)
    return (float(array[0]), float(array[1]), float(array[2]))


def _quat(values: object) -> Quat:
    array = np.asarray(values, dtype=float)
    return (float(array[0]), float(array[1]), float(array[2]), float(array[3]))


def _mat_to_quat(matrix: object) -> Quat:
    result = np.zeros(4)
    mujoco.mju_mat2Quat(result, np.asarray(matrix, dtype=float).reshape(9))
    return _quat(result)


def _split(instance: ConnectorInstanceId) -> tuple[ModuleInstanceId, str]:
    return split_connector_instance_id(instance)
