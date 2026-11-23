"""MuJoCo backend adapter.

The adapter loads a scene, steps real physics, reports observed state, and
realises runtime connections with reserved MuJoCo equality constraints.
Fixed and hinge intents are realised by separate reserved equality pools.
Other physical intents are refused explicitly so ModSim never records a
connection backed by different physics than the Robot Pack requested.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from modsim.backends.base import (
    BackendCapabilities,
    BackendError,
    BackendHandleRegistry,
    ConnectionOutcome,
    ConnectionRequest,
)
from modsim.core.entities import JointCommand
from modsim.core.ids import (
    ConnectorInstanceId,
    ConstraintHandle,
    JointInstanceId,
    ModuleInstanceId,
    split_connector_instance_id,
    split_joint_instance_id,
)
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, ConstraintWrench
from modsim.core.transforms import (
    ZERO_VEC3,
    Quat,
    Transform,
    Vec3,
    quat_conjugate,
    quat_rotate,
)
from modsim.robot_packs.schema import ControlMode, PhysicalConstraintType, RobotPack
from modsim_backend_mujoco.hinges import (
    CONNECT_ANCHOR_BODY_A,
    CONNECT_ANCHOR_BODY_B,
    HingePool,
    HingePoolExhaustedError,
    hinge_anchor_pairs,
)
from modsim_backend_mujoco.sampling import MujocoSample, SampleLayout
from modsim_backend_mujoco.scene import DEFAULT_GRAVITY, GROUND_GEOM, CompiledScene, build_scene
from modsim_backend_mujoco.welds import (
    ANCHOR,
    RELPOSE_POSITION,
    RELPOSE_ROTATION,
    RIGID_TORQUE_SCALE,
    TORQUE_SCALE,
    ContactExclusionPool,
    ContactExclusionPoolExhaustedError,
    WeldPool,
    WeldPoolExhaustedError,
    body_relative_transform,
)

MUJOCO_BACKEND_NAME = "mujoco"


class MuJoCoBackendAdapter:
    """Rigid-body physics backend built on MuJoCo."""

    __slots__ = (
        "_compiled",
        "_constraint_time_constant_s",
        "_contact_exclusions",
        "_data",
        "_forces",
        "_gravity",
        "_ground",
        "_ground_height_m",
        "_hinge_pool_size",
        "_hinges",
        "_immediate_connection_changes",
        "_rows_stale",
        "_sample_layout",
        "_scene",
        "_timestep_s",
        "_weld_pool_size",
        "_welds",
        "_wrench_references",
    )

    def __init__(
        self,
        *,
        gravity: Vec3 | None = None,
        timestep_s: float | None = None,
        weld_pool_size: int | None = None,
        hinge_pool_size: int | None = None,
        ground: bool | None = None,
        ground_height_m: float | None = None,
        constraint_time_constant_s: float | None = None,
        immediate_connection_changes: bool = False,
    ) -> None:
        """Create an unloaded adapter.

        World settings and pool sizes normally come from the scene
        (``SceneSpec.world`` and ``SceneSpec.connection_budget``). The keyword
        arguments here are engine-level overrides: any value other than ``None``
        wins over the scene. ``constraint_time_constant_s`` has no scene
        equivalent because it is a MuJoCo solver parameter.

        ``immediate_connection_changes`` decides when a dock or undock acts.
        Split stepping builds the next solve's constraint rows at the end of
        each ``step``, before ModSim commits connection changes. When False (the
        default), no rebuild happens and a change acts one internal step later;
        the capabilities declare that delay, so per-event cost is independent
        of the world size and controllers wait on
        ``RuntimeSession.connection_change_pending`` where timing matters. When
        True, the first internal step after a change rebuilds those rows with
        ``mj_step1``, so the change acts at once, as with ``mj_step``. That costs
        one extra ``mj_step1`` per ``step`` call following a change, however many
        changes there were.
        """
        if constraint_time_constant_s is not None and (
            not math.isfinite(constraint_time_constant_s) or constraint_time_constant_s <= 0
        ):
            raise ValueError("constraint_time_constant_s must be finite and positive")
        self._constraint_time_constant_s = constraint_time_constant_s
        self._gravity = gravity
        self._timestep_s = timestep_s
        self._weld_pool_size = weld_pool_size
        self._hinge_pool_size = hinge_pool_size
        self._ground = ground
        self._ground_height_m = ground_height_m
        self._compiled: CompiledScene | None = None
        self._sample_layout: SampleLayout | None = None
        self._forces: dict[ConstraintHandle, ConstraintWrench] = {}
        self._wrench_references: dict[ConstraintHandle, tuple[int, np.ndarray]] = {}
        self._immediate_connection_changes = immediate_connection_changes
        self._rows_stale = False
        self._data: mujoco.MjData | None = None
        self._scene: SceneSpec | None = None
        self._welds = WeldPool()
        self._hinges = HingePool()
        self._contact_exclusions = ContactExclusionPool()

    # ------------------------------------------------------------------
    # BackendAdapter protocol
    # ------------------------------------------------------------------

    def capabilities(self) -> BackendCapabilities:
        """Report what this adapter can do.

        Full wrenches are reported for both fixed welds and two-point hinges,
        which enables the core's backend-independent overload path. Split
        stepping fixes each internal step's constraint set before the commands
        between steps are applied, so by default a connection change acts one
        internal step into the next ``step``. With ``immediate_connection_changes``
        the adapter rebuilds that set after a change and declares no delay.
        """
        return BackendCapabilities(
            name=MUJOCO_BACKEND_NAME,
            supports_runtime_constraints=True,
            supports_constraint_removal=True,
            supports_constraint_forces=True,
            supports_constraint_torques=True,
            supports_contact_forces=True,
            supports_module_pose_write=True,
            supports_joint_commands=True,
            supported_joint_control_modes=frozenset({ControlMode.EFFORT}),
            supports_external_viewer=True,
            supports_gravity=True,
            supports_ground_plane=True,
            supports_contact_dynamics=True,
            supported_constraint_types=frozenset(
                {PhysicalConstraintType.FIXED, PhysicalConstraintType.HINGE}
            ),
            connection_change_delay_substeps=0 if self._immediate_connection_changes else 1,
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
        world = scene.world
        budget = scene.connection_budget
        compiled = build_scene(
            pack,
            scene,
            Path(root),
            gravity=_first(self._gravity, world.gravity_m_s2, DEFAULT_GRAVITY),
            timestep_s=_first(self._timestep_s, world.timestep_s),
            weld_pool_size=_first(self._weld_pool_size, budget.fixed),
            hinge_pool_size=_first(self._hinge_pool_size, budget.hinge),
            ground=bool(_first(self._ground, world.ground)),
            ground_height_m=float(_first(self._ground_height_m, world.ground_height_m)),
        )
        self._compiled = compiled
        self._sample_layout = SampleLayout.build(
            compiled.model,
            compiled.body_ids,
            compiled.site_ids,
            compiled.joint_ids,
            compiled.actuator_ids,
        )
        if self._constraint_time_constant_s is not None:
            if self._constraint_time_constant_s < 2 * compiled.model.opt.timestep:
                raise BackendError("constraint_time_constant_s must be at least twice the timestep")
            # Configure only ModSim's reserved runtime constraints. Authored
            # equalities and every other scene keep their original solver settings.
            for equality in (
                *compiled.weld_pool,
                *(e for pair in compiled.hinge_pool for e in pair),
            ):
                compiled.model.eq_solref[equality] = (self._constraint_time_constant_s, 1.0)
        self._scene = scene
        self._data = mujoco.MjData(compiled.model)
        self._welds = WeldPool.over(compiled.weld_pool)
        self._hinges = HingePool.over(compiled.hinge_pool)
        self._contact_exclusions = ContactExclusionPool.over(compiled.contact_exclusion_pool)
        self._wrench_references = {}
        mujoco.mj_forward(compiled.model, self._data)
        self._after_forward(compiled.model, self._data)
        return compiled.handles

    def step(self, dt_s: float) -> None:
        """Advance physics by approximately ``dt_s`` seconds.

        MuJoCo's integrator step is a model property, so the requested interval
        is covered by whole solver steps rather than by changing the timestep
        mid-run. ``snapshot().time_s`` reports the time actually reached.

        Each solver step runs phase-shifted, as ``mj_step2`` (actuation,
        constraint solve, integration) followed by ``mj_step1`` (kinematics,
        collision, and constraint construction for the *new* state). For the
        Euler and implicit integrators this is the same sequence of MuJoCo
        calls as ``mj_step``; it only moves where ModSim observes and acts.
        Between steps every pose, twist, and site frame therefore describes
        the state at ``data.time``, so snapshots never lag their timestamp and
        a topology change needs no kinematics refresh of the whole model. A
        constraint switched on or off between steps is picked up by the next
        ``mj_step1``, so it acts one internal step later (the declared delay);
        with ``immediate_connection_changes`` the first internal step rebuilds
        the rows first, so the change acts at once. Constraint forces
        are captured after the final ``mj_step2``, before ``mj_step1``
        rebuilds the constraint rows.
        """
        if dt_s < 0.0:
            raise BackendError("cannot step backwards")
        model, data = self._require_loaded()
        if dt_s == 0.0:
            mujoco.mj_forward(model, data)
            self._after_forward(model, data)
            return
        steps = max(1, math.floor(dt_s / model.opt.timestep + 0.5))
        anisotropic = bool(self._require_compiled().anisotropic_ground_geoms)
        if self._rows_stale and self._immediate_connection_changes:
            # A connection changed after the rows for this step were built:
            # rebuild them (and the contacts, since exclusions changed too).
            mujoco.mj_step1(model, data)
            if anisotropic:
                self._align_anisotropic_ground_contacts(model, data)
        self._rows_stale = False
        for index in range(steps):
            mujoco.mj_step2(model, data)
            if index == steps - 1:
                self._forces = self._constraint_forces(data)
            mujoco.mj_step1(model, data)
            if anisotropic:
                # MuJoCo's plane collision frame is world-aligned. Rotate tire
                # contacts' tangent onto the axle before the next mj_step2
                # consumes anisotropic pair friction.
                self._align_anisotropic_ground_contacts(model, data)

    def _after_forward(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Restore the between-steps invariant after a full ``mj_forward``."""
        self._rows_stale = False
        self._forces = self._constraint_forces(data)
        if self._require_compiled().anisotropic_ground_geoms:
            self._align_anisotropic_ground_contacts(model, data)

    def snapshot(self) -> BackendStateSnapshot:
        """Report link poses, world twists, joint states, and connector site frames.

        The snapshot holds one bulk copy of the needed ``MjData`` rows; per-body
        and per-site values are materialised lazily on lookup (see
        ``modsim_backend_mujoco.sampling``). Constraint forces are those of the
        last solve, captured by ``step`` (see there).
        """
        _, data = self._require_loaded()
        layout = self._sample_layout
        if layout is None:  # pragma: no cover - set together with _compiled in load()
            raise BackendError("the MuJoCo backend has no sample layout; call load() first")
        sample = MujocoSample(layout, data)
        return BackendStateSnapshot(
            time_s=float(data.time),
            link_states=sample.link_states(),
            joint_states=sample.joint_states(),
            connector_frames=sample.connector_frames(),
            constraint_wrenches=self._active_wrenches(),
        )

    def _active_wrenches(self) -> dict[ConstraintHandle, ConstraintWrench]:
        """Wrenches of the last solve, for connections that still exist."""
        return {
            handle: wrench
            for handle, wrench in self._forces.items()
            if self._welds.slot_for(handle) is not None or self._hinges.slot_for(handle) is not None
        }

    def set_joint_commands(self, commands: tuple[JointCommand, ...]) -> None:
        """Set a validated batch of persistent effort commands atomically."""
        _, data = self._require_loaded()
        compiled = self._require_compiled()
        updates: list[tuple[int, float]] = []
        for command in commands:
            if command.mode is not ControlMode.EFFORT:
                raise BackendError(
                    "the MuJoCo backend currently supports only effort joint commands; "
                    f"received '{command.mode.value}' for '{command.joint}'"
                )
            module_id, joint_id = _split_joint(command.joint)
            try:
                actuator_id = compiled.actuator_ids[(module_id, joint_id)]
            except KeyError as error:
                raise BackendError(
                    f"joint '{command.joint}' has no MuJoCo effort actuator"
                ) from error
            updates.append((actuator_id, command.value))

        # Resolve the entire batch before mutating ctrl so an invalid later
        # target cannot leave an earlier one applied.
        for actuator_id, value in updates:
            data.ctrl[actuator_id] = value

    def clear_joint_commands(
        self,
        joints: tuple[JointInstanceId, ...] | None = None,
    ) -> None:
        """Set selected persistent effort targets, or every target, to zero."""
        _, data = self._require_loaded()
        compiled = self._require_compiled()
        if joints is None:
            actuator_ids = tuple(compiled.actuator_ids.values())
        else:
            resolved: list[int] = []
            for joint in joints:
                module_id, joint_id = _split_joint(joint)
                try:
                    resolved.append(compiled.actuator_ids[(module_id, joint_id)])
                except KeyError as error:
                    raise BackendError(f"joint '{joint}' has no MuJoCo effort actuator") from error
            actuator_ids = tuple(resolved)

        for actuator_id in actuator_ids:
            data.ctrl[actuator_id] = 0.0

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Realise a fixed weld or two-point hinge for this connector pair.

        The constraint kind selects a dedicated precompiled pool. Unsupported
        kinds are refused instead of being approximated by a different joint.
        """
        constraint = request.physical_connection.constraint
        if constraint is PhysicalConstraintType.FIXED:
            outcome = self._create_weld(request)
        elif constraint is PhysicalConstraintType.HINGE:
            outcome = self._create_hinge(request)
        else:
            outcome = None
        if outcome is not None:
            self._rows_stale |= outcome.success
            return outcome
        return ConnectionOutcome.refused(
            "the MuJoCo backend supports fixed and hinge physical connections; "
            f"received '{constraint.value}'"
        )

    def _create_weld(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Claim and configure one reserved weld slot."""

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
        try:
            exclusion = self._contact_exclusions.claim(handle, body_a, body_b)
        except (ContactExclusionPoolExhaustedError, ValueError) as error:
            self._welds.release(handle)
            return ConnectionOutcome.refused(str(error))

        equality = slot.equality_id
        try:
            if request.snap_to_nominal:
                self._snap_to_nominal(request, body_a, body_b)

            relative = body_relative_transform(
                request.connector_a_local,
                request.connector_b_local,
                request.relative_transform,
            )
            model.eq_type[equality] = mujoco.mjtEq.mjEQ_WELD
            model.eq_objtype[equality] = mujoco.mjtObj.mjOBJ_BODY
            model.eq_obj1id[equality] = body_a
            model.eq_obj2id[equality] = body_b
            model.eq_data[equality, ANCHOR] = 0.0
            model.eq_data[equality, RELPOSE_POSITION] = relative.translation
            model.eq_data[equality, RELPOSE_ROTATION] = relative.rotation
            model.eq_data[equality, TORQUE_SCALE] = RIGID_TORQUE_SCALE
            _insert_exclusion(model, exclusion.signature)
            self._wrench_references[handle] = _wrench_reference(body_a, request)
            data.eq_active[equality] = 1
        except Exception:
            data.eq_active[equality] = 0
            self._wrench_references.pop(handle, None)
            self._contact_exclusions.release(handle)
            self._welds.release(handle)
            self._sync_contact_exclusions(model)
            raise
        return ConnectionOutcome.accepted(handle)

    def _create_hinge(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Claim two connect equalities and place them on the requested line."""
        model, data = self._require_loaded()
        hinge_spec = request.physical_connection.hinge
        if hinge_spec is None:  # Defensive: validated Robot Packs cannot reach this branch.
            return ConnectionOutcome.refused("a hinge connection requires hinge geometry")
        try:
            body_a = self._body_id(request.module_a, request.link_a)
            body_b = self._body_id(request.module_b, request.link_b)
        except BackendError as error:
            return ConnectionOutcome.refused(str(error))
        if body_a == body_b:
            return ConnectionOutcome.refused(
                "MuJoCo rejects a hinge whose two operands are the same body"
            )

        handle = ConstraintHandle(f"hinge:{request.connection_id}")
        try:
            slot = self._hinges.claim(handle)
        except HingePoolExhaustedError as error:
            return ConnectionOutcome.refused(str(error))

        try:
            if request.snap_to_nominal:
                self._snap_to_nominal(request, body_a, body_b)

            anchors = hinge_anchor_pairs(
                self._body_world_transform(body_a),
                self._connector_world_transform(request.connector_a),
                self._body_world_transform(body_b),
                self._connector_world_transform(request.connector_b),
                hinge_spec.axis,
                hinge_spec.anchor_separation_m,
            )
            for equality, anchor in zip(slot.equality_ids, anchors, strict=True):
                model.eq_type[equality] = mujoco.mjtEq.mjEQ_CONNECT
                model.eq_objtype[equality] = mujoco.mjtObj.mjOBJ_BODY
                model.eq_obj1id[equality] = body_a
                model.eq_obj2id[equality] = body_b
                model.eq_data[equality, :] = 0.0
                model.eq_data[equality, CONNECT_ANCHOR_BODY_A] = anchor.body_a
                model.eq_data[equality, CONNECT_ANCHOR_BODY_B] = anchor.body_b
            self._wrench_references[handle] = _wrench_reference(body_a, request)
            for equality in slot.equality_ids:
                data.eq_active[equality] = 1
        except Exception:
            for equality in slot.equality_ids:
                data.eq_active[equality] = 0
            self._wrench_references.pop(handle, None)
            self._hinges.release(handle)
            raise
        return ConnectionOutcome.accepted(handle)

    def remove_physical_connection(self, handle: ConstraintHandle) -> bool:
        """Deactivate a weld or hinge and return its slot to its pool."""
        model, data = self._require_loaded()
        self._wrench_references.pop(handle, None)
        self._rows_stale = True
        slot = self._welds.release(handle)
        if slot is not None:
            data.eq_active[slot.equality_id] = 0
            signature = self._contact_exclusions.signature_of(handle)
            self._contact_exclusions.release(handle)
            if signature is not None:
                _remove_exclusion(model, signature)
            return True

        hinge = self._hinges.release(handle)
        if hinge is None:
            return False
        for equality in hinge.equality_ids:
            data.eq_active[equality] = 0
        return True

    def shutdown(self) -> None:
        """Release the compiled model and its data."""
        self._welds.clear()
        self._hinges.clear()
        self._contact_exclusions.clear()
        self._compiled = None
        self._sample_layout = None
        self._forces = {}
        self._wrench_references = {}
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
    def hinge_pool(self) -> HingePool:
        """Return the allocator over reserved pairs of point constraints."""
        return self._hinges

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
        self._after_forward(model, data)

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
        self._after_forward(model, data)

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

    def _body_world_transform(self, body_id: int) -> Transform:
        """Return one MuJoCo body's current world transform."""
        _, data = self._require_loaded()
        return Transform(
            translation=_vec3(data.xpos[body_id]),
            rotation=_quat(data.xquat[body_id]),
        )

    def _connector_world_transform(self, connector: ConnectorInstanceId) -> Transform:
        """Return the current world transform of a materialised connector site."""
        compiled = self._require_compiled()
        _, data = self._require_loaded()
        try:
            site_id = compiled.site_ids[connector]
        except KeyError as error:
            raise BackendError(f"unknown connector site '{connector}'") from error
        return Transform(
            translation=_vec3(data.site_xpos[site_id]),
            rotation=_mat_to_quat(data.site_xmat[site_id]),
        )

    def _constraint_forces(
        self,
        data: mujoco.MjData,
    ) -> dict[ConstraintHandle, ConstraintWrench]:
        """Return the wrench of every active connection from the last solve.

        Must be called while ``data`` still holds the kinematics the solved
        constraint rows were built from (directly after ``mj_step2`` or
        ``mj_forward``), so the rows, anchors and frames agree.
        """
        model = self._require_compiled().model
        wrenches: dict[ConstraintHandle, ConstraintWrench] = {}
        rows = _EqualityForceRows(data)
        for slot in self._welds.slots:
            if slot.handle is not None and data.eq_active[slot.equality_id]:
                wrenches[slot.handle] = _weld_wrench(
                    model, data, rows, slot.equality_id, self._wrench_references.get(slot.handle)
                )
        for slot in self._hinges.slots:
            if slot.handle is not None and all(
                data.eq_active[equality] for equality in slot.equality_ids
            ):
                wrenches[slot.handle] = _hinge_wrench(
                    model, data, rows, slot.equality_ids, self._wrench_references.get(slot.handle)
                )
        return wrenches

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
        _, data = self._require_loaded()
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
        # set_module_pose already runs mj_forward on the new pose.
        self.set_module_pose(request.module_b, target_root_b)

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

    def _sync_contact_exclusions(self, model: mujoco.MjModel) -> None:
        """Publish authored and occupied exclusion signatures in search order."""
        compiled = self._require_compiled()
        active = self._contact_exclusions.active_signatures
        inactive_count = self._contact_exclusions.capacity - len(active)
        signatures = sorted(
            (*compiled.static_contact_exclusions, *(0 for _ in range(inactive_count)), *active)
        )
        if len(signatures) != model.nexclude:
            raise BackendError("MuJoCo contact-exclusion pool is inconsistent with the model")
        model.exclude_signature[:] = signatures

    def _align_anisotropic_ground_contacts(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
    ) -> None:
        """Align tire pair friction with each wheel's current world-frame axle."""
        compiled = self._require_compiled()
        tire_geoms = set(compiled.anisotropic_ground_geoms)
        ground_id = model.geom(GROUND_GEOM).id
        for contact in data.contact:
            first, second = int(contact.geom[0]), int(contact.geom[1])
            if first == ground_id and second in tire_geoms:
                tire = second
            elif second == ground_id and first in tire_geoms:
                tire = first
            else:
                continue

            frame = np.asarray(contact.frame).reshape(3, 3)
            normal = frame[0]
            geom_rotation = data.geom_xmat[tire].reshape(3, 3)
            axle = geom_rotation[:, 2]
            tangent = axle - normal * float(np.dot(axle, normal))
            magnitude = float(np.linalg.norm(tangent))
            if magnitude <= 1e-12:
                continue
            tangent /= magnitude
            frame[1] = tangent
            frame[2] = np.cross(normal, tangent)


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


class _EqualityForceRows:
    """Solver rows of every equality constraint, grouped once per snapshot.

    ``efc_id`` identifies the equality that produced each scalar solver row;
    filtering by both constraint kind and equality id avoids mixing in contact,
    friction, or limit forces. Grouping all rows in one pass (a stable sort, so
    each group keeps constraint order) replaces a full scan per connection,
    which grew quadratically with the number of active welds. The rows, and so
    the magnitudes computed from them, are unchanged.
    """

    __slots__ = ("_bounds", "_forces")

    def __init__(self, data: mujoco.MjData) -> None:
        count = data.nefc
        self._bounds: dict[int, tuple[int, int]] = {}
        if count == 0:
            self._forces = np.empty(0)
            return
        kinds = np.asarray(data.efc_type[:count])
        equality = kinds == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
        identifiers = np.asarray(data.efc_id[:count])[equality]
        forces = np.asarray(data.efc_force[:count])[equality]
        order = np.argsort(identifiers, kind="stable")
        identifiers = identifiers[order]
        self._forces = forces[order]
        if len(identifiers) == 0:
            return
        unique, starts = np.unique(identifiers, return_index=True)
        ends = [*starts[1:].tolist(), len(identifiers)]
        for equality_id, start, end in zip(unique.tolist(), starts.tolist(), ends, strict=True):
            self._bounds[equality_id] = (start, end)

    def __call__(self, equality_id: int) -> np.ndarray:
        """Return solver rows belonging to one equality, in constraint order."""
        bounds = self._bounds.get(equality_id)
        if bounds is None:
            return np.empty(0)
        return self._forces[bounds[0] : bounds[1]]


_ZERO_WRENCH = ConstraintWrench()


def _first(*values: object) -> Any:
    """Return the first value that is not ``None`` (engine override, scene, default)."""
    for value in values:
        if value is not None:
            return value
    return None


def _wrench_reference(body_a: int, request: ConnectionRequest) -> tuple[int, np.ndarray]:
    """Return body A and connector A's origin in body A's frame.

    The wrench is reported about connector A's origin, which is fixed in body
    A, so storing it once keeps each per-step evaluation O(1).
    """
    return body_a, np.asarray(request.connector_a_local.translation, dtype=float)


def _reference_point(data: mujoco.MjData, reference: tuple[int, np.ndarray]) -> np.ndarray:
    body, local = reference
    return data.xpos[body] + data.xmat[body].reshape(3, 3) @ local


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    )


def _weld_rotation_map(model: mujoco.MjModel, data: mujoco.MjData, equality: int) -> np.ndarray:
    """Return ``A`` with rotational rows = ``A @ (omega_a - omega_b)`` (world frame).

    This mirrors ``mj_instantiateEquality`` for a body-semantic weld: the rows
    are ``0.5 * torquescale * vec(neg(q_b) * (0, omega) * q_a * relpose)``. The
    moment the rows apply to body A is therefore ``A.T @ rows``.
    """
    body_a = int(model.eq_obj1id[equality])
    body_b = int(model.eq_obj2id[equality])
    quat = _quat_mul(data.xquat[body_a], model.eq_data[equality, RELPOSE_ROTATION])
    neg_b = data.xquat[body_b] * np.array((1.0, -1.0, -1.0, -1.0))
    scale = 0.5 * float(model.eq_data[equality, TORQUE_SCALE])
    columns = []
    for axis in np.eye(3):
        pure = np.array((0.0, axis[0], axis[1], axis[2]))
        columns.append(_quat_mul(_quat_mul(neg_b, pure), quat)[1:] * scale)
    return np.column_stack(columns)


def _weld_wrench(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    rows_of: _EqualityForceRows,
    equality: int,
    reference: tuple[int, np.ndarray] | None,
) -> ConstraintWrench:
    """Return the wrench a weld applies to body A, about connector A's origin.

    A weld contributes three translational rows, a world-frame force acting on
    body A at its anchor, followed by three rotational rows, which encode a
    couple through MuJoCo's weld torque scale and are mapped back to N m.
    """
    rows = rows_of(equality)
    if len(rows) < 6:
        return _ZERO_WRENCH
    force = rows[:3]
    body_a = int(model.eq_obj1id[equality])
    rotation_a = data.xmat[body_a].reshape(3, 3)
    anchor = data.xpos[body_a] + rotation_a @ model.eq_data[equality, RELPOSE_POSITION]
    torque = _weld_rotation_map(model, data, equality).T @ rows[3:6]
    if reference is not None:
        torque = torque + np.cross(anchor - _reference_point(data, reference), force)
    return ConstraintWrench(force_n=_vec3(force), torque_nm=_vec3(torque))


def _hinge_wrench(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    rows_of: _EqualityForceRows,
    equality_ids: tuple[int, int],
    reference: tuple[int, np.ndarray] | None,
) -> ConstraintWrench:
    """Return the net wrench a two-point hinge applies to body A.

    Each connect equality contributes a world-frame point force acting on body
    A at its anchor. Their sum is the net force; their moments about connector
    A's origin sum to the transmitted moment, whose hinge-axis component is
    zero up to solver tolerance.
    """
    point_forces = [rows_of(equality) for equality in equality_ids]
    if any(len(rows) < 3 for rows in point_forces):
        return _ZERO_WRENCH
    force = point_forces[0][:3] + point_forces[1][:3]
    torque = np.zeros(3)
    origin = (
        _reference_point(data, reference)
        if reference is not None
        else data.xpos[int(model.eq_obj1id[equality_ids[0]])]
    )
    for equality, rows in zip(equality_ids, point_forces, strict=True):
        body_a = int(model.eq_obj1id[equality])
        anchor = (
            data.xpos[body_a]
            + data.xmat[body_a].reshape(3, 3) @ model.eq_data[equality, CONNECT_ANCHOR_BODY_A]
        )
        torque += np.cross(anchor - origin, rows[:3])
    return ConstraintWrench(force_n=_vec3(force), torque_nm=_vec3(torque))


def _split(instance: ConnectorInstanceId) -> tuple[ModuleInstanceId, str]:
    return split_connector_instance_id(instance)


def _split_joint(instance: JointInstanceId) -> tuple[ModuleInstanceId, str]:
    try:
        return split_joint_instance_id(instance)
    except ValueError as error:
        raise BackendError(str(error)) from error


def _insert_exclusion(model: mujoco.MjModel, signature: int) -> None:
    """Publish one body-pair exclusion into the sorted signature array.

    The array holds the authored exclusions, one 0 per free reserved slot, and
    the occupied signatures, all in ascending order (MuJoCo binary-searches
    it). Dropping a leading 0 and inserting ``signature`` in place yields
    exactly the array a full re-sort would, with an O(log P) search instead of
    an O(P log P) sort.
    """
    signatures = model.exclude_signature
    if signatures.size == 0 or signatures[0] != 0:
        raise BackendError("MuJoCo contact-exclusion pool has no free reserved entry")
    position = int(np.searchsorted(signatures, signature, side="right"))
    signatures[: position - 1] = signatures[1:position].copy()
    signatures[position - 1] = signature


def _remove_exclusion(model: mujoco.MjModel, signature: int) -> None:
    """Withdraw one occurrence of ``signature`` and return its entry to the free 0s."""
    signatures = model.exclude_signature
    position = int(np.searchsorted(signatures, signature, side="left"))
    if position >= signatures.size or signatures[position] != signature:
        raise BackendError("MuJoCo contact-exclusion pool is inconsistent with the model")
    signatures[1 : position + 1] = signatures[:position].copy()
    signatures[0] = 0
