"""Fixture-supported SMORES handoff into a four-module vertical chain.

Joint effort and scratch-data kinematics deliberately live in the optional
backend package. This is not the general ModSim actuator API. The planner
never mutates live physics state; only initial staging writes root poses.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import cast

import mujoco
import numpy as np

from modsim.core.events import Event
from modsim.core.ids import ConnectorInstanceId, ConstraintHandle, ModuleInstanceId, connection_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, Vec3, quat_from_axis_angle
from modsim.planning.spatial import (
    Bond,
    JointPath,
    JointPoint,
    SpatialPlanningError,
)
from modsim.robot_packs import RobotPackLoader
from modsim.robot_packs.schema import AlignmentMode
from modsim.runtime.session import RuntimeSession
from modsim.runtime.spatial import (
    SpatialHandoffProblem,
    SpatialObservation,
    SpatialReconfigurationScenario,
)
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter
from modsim_backend_mujoco.scene import PositionServo

MODULES = ("helper", "payload", "receiver", "arm", "upper")
ANCHORS = frozenset(("helper", "receiver"))
INITIAL = frozenset(
    (
        Bond("helper/pan", "payload/bottom"),
        Bond("receiver/pan", "arm/bottom"),
        Bond("arm/pan", "upper/bottom"),
    )
)
CAPTURE = Bond("upper/pan", "payload/left")
RELEASE = Bond("helper/pan", "payload/bottom")
GOAL = (INITIAL | {CAPTURE}) - {RELEASE}
JOINTS = ("joint_left_wheel", "joint_right_wheel", "joint_tilt", "joint_pan")
STEP_RAD = math.pi / 12
MATE = Transform(rotation=quat_from_axis_angle((0.0, 0.0, 1.0), math.pi))


class SpatialExperiment:
    """Measured, bounded-effort execution with explicit terminal outcomes."""

    def __init__(
        self, pack_root: Path, *, torque_limit_nm: float = 1.2, duration_s: float = 45.0
    ) -> None:
        if not math.isfinite(torque_limit_nm) or torque_limit_nm <= 0:
            raise ValueError("torque limit must be finite and positive")
        self._duration_s = duration_s
        self.torque_limit_nm = torque_limit_nm
        self.dt_s = 0.001
        self.adapter = MuJoCoBackendAdapter(
            timestep_s=self.dt_s,
            ground=True,
            weld_pool_size=16,
            exclude_docked_contacts=False,
            position_servos={
                # The longer receiver needs stiffer tracking during load transfer.
                # Gains are provisional; the original effort bound still applies.
                f"{m}/{j}": PositionServo(150, 0.6, torque_limit_nm)
                for m in MODULES
                for j in JOINTS
            },
        )
        try:
            self._initialize(pack_root)
        except Exception:
            self.adapter.shutdown()
            raise

    def _initialize(self, pack_root: Path) -> None:
        loaded = RobotPackLoader().load(pack_root)
        # A benchmark-local override: no nominal snap may move a root at capture.
        pack = loaded.pack.model_copy(deep=True)
        # This benchmark uses the CAD URDF proxies and provisional position
        # servos recorded in the manuscript, independently of planar drive tuning.
        pack = pack.model_copy(
            update={
                "manifest": pack.manifest.model_copy(
                    update={"assets": pack.manifest.assets.model_copy(update={"mujoco": {}})}
                )
            }
        )
        module = pack.hardware_catalog.module_types["smores_ep"]
        pack.hardware_catalog.module_types["smores_ep"] = module.model_copy(
            update={
                "joints": tuple(j.model_copy(update={"control_modes": ()}) for j in module.joints)
            }
        )
        connector = pack.hardware_catalog.connector_types["ep_face"]
        assert connector.acceptance_region is not None
        pack.hardware_catalog.connector_types["ep_face"] = connector.model_copy(
            update={
                "acceptance_region": connector.acceptance_region.model_copy(
                    update={"orientation_tolerance_rad": math.pi / 18}
                ),
                "docking_policy": connector.effective_docking_policy.model_copy(
                    update={"alignment": AlignmentMode.MEASURED}
                ),
            }
        )
        self.session = RuntimeSession.create(
            loaded.with_pack(pack),
            SceneSpec.of(ModulePlacement(ModuleInstanceId(m), "smores_ep") for m in MODULES),
            self.adapter,
        )
        self.model, self.data = self.adapter.model, self.adapter.data
        self.model.eq_solref[:, 0] = 0.002
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.scratch = mujoco.MjData(self.model)
        self.joint_indices = {
            (m, j): (
                int(self.model.joint(f"{m}/{j}").qposadr[0]),
                int(self.model.joint(f"{m}/{j}").dofadr[0]),
            )
            for m in MODULES
            for j in JOINTS
        }
        # Provisional reflected motor/gear inertia, explicitly part of this
        # experimental actuator model rather than a change to the imported CAD.
        for _, dadr in self.joint_indices.values():
            self.model.dof_armature[dadr] = 0.0005
        self.roots = {}
        for m in MODULES:
            body = self.model.body(f"{m}/base_link").id
            joint = int(self.model.body_jntadr[body])
            self.roots[m] = int(self.model.jnt_qposadr[joint])
        self.body_modules = {
            b: (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or "").split("/")[0]
            for b in range(self.model.nbody)
        }
        self.fixture_poses = self._fixture_poses()
        self.initial_point = (0, 0, 0)
        self.goal_point = (6, 6, 6)
        self.final_point = (0, 0, 6)
        self._place(self.data, tuple(map(float, self.initial_point)))
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self.session.world.ingest(self.adapter.snapshot())
        for bond in sorted(INITIAL):
            self._dock(bond)
        for module in sorted(ANCHORS):
            slot = self.adapter.weld_pool.claim(ConstraintHandle(f"fixture/{module}"))
            eq = slot.equality_id
            self.model.eq_obj1id[eq] = 0
            self.model.eq_obj2id[eq] = self.model.body(f"{module}/base_link").id
            pose = self.fixture_poses[module]
            self.model.eq_data[eq, :] = [0, 0, 0, *pose.translation, *pose.rotation, 1]
            self.data.eq_active[eq] = True
        mujoco.mj_forward(self.model, self.data)
        self.rejection_reasons: dict[str, int] = {}
        self._place(self.scratch, tuple(map(float, self.final_point)), transferred=True)
        self._target_positions = {
            m: cast(Vec3, tuple(float(x) for x in self.scratch.body(f"{m}/base_link").xpos))
            for m in MODULES
        }
        self.scenario = SpatialReconfigurationScenario(
            self.session,
            self,
            SpatialHandoffProblem(
                frozenset(MODULES), ANCHORS, INITIAL, GOAL, "helper", final=self.final_point
            ),
            duration_s=self._duration_s,
        )
        self.planned_trace = []
        for point in self.path.points:
            self._place(self.scratch, point)
            self.planned_trace.append(
                tuple(float(x) for x in self.scratch.body("payload/base_link").xpos)
            )

    @property
    def target_positions(self) -> dict[str, Vec3]:
        return self._target_positions

    @property
    def motor_keys(self) -> tuple[tuple[ModuleInstanceId, str], ...]:
        return tuple((ModuleInstanceId(m), j) for m, j in self.joint_indices)

    def feasible(self, point: JointPoint, *, transferred: bool = False) -> bool:
        return self._feasible(point, transferred=transferred)

    def targets(self, point: JointPoint) -> tuple[float, ...]:
        return tuple(self._targets(point))

    def configuration_positions(
        self, point: JointPoint, *, transferred: bool = False
    ) -> dict[str, Vec3]:
        self._place(self.scratch, point, transferred=transferred)
        return {
            m: cast(Vec3, tuple(float(x) for x in self.scratch.body(f"{m}/base_link").xpos))
            for m in MODULES
        }

    def command_positions(self, positions: tuple[float, ...]) -> None:
        if len(positions) != self.model.nu or not all(math.isfinite(q) for q in positions):
            raise ValueError("servo commands must contain one finite position per motor")
        self.data.ctrl[:] = positions

    def observe_safety(self) -> SpatialObservation:
        return SpatialObservation(
            float(self.data.time),
            self._penetration(self.data),
            bool(np.all(np.isfinite(self.data.qpos))),
        )

    def _local(self, module: str, face: str, point: JointPoint) -> Transform:
        """Read connector FK on scratch data, never the executing MjData."""
        self.scratch.qpos[:] = self.model.qpos0
        for _m, addr in self.roots.items():
            self.scratch.qpos[addr : addr + 7] = [0, 0, 0, 1, 0, 0, 0]
        for ((_, _), (addr, _)), value in zip(
            self.joint_indices.items(), self._targets(point), strict=False
        ):
            self.scratch.qpos[addr] = value
        mujoco.mj_kinematics(self.model, self.scratch)
        site = self.scratch.site(f"{module}/connector/{face}")
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, site.xmat)
        return Transform(
            (float(site.xpos[0]), float(site.xpos[1]), float(site.xpos[2])),
            (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
        )

    def _fixture_poses(self) -> dict[str, Transform]:
        # Stage two floor-level fixtures from exact rendezvous FK. The receiver's
        # three-module chain folds toward the helper, then unfolds after capture.
        point = (6.0, 6.0, 6.0)
        receiver = Transform.from_translation((0.0, 0.0, 0.06))
        arm = (
            receiver.compose(self._local("receiver", "pan", point))
            .compose(MATE)
            .compose(self._local("arm", "bottom", point).inverse())
        )
        upper = (
            arm.compose(self._local("arm", "pan", point))
            .compose(MATE)
            .compose(self._local("upper", "bottom", point).inverse())
        )
        payload = (
            upper.compose(self._local("upper", "pan", point))
            .compose(MATE)
            .compose(self._local("payload", "left", point).inverse())
        )
        helper = (
            payload.compose(self._local("payload", "bottom", point))
            .compose(MATE)
            .compose(self._local("helper", "pan", point).inverse())
        )
        return {"helper": helper, "receiver": receiver}

    def _targets(self, point: JointPoint) -> list[float]:
        values = {
            ("helper", "joint_tilt"): -point[0] * STEP_RAD,
            ("payload", "joint_left_wheel"): point[2] * STEP_RAD,
            ("receiver", "joint_tilt"): -math.pi / 2,
            ("arm", "joint_tilt"): point[1] * STEP_RAD,
        }
        return [values.get((m, j), 0.0) for m, j in self.joint_indices]

    def _place(self, data: mujoco.MjData, point: JointPoint, *, transferred: bool = False) -> None:
        """Exact FK for initial staging and separate planning data only."""
        locals_ = {
            (m, f): self._local(m, f, point)
            for m, f in (
                ("helper", "pan"),
                ("payload", "bottom"),
                ("receiver", "pan"),
                ("arm", "bottom"),
                ("upper", "bottom"),
                ("upper", "pan"),
                ("arm", "pan"),
                ("payload", "left"),
            )
        }
        poses = dict(self.fixture_poses)
        for a, af, b, bf in (
            ("helper", "pan", "payload", "bottom"),
            ("receiver", "pan", "arm", "bottom"),
            ("arm", "pan", "upper", "bottom"),
        ):
            poses[b] = (
                poses[a].compose(locals_[a, af]).compose(MATE).compose(locals_[b, bf].inverse())
            )
        if transferred:
            poses["payload"] = (
                poses["upper"]
                .compose(locals_["upper", "pan"])
                .compose(MATE)
                .compose(locals_["payload", "left"].inverse())
            )
        for m, pose in poses.items():
            addr = self.roots[m]
            data.qpos[addr : addr + 7] = [*pose.translation, *pose.rotation]
        for (_, (addr, _)), value in zip(
            self.joint_indices.items(), self._targets(point), strict=False
        ):
            data.qpos[addr] = value
        data.qvel[:] = 0
        mujoco.mj_forward(self.model, data)

    def _penetration(self, data: mujoco.MjData) -> float:
        # Internal contacts belong to the imported module model. Check ALL
        # inter-module/ground contacts, including prospective docking faces.
        return max(
            (
                max(0.0, -float(c.dist))
                for c in data.contact
                if self.body_modules[int(self.model.geom_bodyid[c.geom1])]
                != self.body_modules[int(self.model.geom_bodyid[c.geom2])]
            ),
            default=0.0,
        )

    def _feasible(self, point: JointPoint, *, transferred: bool = False) -> bool:
        self._place(self.scratch, point, transferred=transferred)
        reason = "collision" if self._penetration(self.scratch) > 0.0005 else ""
        # Coarse minimum budget for the helper's payload lift. This does not
        # certify the longer receiver's load capacity; measured bounded-effort
        # execution must also pass tracking, collision, and final-hold checks.
        mass = float(sum(self.model.body_mass)) / len(MODULES)
        if self.torque_limit_nm < 2 * mass * 9.81 * 0.10:
            reason = "gravity torque budget"
        if reason:
            self.rejection_reasons[reason] = self.rejection_reasons.get(reason, 0) + 1
        return not reason

    def _dock(self, bond: Bond) -> None:
        self.session.request_dock(ConnectorInstanceId(bond.a), ConnectorInstanceId(bond.b))
        self.session.process_docking()
        if (
            connection_id(ConnectorInstanceId(bond.a), ConnectorInstanceId(bond.b))
            not in self.session.world.connections
        ):
            raise SpatialPlanningError(f"measured docking refused: {bond}")

    @property
    def terminal(self) -> bool:
        return self.scenario.terminal

    @property
    def phase(self) -> str:
        return self.scenario.phase

    @property
    def detail(self) -> str:
        return self.scenario.detail

    @property
    def path(self) -> JointPath:
        return self.scenario.path

    @property
    def return_path(self) -> JointPath:
        return self.scenario.return_path

    @property
    def waypoint(self) -> int:
        return self.scenario.waypoint

    @property
    def return_waypoint(self) -> int:
        return self.scenario.return_waypoint

    @property
    def peak_effort_nm(self) -> float:
        return self.scenario.peak_effort_nm

    @property
    def peak_penetration_m(self) -> float:
        return self.scenario.peak_penetration_m

    @property
    def history(self) -> list[dict[str, object]]:
        return self.scenario.history

    def stop(self) -> None:
        self.scenario.stop()

    def step(self) -> tuple[Event, ...]:
        return self.scenario.step()

    def target_error_m(self) -> float:
        return self.scenario.target_error_m()

    def report(self) -> dict[str, object]:
        return {
            **self.scenario.report(),
            "benchmark": "smores_vertical_chain",
            "rejections": self.rejection_reasons,
            "torque_limit_nm": self.torque_limit_nm,
        }

    def close(self) -> None:
        self.session.shutdown()


def create_spatial_scenario(pack_root: Path, duration_s: float) -> SpatialReconfigurationScenario:
    """Prepare mechanical services and return the ModSim-owned controller."""
    return SpatialExperiment(pack_root, duration_s=duration_s).scenario
