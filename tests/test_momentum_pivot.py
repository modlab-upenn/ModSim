"""Fast contract tests for the backend-neutral M-Blocks pivot controller."""

from __future__ import annotations

import math
from dataclasses import replace
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest

from modsim.backends.base import (
    BackendCapabilities,
    BackendHandleRegistry,
    ConnectionOutcome,
    ConnectionRequest,
)
from modsim.backends.mock import MockBackendAdapter
from modsim.core.entities import JointCommand
from modsim.core.events import DockCommitted, UndockCommitted
from modsim.core.ids import ConstraintHandle, JointInstanceId, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, JointState
from modsim.core.transforms import Transform, quat_from_axis_angle, quat_rotate, vec_scale
from modsim.model_views import CubicLatticeView, ModelViewContext, ModelViewFactory
from modsim.robot_packs import ControlMode, RobotPack, RobotPackLoader
from modsim.robot_packs.schema import PhysicalConstraintType
from modsim.runtime.momentum_pivot import (
    PUBLISHED_BRAKE_EFFORT_CAP_NM,
    PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S,
    PUBLISHED_SPINUP_EFFORT_CAP_NM,
    MomentumPivotConfig,
    MomentumPivotScenario,
)
from modsim.runtime.reconfiguration import ReconfigurationPhase
from modsim.runtime.session import RuntimeSession

_ROOT = Path(__file__).resolve().parents[1]
_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"
_SCENARIO_PATH = _ROOT / "examples" / "scenarios" / "mblocks_two_module_momentum_pivot.py"


class _MomentumPhysicsSpy(MockBackendAdapter):
    """Tiny deterministic physics stand-in with hinge and flywheel feedback.

    It exists to test controller boundaries, not rigid-body fidelity.  Root
    movement performed inside :meth:`step` represents backend-integrated state;
    calls to the optional root-control API are separately counted and can be
    prohibited after scenario initialization.
    """

    __slots__ = (
        "_current_effort",
        "_hinges",
        "_joint_ids",
        "_joint_velocity",
        "_pivot_angle",
        "_pivot_axis_world",
        "_pivot_origin_world",
        "_pivot_start_pose",
        "_pivot_started",
        "command_batches",
        "root_controls_allowed",
        "root_write_count",
    )

    def __init__(self) -> None:
        super().__init__()
        self._current_effort: dict[JointInstanceId, float] = {}
        self._hinges: set[ConstraintHandle] = set()
        self._joint_ids: dict[ModuleInstanceId, tuple[str, ...]] = {}
        self._joint_velocity: dict[JointInstanceId, float] = {}
        self._pivot_angle = 0.0
        self._pivot_axis_world = (0.0, 1.0, 0.0)
        self._pivot_origin_world = (0.0, 0.0, 0.0)
        self._pivot_started = False
        self._pivot_start_pose: Transform | None = None
        self.command_batches: list[tuple[JointCommand, ...]] = []
        self.root_controls_allowed = True
        self.root_write_count = 0

    def capabilities(self) -> BackendCapabilities:
        return BackendCapabilities(
            name="momentum-physics-spy",
            supports_runtime_constraints=True,
            supports_constraint_removal=True,
            supports_module_pose_write=True,
            supports_joint_commands=True,
            supported_joint_control_modes=frozenset({ControlMode.EFFORT}),
        )

    def load(
        self,
        pack: RobotPack,
        scene: SceneSpec,
        *,
        root: Path | None = None,
    ) -> BackendHandleRegistry:
        self._joint_ids = {
            placement.instance_id: tuple(
                joint.id
                for joint in pack.hardware_catalog.module_types[placement.module_type_id].joints
            )
            for placement in scene.placements
        }
        self._joint_velocity = {
            JointInstanceId(f"{module_id}/{joint_id}"): 0.0
            for module_id, joint_ids in self._joint_ids.items()
            for joint_id in joint_ids
        }
        return super().load(pack, scene, root=root)

    def step(self, dt_s: float) -> None:
        super().step(dt_s)
        rotor_inertia = 8.4e-6
        for joint, effort in self._current_effort.items():
            previous = self._joint_velocity[joint]
            current = previous + effort / rotor_inertia * dt_s
            if previous * current < 0.0:
                current = 0.0
            self._joint_velocity[joint] = current

        if any(effort < 0.0 for effort in self._current_effort.values()):
            self._pivot_started = True
        if self._hinges and self._pivot_start_pose is not None and self._pivot_started:
            self._pivot_angle = min(math.pi, self._pivot_angle + math.pi / 20.0)
            about_edge = (
                Transform.from_translation(self._pivot_origin_world)
                .compose(
                    Transform(
                        rotation=quat_from_axis_angle(self._pivot_axis_world, self._pivot_angle)
                    )
                )
                .compose(Transform.from_translation(vec_scale(self._pivot_origin_world, -1.0)))
            )
            moving = ModuleInstanceId("moving_block")
            body = self._bodies[moving]
            body.pose = about_edge.compose(self._pivot_start_pose)
            body.linear_velocity_m_s = (0.0, 0.0, 0.0)
            body.angular_velocity_rad_s = (0.0, 0.0, 0.0)

    def snapshot(self) -> BackendStateSnapshot:
        snapshot = super().snapshot()
        return BackendStateSnapshot(
            time_s=snapshot.time_s,
            link_states=snapshot.link_states,
            joint_states={
                module_id: {
                    joint_id: JointState(
                        velocity=self._joint_velocity[JointInstanceId(f"{module_id}/{joint_id}")],
                        effort=self._current_effort.get(
                            JointInstanceId(f"{module_id}/{joint_id}"),
                            0.0,
                        ),
                    )
                    for joint_id in joint_ids
                }
                for module_id, joint_ids in self._joint_ids.items()
            },
            connector_frames=snapshot.connector_frames,
            constraint_forces_n=snapshot.constraint_forces_n,
        )

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        if request.physical_connection.constraint is not PhysicalConstraintType.HINGE:
            return super().create_physical_connection(request)
        handle = ConstraintHandle(f"hinge:{request.connection_id}")
        self._hinges.add(handle)
        support_id = ModuleInstanceId("support_block")
        moving_id = ModuleInstanceId("moving_block")
        if request.module_a == support_id:
            support_local = request.connector_a_local
        else:
            assert request.module_b == support_id
            support_local = request.connector_b_local
        support = self._bodies[support_id]
        connector_world = support.pose.compose(support_local)
        assert request.physical_connection.hinge is not None
        self._pivot_origin_world = connector_world.translation
        self._pivot_axis_world = quat_rotate(
            connector_world.rotation,
            request.physical_connection.hinge.axis,
        )
        self._pivot_start_pose = self._bodies[moving_id].pose
        return ConnectionOutcome.accepted(handle)

    def remove_physical_connection(self, handle: ConstraintHandle) -> bool:
        if handle in self._hinges:
            self._hinges.remove(handle)
            return True
        return super().remove_physical_connection(handle)

    def set_joint_commands(self, commands: tuple[JointCommand, ...]) -> None:
        self.command_batches.append(commands)
        for command in commands:
            self._current_effort[command.joint] = command.value

    def clear_joint_commands(
        self,
        joints: tuple[JointInstanceId, ...] | None = None,
    ) -> None:
        selected = tuple(self._current_effort) if joints is None else joints
        for joint in selected:
            self._current_effort.pop(joint, None)

    def set_module_pose(self, module_id: ModuleInstanceId, pose: Transform) -> None:
        self._record_root_control()
        super().set_module_pose(module_id, pose)

    def set_module_twist(
        self,
        module_id: ModuleInstanceId,
        *,
        linear_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._record_root_control()
        super().set_module_twist(
            module_id,
            linear_m_s=linear_m_s,
            angular_rad_s=angular_rad_s,
        )

    def apply_module_wrench(
        self,
        module_id: ModuleInstanceId,
        *,
        force_n: tuple[float, float, float] = (0.0, 0.0, 0.0),
        torque_nm: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        del module_id, force_n, torque_nm
        self._record_root_control()

    def _record_root_control(self) -> None:
        if not self.root_controls_allowed:
            raise AssertionError("scenario used a root pose, twist, or wrench after creation")
        self.root_write_count += 1


def _example() -> tuple[SceneSpec, MomentumPivotConfig]:
    spec = spec_from_file_location("test_mblocks_momentum_example", _SCENARIO_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    scene = module.build_scene()
    config = module.build_config()
    assert isinstance(scene, SceneSpec)
    assert isinstance(config, MomentumPivotConfig)
    return scene, config


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("spinup_effort_nm", PUBLISHED_SPINUP_EFFORT_CAP_NM + 0.001, "0.03 N m"),
        ("brake_effort_nm", PUBLISHED_BRAKE_EFFORT_CAP_NM + 0.1, "2.6 N m"),
        (
            "target_flywheel_speed_rad_s",
            PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S + 1.0,
            "20,000 RPM",
        ),
        ("capture_angle_tolerance_rad", math.pi / 2.0, "below pi/2"),
        ("target_pivot_angle_rad", math.pi + 0.01, "must not exceed pi"),
        ("dt_s", 0.001, "must not exceed"),
    ),
)
def test_config_enforces_reference_actuator_and_timestep_ceilings(
    field: str,
    value: float,
    message: str,
) -> None:
    _, config = _example()

    with pytest.raises(ValueError, match=message):
        replace(config, **{field: value})


def test_controller_completes_face_hinge_face_lifecycle_without_runtime_root_writes() -> None:
    scene, config = _example()
    config = replace(
        config,
        settle_s=0.0,
        target_flywheel_speed_rad_s=20.0,
        connected_hold_s=0.001,
        spinup_timeout_s=0.1,
        brake_timeout_s=0.02,
        pivot_timeout_s=0.1,
    )
    loaded = RobotPackLoader().load(_PACK_PATH)
    adapter = _MomentumPhysicsSpy()
    runtime: RuntimeSession | None = None
    try:
        runtime = RuntimeSession.create(loaded, scene, adapter)
        scenario = MomentumPivotScenario.create(runtime, config)
        initialization_root_writes = adapter.root_write_count
        adapter.root_controls_allowed = False

        for _ in range(1_000):
            scenario.step()
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break

        assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
        assert adapter.root_write_count == initialization_root_writes
        active = set(runtime.world.connections)
        assert active == {config.target_face.connection_id}

        committed = [
            event
            for event in runtime.world.event_log
            if isinstance(event, DockCommitted | UndockCommitted)
        ]
        assert [event.connection_id for event in committed] == [
            config.initial_face.connection_id,
            config.edge_hinge.connection_id,
            config.initial_face.connection_id,
            config.target_face.connection_id,
            config.edge_hinge.connection_id,
        ]
        hinge_commit = committed[1]
        assert isinstance(hinge_commit, DockCommitted)
        assert hinge_commit.constraint is PhysicalConstraintType.HINGE

        moving_efforts = [
            command.value
            for batch in adapter.command_batches
            for command in batch
            if command.joint == config.flywheel
        ]
        assert max(moving_efforts) <= PUBLISHED_SPINUP_EFFORT_CAP_NM
        assert min(moving_efforts) >= -PUBLISHED_BRAKE_EFFORT_CAP_NM
        assert moving_efforts[-1] == 0.0

        telemetry = scenario.telemetry
        assert telemetry.maximum_brake_effort_nm <= PUBLISHED_BRAKE_EFFORT_CAP_NM
        assert telemetry.maximum_brake_effort_nm > 0.0
        assert telemetry.brake_impulse_nms > 0.0
        assert telemetry.brake_started_at_s is not None
        assert telemetry.brake_ended_at_s is not None
        assert telemetry.brake_end_flywheel_speed_rad_s == pytest.approx(0.0)
        assert telemetry.capture_time_s is not None
        assert telemetry.maximum_pivot_angle_rad == pytest.approx(math.pi)
    finally:
        if runtime is not None:
            runtime.shutdown()


def test_controller_rejects_geometric_capture_at_the_wrong_pivot_angle() -> None:
    scene, config = _example()
    config = replace(
        config,
        settle_s=0.0,
        target_flywheel_speed_rad_s=20.0,
        spinup_timeout_s=0.1,
        brake_timeout_s=0.02,
        pivot_timeout_s=0.1,
        target_pivot_angle_rad=math.pi / 2.0,
        capture_angle_tolerance_rad=math.radians(5.0),
    )
    loaded = RobotPackLoader().load(_PACK_PATH)
    adapter = _MomentumPhysicsSpy()
    runtime = RuntimeSession.create(loaded, scene, adapter)
    try:
        scenario = MomentumPivotScenario.create(runtime, config)
        adapter.root_controls_allowed = False

        for _ in range(1_000):
            scenario.step()
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break

        assert scenario.status.phase is ReconfigurationPhase.FAILED
        assert "timed out before" in scenario.status.detail
        assert config.target_face.connection_id not in runtime.world.connections
        assert config.edge_hinge.connection_id in runtime.world.connections
        assert scenario.telemetry.pivot_angle_rad == pytest.approx(math.pi)
        assert adapter.command_batches[-1][-1].value == 0.0
    finally:
        runtime.shutdown()


@pytest.mark.mujoco
@pytest.mark.parametrize("dt_s", (0.0005, 0.00025, 0.0001))
def test_real_mujoco_pivot_completes_and_converges_without_root_control(
    dt_s: float,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Gravity, contact, flywheel torque, and the hinge produce the complete roll."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    scene, base_config = _example()
    config = replace(base_config, dt_s=dt_s)
    loaded = RobotPackLoader().load(_PACK_PATH)
    runtime = RuntimeSession.create(
        loaded,
        scene,
        "mujoco",
        gravity=(0.0, 0.0, -9.81),
        ground=True,
        ground_height_m=0.0,
        timestep_s=dt_s,
        hinge_pool_size=1,
        weld_pool_size=2,
    )
    try:
        scenario = MomentumPivotScenario.create(runtime, config)
        adapter_type = type(runtime.adapter)

        def reject_runtime_root_control(*_args: object, **_kwargs: object) -> None:
            raise AssertionError("physical pivot wrote a root pose, twist, or wrench")

        monkeypatch.setattr(adapter_type, "set_module_pose", reject_runtime_root_control)
        monkeypatch.setattr(adapter_type, "set_module_twist", reject_runtime_root_control)
        monkeypatch.setattr(adapter_type, "apply_module_wrench", reject_runtime_root_control)

        for _ in range(math.ceil(5.0 / dt_s)):
            scenario.step()
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break

        assert scenario.status.phase is ReconfigurationPhase.COMPLETE, scenario.status.detail
        assert set(runtime.world.connections) == {config.target_face.connection_id}
        assert config.initial_face.connection_id not in runtime.world.connections
        assert config.edge_hinge.connection_id not in runtime.world.connections
        target = runtime.world.connections[config.target_face.connection_id]
        assert target.constraint is PhysicalConstraintType.FIXED

        telemetry = scenario.telemetry
        assert 0.84 <= scenario.status.time_s <= 0.90
        assert 3.13 <= telemetry.maximum_pivot_angle_rad <= 3.15
        assert 0.0075 <= telemetry.brake_impulse_nms <= 0.0078
        # The pulse is disabled at the first sampled zero crossing.  The
        # residual shrinks with timestep (about 4.95, 2.08, and 0.35 rad/s for
        # the three regression steps), rather than being hidden by a velocity
        # write.
        assert telemetry.brake_end_flywheel_speed_rad_s is not None
        assert abs(telemetry.brake_end_flywheel_speed_rad_s) <= 5.1
        assert telemetry.maximum_brake_effort_nm <= PUBLISHED_BRAKE_EFFORT_CAP_NM

        recipe = next(
            item
            for item in loaded.pack.manifest.model_views
            if item.id == "mblocks_physics_lattice"
        )
        view = ModelViewFactory().build(
            recipe,
            ModelViewContext(loaded.pack, runtime.world),
        )
        assert isinstance(view, CubicLatticeView)
        nodes = {node.id: node for node in view.nodes}
        assert nodes[str(config.fixed_module)].cell == (0, 0, 0)
        assert nodes[str(config.moving_module)].cell == (1, 0, 0)
        assert not nodes[str(config.fixed_module)].off_lattice
        assert not nodes[str(config.moving_module)].off_lattice
    finally:
        runtime.shutdown()
