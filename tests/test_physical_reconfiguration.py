"""Fast contract tests for wheel-driven reconfiguration orchestration."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import cast

import pytest

from modsim.backends.base import BackendCapabilities, BackendHandleRegistry
from modsim.backends.mock import MockBackendAdapter
from modsim.core.entities import JointCommand
from modsim.core.ids import ConnectorInstanceId, JointInstanceId, ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, JointState
from modsim.core.transforms import Transform
from modsim.robot_packs import ControlMode, LoadedRobotPack, RobotPack
from modsim.runtime.differential_drive import (
    DifferentialDriveGeometry,
    PositionEffortController,
    VelocityEffortController,
)
from modsim.runtime.physical_reconfiguration import (
    DifferentialDriveReconfigurationConfig,
    DifferentialDriveReconfigurationScenario,
    PhysicalActionRoute,
    TargetRelativeWaypoint,
)
from modsim.runtime.physics_docking import (
    DifferentialDriveDockingConfig,
    DifferentialDriveDockingScenario,
    JointHoldTarget,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationAction,
    ReconfigurationPlan,
    ReconfigurationScenarioError,
)
from modsim.runtime.session import RuntimeSession


class _JointFeedbackMock(MockBackendAdapter):
    """Mock kinematics with measured joints and optional effort support."""

    __slots__ = (
        "_joint_ids",
        "_supports_effort",
        "joint_command_batches",
        "pose_write_count",
    )

    def __init__(self, *, supports_effort: bool) -> None:
        super().__init__()
        self._joint_ids: dict[ModuleInstanceId, tuple[str, ...]] = {}
        self._supports_effort = supports_effort
        self.joint_command_batches: list[tuple[JointCommand, ...]] = []
        self.pose_write_count = 0

    def capabilities(self) -> BackendCapabilities:
        modes: frozenset[ControlMode] = (
            frozenset({ControlMode.EFFORT}) if self._supports_effort else frozenset()
        )
        return BackendCapabilities(
            name="joint-feedback-mock",
            supports_runtime_constraints=True,
            supports_constraint_removal=True,
            supports_module_pose_write=True,
            supports_joint_commands=self._supports_effort,
            supported_joint_control_modes=modes,
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
        return super().load(pack, scene, root=root)

    def snapshot(self) -> BackendStateSnapshot:
        snapshot = super().snapshot()
        return BackendStateSnapshot(
            time_s=snapshot.time_s,
            link_states=snapshot.link_states,
            joint_states={
                module_id: dict.fromkeys(joint_ids, JointState())
                for module_id, joint_ids in self._joint_ids.items()
            },
            connector_frames=snapshot.connector_frames,
            constraint_forces_n=snapshot.constraint_forces_n,
        )

    def set_joint_commands(self, commands: tuple[JointCommand, ...]) -> None:
        self.joint_command_batches.append(commands)

    def clear_joint_commands(
        self,
        joints: tuple[JointInstanceId, ...] | None = None,
    ) -> None:
        del joints

    def set_module_pose(self, module_id: ModuleInstanceId, pose: Transform) -> None:
        self.pose_write_count += 1
        super().set_module_pose(module_id, pose)


def _pair(
    fixed_module: str,
    fixed_connector: str,
    moving_module: str,
    moving_connector: str,
) -> ConnectorPairRef:
    return ConnectorPairRef(
        fixed_connector=ConnectorInstanceId(f"{fixed_module}/{fixed_connector}"),
        moving_connector=ConnectorInstanceId(f"{moving_module}/{moving_connector}"),
    )


def _two_module_plan(*, action: ReconfigurationAction | None = None) -> ReconfigurationPlan:
    initial = _pair("module_1", "bottom", "module_2", "pan")
    replacement = ReconfigurationAction(
        label="replace rear-to-front with front-to-rear",
        undock=initial,
        dock=_pair("module_1", "pan", "module_2", "bottom"),
    )
    return ReconfigurationPlan(
        id="physical_pair",
        name="Physical pair",
        module_ids=(ModuleInstanceId("module_1"), ModuleInstanceId("module_2")),
        initial_connections=(initial,),
        actions=(replacement if action is None else action,),
    )


def _config(*, route_count: int = 1) -> DifferentialDriveReconfigurationConfig:
    route = PhysicalActionRoute(
        waypoints=(TargetRelativeWaypoint(0.05, 0.0),),
        approach_direction=1,
    )
    return DifferentialDriveReconfigurationConfig(
        left_wheel_joint="joint_left_wheel",
        right_wheel_joint="joint_right_wheel",
        hold_joints=(
            JointHoldTarget("joint_tilt"),
            JointHoldTarget("joint_pan", continuous=True),
        ),
        connector_roll_joint="joint_pan",
        connector_roll_connector="pan",
        geometry=DifferentialDriveGeometry(
            wheel_radius_m=0.04,
            track_width_m=0.0672,
        ),
        wheel_controller=VelocityEffortController(
            gain_nm_per_rad_s=0.1,
            max_effort_nm=0.04,
        ),
        hold_controller=PositionEffortController(
            position_gain_nm_per_rad=1.0,
            velocity_gain_nm_per_rad_s=0.02,
            max_effort_nm=0.1,
        ),
        routes=(route,) * route_count,
    )


def _docking_config() -> DifferentialDriveDockingConfig:
    return DifferentialDriveDockingConfig(
        fixed_connector=ConnectorInstanceId("module_1/bottom"),
        moving_connector=ConnectorInstanceId("module_2/pan"),
        left_wheel_joint="joint_left_wheel",
        right_wheel_joint="joint_right_wheel",
        hold_joints=(
            JointHoldTarget("joint_tilt"),
            JointHoldTarget("joint_pan", continuous=True),
        ),
        geometry=DifferentialDriveGeometry(
            wheel_radius_m=0.04,
            track_width_m=0.0672,
        ),
        wheel_controller=VelocityEffortController(
            gain_nm_per_rad_s=0.1,
            max_effort_nm=0.04,
        ),
        hold_controller=PositionEffortController(
            position_gain_nm_per_rad=1.0,
            velocity_gain_nm_per_rad_s=0.02,
            max_effort_nm=0.1,
        ),
    )


def _scene() -> SceneSpec:
    return SceneSpec.of(
        ModulePlacement(
            instance_id=ModuleInstanceId(f"module_{index}"),
            module_type_id="smores_ep",
            pose=Transform.from_translation((0.2 * index, 0.0, 0.05)),
        )
        for index in (1, 2)
    )


def _without_left_wheel_effort(pack: RobotPack) -> RobotPack:
    data = pack.model_dump(mode="python")
    module = data["hardware_catalog"]["module_types"]["smores_ep"]
    left_wheel = next(joint for joint in module["joints"] if joint["id"] == "joint_left_wheel")
    left_wheel["control_modes"] = []
    return RobotPack.model_validate(data)


def test_route_value_objects_reject_invalid_direction_and_tolerances() -> None:
    with pytest.raises(ValueError, match="drive_direction must be -1 or 1"):
        TargetRelativeWaypoint(0.0, 0.0, drive_direction=0)
    with pytest.raises(ValueError, match="position_tolerance_m must be greater than zero"):
        TargetRelativeWaypoint(0.0, 0.0, position_tolerance_m=0.0)
    with pytest.raises(ValueError, match="waypoint heading_rad must be finite"):
        TargetRelativeWaypoint(0.0, 0.0, heading_rad=math.nan)
    with pytest.raises(ValueError, match="approach_direction must be -1 or 1"):
        PhysicalActionRoute(waypoints=(), approach_direction=0)


def test_configuration_rejects_ambiguous_or_unsafe_joint_assignments() -> None:
    config = _config()

    with pytest.raises(ValueError, match="left and right wheel joints must differ"):
        replace(config, right_wheel_joint=config.left_wheel_joint)
    with pytest.raises(ValueError, match="duplicate joint IDs"):
        replace(
            config,
            hold_joints=(JointHoldTarget("joint_tilt"), JointHoldTarget("joint_tilt")),
        )
    with pytest.raises(ValueError, match="wheel joints must not also be hold joints"):
        replace(config, hold_joints=(JointHoldTarget(config.left_wheel_joint),))
    with pytest.raises(ValueError, match="must be supplied together"):
        replace(config, connector_roll_connector=None)
    with pytest.raises(ValueError, match="must identify one of the hold joints"):
        replace(config, connector_roll_joint="joint_missing")


def test_configuration_rejects_unstable_timing_and_feedback_values() -> None:
    config = _config()

    with pytest.raises(ValueError, match="dt_s must not exceed"):
        replace(config, dt_s=0.006)
    with pytest.raises(ValueError, match="release_delay_s must not be negative"):
        replace(config, release_delay_s=-0.01)
    with pytest.raises(ValueError, match="action_timeout_s must be greater than zero"):
        replace(config, action_timeout_s=0.0)
    with pytest.raises(ValueError, match="maximum_wheel_speed_rad_s must be finite"):
        replace(config, maximum_wheel_speed_rad_s=math.inf)


def test_scenario_rejects_invalid_plan_shape_before_backend_use() -> None:
    valid_plan = _two_module_plan()
    zero_time = cast(
        RuntimeSession,
        SimpleNamespace(world=SimpleNamespace(time_s=0.0)),
    )

    with pytest.raises(ReconfigurationScenarioError, match=r"1 action\(s\).*0 route\(s\)"):
        DifferentialDriveReconfigurationScenario.create(
            zero_time,
            valid_plan,
            _config(route_count=0),
        )

    dock_only = ReconfigurationAction(
        label="dock only",
        dock=_pair("module_1", "pan", "module_2", "bottom"),
    )
    with pytest.raises(ReconfigurationScenarioError, match="must replace one connection"):
        DifferentialDriveReconfigurationScenario.create(
            zero_time,
            _two_module_plan(action=dock_only),
            _config(),
        )

    advanced = cast(
        RuntimeSession,
        SimpleNamespace(world=SimpleNamespace(time_s=0.01)),
    )
    with pytest.raises(ReconfigurationScenarioError, match="simulation time zero"):
        DifferentialDriveReconfigurationScenario.create(
            advanced,
            valid_plan,
            _config(),
        )


@pytest.mark.parametrize(
    ("supports_effort", "remove_pack_effort", "message"),
    (
        (False, False, "does not support joint commands"),
        (True, True, "does not declare 'effort' control"),
    ),
)
def test_joint_effort_preflight_fails_before_staging_mutates_the_world(
    smores_loaded_pack: LoadedRobotPack,
    supports_effort: bool,
    remove_pack_effort: bool,
    message: str,
) -> None:
    pack = (
        _without_left_wheel_effort(smores_loaded_pack.pack)
        if remove_pack_effort
        else smores_loaded_pack.pack
    )
    adapter = _JointFeedbackMock(supports_effort=supports_effort)
    session = RuntimeSession.create(pack, _scene(), adapter)

    try:
        with pytest.raises(ReconfigurationScenarioError, match=message):
            DifferentialDriveReconfigurationScenario.create(
                session,
                _two_module_plan(),
                _config(),
            )

        assert session.world.time_s == 0.0
        assert not session.world.connections
        assert session.world.assemblies.count == 2
        assert len(session.world.event_log) == 0
        assert adapter.pose_write_count == 0
        assert not adapter.joint_command_batches
    finally:
        session.shutdown()


@pytest.mark.parametrize(
    ("supports_effort", "remove_pack_effort", "message"),
    (
        (False, False, "does not support joint commands"),
        (True, True, "does not declare 'effort' control"),
    ),
)
def test_pair_joint_effort_preflight_fails_without_mutating_root_state(
    smores_loaded_pack: LoadedRobotPack,
    supports_effort: bool,
    remove_pack_effort: bool,
    message: str,
) -> None:
    pack = (
        _without_left_wheel_effort(smores_loaded_pack.pack)
        if remove_pack_effort
        else smores_loaded_pack.pack
    )
    adapter = _JointFeedbackMock(supports_effort=supports_effort)
    session = RuntimeSession.create(pack, _scene(), adapter)
    adapter.set_module_twist(
        ModuleInstanceId("module_1"),
        linear_m_s=(0.1, -0.2, 0.3),
        angular_rad_s=(0.4, -0.5, 0.6),
    )
    adapter.set_module_twist(
        ModuleInstanceId("module_2"),
        linear_m_s=(-0.1, 0.2, -0.3),
        angular_rad_s=(-0.4, 0.5, -0.6),
    )
    session.world.ingest(adapter.snapshot())
    backend_before = adapter.snapshot()
    world_before = {
        module_id: (
            module.pose,
            module.linear_velocity_m_s,
            module.angular_velocity_rad_s,
        )
        for module_id, module in session.world.modules.items()
    }

    try:
        with pytest.raises(ReconfigurationScenarioError, match=message):
            DifferentialDriveDockingScenario.create(
                session,
                _docking_config(),
            )

        assert adapter.snapshot() == backend_before
        assert {
            module_id: (
                module.pose,
                module.linear_velocity_m_s,
                module.angular_velocity_rad_s,
            )
            for module_id, module in session.world.modules.items()
        } == world_before
        assert session.world.time_s == 0.0
        assert not session.world.connections
        assert session.world.assemblies.count == 2
        assert len(session.world.event_log) == 0
        assert adapter.welds == ()
        assert adapter.pose_write_count == 0
        assert not adapter.joint_command_batches
    finally:
        session.shutdown()
