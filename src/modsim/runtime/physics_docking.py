"""Physical two-module docking driven through differential-drive wheel joints.

Unlike the scripted reconfiguration scenario, this controller uses a root-pose
write only for initial placement.  Every subsequent approach and retraction
step is produced by bounded joint efforts while the backend integrates gravity,
contact, inertia, and the docking constraint.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modsim.backends.base import BackendError
from modsim.core.entities import JointCommand
from modsim.core.events import DockFailed, Event, UndockFailed
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    JointInstanceId,
    ModuleInstanceId,
    connection_id,
    joint_instance_id,
    split_connector_instance_id,
)
from modsim.core.transforms import quat_rotate, vec_dot, vec_norm, vec_sub
from modsim.core.validation import (
    require_finite,
    require_finite_nonnegative,
    require_finite_positive,
)
from modsim.robot_packs.schema import ControlMode
from modsim.runtime.differential_drive import (
    DifferentialDriveGeometry,
    PositionEffortController,
    VelocityEffortController,
)
from modsim.runtime.reconfiguration import (
    ReconfigurationPhase,
    ReconfigurationScenarioError,
    ReconfigurationStatus,
    stage_docking_assembly_pair,
)
from modsim.runtime.session import JointCommandError, RuntimeSession

_EPSILON = 1e-12
MAX_PHYSICAL_TIMESTEP_S = 0.005


@dataclass(frozen=True, slots=True)
class JointHoldTarget:
    """One module-local angular joint held during ground locomotion."""

    joint_id: str
    target_rad: float = 0.0
    continuous: bool = False

    def __post_init__(self) -> None:
        if not self.joint_id.strip():
            raise ValueError("hold joint_id must not be empty")
        require_finite(self.target_rad, "hold target_rad")


@dataclass(frozen=True, slots=True)
class DifferentialDriveDockingConfig:
    """Platform parameters and timing for one physical docking cycle."""

    fixed_connector: ConnectorInstanceId
    moving_connector: ConnectorInstanceId
    left_wheel_joint: str
    right_wheel_joint: str
    hold_joints: tuple[JointHoldTarget, ...]
    geometry: DifferentialDriveGeometry
    wheel_controller: VelocityEffortController
    hold_controller: PositionEffortController
    dt_s: float = 0.001
    controller_period_s: float = 0.025
    initial_gap_m: float = 0.08
    approach_speed_m_s: float = 0.03
    retract_speed_m_s: float = 0.03
    maximum_wheel_speed_rad_s: float = math.pi / 2.0
    settle_s: float = 0.5
    connected_hold_s: float = 1.0
    release_delay_s: float = 0.08
    retract_distance_m: float = 0.04
    latch_distance_m: float = 0.001
    approach_timeout_s: float = 6.0
    heading_gain_per_s: float = 2.0
    lateral_gain_rad_per_m_s: float = 8.0
    maximum_yaw_rate_rad_s: float = 0.6
    plan_id: str = "physical_diff_drive_dock_undock"
    plan_name: str = "Physical differential-drive dock and undock"

    def __post_init__(self) -> None:
        fixed_module, _ = split_connector_instance_id(self.fixed_connector)
        moving_module, _ = split_connector_instance_id(self.moving_connector)
        if fixed_module == moving_module:
            raise ValueError("physical docking requires two different modules")
        for label, value in (
            ("left_wheel_joint", self.left_wheel_joint),
            ("right_wheel_joint", self.right_wheel_joint),
            ("plan_id", self.plan_id),
            ("plan_name", self.plan_name),
        ):
            if not value.strip():
                raise ValueError(f"{label} must not be empty")
        if self.left_wheel_joint == self.right_wheel_joint:
            raise ValueError("left and right wheel joints must differ")
        hold_ids = tuple(target.joint_id for target in self.hold_joints)
        if len(set(hold_ids)) != len(hold_ids):
            raise ValueError("hold_joints must not contain duplicate joint IDs")
        if self.left_wheel_joint in hold_ids or self.right_wheel_joint in hold_ids:
            raise ValueError("wheel joints must not also be hold joints")
        for name, value in (
            ("dt_s", self.dt_s),
            ("controller_period_s", self.controller_period_s),
            ("approach_speed_m_s", self.approach_speed_m_s),
            ("retract_speed_m_s", self.retract_speed_m_s),
            ("maximum_wheel_speed_rad_s", self.maximum_wheel_speed_rad_s),
            ("latch_distance_m", self.latch_distance_m),
            ("approach_timeout_s", self.approach_timeout_s),
            ("heading_gain_per_s", self.heading_gain_per_s),
            ("lateral_gain_rad_per_m_s", self.lateral_gain_rad_per_m_s),
            ("maximum_yaw_rate_rad_s", self.maximum_yaw_rate_rad_s),
        ):
            require_finite_positive(value, name)
        if self.dt_s > MAX_PHYSICAL_TIMESTEP_S:
            raise ValueError(
                f"dt_s must not exceed {MAX_PHYSICAL_TIMESTEP_S:g} for physical docking"
            )
        for name, value in (
            ("initial_gap_m", self.initial_gap_m),
            ("settle_s", self.settle_s),
            ("connected_hold_s", self.connected_hold_s),
            ("release_delay_s", self.release_delay_s),
            ("retract_distance_m", self.retract_distance_m),
        ):
            require_finite_nonnegative(value, name)

    @property
    def fixed_module(self) -> ModuleInstanceId:
        """Return the module that acts as the stationary docking target."""
        return split_connector_instance_id(self.fixed_connector)[0]

    @property
    def moving_module(self) -> ModuleInstanceId:
        """Return the module driven toward and away from the target."""
        return split_connector_instance_id(self.moving_connector)[0]

    @property
    def connection(self) -> ConnectionId:
        """Return the canonical connection created by this cycle."""
        return connection_id(self.fixed_connector, self.moving_connector)


@dataclass(slots=True)
class DifferentialDriveDockingScenario:
    """Drive, latch, release, and reverse one module under backend dynamics."""

    session: RuntimeSession
    config: DifferentialDriveDockingConfig
    _phase: ReconfigurationPhase
    _phase_started_at_s: float
    _detail: str
    _next_control_at_s: float
    _retract_origin: tuple[float, float, float] | None = None
    _retract_direction: tuple[float, float, float] | None = None

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        config: DifferentialDriveDockingConfig,
    ) -> DifferentialDriveDockingScenario:
        """Preflight physical control, place the pair once, and begin settling."""
        scenario = cls(
            session=session,
            config=config,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _detail="Settling both free modules on the ground under joint hold control",
            _next_control_at_s=session.world.time_s,
        )
        scenario._require_feedback()
        scenario._require_joint_control()
        try:
            stage_docking_assembly_pair(
                session,
                config.fixed_connector,
                config.moving_connector,
                gap_m=config.initial_gap_m,
                # A ground robot must remain upright. Connector-frame roll is
                # resolved by the eventual acceptance/weld policy; initial
                # placement aligns only the opposing docking axes.
                preserve_orientation=True,
            )
        except ReconfigurationScenarioError:
            raise
        except Exception as error:
            raise ReconfigurationScenarioError(
                f"could not stage the physical docking pair: {error}"
            ) from error

        scenario._apply_control()
        return scenario

    @property
    def status(self) -> ReconfigurationStatus:
        """Return immutable progress for the Runtime Inspector."""
        action_index: int | None
        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            action_index = None
        elif self._phase in {
            ReconfigurationPhase.APPROACHING,
            ReconfigurationPhase.DOCKING,
            ReconfigurationPhase.HOLDING_CONNECTED,
        }:
            action_index = 0
        else:
            action_index = 1
        return ReconfigurationStatus(
            phase=self._phase,
            time_s=self.session.world.time_s,
            plan_id=self.config.plan_id,
            plan_name=self.config.plan_name,
            action_index=action_index,
            action_count=2,
            detail=self._detail,
        )

    def step(self) -> tuple[Event, ...]:
        """Advance one dynamic step and update the docking controller."""
        self._update_control_if_due()
        events: list[Event] = list(self.session.step(self.config.dt_s))

        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            if self._held_for(self.config.settle_s):
                self._enter_phase(
                    ReconfigurationPhase.APPROACHING,
                    "Driving the moving module toward the target with its wheel joints",
                )
        elif self._phase is ReconfigurationPhase.APPROACHING:
            events.extend(self._advance_approach())
        elif self._phase is ReconfigurationPhase.DOCKING:
            if self.config.connection in self.session.world.connections:
                self._enter_phase(
                    ReconfigurationPhase.HOLDING_CONNECTED,
                    "Physical approach complete; holding the idealized fixed EP-face weld",
                )
            else:
                self._fail("The requested physical dock did not remain committed")
        elif self._phase is ReconfigurationPhase.HOLDING_CONNECTED:
            if self._held_for(self.config.connected_hold_s):
                self._enter_phase(
                    ReconfigurationPhase.UNDOCKING,
                    "Waiting through the modeled EP-face release delay before removing the weld",
                )
        elif self._phase is ReconfigurationPhase.UNDOCKING:
            if self._held_for(self.config.release_delay_s):
                events.extend(self._release())
        elif self._phase is ReconfigurationPhase.HOLDING_SEPARATED:
            self._advance_retraction()

        return tuple(events)

    def _advance_approach(self) -> tuple[Event, ...]:
        if self._held_for(self.config.approach_timeout_s):
            self._fail("Timed out before the measured connector frames entered acceptance")
            return ()

        proposal = next(
            (
                item
                for item in self.session.proposals()
                if item.connection_id == self.config.connection
            ),
            None,
        )
        if proposal is None or not (
            proposal.compatibility.compatible and proposal.acceptance.satisfied
        ):
            return ()

        # The pack-level acceptance region describes where capture is allowed,
        # not where this first-pass dynamics model should materialize its ideal
        # weld. With magnetic attraction still deferred, continue the physical
        # wheel approach to near contact so a 6 mm tolerance cannot create a
        # visibly floating connection.
        position_error_m = proposal.acceptance.criterion("position").measured
        if position_error_m > self.config.latch_distance_m:
            return ()

        self._apply_control(linear_m_s=0.0, yaw_rate_rad_s=0.0)
        self.session.request_dock(
            self.config.fixed_connector,
            self.config.moving_connector,
        )
        events = self.session.process_docking()
        failure = next(
            (event for event in events if isinstance(event, DockFailed)),
            None,
        )
        if failure is not None:
            self._fail(f"Physical dock failed ({failure.reason.value}): {failure.detail}")
        elif self.config.connection in self.session.world.connections:
            self._enter_phase(
                ReconfigurationPhase.DOCKING,
                "Near-contact connector acceptance satisfied; fixed weld committed",
            )
        else:
            self._fail("Dock request produced no committed connection")
        return events

    def _release(self) -> tuple[Event, ...]:
        self.session.request_undock(self.config.connection)
        events = self.session.process_docking()
        failure = next((event for event in events if isinstance(event, UndockFailed)), None)
        if failure is not None:
            self._fail(f"Physical undock failed: {failure.detail}")
            return events
        if self.config.connection in self.session.world.connections:
            self._fail("Undock request left the physical connection active")
            return events

        self._retract_origin = self.session.world.modules[
            self.config.moving_module
        ].pose.translation
        fixed_axis = self.session.world.connector(self.config.fixed_connector).world_docking_axis
        self._retract_direction = _planar_unit(fixed_axis)
        self._enter_phase(
            ReconfigurationPhase.HOLDING_SEPARATED,
            "Weld released; reversing the moving module through wheel effort",
        )
        return events

    def _advance_retraction(self) -> None:
        if self.config.retract_distance_m <= 0.0:
            self._complete()
            return
        origin = self._retract_origin
        direction = self._retract_direction
        if origin is None or direction is None:  # pragma: no cover - phase invariant
            raise AssertionError("retraction phase requires a recorded origin and direction")
        current = self.session.world.modules[self.config.moving_module].pose.translation
        travelled = vec_dot(vec_sub(current, origin), direction)
        if travelled + _EPSILON >= self.config.retract_distance_m:
            self._complete()

    def _update_control_if_due(self) -> None:
        if self.session.world.time_s + _EPSILON < self._next_control_at_s:
            return
        linear_m_s = 0.0
        yaw_rate_rad_s = 0.0
        if self._phase is ReconfigurationPhase.APPROACHING:
            linear_m_s, yaw_rate_rad_s = self._approach_twist()
        elif self._phase is ReconfigurationPhase.HOLDING_SEPARATED:
            linear_m_s = -self.config.retract_speed_m_s
        self._apply_control(linear_m_s=linear_m_s, yaw_rate_rad_s=yaw_rate_rad_s)
        while self._next_control_at_s <= self.session.world.time_s + _EPSILON:
            self._next_control_at_s += self.config.controller_period_s

    def _approach_twist(self) -> tuple[float, float]:
        fixed = self.session.world.connector(self.config.fixed_connector)
        moving = self.session.world.connector(self.config.moving_connector)
        desired_forward = _planar_unit(
            (-fixed.world_docking_axis[0], -fixed.world_docking_axis[1], 0.0)
        )
        root = self.session.world.modules[self.config.moving_module]
        current_forward = _planar_unit(quat_rotate(root.pose.rotation, (1.0, 0.0, 0.0)))
        heading_error = math.atan2(
            current_forward[0] * desired_forward[1] - current_forward[1] * desired_forward[0],
            current_forward[0] * desired_forward[0] + current_forward[1] * desired_forward[1],
        )
        error = vec_sub(fixed.world_pose.translation, moving.world_pose.translation)
        left = (-desired_forward[1], desired_forward[0], 0.0)
        lateral_error_m = vec_dot(error, left)
        yaw_rate = (
            self.config.heading_gain_per_s * heading_error
            + self.config.lateral_gain_rad_per_m_s * lateral_error_m
        )
        yaw_rate = max(
            -self.config.maximum_yaw_rate_rad_s,
            min(self.config.maximum_yaw_rate_rad_s, yaw_rate),
        )
        axial_error_m = max(0.0, vec_dot(error, desired_forward))
        linear = min(self.config.approach_speed_m_s, max(0.004, axial_error_m * 1.2))
        if abs(heading_error) > 0.35:
            linear = 0.0
        return linear, yaw_rate

    def _apply_control(
        self,
        *,
        linear_m_s: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
    ) -> None:
        left_target, right_target = self.config.geometry.wheel_velocity_targets(
            linear_m_s,
            yaw_rate_rad_s,
        )
        maximum = self.config.maximum_wheel_speed_rad_s
        left_target = max(-maximum, min(maximum, left_target))
        right_target = max(-maximum, min(maximum, right_target))

        commands: list[JointCommand] = []
        for module_id, targets in (
            (self.config.fixed_module, (0.0, 0.0)),
            (self.config.moving_module, (left_target, right_target)),
        ):
            commands.extend(self._wheel_commands(module_id, *targets))
            commands.extend(self._hold_commands(module_id))
        try:
            self.session.set_joint_commands(commands)
        except JointCommandError as error:
            raise ReconfigurationScenarioError(
                f"could not command physical docking joints: {error}"
            ) from error

    def _wheel_commands(
        self,
        module_id: ModuleInstanceId,
        left_target_rad_s: float,
        right_target_rad_s: float,
    ) -> tuple[JointCommand, JointCommand]:
        left = joint_instance_id(module_id, self.config.left_wheel_joint)
        right = joint_instance_id(module_id, self.config.right_wheel_joint)
        left_effort = self.config.wheel_controller.effort_nm(
            left_target_rad_s,
            self.session.world.joint_state(left).velocity,
        )
        right_effort = self.config.wheel_controller.effort_nm(
            right_target_rad_s,
            self.session.world.joint_state(right).velocity,
        )
        return (
            JointCommand(left, ControlMode.EFFORT, left_effort),
            JointCommand(right, ControlMode.EFFORT, right_effort),
        )

    def _hold_commands(self, module_id: ModuleInstanceId) -> tuple[JointCommand, ...]:
        commands: list[JointCommand] = []
        for target in self.config.hold_joints:
            joint = joint_instance_id(module_id, target.joint_id)
            state = self.session.world.joint_state(joint)
            effort = self.config.hold_controller.effort_nm(
                target.target_rad,
                state.position,
                state.velocity,
                continuous=target.continuous,
            )
            commands.append(JointCommand(joint, ControlMode.EFFORT, effort))
        return tuple(commands)

    def _require_feedback(self) -> None:
        joints: list[JointInstanceId] = []
        for module_id in (self.config.fixed_module, self.config.moving_module):
            joints.extend(
                (
                    joint_instance_id(module_id, self.config.left_wheel_joint),
                    joint_instance_id(module_id, self.config.right_wheel_joint),
                )
            )
            joints.extend(
                joint_instance_id(module_id, item.joint_id) for item in self.config.hold_joints
            )
        missing: list[str] = []
        for joint in joints:
            try:
                self.session.world.joint_state(joint)
            except (KeyError, ValueError):
                missing.append(str(joint))
        if missing:
            raise ReconfigurationScenarioError(
                "backend did not report required physical docking joint state(s): "
                + ", ".join(missing)
            )

    def _require_joint_control(self) -> None:
        """Validate the complete zero-effort batch before staging either root."""
        commands: list[JointCommand] = []
        for module_id in (self.config.fixed_module, self.config.moving_module):
            local_ids = (
                self.config.left_wheel_joint,
                self.config.right_wheel_joint,
                *(target.joint_id for target in self.config.hold_joints),
            )
            commands.extend(
                JointCommand(
                    joint_instance_id(module_id, local_id),
                    ControlMode.EFFORT,
                    0.0,
                )
                for local_id in local_ids
            )
        try:
            self.session.set_joint_commands(commands)
        except (JointCommandError, BackendError) as error:
            raise ReconfigurationScenarioError(
                f"backend cannot execute physical docking joint efforts: {error}"
            ) from error

    def _held_for(self, duration_s: float) -> bool:
        return self.session.world.time_s - self._phase_started_at_s + _EPSILON >= duration_s

    def _enter_phase(self, phase: ReconfigurationPhase, detail: str) -> None:
        self._phase = phase
        self._phase_started_at_s = self.session.world.time_s
        self._detail = detail
        self._next_control_at_s = self.session.world.time_s

    def _fail(self, detail: str) -> None:
        self._enter_phase(ReconfigurationPhase.FAILED, detail)
        self._apply_control()

    def _complete(self) -> None:
        self._enter_phase(
            ReconfigurationPhase.COMPLETE,
            "Completed physical wheel-driven dock, release, and retraction",
        )
        self._apply_control()


def _planar_unit(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    planar = (vector[0], vector[1], 0.0)
    magnitude = vec_norm(planar)
    if magnitude <= _EPSILON:
        raise ReconfigurationScenarioError("physical docking direction has no planar component")
    return (planar[0] / magnitude, planar[1] / magnitude, 0.0)


__all__ = [
    "MAX_PHYSICAL_TIMESTEP_S",
    "DifferentialDriveDockingConfig",
    "DifferentialDriveDockingScenario",
    "JointHoldTarget",
]
