"""Wheel-driven execution of an authored modular-robot reconfiguration plan.

The initial connection forest is staged once before simulation time advances.
After that boundary this scenario issues only joint efforts and ordinary
dock/undock requests.  MuJoCo (or another joint-command physics backend) owns
gravity, contact, rigid-body motion, and connection constraints.

Routes are expressed relative to the final moving-module root pose for each
dock.  That keeps platform-specific collision-clearance choices outside the
generic controller while allowing a moving connected component to follow the
same target if the stationary component shifts under contact.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modsim.backends.base import BackendError
from modsim.core.entities import JointCommand
from modsim.core.events import DockFailed, Event, UndockFailed
from modsim.core.ids import (
    ConnectorInstanceId,
    JointInstanceId,
    ModuleInstanceId,
    joint_instance_id,
    split_connector_instance_id,
)
from modsim.core.transforms import (
    Transform,
    Vec3,
    project_onto_plane,
    quat_rotate,
    signed_angle_about,
    vec_norm,
    wrap_angle,
)
from modsim.core.validation import (
    require_finite,
    require_finite_nonnegative,
    require_finite_positive,
)
from modsim.robot_packs.schema import AlignmentMode, ControlMode, OrientationMode
from modsim.runtime.differential_drive import (
    DifferentialDriveGeometry,
    PositionEffortController,
    VelocityEffortController,
    allocate_rigid_assembly_wheel_targets,
)
from modsim.runtime.physics_docking import MAX_PHYSICAL_TIMESTEP_S, JointHoldTarget
from modsim.runtime.reconfiguration import (
    ReconfigurationAction,
    ReconfigurationPhase,
    ReconfigurationPlan,
    ReconfigurationScenarioError,
    ReconfigurationStatus,
    initialize_reconfiguration_plan,
)
from modsim.runtime.session import JointCommandError, RuntimeSession

_EPSILON = 1e-12
_WAYPOINT_HYSTERESIS = 1.05
_TURN_IN_PLACE_ENTER_RAD = 0.5
_TURN_IN_PLACE_EXIT_RAD = 0.18


@dataclass(frozen=True, slots=True)
class TargetRelativeWaypoint:
    """One planar waypoint relative to the action's final moving-root pose.

    ``forward_m`` and ``left_m`` are resolved in the final root heading.
    ``heading_rad`` is an offset from that same heading. ``drive_direction``
    is ``1`` for forward travel and ``-1`` for reverse travel.
    """

    forward_m: float
    left_m: float
    heading_rad: float = 0.0
    drive_direction: int = 1
    position_tolerance_m: float = 0.012
    heading_tolerance_rad: float = 0.12

    def __post_init__(self) -> None:
        require_finite(self.forward_m, "waypoint forward_m")
        require_finite(self.left_m, "waypoint left_m")
        require_finite(self.heading_rad, "waypoint heading_rad")
        if self.drive_direction not in {-1, 1}:
            raise ValueError("waypoint drive_direction must be -1 or 1")
        require_finite_positive(
            self.position_tolerance_m,
            "waypoint position_tolerance_m",
        )
        require_finite_positive(
            self.heading_tolerance_rad,
            "waypoint heading_tolerance_rad",
        )


@dataclass(frozen=True, slots=True)
class PhysicalActionRoute:
    """Authored navigation corridor and final approach direction for one action."""

    waypoints: tuple[TargetRelativeWaypoint, ...]
    approach_direction: int

    def __post_init__(self) -> None:
        if self.approach_direction not in {-1, 1}:
            raise ValueError("route approach_direction must be -1 or 1")


@dataclass(frozen=True, slots=True)
class DifferentialDriveReconfigurationConfig:
    """Platform mechanics, route data, and bounded feedback parameters."""

    left_wheel_joint: str
    right_wheel_joint: str
    hold_joints: tuple[JointHoldTarget, ...]
    connector_roll_joint: str | None
    connector_roll_connector: str | None
    geometry: DifferentialDriveGeometry
    wheel_controller: VelocityEffortController
    hold_controller: PositionEffortController
    routes: tuple[PhysicalActionRoute, ...]
    dt_s: float = 0.002
    controller_period_s: float = 0.002
    navigation_speed_m_s: float = 0.04
    approach_speed_m_s: float = 0.02
    maximum_wheel_speed_rad_s: float = math.pi / 2.0
    maximum_yaw_rate_rad_s: float = 0.55
    heading_gain_per_s: float = 2.5
    heading_velocity_gain: float = 0.8
    approach_lateral_gain_rad_per_m_s: float = 8.0
    approach_orientation_gain_per_s: float = 3.0
    position_gain_per_s: float = 1.2
    lateral_residual_limit_m_s: float = 0.08
    settle_s: float = 0.5
    release_delay_s: float = 0.08
    connected_hold_s: float = 0.3
    latch_distance_m: float = 0.001
    action_timeout_s: float = 45.0

    def __post_init__(self) -> None:
        for label, value in (
            ("left_wheel_joint", self.left_wheel_joint),
            ("right_wheel_joint", self.right_wheel_joint),
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
        if self.connector_roll_joint is not None and self.connector_roll_joint not in hold_ids:
            raise ValueError("connector_roll_joint must identify one of the hold joints")
        if (self.connector_roll_joint is None) != (self.connector_roll_connector is None):
            raise ValueError(
                "connector_roll_joint and connector_roll_connector must be supplied together"
            )
        for name, value in (
            ("dt_s", self.dt_s),
            ("controller_period_s", self.controller_period_s),
            ("navigation_speed_m_s", self.navigation_speed_m_s),
            ("approach_speed_m_s", self.approach_speed_m_s),
            ("maximum_wheel_speed_rad_s", self.maximum_wheel_speed_rad_s),
            ("maximum_yaw_rate_rad_s", self.maximum_yaw_rate_rad_s),
            ("heading_gain_per_s", self.heading_gain_per_s),
            (
                "approach_lateral_gain_rad_per_m_s",
                self.approach_lateral_gain_rad_per_m_s,
            ),
            (
                "approach_orientation_gain_per_s",
                self.approach_orientation_gain_per_s,
            ),
            ("position_gain_per_s", self.position_gain_per_s),
            ("lateral_residual_limit_m_s", self.lateral_residual_limit_m_s),
            ("latch_distance_m", self.latch_distance_m),
            ("action_timeout_s", self.action_timeout_s),
        ):
            require_finite_positive(value, name)
        require_finite_nonnegative(
            self.heading_velocity_gain,
            "heading_velocity_gain",
        )
        if self.dt_s > MAX_PHYSICAL_TIMESTEP_S:
            raise ValueError(
                f"dt_s must not exceed {MAX_PHYSICAL_TIMESTEP_S:g} for physical reconfiguration"
            )
        for name, value in (
            ("settle_s", self.settle_s),
            ("release_delay_s", self.release_delay_s),
            ("connected_hold_s", self.connected_hold_s),
        ):
            require_finite_nonnegative(value, name)


@dataclass(frozen=True, slots=True)
class _PlanarTarget:
    position_m: Vec3
    yaw_rad: float


@dataclass(slots=True)
class DifferentialDriveReconfigurationScenario:
    """Execute a reconfiguration plan with wheel effort after initial staging."""

    session: RuntimeSession
    plan: ReconfigurationPlan
    config: DifferentialDriveReconfigurationConfig
    _phase: ReconfigurationPhase
    _phase_started_at_s: float
    _detail: str
    _next_control_at_s: float
    _action_index: int | None = None
    _waypoint_index: int = 0
    _turning_to_path: bool = False
    _moving_modules: tuple[ModuleInstanceId, ...] = ()
    _action_motion_started_at_s: float | None = None
    _maximum_requested_lateral_residual_m_s: float = 0.0
    _maximum_lateral_residual_m_s: float = 0.0
    _maximum_wheel_speed_rad_s_observed: float = 0.0

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        plan: ReconfigurationPlan,
        config: DifferentialDriveReconfigurationConfig,
    ) -> DifferentialDriveReconfigurationScenario:
        """Stage an upright initial forest, validate feedback, and begin settling."""
        if len(config.routes) != len(plan.actions):
            raise ReconfigurationScenarioError(
                f"physical plan '{plan.id}' has {len(plan.actions)} action(s) but "
                f"{len(config.routes)} route(s)"
            )
        for index, action in enumerate(plan.actions):
            if action.undock is None or action.dock is None:
                raise ReconfigurationScenarioError(
                    f"physical action {index + 1} '{action.label}' must replace one connection"
                )
        if abs(session.world.time_s) > _EPSILON:
            raise ReconfigurationScenarioError(
                "physical reconfiguration must be created at simulation time zero"
            )
        scenario = cls(
            session=session,
            plan=plan,
            config=config,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _detail=(
                f"Settling the upright initial configuration for '{plan.name}' "
                "under gravity and wheel brakes"
            ),
            _next_control_at_s=session.world.time_s,
        )
        scenario._require_feedback()
        scenario._require_measured_alignment()
        scenario._require_joint_control()
        initialize_reconfiguration_plan(
            session,
            plan,
            preserve_orientation=True,
        )
        scenario._apply_control()
        return scenario

    @property
    def status(self) -> ReconfigurationStatus:
        """Return immutable progress for the Runtime Inspector."""
        return ReconfigurationStatus(
            phase=self._phase,
            time_s=self.session.world.time_s,
            plan_id=self.plan.id,
            plan_name=self.plan.name,
            action_index=self._action_index,
            action_count=len(self.plan.actions),
            detail=self._detail,
        )

    @property
    def maximum_lateral_residual_m_s(self) -> float:
        """Return the largest applied nonholonomic slip component so far."""
        return self._maximum_lateral_residual_m_s

    @property
    def maximum_requested_lateral_residual_m_s(self) -> float:
        """Return the largest pre-scaling nonholonomic slip request so far."""
        return self._maximum_requested_lateral_residual_m_s

    @property
    def maximum_wheel_speed_rad_s_observed(self) -> float:
        """Return the largest absolute measured wheel speed so far."""
        return self._maximum_wheel_speed_rad_s_observed

    def step(self) -> tuple[Event, ...]:
        """Advance one physics step and update the route controller."""
        self._update_control_if_due()
        events: list[Event] = list(self.session.step(self.config.dt_s))

        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            if self._held_for(self.config.settle_s):
                if self.plan.actions:
                    self._start_action(0)
                else:
                    self._complete()
        elif self._phase is ReconfigurationPhase.UNDOCKING:
            if self._held_for(self.config.release_delay_s):
                events.extend(self._release_current_connection())
        elif self._phase is ReconfigurationPhase.HOLDING_SEPARATED:
            self._advance_waypoint()
        elif self._phase is ReconfigurationPhase.APPROACHING:
            events.extend(self._advance_approach())
        elif self._phase is ReconfigurationPhase.DOCKING:
            action = self._current_action()
            assert action.dock is not None
            if action.dock.connection_id in self.session.world.connections:
                self._enter_phase(
                    ReconfigurationPhase.HOLDING_CONNECTED,
                    f"{action.label}: holding the measured connection constraint",
                )
            else:
                self._fail(f"{action.label}: requested weld did not remain committed")
        elif self._phase is ReconfigurationPhase.HOLDING_CONNECTED:
            action = self._current_action()
            assert action.dock is not None
            if action.dock.connection_id not in self.session.world.connections:
                self._fail(f"{action.label}: committed connection was lost during its hold")
            elif self._held_for(self.config.connected_hold_s):
                assert self._action_index is not None
                next_index = self._action_index + 1
                if next_index >= len(self.plan.actions):
                    self._complete()
                else:
                    self._start_action(next_index)

        return tuple(events)

    def _start_action(self, index: int) -> None:
        action = self.plan.actions[index]
        assert action.undock is not None
        if action.undock.connection_id not in self.session.world.connections:
            self._fail(
                f"action {index + 1} '{action.label}' expected active connection "
                f"'{action.undock.connection_id}'"
            )
            return
        self._action_index = index
        self._waypoint_index = 0
        self._turning_to_path = False
        self._moving_modules = ()
        self._action_motion_started_at_s = None
        self._enter_phase(
            ReconfigurationPhase.UNDOCKING,
            f"{action.label}: waiting {self.config.release_delay_s:.3f} s before "
            "releasing the physical connection",
        )

    def _release_current_connection(self) -> tuple[Event, ...]:
        action = self._current_action()
        assert action.undock is not None and action.dock is not None
        self.session.request_undock(action.undock.connection_id)
        events = self.session.process_docking()
        failure = next((event for event in events if isinstance(event, UndockFailed)), None)
        if failure is not None:
            self._fail(f"{action.label}: physical undock failed: {failure.detail}")
            return events
        if action.undock.connection_id in self.session.world.connections:
            self._fail(f"{action.label}: weld release left the connection active")
            return events

        moving_module = split_connector_instance_id(action.dock.moving_connector)[0]
        moving_assembly = self.session.world.assemblies.assembly_of(moving_module)
        self._moving_modules = tuple(sorted(self.session.world.assemblies.members(moving_assembly)))
        fixed_module = split_connector_instance_id(action.dock.fixed_connector)[0]
        if fixed_module in self._moving_modules:
            self._fail(f"{action.label}: release did not separate the target components")
            return events
        self._action_motion_started_at_s = self.session.world.time_s
        route = self._current_route()
        if route.waypoints:
            self._enter_phase(
                ReconfigurationPhase.HOLDING_SEPARATED,
                self._waypoint_detail(action, route, 0),
            )
        else:
            self._enter_phase(
                ReconfigurationPhase.APPROACHING,
                f"{action.label}: beginning straight measured-frame approach",
            )
        return events

    def _advance_waypoint(self) -> None:
        if self._action_timed_out():
            self._fail(f"{self._current_action().label}: timed out following physical route")
            return
        target = self._target_root_pose()
        waypoint = self._current_waypoint()
        desired = self._resolve_waypoint(target, waypoint)
        moving_module = self._moving_reference_module()
        current = self.session.world.modules[moving_module].pose
        distance = math.hypot(
            desired.position_m[0] - current.translation[0],
            desired.position_m[1] - current.translation[1],
        )
        if distance > waypoint.position_tolerance_m * _WAYPOINT_HYSTERESIS:
            return
        if abs(wrap_angle(desired.yaw_rad - _yaw_of(current.rotation))) > (
            waypoint.heading_tolerance_rad
        ):
            return
        self._waypoint_index += 1
        self._turning_to_path = False
        route = self._current_route()
        action = self._current_action()
        if self._waypoint_index < len(route.waypoints):
            self._detail = self._waypoint_detail(action, route, self._waypoint_index)
            return
        self._enter_phase(
            ReconfigurationPhase.APPROACHING,
            f"{action.label}: route clear; beginning straight measured-frame approach",
        )

    def _advance_approach(self) -> tuple[Event, ...]:
        action = self._current_action()
        assert action.dock is not None
        if self._action_timed_out():
            self._fail(f"{action.label}: timed out before connector capture")
            return ()
        proposal = next(
            (
                item
                for item in self.session.proposals()
                if item.connection_id == action.dock.connection_id
            ),
            None,
        )
        if proposal is None or not (
            proposal.compatibility.compatible and proposal.acceptance.satisfied
        ):
            return ()
        position_error_m = proposal.acceptance.criterion("position").measured
        if position_error_m > self.config.latch_distance_m:
            return ()

        self._apply_control()
        self.session.request_dock(
            action.dock.fixed_connector,
            action.dock.moving_connector,
        )
        events = self.session.process_docking()
        failure = next((event for event in events if isinstance(event, DockFailed)), None)
        if failure is not None:
            self._fail(
                f"{action.label}: physical dock failed ({failure.reason.value}): {failure.detail}"
            )
        elif action.dock.connection_id in self.session.world.connections:
            self._enter_phase(
                ReconfigurationPhase.DOCKING,
                f"{action.label}: near-contact acceptance satisfied; weld committed",
            )
        else:
            self._fail(f"{action.label}: dock request produced no connection")
        return events

    def _update_control_if_due(self) -> None:
        if self.session.world.time_s + _EPSILON < self._next_control_at_s:
            return
        linear_m_s = 0.0
        yaw_rate_rad_s = 0.0
        if self._phase is ReconfigurationPhase.HOLDING_SEPARATED:
            linear_m_s, yaw_rate_rad_s = self._navigation_twist()
        elif self._phase is ReconfigurationPhase.APPROACHING:
            linear_m_s, yaw_rate_rad_s = self._approach_twist()
        self._apply_control(
            linear_m_s=linear_m_s,
            yaw_rate_rad_s=yaw_rate_rad_s,
        )
        while self._next_control_at_s <= self.session.world.time_s + _EPSILON:
            self._next_control_at_s += self.config.controller_period_s

    def _navigation_twist(self) -> tuple[float, float]:
        waypoint = self._current_waypoint()
        target = self._resolve_waypoint(self._target_root_pose(), waypoint)
        return self._pose_control_twist(
            target,
            drive_direction=waypoint.drive_direction,
            position_tolerance_m=waypoint.position_tolerance_m,
            heading_tolerance_rad=waypoint.heading_tolerance_rad,
            speed_m_s=self.config.navigation_speed_m_s,
            allow_near_target_reversal=False,
        )

    def _approach_twist(self) -> tuple[float, float]:
        route = self._current_route()
        target = self._target_root_pose()
        moving = self.session.world.modules[self._moving_reference_module()]
        dx = target.position_m[0] - moving.pose.translation[0]
        dy = target.position_m[1] - moving.pose.translation[1]
        distance_m = math.hypot(dx, dy)
        current_yaw = _yaw_of(moving.pose.rotation)
        direction = route.approach_direction
        effective_yaw = current_yaw if direction > 0 else wrap_angle(current_yaw + math.pi)
        goal_effective_yaw = (
            target.yaw_rad if direction > 0 else wrap_angle(target.yaw_rad + math.pi)
        )
        effective_forward = (
            math.cos(goal_effective_yaw),
            math.sin(goal_effective_yaw),
        )
        effective_left = (-effective_forward[1], effective_forward[0])
        axial_error_m = dx * effective_forward[0] + dy * effective_forward[1]
        lateral_error_m = dx * effective_left[0] + dy * effective_left[1]
        orientation_error = wrap_angle(goal_effective_yaw - effective_yaw)
        turn_rate = self._turn_in_place_rate(orientation_error)
        if turn_rate is not None:
            return 0.0, turn_rate
        yaw_rate = self._bounded_yaw(
            self.config.approach_orientation_gain_per_s * orientation_error
            + self.config.approach_lateral_gain_rad_per_m_s * lateral_error_m
            - self.config.heading_velocity_gain * moving.angular_velocity_rad_s[2]
        )
        signed_effective_speed = max(
            -self.config.approach_speed_m_s,
            min(
                self.config.approach_speed_m_s,
                self.config.position_gain_per_s * axial_error_m,
            ),
        )
        if abs(axial_error_m) > self.config.latch_distance_m * 0.25:
            signed_effective_speed = math.copysign(
                max(0.003, abs(signed_effective_speed)),
                signed_effective_speed,
            )
        if distance_m <= _EPSILON:
            signed_effective_speed = 0.0
        return direction * signed_effective_speed, yaw_rate

    def _pose_control_twist(
        self,
        target: _PlanarTarget,
        *,
        drive_direction: int,
        position_tolerance_m: float,
        heading_tolerance_rad: float,
        speed_m_s: float,
        allow_near_target_reversal: bool,
    ) -> tuple[float, float]:
        moving_state = self.session.world.modules[self._moving_reference_module()]
        moving = moving_state.pose
        dx = target.position_m[0] - moving.translation[0]
        dy = target.position_m[1] - moving.translation[1]
        distance = math.hypot(dx, dy)
        current_yaw = _yaw_of(moving.rotation)
        if distance <= position_tolerance_m * _WAYPOINT_HYSTERESIS:
            heading_error = wrap_angle(target.yaw_rad - current_yaw)
            if abs(heading_error) <= heading_tolerance_rad:
                return 0.0, 0.0
            return 0.0, self._heading_rate(heading_error)

        path_heading = math.atan2(dy, dx)
        resolved_direction = drive_direction
        desired_heading = (
            path_heading if resolved_direction > 0 else wrap_angle(path_heading + math.pi)
        )
        heading_error = wrap_angle(desired_heading - current_yaw)
        if allow_near_target_reversal and distance <= max(
            0.04,
            position_tolerance_m * 3.0,
        ):
            alternate_heading = wrap_angle(desired_heading + math.pi)
            alternate_error = wrap_angle(alternate_heading - current_yaw)
            if abs(alternate_error) + 0.1 < abs(heading_error):
                resolved_direction *= -1
                heading_error = alternate_error
        turn_rate = self._turn_in_place_rate(heading_error)
        if turn_rate is not None:
            return 0.0, turn_rate
        yaw_rate = self._heading_rate(heading_error)
        # Slow down inside the final few centimetres so a low-friction skid
        # does not carry the assembly through the waypoint and into an orbit.
        # A small floor is retained outside the arrival tolerance to overcome
        # static contact friction.
        remaining_m = max(0.0, distance - position_tolerance_m * 0.5)
        magnitude = min(
            speed_m_s,
            max(0.002, self.config.position_gain_per_s * remaining_m),
        )
        magnitude *= max(0.25, math.cos(heading_error))
        return resolved_direction * magnitude, yaw_rate

    def _apply_control(
        self,
        *,
        linear_m_s: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
    ) -> None:
        wheel_targets: dict[ModuleInstanceId, tuple[float, float]] = {}
        if self._moving_modules and (abs(linear_m_s) > _EPSILON or abs(yaw_rate_rad_s) > _EPSILON):
            reference_module = self._moving_reference_module()
            reference_pose = self.session.world.modules[reference_module].pose
            forward = _planar_unit(quat_rotate(reference_pose.rotation, (1.0, 0.0, 0.0)))
            reference_velocity = (
                forward[0] * linear_m_s,
                forward[1] * linear_m_s,
                0.0,
            )
            module_poses = {
                module_id: self.session.world.modules[module_id].pose
                for module_id in self._moving_modules
            }
            allocation = allocate_rigid_assembly_wheel_targets(
                self.config.geometry,
                module_poses,
                reference_position_m=reference_pose.translation,
                reference_linear_velocity_m_s=reference_velocity,
                yaw_rate_rad_s=yaw_rate_rad_s,
                lateral_residual_limit_m_s=self.config.lateral_residual_limit_m_s,
            )
            self._maximum_requested_lateral_residual_m_s = max(
                self._maximum_requested_lateral_residual_m_s,
                allocation.maximum_lateral_residual_m_s,
            )
            if not allocation.within_lateral_residual_limit:
                scale = (
                    self.config.lateral_residual_limit_m_s / allocation.maximum_lateral_residual_m_s
                )
                allocation = allocate_rigid_assembly_wheel_targets(
                    self.config.geometry,
                    module_poses,
                    reference_position_m=reference_pose.translation,
                    reference_linear_velocity_m_s=(
                        reference_velocity[0] * scale,
                        reference_velocity[1] * scale,
                        0.0,
                    ),
                    yaw_rate_rad_s=yaw_rate_rad_s * scale,
                    lateral_residual_limit_m_s=self.config.lateral_residual_limit_m_s,
                )
            self._maximum_lateral_residual_m_s = max(
                self._maximum_lateral_residual_m_s,
                allocation.maximum_lateral_residual_m_s,
            )
            peak = max(
                (
                    abs(value)
                    for target in allocation.targets
                    for value in (target.left_rad_s, target.right_rad_s)
                ),
                default=0.0,
            )
            wheel_scale = min(
                1.0,
                self.config.maximum_wheel_speed_rad_s / peak if peak > 0.0 else 1.0,
            )
            wheel_targets = {
                target.module_id: (
                    target.left_rad_s * wheel_scale,
                    target.right_rad_s * wheel_scale,
                )
                for target in allocation.targets
            }

        commands: list[JointCommand] = []
        for module_id in sorted(self.session.world.modules):
            left_target, right_target = wheel_targets.get(module_id, (0.0, 0.0))
            commands.extend(self._wheel_commands(module_id, left_target, right_target))
            commands.extend(self._hold_commands(module_id))
        try:
            self.session.set_joint_commands(commands)
        except JointCommandError as error:
            raise ReconfigurationScenarioError(
                f"could not command physical reconfiguration joints: {error}"
            ) from error

    def _wheel_commands(
        self,
        module_id: ModuleInstanceId,
        left_target_rad_s: float,
        right_target_rad_s: float,
    ) -> tuple[JointCommand, JointCommand]:
        left = joint_instance_id(module_id, self.config.left_wheel_joint)
        right = joint_instance_id(module_id, self.config.right_wheel_joint)
        self._maximum_wheel_speed_rad_s_observed = max(
            self._maximum_wheel_speed_rad_s_observed,
            abs(self.session.world.joint_state(left).velocity),
            abs(self.session.world.joint_state(right).velocity),
        )
        return (
            JointCommand(
                left,
                ControlMode.EFFORT,
                self.config.wheel_controller.effort_nm(
                    left_target_rad_s,
                    self.session.world.joint_state(left).velocity,
                ),
            ),
            JointCommand(
                right,
                ControlMode.EFFORT,
                self.config.wheel_controller.effort_nm(
                    right_target_rad_s,
                    self.session.world.joint_state(right).velocity,
                ),
            ),
        )

    def _hold_commands(self, module_id: ModuleInstanceId) -> tuple[JointCommand, ...]:
        commands: list[JointCommand] = []
        roll_endpoint = self._connector_roll_endpoint()
        for target in self.config.hold_joints:
            joint = joint_instance_id(module_id, target.joint_id)
            state = self.session.world.joint_state(joint)
            target_rad = target.target_rad
            if (
                self.config.connector_roll_joint == target.joint_id
                and roll_endpoint is not None
                and module_id == split_connector_instance_id(roll_endpoint[0])[0]
                and self._phase
                in {
                    ReconfigurationPhase.APPROACHING,
                    ReconfigurationPhase.DOCKING,
                    ReconfigurationPhase.HOLDING_CONNECTED,
                }
            ):
                target_rad = self._connector_roll_target(
                    state.position,
                    controlled_connector=roll_endpoint[0],
                    other_connector=roll_endpoint[1],
                )
            effort = self.config.hold_controller.effort_nm(
                target_rad,
                state.position,
                state.velocity,
                continuous=target.continuous,
            )
            commands.append(JointCommand(joint, ControlMode.EFFORT, effort))
        return tuple(commands)

    def _connector_roll_endpoint(
        self,
    ) -> tuple[ConnectorInstanceId, ConnectorInstanceId] | None:
        if self._action_index is None or self.config.connector_roll_connector is None:
            return None
        action = self._current_action()
        assert action.dock is not None
        fixed_local = split_connector_instance_id(action.dock.fixed_connector)[1]
        moving_local = split_connector_instance_id(action.dock.moving_connector)[1]
        if fixed_local == self.config.connector_roll_connector:
            return action.dock.fixed_connector, action.dock.moving_connector
        if moving_local == self.config.connector_roll_connector:
            return action.dock.moving_connector, action.dock.fixed_connector
        return None

    def _connector_roll_target(
        self,
        current_position_rad: float,
        *,
        controlled_connector: ConnectorInstanceId,
        other_connector: ConnectorInstanceId,
    ) -> float:
        controlled = self.session.world.connector(controlled_connector)
        other = self.session.world.connector(other_connector)
        orientations = self.session.world.connector_type(controlled_connector).allowed_orientations
        if orientations is None or orientations.mode is OrientationMode.CONTINUOUS:
            return current_position_rad
        axis = controlled.world_docking_axis
        controlled_reference = _frame_reference(controlled.world_pose, axis)
        other_reference = _frame_reference(other.world_pose, axis)
        roll = signed_angle_about(controlled_reference, other_reference, axis)
        correction = min(
            (wrap_angle(roll - allowed) for allowed in orientations.values_rad),
            key=abs,
        )
        return current_position_rad + correction

    def _target_root_pose(self) -> _PlanarTarget:
        action = self._current_action()
        assert action.dock is not None
        fixed = self.session.world.connector(action.dock.fixed_connector)
        moving = self.session.world.connector(action.dock.moving_connector)
        root = self.session.world.modules[moving.module_id].pose
        fixed_axis = _planar_unit(fixed.world_docking_axis)
        moving_axis = _planar_unit(moving.world_docking_axis)
        desired_axis = (-fixed_axis[0], -fixed_axis[1], 0.0)
        alignment_delta = math.atan2(
            moving_axis[0] * desired_axis[1] - moving_axis[1] * desired_axis[0],
            moving_axis[0] * desired_axis[0] + moving_axis[1] * desired_axis[1],
        )
        current_yaw = _yaw_of(root.rotation)
        target_yaw = wrap_angle(current_yaw + alignment_delta)
        relative_x = moving.world_pose.translation[0] - root.translation[0]
        relative_y = moving.world_pose.translation[1] - root.translation[1]
        cosine = math.cos(alignment_delta)
        sine = math.sin(alignment_delta)
        target_relative_x = cosine * relative_x - sine * relative_y
        target_relative_y = sine * relative_x + cosine * relative_y
        return _PlanarTarget(
            position_m=(
                fixed.world_pose.translation[0] - target_relative_x,
                fixed.world_pose.translation[1] - target_relative_y,
                root.translation[2],
            ),
            yaw_rad=target_yaw,
        )

    @staticmethod
    def _resolve_waypoint(
        target: _PlanarTarget,
        waypoint: TargetRelativeWaypoint,
    ) -> _PlanarTarget:
        forward = (math.cos(target.yaw_rad), math.sin(target.yaw_rad))
        left = (-forward[1], forward[0])
        return _PlanarTarget(
            position_m=(
                target.position_m[0] + forward[0] * waypoint.forward_m + left[0] * waypoint.left_m,
                target.position_m[1] + forward[1] * waypoint.forward_m + left[1] * waypoint.left_m,
                target.position_m[2],
            ),
            yaw_rad=wrap_angle(target.yaw_rad + waypoint.heading_rad),
        )

    def _current_action(self) -> ReconfigurationAction:
        if self._action_index is None:
            raise AssertionError("physical action phase requires an action")
        return self.plan.actions[self._action_index]

    def _current_route(self) -> PhysicalActionRoute:
        if self._action_index is None:
            raise AssertionError("physical route phase requires an action")
        return self.config.routes[self._action_index]

    def _current_waypoint(self) -> TargetRelativeWaypoint:
        route = self._current_route()
        try:
            return route.waypoints[self._waypoint_index]
        except IndexError as error:  # pragma: no cover - phase invariant
            raise AssertionError("physical route has no current waypoint") from error

    def _moving_reference_module(self) -> ModuleInstanceId:
        action = self._current_action()
        assert action.dock is not None
        return split_connector_instance_id(action.dock.moving_connector)[0]

    def _require_feedback(self) -> None:
        missing: list[str] = []
        for module_id in sorted(self.session.world.modules):
            local_ids = (
                self.config.left_wheel_joint,
                self.config.right_wheel_joint,
                *(target.joint_id for target in self.config.hold_joints),
            )
            for local_id in local_ids:
                joint: JointInstanceId = joint_instance_id(module_id, local_id)
                try:
                    self.session.world.joint_state(joint)
                except (KeyError, ValueError):
                    missing.append(str(joint))
        if missing:
            raise ReconfigurationScenarioError(
                "backend did not report required physical reconfiguration joint state(s): "
                + ", ".join(missing)
            )

    def _require_measured_alignment(self) -> None:
        for index, action in enumerate(self.plan.actions):
            assert action.dock is not None
            for connector in (
                action.dock.fixed_connector,
                action.dock.moving_connector,
            ):
                connector_type = self.session.world.connector_type(connector)
                if connector_type.effective_docking_policy.alignment is not AlignmentMode.MEASURED:
                    raise ReconfigurationScenarioError(
                        f"physical action {index + 1} '{action.label}' requires measured "
                        f"alignment, but connector '{connector}' declares "
                        f"'{connector_type.effective_docking_policy.alignment.value}'"
                    )

    def _require_joint_control(self) -> None:
        """Validate the complete zero-effort batch before staging connections."""
        commands: list[JointCommand] = []
        for module_id in sorted(self.session.world.modules):
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
                f"backend cannot execute physical reconfiguration joint efforts: {error}"
            ) from error

    def _waypoint_detail(
        self,
        action: ReconfigurationAction,
        route: PhysicalActionRoute,
        index: int,
    ) -> str:
        return (
            f"{action.label}: driving {len(self._moving_modules)} module(s) through "
            f"route waypoint {index + 1}/{len(route.waypoints)}"
        )

    def _action_timed_out(self) -> bool:
        started = self._action_motion_started_at_s
        return started is not None and (
            self.session.world.time_s - started + _EPSILON >= self.config.action_timeout_s
        )

    def _bounded_yaw(self, value: float) -> float:
        return max(
            -self.config.maximum_yaw_rate_rad_s,
            min(self.config.maximum_yaw_rate_rad_s, value),
        )

    def _heading_rate(self, heading_error_rad: float) -> float:
        measured = self.session.world.modules[
            self._moving_reference_module()
        ].angular_velocity_rad_s[2]
        return self._bounded_yaw(
            self.config.heading_gain_per_s * heading_error_rad
            - self.config.heading_velocity_gain * measured
        )

    def _turn_in_place_rate(self, heading_error_rad: float) -> float | None:
        """Return a turn command while a path-heading hysteresis latch is active."""
        error = abs(heading_error_rad)
        if self._turning_to_path:
            if error <= _TURN_IN_PLACE_EXIT_RAD:
                self._turning_to_path = False
                return None
            return self._heading_rate(heading_error_rad)
        if error >= _TURN_IN_PLACE_ENTER_RAD:
            self._turning_to_path = True
            return self._heading_rate(heading_error_rad)
        return None

    def _held_for(self, duration_s: float) -> bool:
        return self.session.world.time_s - self._phase_started_at_s + _EPSILON >= duration_s

    def _enter_phase(self, phase: ReconfigurationPhase, detail: str) -> None:
        self._phase = phase
        self._phase_started_at_s = self.session.world.time_s
        self._detail = detail
        self._next_control_at_s = self.session.world.time_s

    def _fail(self, detail: str) -> None:
        self._enter_phase(ReconfigurationPhase.FAILED, detail)
        self._moving_modules = ()
        self._apply_control()

    def _complete(self) -> None:
        expected = {pair.connection_id for pair in self.plan.initial_connections}
        for action in self.plan.actions:
            if action.undock is not None:
                expected.discard(action.undock.connection_id)
            if action.dock is not None:
                expected.add(action.dock.connection_id)
        actual = set(self.session.world.connections)
        if actual != expected:
            missing = ", ".join(sorted(expected - actual)) or "none"
            unexpected = ", ".join(sorted(actual - expected)) or "none"
            self._fail(
                "physical reconfiguration ended with the wrong topology "
                f"(missing: {missing}; unexpected: {unexpected})"
            )
            return
        self._enter_phase(
            ReconfigurationPhase.COMPLETE,
            f"Completed the physical wheel-driven '{self.plan.name}' reconfiguration",
        )
        self._moving_modules = ()
        self._apply_control()


def _yaw_of(rotation: tuple[float, float, float, float]) -> float:
    forward = quat_rotate(rotation, (1.0, 0.0, 0.0))
    planar = _planar_unit(forward)
    return math.atan2(planar[1], planar[0])


def _planar_unit(vector: Vec3) -> Vec3:
    planar = (vector[0], vector[1], 0.0)
    magnitude = vec_norm(planar)
    if magnitude <= _EPSILON:
        raise ReconfigurationScenarioError("physical route direction has no planar component")
    return (planar[0] / magnitude, planar[1] / magnitude, 0.0)


def _frame_reference(pose: Transform, axis: Vec3) -> Vec3:
    for cardinal in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        projected = project_onto_plane(pose.apply_direction(cardinal), axis)
        if vec_norm(projected) > 1e-6:
            return projected
    raise ReconfigurationScenarioError("connector frame has no direction normal to docking axis")


__all__ = [
    "DifferentialDriveReconfigurationConfig",
    "DifferentialDriveReconfigurationScenario",
    "PhysicalActionRoute",
    "TargetRelativeWaypoint",
]
