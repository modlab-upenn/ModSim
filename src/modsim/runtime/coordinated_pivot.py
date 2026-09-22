"""Coordinated rigid-slab pivots for lattice modular robots.

This module generalizes the single-module M-Blocks pivot primitives to a
rigid group.  A plan declares every fixed face released at take-off, every
fixed face created at landing, one temporary edge hinge, and the modules whose
flywheels act together.  The kinematic scenario is a deterministic reference
executor; the momentum scenario never writes module roots after construction
and leaves all motion to the backend's gravity, contact, hinge, and actuator
dynamics.

The plan deliberately supports cyclic fixed-face topologies.  A rectangular
M-Blocks mat and a staircase both contain loops, even though each moving slab
must become exactly one detached component when its authored cut is applied.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field

from modsim.backends.base import BackendError, SupportsModuleKinematics
from modsim.core.entities import JointCommand
from modsim.core.events import DockFailed, Event, UndockFailed
from modsim.core.ids import (
    ConnectionId,
    JointInstanceId,
    ModuleInstanceId,
    connection_id,
    joint_instance_id,
    split_connector_instance_id,
)
from modsim.core.transforms import (
    Transform,
    Vec3,
    quat_from_axis_angle,
    quat_rotate,
    signed_angle_about,
    vec_scale,
)
from modsim.robot_packs.schema import ControlMode, PhysicalConstraintType
from modsim.runtime.kinematic_pivot import KinematicPivotConfig, KinematicPivotRoute
from modsim.runtime.momentum_pivot import (
    PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S,
    MomentumPivotConfig,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationPhase,
    ReconfigurationScenarioError,
    ReconfigurationStatus,
)
from modsim.runtime.session import JointCommandError, RuntimeSession

_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class CoordinatedPivotAction:
    """One multi-module face-to-hinge-to-face pivot.

    Pair directions are semantic: each face and hinge points from a stationary
    module to a member of ``moving_modules``.  ``momentum`` supplies the
    primary row's measured pivot and bounded actuator parameters; the same
    command law is applied to every moving module's flywheel.
    """

    label: str
    moving_modules: tuple[ModuleInstanceId, ...]
    release_faces: tuple[ConnectorPairRef, ...]
    target_faces: tuple[ConnectorPairRef, ...]
    route: KinematicPivotRoute
    momentum: MomentumPivotConfig

    def __post_init__(self) -> None:
        if not self.label.strip():
            raise ValueError("coordinated pivot label must not be empty")
        if not self.moving_modules:
            raise ValueError("coordinated pivot requires at least one moving module")
        if len(set(self.moving_modules)) != len(self.moving_modules):
            raise ValueError("coordinated pivot moving_modules must be unique")
        if not self.release_faces:
            raise ValueError("coordinated pivot requires at least one released face")
        if not self.target_faces:
            raise ValueError("coordinated pivot requires at least one target face")
        _require_unique_pairs(self.release_faces, "release_faces")
        _require_unique_pairs(self.target_faces, "target_faces")

        moving = set(self.moving_modules)
        if self.route.moving_module not in moving:
            raise ValueError("route moving_module must belong to moving_modules")
        if self.momentum.moving_module not in moving:
            raise ValueError("momentum moving module must belong to moving_modules")
        if self.route.moving_module != self.momentum.moving_module:
            raise ValueError("route and primary momentum pivot must select the same module")
        if self.route.reference_module != self.momentum.fixed_module:
            raise ValueError("route reference_module must match the momentum hinge support")
        if not math.isclose(
            abs(self.route.angle_rad),
            self.momentum.target_pivot_angle_rad,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError("route angle must match the momentum target pivot angle")
        if self.momentum.initial_face not in self.release_faces:
            raise ValueError("primary momentum initial_face must be present in release_faces")
        if self.momentum.target_face not in self.target_faces:
            raise ValueError("primary momentum target_face must be present in target_faces")

        for label, pairs in (
            ("release face", self.release_faces),
            ("target face", self.target_faces),
            ("edge hinge", (self.momentum.edge_hinge,)),
        ):
            for pair in pairs:
                fixed = split_connector_instance_id(pair.fixed_connector)[0]
                moving_module = split_connector_instance_id(pair.moving_connector)[0]
                if fixed in moving or moving_module not in moving:
                    raise ValueError(
                        f"{label} pairs must be directed from the stationary structure "
                        "to a moving module"
                    )


@dataclass(frozen=True, slots=True)
class CoordinatedPivotPlan:
    """An exact cyclic-capable topology and its ordered slab pivots."""

    id: str
    name: str
    module_ids: tuple[ModuleInstanceId, ...]
    initial_connections: tuple[ConnectorPairRef, ...]
    actions: tuple[CoordinatedPivotAction, ...]
    source_url: str | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("coordinated pivot plan id must not be empty")
        if not self.name.strip():
            raise ValueError("coordinated pivot plan name must not be empty")
        if not self.module_ids:
            raise ValueError("coordinated pivot plan requires modules")
        if len(set(self.module_ids)) != len(self.module_ids):
            raise ValueError("coordinated pivot plan module_ids must be unique")
        if not self.actions:
            raise ValueError("coordinated pivot plan requires at least one action")
        if self.source_url is not None and not self.source_url.strip():
            raise ValueError("coordinated pivot plan source_url must not be empty")
        _require_unique_pairs(self.initial_connections, "initial_connections")

        modules = set(self.module_ids)
        all_pairs: list[tuple[str, ConnectorPairRef]] = [
            (f"initial connection {index + 1}", pair)
            for index, pair in enumerate(self.initial_connections)
        ]
        for index, action in enumerate(self.actions):
            if action.momentum.plan_id != self.id or action.momentum.plan_name != self.name:
                raise ValueError(
                    f"action {index + 1} momentum plan identity must match its parent plan"
                )
            if not set(action.moving_modules) <= modules:
                raise ValueError(f"action {index + 1} references an unknown moving module")
            all_pairs.extend(
                (f"action {index + 1} {kind}", pair)
                for kind, pairs in (
                    ("release", action.release_faces),
                    ("target", action.target_faces),
                    ("hinge", (action.momentum.edge_hinge,)),
                )
                for pair in pairs
            )
        for label, pair in all_pairs:
            referenced = {
                split_connector_instance_id(connector)[0] for connector in pair.connectors
            }
            unknown = referenced - modules
            if unknown:
                raise ValueError(
                    f"{label} references module(s) absent from the plan: "
                    f"{', '.join(sorted(unknown))}"
                )

        dt_s = self.actions[0].momentum.dt_s
        active = {pair.connection_id: pair for pair in self.initial_connections}
        _require_connector_exclusivity(active.values(), "initial topology")
        _require_connected(modules, active.values(), "initial topology")
        for index, action in enumerate(self.actions):
            context = f"action {index + 1} '{action.label}'"
            if not math.isclose(
                action.momentum.dt_s,
                dt_s,
                rel_tol=0.0,
                abs_tol=_EPSILON,
            ):
                raise ValueError("all coordinated momentum actions must use one fixed timestep")
            release_ids = {pair.connection_id for pair in action.release_faces}
            missing = release_ids - set(active)
            if missing:
                raise ValueError(
                    f"{context} releases inactive connection(s): {', '.join(sorted(missing))}"
                )
            for connection in release_ids:
                del active[connection]

            moving = set(action.moving_modules)
            components = _components(modules, active.values())
            if moving not in components:
                raise ValueError(
                    f"{context} release cut must detach exactly moving_modules as one component"
                )
            stationary = modules - moving
            if stationary and stationary not in components:
                raise ValueError(
                    f"{context} release cut must leave the stationary structure connected"
                )

            target_ids = {pair.connection_id for pair in action.target_faces}
            already_active = target_ids & set(active)
            if already_active:
                raise ValueError(
                    f"{context} targets active connection(s): {', '.join(sorted(already_active))}"
                )
            candidate = (*active.values(), *action.target_faces)
            _require_connector_exclusivity(candidate, f"{context} target topology")
            for pair in action.target_faces:
                active[pair.connection_id] = pair
            _require_connected(modules, active.values(), f"{context} target topology")

    @property
    def final_connections(self) -> tuple[ConnectorPairRef, ...]:
        """Return the exact fixed-face topology after every action."""
        active = {pair.connection_id: pair for pair in self.initial_connections}
        for action in self.actions:
            for pair in action.release_faces:
                del active[pair.connection_id]
            active.update((pair.connection_id, pair) for pair in action.target_faces)
        return tuple(active[identifier] for identifier in sorted(active))


@dataclass(frozen=True, slots=True)
class CoordinatedMomentumTelemetry:
    """Measurements for one completed physical slab pivot."""

    flywheel_speeds_rad_s: tuple[tuple[JointInstanceId, float], ...]
    pivot_angle_rad: float
    maximum_pivot_angle_rad: float
    brake_impulse_nms: float
    maximum_brake_effort_nm: float
    brake_started_at_s: float | None
    brake_ended_at_s: float | None
    capture_time_s: float | None


@dataclass(slots=True)
class CoordinatedKinematicPivotScenario:
    """Execute a coordinated plan as deterministic rigid analytical arcs."""

    session: RuntimeSession
    plan: CoordinatedPivotPlan
    config: KinematicPivotConfig
    _phase: ReconfigurationPhase
    _phase_started_at_s: float
    _action_index: int | None
    _detail: str
    _moving_modules: tuple[ModuleInstanceId, ...] = ()
    _start_relative_poses: dict[ModuleInstanceId, Transform] = field(
        default_factory=dict[ModuleInstanceId, Transform]
    )
    _baseline_connections: frozenset[ConnectionId] = frozenset()

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        plan: CoordinatedPivotPlan,
        config: KinematicPivotConfig | None = None,
    ) -> CoordinatedKinematicPivotScenario:
        """Commit the already-posed cyclic topology and begin its hold."""
        resolved = config if config is not None else KinematicPivotConfig()
        _preflight_plan(session, plan, require_joint_dynamics=False)
        _commit_exact_initial_connections(session, plan)
        scenario = cls(
            session=session,
            plan=plan,
            config=resolved,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _action_index=None,
            _detail=(
                f"Holding cyclic reference topology with "
                f"{len(plan.initial_connections)} fixed faces"
            ),
        )
        if resolved.initial_hold_s == 0.0:
            scenario._start_action(0)
        return scenario

    @property
    def status(self) -> ReconfigurationStatus:
        return ReconfigurationStatus(
            phase=self._phase,
            time_s=self.session.world.time_s,
            plan_id=self.plan.id,
            plan_name=self.plan.name,
            action_index=self._action_index,
            action_count=len(self.plan.actions),
            detail=self._detail,
        )

    def step(self) -> tuple[Event, ...]:
        """Advance one deterministic reference step."""
        if self._phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            return self.session.step(self.config.dt_s, process_connectors=False)
        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            events = self.session.step(self.config.dt_s, process_connectors=False)
            if self._held_for(self.config.initial_hold_s):
                self._start_action(0)
            return events
        if self._phase is ReconfigurationPhase.UNDOCKING:
            return self._step_undocking()
        if self._phase is ReconfigurationPhase.PIVOTING:
            return self._step_pivoting()
        if self._phase is ReconfigurationPhase.DOCKING:
            return self._step_docking()
        if self._phase is ReconfigurationPhase.HOLDING_CONNECTED:
            return self._step_connected_hold()
        raise AssertionError(f"unhandled coordinated kinematic phase '{self._phase}'")

    def _start_action(self, index: int) -> None:
        action = self.plan.actions[index]
        self._action_index = index
        self._moving_modules = ()
        self._start_relative_poses.clear()
        self._baseline_connections = frozenset(self.session.world.connections)
        missing = {pair.connection_id for pair in action.release_faces}.difference(
            self._baseline_connections
        )
        if missing:
            raise ReconfigurationScenarioError(
                f"{action.label}: expected release face(s) are inactive: "
                f"{', '.join(sorted(missing))}"
            )
        for pair in action.release_faces:
            self.session.request_undock(pair.connection_id)
        self._enter_phase(
            ReconfigurationPhase.UNDOCKING,
            f"{action.label}: releasing {len(action.release_faces)} fixed faces",
        )

    def _step_undocking(self) -> tuple[Event, ...]:
        action = self._current_action()
        events = self.session.step(self.config.dt_s)
        release_ids = {pair.connection_id for pair in action.release_faces}
        failures = [
            event
            for event in events
            if isinstance(event, UndockFailed) and event.connection_id in release_ids
        ]
        if failures:
            self._fail(f"{action.label}: face release failed: {failures[0].detail}")
            return events
        remaining = release_ids.intersection(self.session.world.connections)
        if remaining:
            self._fail(
                f"{action.label}: face release left active connection(s): "
                f"{', '.join(sorted(remaining))}"
            )
            return events
        expected = set(self._baseline_connections) - release_ids
        if not self._require_topology(expected, f"{action.label}: release"):
            return events

        assembly = self.session.world.assemblies.assembly_of(action.route.moving_module)
        members = tuple(sorted(self.session.world.assemblies.members(assembly)))
        if set(members) != set(action.moving_modules):
            self._fail(
                f"{action.label}: release detached {', '.join(members)}, expected "
                f"{', '.join(action.moving_modules)}"
            )
            return events
        reference = self._reference_pose(action.route)
        self._moving_modules = members
        self._start_relative_poses = {
            module: self.session.world.modules[module].pose.relative_to(reference)
            for module in members
        }
        self._enter_phase(
            ReconfigurationPhase.PIVOTING,
            f"{action.label}: rotating a {len(members)}-module slab",
        )
        return events

    def _step_pivoting(self) -> tuple[Event, ...]:
        action = self._current_action()
        route = action.route
        elapsed_s = self.session.world.time_s - self._phase_started_at_s
        next_elapsed_s = min(route.duration_s, elapsed_s + self.config.dt_s)
        fraction = next_elapsed_s / route.duration_s
        self._apply_pivot(fraction)
        events: list[Event] = list(self.session.step(self.config.dt_s, process_connectors=False))
        progress = min(100.0, 100.0 * fraction)
        self._detail = f"{action.label}: reference pivot {progress:.0f}% complete"
        if next_elapsed_s + _EPSILON < route.duration_s:
            return tuple(events)

        self._apply_pivot(1.0)
        self.session.world.ingest(self.session.adapter.snapshot())
        for pair in action.target_faces:
            self.session.request_dock(*pair.connectors)
        commit_events = self.session.process_docking()
        events.extend(commit_events)
        target_ids = {pair.connection_id for pair in action.target_faces}
        failures = [
            event
            for event in commit_events
            if isinstance(event, DockFailed)
            and connection_id(event.connector_a, event.connector_b) in target_ids
        ]
        if failures:
            failure = failures[0]
            self._fail(f"{action.label}: landing failed ({failure.reason.value}): {failure.detail}")
            return tuple(events)
        if not target_ids <= set(self.session.world.connections):
            missing = target_ids - set(self.session.world.connections)
            self._fail(
                f"{action.label}: landing omitted connection(s): {', '.join(sorted(missing))}"
            )
            return tuple(events)
        expected = (
            set(self._baseline_connections) - {pair.connection_id for pair in action.release_faces}
        ) | target_ids
        if self._require_topology(expected, f"{action.label}: landing"):
            self._enter_phase(
                ReconfigurationPhase.DOCKING,
                f"{action.label}: committed {len(target_ids)} landing faces",
            )
        return tuple(events)

    def _step_docking(self) -> tuple[Event, ...]:
        action = self._current_action()
        events = self.session.step(self.config.dt_s, process_connectors=False)
        targets = {pair.connection_id for pair in action.target_faces}
        if not targets <= set(self.session.world.connections):
            self._fail(f"{action.label}: a landing face was not retained")
        else:
            self._enter_phase(
                ReconfigurationPhase.HOLDING_CONNECTED,
                f"{action.label}: holding the landed slab",
            )
        return events

    def _step_connected_hold(self) -> tuple[Event, ...]:
        action = self._current_action()
        events = self.session.step(self.config.dt_s, process_connectors=False)
        targets = {pair.connection_id for pair in action.target_faces}
        if not targets <= set(self.session.world.connections):
            self._fail(f"{action.label}: target topology was lost during hold")
        elif self._held_for(self.config.connected_hold_s):
            assert self._action_index is not None
            next_index = self._action_index + 1
            if next_index == len(self.plan.actions):
                self._enter_phase(
                    ReconfigurationPhase.COMPLETE,
                    f"Completed {len(self.plan.actions)} coordinated reference pivots",
                )
            else:
                self._start_action(next_index)
        return events

    def _apply_pivot(self, fraction: float) -> None:
        action = self._current_action()
        route = action.route
        adapter = self.session.adapter
        if not isinstance(adapter, SupportsModuleKinematics):  # pragma: no cover - preflight
            raise ReconfigurationScenarioError("backend lost module-pose-write support")
        reference = self._reference_pose(route)
        rotation = Transform(
            rotation=quat_from_axis_angle(route.pivot_axis, route.angle_rad * fraction)
        )
        about_edge = (
            Transform.from_translation(route.pivot_point_m)
            .compose(rotation)
            .compose(Transform.from_translation(vec_scale(route.pivot_point_m, -1.0)))
        )
        try:
            for module in self._moving_modules:
                adapter.set_module_pose(
                    module,
                    reference.compose(about_edge).compose(self._start_relative_poses[module]),
                )
        except BackendError as error:
            raise ReconfigurationScenarioError(
                f"backend could not apply coordinated kinematic pivot: {error}"
            ) from error

    def _current_action(self) -> CoordinatedPivotAction:
        if self._action_index is None:  # pragma: no cover - phase invariant
            raise AssertionError("coordinated pivot phase requires an action")
        return self.plan.actions[self._action_index]

    def _reference_pose(self, route: KinematicPivotRoute) -> Transform:
        if route.reference_module is None:
            return Transform.identity()
        return self.session.world.modules[route.reference_module].pose

    def _held_for(self, duration_s: float) -> bool:
        return self.session.world.time_s - self._phase_started_at_s + _EPSILON >= duration_s

    def _require_topology(self, expected: set[ConnectionId], context: str) -> bool:
        actual = set(self.session.world.connections)
        if actual == expected:
            return True
        self._fail(
            f"{context} changed unrelated topology (expected: "
            f"{', '.join(sorted(expected)) or 'none'}; active: "
            f"{', '.join(sorted(actual)) or 'none'})"
        )
        return False

    def _enter_phase(self, phase: ReconfigurationPhase, detail: str) -> None:
        self._phase = phase
        self._phase_started_at_s = self.session.world.time_s
        self._detail = detail

    def _fail(self, detail: str) -> None:
        self._enter_phase(ReconfigurationPhase.FAILED, detail)


@dataclass(slots=True)
class CoordinatedMomentumPivotScenario:
    """Execute a coordinated plan using only joint effort and constraints."""

    session: RuntimeSession
    plan: CoordinatedPivotPlan
    _phase: ReconfigurationPhase
    _phase_started_at_s: float
    _action_index: int
    _detail: str
    _baseline_connections: frozenset[ConnectionId] = frozenset()
    _initial_pivot_reference: Vec3 = (0.0, 0.0, 1.0)
    _pivot_angle_rad: float = 0.0
    _maximum_pivot_angle_rad: float = 0.0
    _brake_impulse_nms: float = 0.0
    _maximum_brake_effort_nm: float = 0.0
    _brake_started_at_s: float | None = None
    _brake_ended_at_s: float | None = None
    _capture_time_s: float | None = None
    _capture_ready: Callable[[], bool] | None = None
    _commanded_efforts_nm: dict[JointInstanceId, float] = field(
        default_factory=dict[JointInstanceId, float]
    )
    _completed_telemetry: list[CoordinatedMomentumTelemetry] = field(
        default_factory=list[CoordinatedMomentumTelemetry]
    )

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        plan: CoordinatedPivotPlan,
    ) -> CoordinatedMomentumPivotScenario:
        """Commit the exact initial topology and pre-engage the first hinge."""
        _preflight_plan(session, plan, require_joint_dynamics=True)
        _commit_exact_initial_connections(session, plan)
        scenario = cls(
            session=session,
            plan=plan,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _action_index=0,
            _detail="",
        )
        scenario._activate_action(0)
        return scenario

    @classmethod
    def create_from_existing(
        cls,
        session: RuntimeSession,
        plan: CoordinatedPivotPlan,
        *,
        capture_ready: Callable[[], bool] | None = None,
    ) -> CoordinatedMomentumPivotScenario:
        """Activate a generated pivot without staging, resetting, or moving roots."""
        _preflight_plan(session, plan, require_joint_dynamics=True, require_fresh_world=False)
        _raise_topology_mismatch(
            session,
            {pair.connection_id for pair in plan.initial_connections},
            "online pivot activation",
        )
        scenario = cls(
            session=session,
            plan=plan,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _action_index=0,
            _detail="",
            _capture_ready=capture_ready,
        )
        scenario._activate_action(0)
        return scenario

    @property
    def status(self) -> ReconfigurationStatus:
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
    def completed_telemetry(self) -> tuple[CoordinatedMomentumTelemetry, ...]:
        return tuple(self._completed_telemetry)

    @property
    def current_telemetry(self) -> CoordinatedMomentumTelemetry:
        return self._telemetry()

    def step(self) -> tuple[Event, ...]:
        """Advance one backend-integrated physics step and controller tick."""
        config = self._current_action().momentum
        if self._phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            return self.session.step(config.dt_s, process_connectors=False)

        if self._phase is ReconfigurationPhase.PIVOTING and self._brake_ended_at_s is None:
            efforts = tuple(abs(value) for value in self._commanded_efforts_nm.values())
            self._brake_impulse_nms += sum(efforts) * config.dt_s
            self._maximum_brake_effort_nm = max((self._maximum_brake_effort_nm, *efforts))
        events: list[Event] = list(self.session.step(config.dt_s, process_connectors=False))

        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            if self._held_for(config.settle_s):
                self._enter_phase(
                    ReconfigurationPhase.APPROACHING,
                    f"Spinning {len(self._current_action().moving_modules)} flywheels",
                )
                self._set_spinup_efforts()
        elif self._phase is ReconfigurationPhase.APPROACHING:
            events.extend(self._advance_spinup())
        elif self._phase is ReconfigurationPhase.PIVOTING:
            events.extend(self._advance_pivot())
        elif self._phase is ReconfigurationPhase.HOLDING_CONNECTED:
            targets = {pair.connection_id for pair in self._current_action().target_faces}
            if not targets <= set(self.session.world.connections):
                self._fail("captured slab topology did not remain connected")
            elif self._held_for(config.connected_hold_s):
                events.extend(self._finish_action())
        else:  # pragma: no cover - private phase invariant
            raise AssertionError(f"unhandled coordinated momentum phase '{self._phase}'")
        return tuple(events)

    def _activate_action(self, index: int) -> tuple[Event, ...]:
        self._action_index = index
        action = self._current_action()
        config = action.momentum
        self._reset_action_telemetry()
        self._baseline_connections = frozenset(self.session.world.connections)
        release_ids = {pair.connection_id for pair in action.release_faces}
        missing = release_ids - set(self._baseline_connections)
        if missing:
            raise ReconfigurationScenarioError(
                f"{action.label}: expected release face(s) are inactive: "
                f"{', '.join(sorted(missing))}"
            )
        hinge = config.edge_hinge.connection_id
        if hinge in self.session.world.connections:
            raise ReconfigurationScenarioError(
                f"{action.label}: transient hinge '{hinge}' is already active"
            )
        self.session.request_dock(*config.edge_hinge.connectors)
        events = self.session.process_docking()
        if hinge not in self.session.world.connections:
            failure = _dock_failure(events, hinge)
            detail = failure.detail if failure is not None else "no committed hinge"
            raise ReconfigurationScenarioError(
                f"{action.label}: could not pre-engage transient hinge: {detail}"
            )
        expected = set(self._baseline_connections) | {hinge}
        _raise_topology_mismatch(self.session, expected, f"{action.label}: hinge activation")

        fixed = self.session.world.modules[config.fixed_module].pose
        moving = self.session.world.modules[config.moving_module].pose
        relative = moving.relative_to(fixed)
        self._initial_pivot_reference = quat_rotate(
            relative.rotation,
            config.pivot_reference_in_moving_frame,
        )
        self._set_zero_efforts()
        self._enter_phase(
            ReconfigurationPhase.HOLDING_INITIAL,
            f"{action.label}: settling {len(action.release_faces)} faces with edge hinge active",
        )
        return events

    def _advance_spinup(self) -> tuple[Event, ...]:
        action = self._current_action()
        config = action.momentum
        projected = self._projected_speeds()
        minimum = min(projected)
        maximum = max(projected)
        self._detail = (
            f"{action.label}: synchronized spin-up {minimum * 60.0 / math.tau:.0f}.."
            f"{maximum * 60.0 / math.tau:.0f} RPM / "
            f"{config.target_flywheel_speed_rad_s * 60.0 / math.tau:.0f} RPM"
        )
        if any(
            abs(self._flywheel_velocity(module)) > PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S + 1e-6
            for module in action.moving_modules
        ):
            self._fail("a coordinated flywheel exceeded the 20,000 RPM safety ceiling")
            return ()
        if all(
            speed + config.spinup_speed_tolerance_rad_s + _EPSILON
            >= config.target_flywheel_speed_rad_s
            for speed in projected
        ):
            self._set_zero_efforts()
            events = self._release_faces_to_hinge()
            if self._phase is ReconfigurationPhase.FAILED:
                return events
            self._brake_started_at_s = self.session.world.time_s
            self._enter_phase(
                ReconfigurationPhase.PIVOTING,
                f"{action.label}: released onto hinge; applying synchronized brake impulse",
            )
            self._set_brake_efforts()
            return events
        if self._held_for(config.spinup_timeout_s):
            self._fail("timed out spinning all coordinated flywheels to target speed")
            return ()
        self._set_spinup_efforts()
        return ()

    def _release_faces_to_hinge(self) -> tuple[Event, ...]:
        action = self._current_action()
        hinge = action.momentum.edge_hinge.connection_id
        if hinge not in self.session.world.connections:
            self._fail("transient hinge disappeared before face release")
            return ()
        for pair in action.release_faces:
            self.session.request_undock(pair.connection_id)
        events = self.session.process_docking()
        release_ids = {pair.connection_id for pair in action.release_faces}
        failure = next(
            (
                event
                for event in events
                if isinstance(event, UndockFailed) and event.connection_id in release_ids
            ),
            None,
        )
        if failure is not None:
            self._fail(f"could not release coordinated face cut: {failure.detail}")
        elif release_ids & set(self.session.world.connections):
            self._fail("one or more coordinated face bonds remained active after release")
        else:
            expected = (set(self._baseline_connections) - release_ids) | {hinge}
            mismatch = _topology_mismatch(self.session, expected)
            if mismatch is not None:
                self._fail("coordinated face release changed unrelated topology " + mismatch)
        return events

    def _advance_pivot(self) -> tuple[Event, ...]:
        action = self._current_action()
        config = action.momentum
        events: list[Event] = []
        self._measure_pivot_angle()
        projected = self._projected_speeds()
        mode = "braking" if self._brake_ended_at_s is None else "coasting"
        self._detail = (
            f"{action.label}: edge pivot {mode}; angle "
            f"{math.degrees(self._pivot_angle_rad):.1f}/"
            f"{math.degrees(config.target_pivot_angle_rad):.1f} deg; flywheels "
            f"{min(projected):.1f}..{max(projected):.1f} rad/s"
        )
        if self._brake_ended_at_s is None:
            if all(speed <= config.flywheel_stop_speed_rad_s + _EPSILON for speed in projected):
                self._finish_brake()
            elif (
                self._brake_started_at_s is not None
                and self.session.world.time_s - self._brake_started_at_s + _EPSILON
                >= config.brake_timeout_s
            ):
                self._fail("mechanical brake did not stop every coordinated flywheel in time")
                return ()
            else:
                self._set_brake_efforts()

        events.extend(self._capture_target_faces())
        if self._phase is ReconfigurationPhase.FAILED:
            return tuple(events)
        targets = {pair.connection_id for pair in action.target_faces}
        if targets <= set(self.session.world.connections) and self._brake_ended_at_s is not None:
            release_events, released = self._release_hinge()
            events.extend(release_events)
            if released:
                self._enter_phase(
                    ReconfigurationPhase.HOLDING_CONNECTED,
                    f"{action.label}: measured landing committed; hinge released",
                )
            return tuple(events)
        if self._held_for(config.pivot_timeout_s):
            self._fail(
                "timed out before every slab landing face entered measured acceptance "
                f"(pivot angle {self._pivot_angle_rad:.6g} rad)"
            )
        return tuple(events)

    def _capture_target_faces(self) -> tuple[Event, ...]:
        if self._capture_ready is not None and not self._capture_ready():
            return ()
        action = self._current_action()
        config = action.momentum
        target_ids = {pair.connection_id for pair in action.target_faces}
        if target_ids <= set(self.session.world.connections):
            if self._capture_time_s is None:
                self._capture_time_s = self.session.world.time_s
            return ()

        proposals = {proposal.connection_id: proposal for proposal in self.session.proposals()}
        selected = [proposals.get(identifier) for identifier in target_ids]
        if any(
            proposal is None
            or not proposal.compatibility.compatible
            or not proposal.acceptance.satisfied
            for proposal in selected
        ):
            return ()
        assert all(proposal is not None for proposal in selected)
        if any(
            proposal.acceptance.criterion("position").measured > config.capture_distance_m
            for proposal in selected
            if proposal is not None
        ):
            return ()
        angle_error = abs(abs(self._pivot_angle_rad) - config.target_pivot_angle_rad)
        half_turn = math.isclose(
            config.target_pivot_angle_rad,
            math.pi,
            rel_tol=0.0,
            abs_tol=_EPSILON,
        )
        if (
            not half_turn and self._pivot_angle_rad <= 0.0
        ) or angle_error > config.capture_angle_tolerance_rad:
            return ()

        for pair in action.target_faces:
            self.session.request_dock(*pair.connectors)
        events = self.session.process_docking()
        failure = next(
            (
                event
                for event in events
                if isinstance(event, DockFailed)
                and connection_id(event.connector_a, event.connector_b) in target_ids
            ),
            None,
        )
        if failure is not None:
            self._fail(f"slab landing failed ({failure.reason.value}): {failure.detail}")
        elif target_ids <= set(self.session.world.connections):
            self._capture_time_s = self.session.world.time_s
            expected = (
                (
                    set(self._baseline_connections)
                    - {pair.connection_id for pair in action.release_faces}
                )
                | {action.momentum.edge_hinge.connection_id}
                | target_ids
            )
            mismatch = _topology_mismatch(self.session, expected)
            if mismatch is not None:
                self._fail("slab landing changed unrelated topology " + mismatch)
        else:
            self._fail("slab landing batch did not commit every target face")
        return events

    def _release_hinge(self) -> tuple[tuple[Event, ...], bool]:
        action = self._current_action()
        hinge = action.momentum.edge_hinge.connection_id
        self.session.request_undock(hinge)
        events = self.session.process_docking()
        failure = next(
            (
                event
                for event in events
                if isinstance(event, UndockFailed) and event.connection_id == hinge
            ),
            None,
        )
        if failure is not None:
            self._fail(f"could not release transient slab hinge: {failure.detail}")
            return events, False
        if hinge in self.session.world.connections:
            self._fail("transient slab hinge remained active after release")
            return events, False
        expected = (
            set(self._baseline_connections) - {pair.connection_id for pair in action.release_faces}
        ) | {pair.connection_id for pair in action.target_faces}
        mismatch = _topology_mismatch(self.session, expected)
        if mismatch is not None:
            self._fail("slab hinge release changed unrelated topology " + mismatch)
            return events, False
        return events, True

    def _finish_action(self) -> tuple[Event, ...]:
        self._completed_telemetry.append(self._telemetry())
        next_index = self._action_index + 1
        if next_index == len(self.plan.actions):
            expected = {pair.connection_id for pair in self.plan.final_connections}
            mismatch = _topology_mismatch(self.session, expected)
            if mismatch is not None:
                self._fail("physical staircase ended with the wrong topology " + mismatch)
                return ()
            self._set_zero_efforts()
            total_impulse = sum(item.brake_impulse_nms for item in self._completed_telemetry)
            self._enter_phase(
                ReconfigurationPhase.COMPLETE,
                f"Completed {len(self.plan.actions)} coordinated physics pivots; "
                f"total brake impulse {total_impulse:.5g} N m s",
            )
            return ()
        try:
            return self._activate_action(next_index)
        except ReconfigurationScenarioError as error:
            self._fail(f"Could not start action {next_index + 1}: {error}")
            return ()

    def _set_spinup_efforts(self) -> None:
        config = self._current_action().momentum
        efforts: dict[JointInstanceId, float] = {}
        for module in self._current_action().moving_modules:
            projected = max(0.0, config.spin_direction * self._flywheel_velocity(module))
            remaining = max(0.0, config.target_flywheel_speed_rad_s - projected)
            one_step = config.flywheel_axial_inertia_kg_m2 * remaining / config.dt_s
            effort = min(config.spinup_effort_nm, one_step)
            efforts[self._flywheel(module)] = config.spin_direction * effort
        self._set_efforts(efforts)

    def _set_brake_efforts(self) -> None:
        config = self._current_action().momentum
        efforts: dict[JointInstanceId, float] = {}
        for module in self._current_action().moving_modules:
            projected = max(0.0, config.spin_direction * self._flywheel_velocity(module))
            one_step = config.flywheel_axial_inertia_kg_m2 * projected / config.dt_s
            effort = min(config.brake_effort_nm, one_step)
            efforts[self._flywheel(module)] = -config.spin_direction * effort
        self._set_efforts(efforts)

    def _set_zero_efforts(self) -> None:
        self._set_efforts(
            {self._flywheel(module): 0.0 for module in self._current_action().moving_modules}
        )

    def _set_efforts(self, efforts: dict[JointInstanceId, float]) -> None:
        try:
            self.session.set_joint_commands(
                JointCommand(joint, ControlMode.EFFORT, effort) for joint, effort in efforts.items()
            )
        except (JointCommandError, BackendError) as error:
            raise ReconfigurationScenarioError(
                f"could not command coordinated flywheel efforts: {error}"
            ) from error
        self._commanded_efforts_nm = dict(efforts)

    def _finish_brake(self) -> None:
        self._brake_ended_at_s = self.session.world.time_s
        self._set_zero_efforts()

    def _measure_pivot_angle(self) -> None:
        config = self._current_action().momentum
        fixed = self.session.world.modules[config.fixed_module].pose
        moving = self.session.world.modules[config.moving_module].pose
        relative = moving.relative_to(fixed)
        current_reference = quat_rotate(
            relative.rotation,
            config.pivot_reference_in_moving_frame,
        )
        angle = signed_angle_about(
            self._initial_pivot_reference,
            current_reference,
            config.pivot_axis_in_fixed_frame,
        )
        self._pivot_angle_rad = angle
        self._maximum_pivot_angle_rad = max(self._maximum_pivot_angle_rad, abs(angle))

    def _telemetry(self) -> CoordinatedMomentumTelemetry:
        return CoordinatedMomentumTelemetry(
            flywheel_speeds_rad_s=tuple(
                (self._flywheel(module), self._flywheel_velocity(module))
                for module in self._current_action().moving_modules
            ),
            pivot_angle_rad=self._pivot_angle_rad,
            maximum_pivot_angle_rad=self._maximum_pivot_angle_rad,
            brake_impulse_nms=self._brake_impulse_nms,
            maximum_brake_effort_nm=self._maximum_brake_effort_nm,
            brake_started_at_s=self._brake_started_at_s,
            brake_ended_at_s=self._brake_ended_at_s,
            capture_time_s=self._capture_time_s,
        )

    def _reset_action_telemetry(self) -> None:
        self._pivot_angle_rad = 0.0
        self._maximum_pivot_angle_rad = 0.0
        self._brake_impulse_nms = 0.0
        self._maximum_brake_effort_nm = 0.0
        self._brake_started_at_s = None
        self._brake_ended_at_s = None
        self._capture_time_s = None
        self._commanded_efforts_nm.clear()

    def _projected_speeds(self) -> tuple[float, ...]:
        config = self._current_action().momentum
        return tuple(
            config.spin_direction * self._flywheel_velocity(module)
            for module in self._current_action().moving_modules
        )

    def _flywheel(self, module: ModuleInstanceId) -> JointInstanceId:
        return joint_instance_id(module, self._current_action().momentum.flywheel_joint)

    def _flywheel_velocity(self, module: ModuleInstanceId) -> float:
        return self.session.world.joint_state(self._flywheel(module)).velocity

    def _current_action(self) -> CoordinatedPivotAction:
        return self.plan.actions[self._action_index]

    def _held_for(self, duration_s: float) -> bool:
        return self.session.world.time_s - self._phase_started_at_s + _EPSILON >= duration_s

    def _enter_phase(self, phase: ReconfigurationPhase, detail: str) -> None:
        self._phase = phase
        self._phase_started_at_s = self.session.world.time_s
        self._detail = detail

    def _fail(self, detail: str) -> None:
        self._enter_phase(ReconfigurationPhase.FAILED, detail)
        self._set_zero_efforts()


def _preflight_plan(
    session: RuntimeSession,
    plan: CoordinatedPivotPlan,
    *,
    require_joint_dynamics: bool,
    require_fresh_world: bool = True,
) -> None:
    if require_fresh_world and abs(session.world.time_s) > _EPSILON:
        raise ReconfigurationScenarioError(
            "coordinated pivot scenario must be created at simulation time zero"
        )
    if require_fresh_world and session.world.connections:
        raise ReconfigurationScenarioError(
            "coordinated pivot scenario requires a fresh world with no active connections"
        )
    actual_modules = set(session.world.modules)
    expected_modules = set(plan.module_ids)
    if actual_modules != expected_modules:
        missing = ", ".join(sorted(expected_modules - actual_modules)) or "none"
        unexpected = ", ".join(sorted(actual_modules - expected_modules)) or "none"
        raise ReconfigurationScenarioError(
            "coordinated pivot scene does not match its plan "
            f"(missing modules: {missing}; unexpected modules: {unexpected})"
        )
    if not require_joint_dynamics and not isinstance(session.adapter, SupportsModuleKinematics):
        raise ReconfigurationScenarioError(
            f"backend '{session.adapter.capabilities().name}' cannot execute kinematic pivots"
        )

    face_pairs = (
        *plan.initial_connections,
        *(
            pair
            for action in plan.actions
            for pair in (*action.release_faces, *action.target_faces)
        ),
    )
    hinge_pairs = tuple(action.momentum.edge_hinge for action in plan.actions)
    for label, pairs, expected_constraint in (
        ("face", face_pairs, PhysicalConstraintType.FIXED),
        ("hinge", hinge_pairs, PhysicalConstraintType.HINGE),
    ):
        for pair in pairs:
            for connector_id in pair.connectors:
                try:
                    connector = session.world.connector(connector_id)
                    connector_type = session.world.connector_type(connector_id)
                except KeyError as error:
                    raise ReconfigurationScenarioError(
                        f"{label} pair references unknown connector '{connector_id}'"
                    ) from error
                if not connector.resolved:
                    raise ReconfigurationScenarioError(
                        f"{label} connector frame '{connector_id}' is unresolved"
                    )
                physical = connector_type.physical_connection
                if physical is None or physical.constraint is not expected_constraint:
                    actual = "undefined" if physical is None else physical.constraint.value
                    raise ReconfigurationScenarioError(
                        f"{label} connector '{connector_id}' requires "
                        f"'{expected_constraint.value}', got '{actual}'"
                    )

    if require_joint_dynamics:
        capabilities = session.adapter.capabilities()
        if not capabilities.supports_joint_commands:
            raise ReconfigurationScenarioError(
                f"backend '{capabilities.name}' cannot command flywheel joints"
            )
        checked: set[JointInstanceId] = set()
        for action in plan.actions:
            config = action.momentum
            for module in action.moving_modules:
                joint = joint_instance_id(module, config.flywheel_joint)
                if joint in checked:
                    continue
                checked.add(joint)
                try:
                    state = session.world.joint_state(joint)
                    spec = session.world.joint_spec(joint)
                except (KeyError, ValueError) as error:
                    raise ReconfigurationScenarioError(
                        f"backend did not report required flywheel joint '{joint}'"
                    ) from error
                if not all(math.isfinite(value) for value in (state.position, state.velocity)):
                    raise ReconfigurationScenarioError(
                        f"backend reported non-finite flywheel state for '{joint}'"
                    )
                limits = spec.limits
                if limits is None or limits.max_effort_nm is None:
                    raise ReconfigurationScenarioError(
                        f"flywheel joint '{joint}' requires a finite effort limit"
                    )
                if limits.max_effort_nm + _EPSILON < config.brake_effort_nm:
                    raise ReconfigurationScenarioError(
                        f"flywheel joint '{joint}' effort limit is below requested brake effort"
                    )
                if limits.max_velocity_rad_per_s is None or (
                    limits.max_velocity_rad_per_s + _EPSILON < config.target_flywheel_speed_rad_s
                ):
                    raise ReconfigurationScenarioError(
                        f"flywheel joint '{joint}' velocity limit is below target speed"
                    )


def _commit_exact_initial_connections(
    session: RuntimeSession,
    plan: CoordinatedPivotPlan,
) -> tuple[Event, ...]:
    for pair in plan.initial_connections:
        session.request_dock(*pair.connectors)
    events = session.process_docking()
    expected = {pair.connection_id for pair in plan.initial_connections}
    actual = set(session.world.connections)
    if actual != expected:
        failures = [
            event
            for event in events
            if isinstance(event, DockFailed)
            and connection_id(event.connector_a, event.connector_b) in expected - actual
        ]
        detail = "; ".join(
            f"{connection_id(event.connector_a, event.connector_b)}: {event.detail}"
            for event in failures
        )
        missing = ", ".join(sorted(expected - actual)) or "none"
        unexpected = ", ".join(sorted(actual - expected)) or "none"
        suffix = f"; failures: {detail}" if detail else ""
        raise ReconfigurationScenarioError(
            "could not commit exact coordinated initial topology "
            f"(missing: {missing}; unexpected: {unexpected}{suffix})"
        )
    return events


def _require_unique_pairs(pairs: tuple[ConnectorPairRef, ...], name: str) -> None:
    identifiers = [pair.connection_id for pair in pairs]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError(f"{name} must not contain duplicate connections")


def _require_connector_exclusivity(
    pairs: Iterable[ConnectorPairRef],
    context: str,
) -> None:
    connectors: set[str] = set()
    for pair in pairs:
        for connector in pair.connectors:
            if connector in connectors:
                raise ValueError(
                    f"{context} uses connector '{connector}' in more than one connection"
                )
            connectors.add(connector)


def _components(
    modules: set[ModuleInstanceId],
    pairs: Iterable[ConnectorPairRef],
) -> list[set[ModuleInstanceId]]:
    adjacency = {module: set[ModuleInstanceId]() for module in modules}
    for pair in pairs:
        left = split_connector_instance_id(pair.fixed_connector)[0]
        right = split_connector_instance_id(pair.moving_connector)[0]
        adjacency[left].add(right)
        adjacency[right].add(left)
    remaining = set(modules)
    components: list[set[ModuleInstanceId]] = []
    while remaining:
        root = min(remaining)
        pending = [root]
        component: set[ModuleInstanceId] = set()
        while pending:
            module = pending.pop()
            if module in component:
                continue
            component.add(module)
            pending.extend(adjacency[module] - component)
        remaining -= component
        components.append(component)
    return components


def _require_connected(
    modules: set[ModuleInstanceId],
    pairs: Iterable[ConnectorPairRef],
    context: str,
) -> None:
    if len(_components(modules, pairs)) != 1:
        raise ValueError(f"{context} must connect every plan module")


def _dock_failure(events: tuple[Event, ...], target: ConnectionId) -> DockFailed | None:
    return next(
        (
            event
            for event in events
            if isinstance(event, DockFailed)
            and connection_id(event.connector_a, event.connector_b) == target
        ),
        None,
    )


def _topology_mismatch(session: RuntimeSession, expected: set[ConnectionId]) -> str | None:
    actual = set(session.world.connections)
    if actual == expected:
        return None
    return (
        f"(expected: {', '.join(sorted(expected)) or 'none'}; "
        f"active: {', '.join(sorted(actual)) or 'none'})"
    )


def _raise_topology_mismatch(
    session: RuntimeSession,
    expected: set[ConnectionId],
    context: str,
) -> None:
    mismatch = _topology_mismatch(session, expected)
    if mismatch is not None:
        raise ReconfigurationScenarioError(f"{context} changed unrelated topology {mismatch}")


__all__ = [
    "CoordinatedKinematicPivotScenario",
    "CoordinatedMomentumPivotScenario",
    "CoordinatedMomentumTelemetry",
    "CoordinatedPivotAction",
    "CoordinatedPivotPlan",
]
