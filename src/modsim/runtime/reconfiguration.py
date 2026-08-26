"""Backend-neutral orchestration for scripted modular-robot reconfiguration.

The primitives in this module describe topology changes without naming a
specific robot platform.  A platform preset supplies full runtime connector
IDs and an ordered plan; the scenario then drives those commands through the
ordinary :class:`~modsim.runtime.session.RuntimeSession` docking pipeline.

Pose writes are intentionally limited to deterministic scenario staging. They
are not canonical robot commands and they do not replace a future actuator or
reconfiguration-planning API. Dock, undock, assembly, and failure events are
still emitted exclusively by ``RuntimeSession``.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from enum import StrEnum

from modsim.backends.base import BackendError, SupportsModuleKinematics
from modsim.connectors.acceptance import nominal_relative_transform
from modsim.core.events import DockFailed, Event, UndockFailed
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ModuleInstanceId,
    connection_id,
    split_connector_instance_id,
)
from modsim.core.transforms import (
    Transform,
    Vec3,
    minimal_rotation,
    quat_angle,
    quat_multiply,
    quat_normalize,
    vec_norm,
    vec_scale,
    vec_sub,
)
from modsim.core.validation import (
    require_finite,
    require_finite_nonnegative,
    require_finite_positive,
)
from modsim.runtime.session import RuntimeSession

_STAGING_POSITION_TOLERANCE_M = 1e-4
_STAGING_ORIENTATION_TOLERANCE_RAD = 1e-3


class ReconfigurationPlanError(ValueError):
    """Raised when a reconfiguration plan is structurally invalid for a world."""


class ReconfigurationScenarioError(RuntimeError):
    """Raised when a backend cannot execute a valid reconfiguration plan."""


@dataclass(frozen=True, slots=True)
class ConnectorPairRef:
    """A directed pair of full runtime connector instance identifiers.

    ``fixed_connector`` identifies the component that stays in place during
    staging. ``moving_connector`` identifies the component transformed toward
    it. The resulting connection ID remains order-independent.
    """

    fixed_connector: ConnectorInstanceId
    moving_connector: ConnectorInstanceId

    def __post_init__(self) -> None:
        fixed_module = _module_from_full_connector_id(
            self.fixed_connector,
            field_name="fixed_connector",
        )
        moving_module = _module_from_full_connector_id(
            self.moving_connector,
            field_name="moving_connector",
        )
        if self.fixed_connector == self.moving_connector:
            raise ValueError("fixed_connector and moving_connector must be different")
        if fixed_module == moving_module:
            raise ValueError("a connector pair must reference two different modules")

    @property
    def connection_id(self) -> ConnectionId:
        """Return the canonical, order-independent connection identifier."""
        return connection_id(self.fixed_connector, self.moving_connector)

    @property
    def connectors(self) -> tuple[ConnectorInstanceId, ConnectorInstanceId]:
        """Return the directed fixed/moving connector tuple."""
        return self.fixed_connector, self.moving_connector


@dataclass(frozen=True, slots=True)
class ReconfigurationAction:
    """Dock, undock, or replace one connection in a scripted plan."""

    label: str
    undock: ConnectorPairRef | None = None
    dock: ConnectorPairRef | None = None

    def __post_init__(self) -> None:
        if not self.label.strip():
            raise ValueError("reconfiguration action label must not be empty")
        if self.undock is None and self.dock is None:
            raise ValueError("reconfiguration action must dock, undock, or do both")


@dataclass(frozen=True, slots=True)
class ReconfigurationPlan:
    """Immutable initial tree and ordered edge-replacement sequence."""

    id: str
    name: str
    module_ids: tuple[ModuleInstanceId, ...]
    initial_connections: tuple[ConnectorPairRef, ...]
    actions: tuple[ReconfigurationAction, ...]
    source_url: str | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("reconfiguration plan id must not be empty")
        if not self.name.strip():
            raise ValueError("reconfiguration plan name must not be empty")
        if not self.module_ids:
            raise ValueError("reconfiguration plan must contain at least one module")
        if len(set(self.module_ids)) != len(self.module_ids):
            raise ValueError("reconfiguration plan module_ids must be unique")
        if self.source_url is not None and not self.source_url.strip():
            raise ValueError("reconfiguration plan source_url must not be empty")


@dataclass(frozen=True, slots=True)
class ScriptedReconfigurationConfig:
    """Timing and approach controls for a scripted reconfiguration run."""

    dt_s: float = 0.002
    gap_m: float = 0.02
    approach_speed_m_s: float = 0.03
    initial_hold_s: float = 1.0
    separated_hold_s: float = 0.5
    connected_hold_s: float = 1.0
    retract_speed_m_s: float = 0.0
    orientation_rad: float = 0.0
    release_after_s: float | None = None

    def __post_init__(self) -> None:
        require_finite_positive(self.dt_s, "dt_s")
        require_finite_nonnegative(self.gap_m, "gap_m")
        require_finite_nonnegative(self.approach_speed_m_s, "approach_speed_m_s")
        require_finite_nonnegative(self.initial_hold_s, "initial_hold_s")
        require_finite_nonnegative(self.separated_hold_s, "separated_hold_s")
        require_finite_nonnegative(self.connected_hold_s, "connected_hold_s")
        require_finite_nonnegative(self.retract_speed_m_s, "retract_speed_m_s")
        require_finite(self.orientation_rad, "orientation_rad")
        if self.release_after_s is not None:
            require_finite_nonnegative(self.release_after_s, "release_after_s")


class ReconfigurationPhase(StrEnum):
    """Presentation-neutral phase of a scripted reconfiguration."""

    HOLDING_INITIAL = "holding_initial"
    UNDOCKING = "undocking"
    HOLDING_SEPARATED = "holding_separated"
    APPROACHING = "approaching"
    DOCKING = "docking"
    HOLDING_CONNECTED = "holding_connected"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class ReconfigurationStatus:
    """Immutable progress copied out of a running scripted scenario.

    ``action_index`` is zero-based and is ``None`` while the initial
    configuration is being held or when a plan contains no actions.
    """

    phase: ReconfigurationPhase
    time_s: float
    plan_id: str
    plan_name: str
    action_index: int | None
    action_count: int
    detail: str


@dataclass(frozen=True, slots=True)
class AssemblyDockingSetup:
    """Result of rigidly staging one connected component for docking."""

    fixed_connector: ConnectorInstanceId
    moving_connector: ConnectorInstanceId
    moving_modules: tuple[ModuleInstanceId, ...]
    approach_direction: Vec3
    gap_m: float


@dataclass(slots=True)
class ScriptedReconfigurationScenario:
    """Execute an initial tree and ordered edge replacements through one session."""

    session: RuntimeSession
    plan: ReconfigurationPlan
    config: ScriptedReconfigurationConfig
    _phase: ReconfigurationPhase
    _scenario_started_at_s: float
    _phase_started_at_s: float
    _action_index: int | None
    _detail: str
    _approach_setup: AssemblyDockingSetup | None = None
    _approach_gap_m: float | None = None

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        plan: ReconfigurationPlan,
        config: ScriptedReconfigurationConfig | None = None,
    ) -> ScriptedReconfigurationScenario:
        """Validate ``plan``, build its initial tree, and begin the initial hold."""
        resolved_config = config if config is not None else ScriptedReconfigurationConfig()
        initialize_reconfiguration_plan(session, plan)
        scenario = cls(
            session=session,
            plan=plan,
            config=resolved_config,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _scenario_started_at_s=session.world.time_s,
            _phase_started_at_s=session.world.time_s,
            _action_index=None,
            _detail=(
                f"Holding initial configuration with {len(plan.initial_connections)} connections"
            ),
        )
        if resolved_config.initial_hold_s == 0.0 and plan.actions:
            scenario._start_action(0)
        return scenario

    @property
    def status(self) -> ReconfigurationStatus:
        """Return immutable progress safe for an inspector frame."""
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
        """Advance one configured runtime step and update scenario progress."""
        if self._phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            return self.session.step(self.config.dt_s)
        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            return self._step_initial_hold()
        if self._phase is ReconfigurationPhase.UNDOCKING:
            return self._step_undocking()
        if self._phase is ReconfigurationPhase.HOLDING_SEPARATED:
            return self._step_separated_hold()
        if self._phase is ReconfigurationPhase.APPROACHING:
            return self._step_approach()
        if self._phase is ReconfigurationPhase.DOCKING:
            return self._step_docking()
        if self._phase is ReconfigurationPhase.HOLDING_CONNECTED:
            return self._step_connected_hold()
        raise AssertionError(f"unhandled reconfiguration phase '{self._phase}'")

    def _step_initial_hold(self) -> tuple[Event, ...]:
        events = self.session.step(self.config.dt_s)
        if self._held_for(self.config.initial_hold_s):
            if not self.plan.actions:
                self._complete()
            else:
                self._start_action(0)
        return events

    def _step_undocking(self) -> tuple[Event, ...]:
        action = self._current_action()
        if action.undock is None:  # pragma: no cover - phase invariant
            raise AssertionError("undocking phase requires an undock pair")
        events = self.session.step(self.config.dt_s)
        failure = next(
            (
                event
                for event in events
                if isinstance(event, UndockFailed)
                and event.connection_id == action.undock.connection_id
            ),
            None,
        )
        if failure is not None:
            self._fail(f"{action.label}: undock failed ({failure.reason.value}): {failure.detail}")
        elif action.undock.connection_id not in self.session.world.connections:
            if action.dock is None:
                self._start_retraction(action.undock)
                self._advance_action()
            else:
                self._enter_phase(
                    ReconfigurationPhase.HOLDING_SEPARATED,
                    f"{action.label}: assemblies separated",
                )
        return events

    def _step_separated_hold(self) -> tuple[Event, ...]:
        events = self.session.step(self.config.dt_s)
        if not self._held_for(self.config.separated_hold_s):
            return events

        action = self._current_action()
        if action.dock is None:  # pragma: no cover - phase invariant
            raise AssertionError("separated hold requires a docking pair")
        try:
            setup = stage_docking_assembly_pair(
                self.session,
                action.dock.fixed_connector,
                action.dock.moving_connector,
                gap_m=self.config.gap_m,
                orientation_rad=self.config.orientation_rad,
            )
        except (ReconfigurationPlanError, ReconfigurationScenarioError) as error:
            raise ReconfigurationScenarioError(
                f"{action.label}: could not start docking approach: {error}"
            ) from error
        self._approach_setup = setup
        self._approach_gap_m = self.config.gap_m
        self._enter_phase(
            ReconfigurationPhase.APPROACHING,
            f"{action.label}: moving {len(setup.moving_modules)} module(s) toward "
            f"{action.dock.fixed_connector}",
        )
        return events

    def _step_approach(self) -> tuple[Event, ...]:
        action = self._current_action()
        if action.dock is None:  # pragma: no cover - phase invariant
            raise AssertionError("approach phase requires a docking pair")
        events = self.session.step(self.config.dt_s)
        target = action.dock.connection_id
        if target in self.session.world.connections:
            self._enter_phase(
                ReconfigurationPhase.HOLDING_CONNECTED,
                f"{action.label}: connection committed",
            )
            return events

        if self._approach_setup is None or self._approach_gap_m is None:
            raise AssertionError("approach phase requires staged assembly state")
        self._approach_gap_m = max(
            0.0,
            self._approach_gap_m - self.config.approach_speed_m_s * self.config.dt_s,
        )
        self._approach_setup = stage_docking_assembly_pair(
            self.session,
            action.dock.fixed_connector,
            action.dock.moving_connector,
            gap_m=self._approach_gap_m,
            preserve_orientation=True,
        )

        proposal = next(
            (item for item in self.session.proposals() if item.connection_id == target),
            None,
        )
        if proposal is None or not (
            proposal.compatibility.compatible and proposal.acceptance.satisfied
        ):
            return events

        try:
            exact = stage_docking_assembly_pair(
                self.session,
                action.dock.fixed_connector,
                action.dock.moving_connector,
                gap_m=0.0,
                preserve_orientation=True,
            )
        except (ReconfigurationPlanError, ReconfigurationScenarioError) as error:
            raise ReconfigurationScenarioError(
                f"{action.label}: could not exact-align the moving assembly: {error}"
            ) from error
        self._approach_setup = exact
        self.session.request_dock(
            action.dock.fixed_connector,
            action.dock.moving_connector,
        )
        commit_events = self.session.process_docking()
        failure = next(
            (
                event
                for event in commit_events
                if isinstance(event, DockFailed)
                and connection_id(event.connector_a, event.connector_b) == target
            ),
            None,
        )
        if failure is not None:
            self._fail(f"{action.label}: dock failed ({failure.reason.value}): {failure.detail}")
            return (*events, *commit_events)
        self._enter_phase(
            ReconfigurationPhase.DOCKING,
            f"{action.label}: exact-aligned assembly; dock requested",
        )
        return (*events, *commit_events)

    def _step_docking(self) -> tuple[Event, ...]:
        action = self._current_action()
        if action.dock is None:  # pragma: no cover - phase invariant
            raise AssertionError("docking phase requires a docking pair")
        events = self.session.step(self.config.dt_s)
        failure = next(
            (
                event
                for event in events
                if isinstance(event, DockFailed)
                and connection_id(event.connector_a, event.connector_b) == action.dock.connection_id
            ),
            None,
        )
        if failure is not None:
            self._fail(f"{action.label}: dock failed ({failure.reason.value}): {failure.detail}")
        elif action.dock.connection_id in self.session.world.connections:
            self._enter_phase(
                ReconfigurationPhase.HOLDING_CONNECTED,
                f"{action.label}: connection committed",
            )
        return events

    def _step_connected_hold(self) -> tuple[Event, ...]:
        events = self.session.step(self.config.dt_s)
        current = self._action_index
        if current is None:  # pragma: no cover - phase invariant
            raise AssertionError("connected hold requires a current action")
        next_index = current + 1
        next_action = self.plan.actions[next_index] if next_index < len(self.plan.actions) else None
        scheduled_release = (
            next_action is not None
            and next_action.undock is not None
            and next_action.dock is None
            and self.config.release_after_s is not None
        )
        if scheduled_release:
            if (
                self.session.world.time_s + 1e-12
                < self._scenario_started_at_s + self.config.release_after_s  # type: ignore[operator]
            ):
                return events
        elif not self._held_for(self.config.connected_hold_s):
            return events
        self._advance_action()
        return events

    def _start_action(self, index: int) -> None:
        action = self.plan.actions[index]
        self._action_index = index
        self._approach_setup = None
        self._approach_gap_m = None
        if action.undock is None:
            self._start_docking(action)
            return
        if action.undock.connection_id not in self.session.world.connections:
            raise ReconfigurationScenarioError(
                f"action {index + 1} '{action.label}' expected active connection "
                f"'{action.undock.connection_id}'"
            )
        self.session.request_undock(action.undock.connection_id)
        self._enter_phase(
            ReconfigurationPhase.UNDOCKING,
            f"{action.label}: undock requested",
        )

    def _start_docking(self, action: ReconfigurationAction) -> None:
        if action.dock is None:  # pragma: no cover - caller invariant
            raise AssertionError("docking action requires a pair")
        try:
            setup = stage_docking_assembly_pair(
                self.session,
                action.dock.fixed_connector,
                action.dock.moving_connector,
                gap_m=self.config.gap_m,
                orientation_rad=self.config.orientation_rad,
            )
        except (ReconfigurationPlanError, ReconfigurationScenarioError) as error:
            raise ReconfigurationScenarioError(
                f"{action.label}: could not start docking approach: {error}"
            ) from error
        self._approach_setup = setup
        self._approach_gap_m = self.config.gap_m
        self._enter_phase(
            ReconfigurationPhase.APPROACHING,
            f"{action.label}: moving {len(setup.moving_modules)} module(s) toward "
            f"{action.dock.fixed_connector}",
        )

    def _start_retraction(self, pair: ConnectorPairRef) -> None:
        speed = self.config.retract_speed_m_s
        if speed == 0.0:
            return
        adapter = _require_kinematic_adapter(self.session)
        fixed = self.session.world.connector(pair.fixed_connector)
        moving = self.session.world.connector(pair.moving_connector)
        moving_assembly = self.session.world.assemblies.assembly_of(moving.module_id)
        direction = fixed.world_docking_axis
        try:
            for module_id in self.session.world.assemblies.members(moving_assembly):
                adapter.set_module_twist(module_id, linear_m_s=vec_scale(direction, speed))
        except BackendError as error:
            raise ReconfigurationScenarioError(
                f"backend could not start retraction: {error}"
            ) from error

    def _advance_action(self) -> None:
        current = self._action_index
        if current is None:  # pragma: no cover - phase invariant
            raise AssertionError("action phase requires a current action")
        next_index = current + 1
        if next_index >= len(self.plan.actions):
            self._complete()
        else:
            self._start_action(next_index)

    def _current_action(self) -> ReconfigurationAction:
        if self._action_index is None:
            raise AssertionError("scenario phase requires a current action")
        return self.plan.actions[self._action_index]

    def _held_for(self, duration_s: float) -> bool:
        elapsed = self.session.world.time_s - self._phase_started_at_s
        return elapsed + 1e-12 >= duration_s

    def _enter_phase(self, phase: ReconfigurationPhase, detail: str) -> None:
        self._phase = phase
        self._phase_started_at_s = self.session.world.time_s
        self._detail = detail

    def _fail(self, detail: str) -> None:
        self._enter_phase(ReconfigurationPhase.FAILED, detail)

    def _complete(self) -> None:
        self._enter_phase(
            ReconfigurationPhase.COMPLETE,
            f"Completed {len(self.plan.actions)} reconfiguration action(s)",
        )


def stage_docking_assembly_pair(
    session: RuntimeSession,
    fixed_connector: ConnectorInstanceId,
    moving_connector: ConnectorInstanceId,
    *,
    gap_m: float,
    orientation_rad: float = 0.0,
    preserve_orientation: bool = False,
) -> AssemblyDockingSetup:
    """Rigidly place an entire moving component in front of a fixed connector.

    Every module root in the moving connector's current assembly receives the
    same world-frame rigid transform. This preserves all existing connection
    geometry and avoids moving only one root of a welded component. With
    ``preserve_orientation``, the helper retains the component's accepted roll
    and applies only the minimum rotation needed to restore opposing docking
    axes.
    """
    require_finite_nonnegative(gap_m, "gap_m")
    require_finite(orientation_rad, "orientation_rad")
    adapter = _require_kinematic_adapter(session)

    try:
        fixed = session.world.connector(fixed_connector)
        moving = session.world.connector(moving_connector)
    except KeyError as error:
        raise ReconfigurationPlanError(str(error)) from error
    if fixed.module_id == moving.module_id:
        raise ReconfigurationPlanError("docking requires connectors on different modules")
    if not fixed.resolved or not moving.resolved:
        raise ReconfigurationScenarioError(
            "connector frames must be resolved before staging an assembly"
        )

    fixed_assembly = session.world.assemblies.assembly_of(fixed.module_id)
    moving_assembly = session.world.assemblies.assembly_of(moving.module_id)
    if fixed_assembly == moving_assembly:
        raise ReconfigurationPlanError(
            f"connectors '{fixed_connector}' and '{moving_connector}' already belong to "
            f"assembly '{fixed_assembly}'"
        )
    moving_modules = tuple(sorted(session.world.assemblies.members(moving_assembly)))
    fixed_modules = tuple(sorted(session.world.assemblies.members(fixed_assembly)))

    if preserve_orientation:
        # Once an articulated assembly enters acceptance, close only its
        # positional gap and restore antiparallel docking axes with the minimum
        # rotation. This preserves its established discrete roll while
        # correcting passive-joint drift from the preceding physics step.
        align = minimal_rotation(
            moving.world_docking_axis,
            vec_scale(fixed.world_docking_axis, -1.0),
        )
        target_connector = Transform(
            rotation=quat_normalize(quat_multiply(align, moving.world_pose.rotation))
        )
    else:
        # DockingManager canonicalizes connector IDs before evaluating roll.
        # Build the nominal pose in that same order, then solve for the moving
        # side when the directed staging order is reversed.
        if fixed_connector < moving_connector:
            nominal = nominal_relative_transform(fixed, moving, orientation_rad)
            target_connector = fixed.world_pose.compose(nominal)
        else:
            nominal = nominal_relative_transform(moving, fixed, orientation_rad)
            target_connector = fixed.world_pose.compose(nominal.inverse())
    target_connector = Transform(
        translation=(
            fixed.world_pose.translation[0] + fixed.world_docking_axis[0] * gap_m,
            fixed.world_pose.translation[1] + fixed.world_docking_axis[1] * gap_m,
            fixed.world_pose.translation[2] + fixed.world_docking_axis[2] * gap_m,
        ),
        rotation=target_connector.rotation,
    )
    world_delta = target_connector.compose(moving.world_pose.inverse())
    target_poses = {
        module_id: world_delta.compose(session.world.modules[module_id].pose)
        for module_id in moving_modules
    }
    fixed_poses = {module_id: session.world.modules[module_id].pose for module_id in fixed_modules}

    try:
        for module_id in moving_modules:
            adapter.set_module_pose(module_id, target_poses[module_id])
        # Staging is deliberately kinematic. Hold the reference assembly at its
        # measured pose and clear both root and articulated motion so passive
        # joints cannot make acceptance depend on contact impulses.
        for module_id in fixed_modules:
            adapter.set_module_pose(module_id, fixed_poses[module_id])
        for module_id in moving_modules:
            adapter.set_module_twist(module_id)
        session.world.ingest(session.adapter.snapshot())
    except BackendError as error:
        raise ReconfigurationScenarioError(
            f"backend could not rigidly stage moving assembly '{moving_assembly}': {error}"
        ) from error

    measured = session.world.connector(moving_connector).world_pose
    if not measured.is_close(
        target_connector,
        position_tolerance_m=_STAGING_POSITION_TOLERANCE_M,
        orientation_tolerance_rad=_STAGING_ORIENTATION_TOLERANCE_RAD,
    ):
        position_error_m = vec_norm(vec_sub(measured.translation, target_connector.translation))
        orientation_error_rad = quat_angle(measured.relative_to(target_connector).rotation)
        raise ReconfigurationScenarioError(
            f"backend did not place moving connector '{moving_connector}' at the requested "
            f"assembly pose (position error {position_error_m:.6g} m, orientation error "
            f"{orientation_error_rad:.6g} rad)"
        )

    return AssemblyDockingSetup(
        fixed_connector=fixed_connector,
        moving_connector=moving_connector,
        moving_modules=moving_modules,
        approach_direction=vec_scale(fixed.world_docking_axis, -1.0),
        gap_m=gap_m,
    )


def connector_pair_plan(
    fixed_connector: ConnectorInstanceId,
    moving_connector: ConnectorInstanceId,
    *,
    include_undock: bool,
) -> ReconfigurationPlan:
    """Build a minimal two-module dock or dock/undock demonstration plan."""
    pair = ConnectorPairRef(fixed_connector, moving_connector)
    module_ids = tuple(
        sorted(
            (
                _module_from_full_connector_id(fixed_connector, field_name="fixed_connector"),
                _module_from_full_connector_id(moving_connector, field_name="moving_connector"),
            )
        )
    )
    actions = [ReconfigurationAction(label="Dock connector pair", dock=pair)]
    if include_undock:
        actions.append(ReconfigurationAction(label="Undock connector pair", undock=pair))
    return ReconfigurationPlan(
        id="dock_undock" if include_undock else "dock",
        name="Two-module dock and undock" if include_undock else "Two-module dock",
        module_ids=module_ids,
        initial_connections=(),
        actions=tuple(actions),
    )


def _require_kinematic_adapter(session: RuntimeSession) -> SupportsModuleKinematics:
    adapter = session.adapter
    if not isinstance(adapter, SupportsModuleKinematics):
        raise ReconfigurationScenarioError(
            f"backend '{adapter.capabilities().name}' cannot reposition and drive modules"
        )
    return adapter


def initialize_reconfiguration_plan(
    session: RuntimeSession,
    plan: ReconfigurationPlan,
    *,
    preserve_orientation: bool = False,
) -> None:
    """Validate and establish a plan's initial connection forest at time zero.

    This helper is scenario initialization, not a runtime motion command.  It
    may reposition roots before stepping begins so a demonstration can start
    from the authored topology.  Physical ground demos set
    ``preserve_orientation`` to keep mobile modules upright; the legacy
    scripted scenario retains nominal connector-frame staging.
    """
    _validate_plan_against_session(session, plan)
    _require_kinematic_adapter(session)
    _build_initial_tree(
        session,
        plan,
        preserve_orientation=preserve_orientation,
    )


def _build_initial_tree(
    session: RuntimeSession,
    plan: ReconfigurationPlan,
    *,
    preserve_orientation: bool,
) -> None:
    expected: set[ConnectionId] = set()
    for index, pair in enumerate(plan.initial_connections):
        try:
            stage_docking_assembly_pair(
                session,
                pair.fixed_connector,
                pair.moving_connector,
                gap_m=0.0,
                preserve_orientation=preserve_orientation,
            )
        except (ReconfigurationPlanError, ReconfigurationScenarioError) as error:
            raise ReconfigurationScenarioError(
                f"could not stage initial connection {index + 1} '{pair.connection_id}': {error}"
            ) from error
        session.request_dock(pair.fixed_connector, pair.moving_connector)
        events = session.process_docking()
        expected.add(pair.connection_id)
        if pair.connection_id not in session.world.connections:
            failure = next(
                (
                    event
                    for event in events
                    if isinstance(event, DockFailed)
                    and connection_id(event.connector_a, event.connector_b) == pair.connection_id
                ),
                None,
            )
            detail = (
                f"{failure.reason.value}: {failure.detail}"
                if failure is not None
                else "runtime produced no committed connection"
            )
            raise ReconfigurationScenarioError(
                f"initial connection {index + 1} '{pair.connection_id}' failed: {detail}"
            )
        actual = set(session.world.connections)
        if actual != expected:
            unexpected = ", ".join(sorted(str(item) for item in actual - expected)) or "none"
            raise ReconfigurationScenarioError(
                f"initial tree acquired unexpected connection(s): {unexpected}"
            )


def _validate_plan_against_session(
    session: RuntimeSession,
    plan: ReconfigurationPlan,
) -> None:
    world_modules = set(session.world.modules)
    plan_modules = set(plan.module_ids)
    if world_modules != plan_modules:
        missing = sorted(str(item) for item in plan_modules - world_modules)
        unexpected = sorted(str(item) for item in world_modules - plan_modules)
        detail: list[str] = []
        if missing:
            detail.append("missing from world: " + ", ".join(missing))
        if unexpected:
            detail.append("not declared by plan: " + ", ".join(unexpected))
        raise ReconfigurationPlanError(
            f"plan '{plan.id}' module_ids do not match the runtime scene ({'; '.join(detail)})"
        )
    if session.world.connections:
        active_connections_text = ", ".join(sorted(str(item) for item in session.world.connections))
        raise ReconfigurationPlanError(
            f"plan '{plan.id}' requires a fresh world with no connections; "
            f"active: {active_connections_text}"
        )
    if len(plan.initial_connections) > len(plan.module_ids) - 1:
        raise ReconfigurationPlanError(
            f"plan '{plan.id}' initial forest has too many connections: "
            f"{len(plan.initial_connections)} for {len(plan.module_ids)} modules"
        )

    referenced_pairs = [
        *plan.initial_connections,
        *(action.undock for action in plan.actions if action.undock is not None),
        *(action.dock for action in plan.actions if action.dock is not None),
    ]
    module_by_connector: dict[ConnectorInstanceId, ModuleInstanceId] = {}
    for pair in referenced_pairs:
        for connector_id in pair.connectors:
            try:
                connector = session.world.connector(connector_id)
            except KeyError as error:
                raise ReconfigurationPlanError(
                    f"plan '{plan.id}' references unknown connector '{connector_id}'"
                ) from error
            if connector.module_id not in plan_modules:
                raise ReconfigurationPlanError(
                    f"connector '{connector_id}' belongs to module '{connector.module_id}', "
                    f"which is not in plan '{plan.id}'"
                )
            encoded_module = _module_from_full_connector_id(
                connector_id,
                field_name="connector",
            )
            if encoded_module != connector.module_id:
                raise ReconfigurationPlanError(
                    f"connector '{connector_id}' resolves to module '{connector.module_id}', "
                    f"not encoded module '{encoded_module}'"
                )
            module_by_connector[connector_id] = connector.module_id

    active: dict[ConnectionId, ConnectorPairRef] = {}
    engaged: dict[ConnectorInstanceId, ConnectionId] = {}
    for index, pair in enumerate(plan.initial_connections):
        components = _components(plan.module_ids, active.values(), module_by_connector)
        if (
            components[module_by_connector[pair.fixed_connector]]
            == components[module_by_connector[pair.moving_connector]]
        ):
            raise ReconfigurationPlanError(
                f"plan '{plan.id}' initial connection {index + 1} creates a cycle"
            )
        _add_planned_connection(
            plan,
            pair,
            active,
            engaged,
            module_by_connector,
            context=f"initial connection {index + 1}",
        )
    for index, action in enumerate(plan.actions):
        context = f"action {index + 1} '{action.label}'"
        if action.undock is not None:
            existing = active.pop(action.undock.connection_id, None)
            if existing is None:
                raise ReconfigurationPlanError(
                    f"{context} cannot undock inactive connection '{action.undock.connection_id}'"
                )
            for connector_id in existing.connectors:
                engaged.pop(connector_id)

        if action.dock is not None:
            components = _components(plan.module_ids, active.values(), module_by_connector)
            fixed_module = module_by_connector[action.dock.fixed_connector]
            moving_module = module_by_connector[action.dock.moving_connector]
            if components[fixed_module] == components[moving_module]:
                raise ReconfigurationPlanError(
                    f"{context} dock '{action.dock.connection_id}' would connect modules "
                    "already in the same assembly"
                )
            _add_planned_connection(
                plan,
                action.dock,
                active,
                engaged,
                module_by_connector,
                context=f"{context} dock",
            )


def _add_planned_connection(
    plan: ReconfigurationPlan,
    pair: ConnectorPairRef,
    active: dict[ConnectionId, ConnectorPairRef],
    engaged: dict[ConnectorInstanceId, ConnectionId],
    module_by_connector: dict[ConnectorInstanceId, ModuleInstanceId],
    *,
    context: str,
) -> None:
    if pair.connection_id in active:
        raise ReconfigurationPlanError(
            f"plan '{plan.id}' {context} duplicates connection '{pair.connection_id}'"
        )
    for connector_id in pair.connectors:
        occupied_by = engaged.get(connector_id)
        if occupied_by is not None:
            raise ReconfigurationPlanError(
                f"plan '{plan.id}' {context} reuses engaged connector '{connector_id}' "
                f"from connection '{occupied_by}'"
            )
    fixed_module = module_by_connector[pair.fixed_connector]
    moving_module = module_by_connector[pair.moving_connector]
    if fixed_module == moving_module:
        raise ReconfigurationPlanError(
            f"plan '{plan.id}' {context} connects module '{fixed_module}' to itself"
        )
    active[pair.connection_id] = pair
    for connector_id in pair.connectors:
        engaged[connector_id] = pair.connection_id


def _components(
    module_ids: tuple[ModuleInstanceId, ...],
    pairs: Iterable[ConnectorPairRef],
    module_by_connector: dict[ConnectorInstanceId, ModuleInstanceId],
) -> dict[ModuleInstanceId, frozenset[ModuleInstanceId]]:
    adjacency: dict[ModuleInstanceId, set[ModuleInstanceId]] = {
        module_id: set() for module_id in module_ids
    }
    for pair in pairs:
        fixed = module_by_connector[pair.fixed_connector]
        moving = module_by_connector[pair.moving_connector]
        adjacency[fixed].add(moving)
        adjacency[moving].add(fixed)
    result: dict[ModuleInstanceId, frozenset[ModuleInstanceId]] = {}
    remaining: set[ModuleInstanceId] = set(module_ids)
    while remaining:
        seed = min(remaining)
        members: set[ModuleInstanceId] = {seed}
        queue: list[ModuleInstanceId] = [seed]
        while queue:
            current = queue.pop()
            for neighbour in adjacency[current]:
                if neighbour not in members:
                    members.add(neighbour)
                    queue.append(neighbour)
        component = frozenset(members)
        remaining -= component
        for member in component:
            result[member] = component
    return result


def _module_from_full_connector_id(
    connector_id: ConnectorInstanceId,
    *,
    field_name: str,
) -> ModuleInstanceId:
    try:
        module_id, local_id = split_connector_instance_id(connector_id)
    except ValueError as error:
        raise ValueError(
            f"{field_name} must be a full runtime connector ID such as 'module_1/pan'"
        ) from error
    if not module_id or not local_id:
        raise ValueError(f"{field_name} must be a full runtime connector ID such as 'module_1/pan'")
    return module_id
