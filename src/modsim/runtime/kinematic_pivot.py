"""Backend-neutral kinematic pivot execution for authored reconfiguration routes.

The scenario in this module is deliberately a visual and semantic proof, not a
physics controller.  It releases an ordinary ModSim connection, writes the
detached assembly's root poses along a rigid arc about an authored edge, and
then asks :class:`~modsim.runtime.session.RuntimeSession` to commit the target
connection through the normal docking lifecycle.  Gravity, contact impulses,
magnetic attraction, and momentum transfer are not modelled by this scenario.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from modsim.backends.base import BackendError, SupportsModuleKinematics
from modsim.core.events import DockFailed, Event, UndockFailed
from modsim.core.ids import (
    ConnectionId,
    ModuleInstanceId,
    connection_id,
    split_connector_instance_id,
)
from modsim.core.transforms import (
    Transform,
    Vec3,
    quat_from_axis_angle,
    vec_norm,
    vec_scale,
)
from modsim.core.validation import (
    require_finite,
    require_finite_nonnegative,
    require_finite_positive,
)
from modsim.runtime.reconfiguration import (
    ReconfigurationAction,
    ReconfigurationPhase,
    ReconfigurationPlan,
    ReconfigurationScenarioError,
    ReconfigurationStatus,
    initialize_reconfiguration_plan,
)
from modsim.runtime.session import RuntimeSession

_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class KinematicPivotRoute:
    """One rigid arc about an authored edge.

    ``moving_module`` selects the detached assembly that follows the arc.
    ``pivot_point_m`` and ``pivot_axis`` are expressed in
    ``reference_module``'s root frame.  When ``reference_module`` is ``None``,
    they are world-frame values.  The reference frame is re-evaluated on every
    step, so a route remains attached to its reference module if that module's
    assembly moves.
    """

    moving_module: ModuleInstanceId
    pivot_point_m: Vec3
    pivot_axis: Vec3
    angle_rad: float
    duration_s: float
    reference_module: ModuleInstanceId | None = None

    def __post_init__(self) -> None:
        if not str(self.moving_module).strip():
            raise ValueError("moving_module must not be empty")
        if self.reference_module is not None:
            if not str(self.reference_module).strip():
                raise ValueError("reference_module must not be empty")
            if self.reference_module == self.moving_module:
                raise ValueError("reference_module and moving_module must differ")
        _require_finite_vec3(self.pivot_point_m, "pivot_point_m")
        _require_finite_vec3(self.pivot_axis, "pivot_axis")
        if vec_norm(self.pivot_axis) <= _EPSILON:
            raise ValueError("pivot_axis must not be a zero-length vector")
        require_finite(self.angle_rad, "angle_rad")
        if abs(self.angle_rad) <= _EPSILON:
            raise ValueError("angle_rad must be non-zero")
        require_finite_positive(self.duration_s, "duration_s")


@dataclass(frozen=True, slots=True)
class KinematicPivotConfig:
    """Shared stepping and hold times for an authored pivot plan."""

    dt_s: float = 0.002
    initial_hold_s: float = 0.75
    connected_hold_s: float = 0.5

    def __post_init__(self) -> None:
        require_finite_positive(self.dt_s, "dt_s")
        require_finite_nonnegative(self.initial_hold_s, "initial_hold_s")
        require_finite_nonnegative(self.connected_hold_s, "connected_hold_s")


@dataclass(slots=True)
class KinematicPivotScenario:
    """Execute connector replacements through deterministic rigid pivot arcs."""

    session: RuntimeSession
    plan: ReconfigurationPlan
    routes: tuple[KinematicPivotRoute, ...]
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
        plan: ReconfigurationPlan,
        routes: tuple[KinematicPivotRoute, ...],
        config: KinematicPivotConfig | None = None,
    ) -> KinematicPivotScenario:
        """Validate and stage a plan before beginning its initial hold."""
        resolved_config = config if config is not None else KinematicPivotConfig()
        resolved_routes = tuple(routes)
        _validate_routes_against_plan(session, plan, resolved_routes)
        # A pivot route is authored against stable cube/root frames. Preserve
        # their existing lattice orientation while staging the initial tree;
        # otherwise nominal face alignment may choose an equivalent connector
        # roll that changes which local cube face points upward.
        initialize_reconfiguration_plan(session, plan, preserve_orientation=True)
        scenario = cls(
            session=session,
            plan=plan,
            routes=resolved_routes,
            config=resolved_config,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _action_index=None,
            _detail=(
                f"Holding initial kinematic configuration with "
                f"{len(plan.initial_connections)} connections"
            ),
        )
        if resolved_config.initial_hold_s == 0.0:
            if plan.actions:
                scenario._start_action(0)
            else:
                scenario._complete()
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

    def step(self) -> tuple[Event, ...]:
        """Advance one fixed-duration kinematic scenario step."""
        if self._phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            return self.session.step(self.config.dt_s)
        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            return self._step_initial_hold()
        if self._phase is ReconfigurationPhase.UNDOCKING:
            return self._step_undocking()
        if self._phase is ReconfigurationPhase.PIVOTING:
            return self._step_pivoting()
        if self._phase is ReconfigurationPhase.DOCKING:
            return self._step_docking()
        if self._phase is ReconfigurationPhase.HOLDING_CONNECTED:
            return self._step_connected_hold()
        raise AssertionError(f"unhandled kinematic pivot phase '{self._phase}'")

    def _step_initial_hold(self) -> tuple[Event, ...]:
        events = self.session.step(self.config.dt_s)
        if self._held_for(self.config.initial_hold_s):
            if self.plan.actions:
                self._start_action(0)
            else:
                self._complete()
        return events

    def _step_undocking(self) -> tuple[Event, ...]:
        action = self._current_action()
        assert action.undock is not None
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
            self._begin_pivot()
        return events

    def _begin_pivot(self) -> None:
        action = self._current_action()
        route = self._current_route()
        assert action.dock is not None and action.undock is not None

        moving_assembly = self.session.world.assemblies.assembly_of(route.moving_module)
        moving_modules = tuple(sorted(self.session.world.assemblies.members(moving_assembly)))
        moving_members = set(moving_modules)
        dock_fixed_module = split_connector_instance_id(action.dock.fixed_connector)[0]
        dock_moving_module = split_connector_instance_id(action.dock.moving_connector)[0]
        if dock_moving_module not in moving_members or dock_fixed_module in moving_members:
            raise ReconfigurationScenarioError(
                f"{action.label}: directed dock pair does not preserve the detached moving "
                f"assembly selected by '{route.moving_module}'"
            )
        if route.reference_module is not None and route.reference_module in moving_members:
            raise ReconfigurationScenarioError(
                f"{action.label}: reference module '{route.reference_module}' belongs to the "
                "moving assembly after release"
            )

        reference_pose = self._reference_pose(route)
        self._moving_modules = moving_modules
        self._start_relative_poses = {
            module_id: self.session.world.modules[module_id].pose.relative_to(reference_pose)
            for module_id in moving_modules
        }
        self._baseline_connections = frozenset(self.session.world.connections)
        self._enter_phase(
            ReconfigurationPhase.PIVOTING,
            f"{action.label}: pivoting {len(moving_modules)} module(s) through "
            f"{route.angle_rad:.6g} rad",
        )

    def _step_pivoting(self) -> tuple[Event, ...]:
        action = self._current_action()
        route = self._current_route()
        assert action.dock is not None
        elapsed_s = self.session.world.time_s - self._phase_started_at_s
        next_elapsed_s = min(route.duration_s, elapsed_s + self.config.dt_s)
        fraction = next_elapsed_s / route.duration_s
        self._apply_pivot(fraction)
        # The path itself is authored, not discovered by passive connector
        # capture. Keep backend stepping, ingestion, and overload handling under
        # RuntimeSession while suppressing connector processing so a different
        # compatible cube face cannot auto-latch just before the analytical
        # endpoint. The endpoint below still uses the ordinary requested docking
        # pipeline and emits canonical events.
        events: list[Event] = list(self.session.step(self.config.dt_s, process_connectors=False))

        target = action.dock.connection_id
        new_connections = set(self.session.world.connections).difference(self._baseline_connections)
        unexpected = new_connections.difference((target,))
        if unexpected:
            names = ", ".join(sorted(str(item) for item in unexpected))
            self._fail(f"{action.label}: unexpected connection committed during pivot: {names}")
            return tuple(events)
        progress = min(100.0, 100.0 * fraction)
        self._detail = f"{action.label}: kinematic pivot {progress:.0f}% complete"
        if next_elapsed_s + _EPSILON < route.duration_s:
            return tuple(events)

        # Re-apply the exact analytical endpoint after the backend step. This
        # corrects contact-solver displacement without introducing a hidden
        # staging jump: it is the same endpoint reached by the authored arc.
        self._apply_pivot(1.0)
        self.session.world.ingest(self.session.adapter.snapshot())
        self.session.request_dock(action.dock.fixed_connector, action.dock.moving_connector)
        commit_events = self.session.process_docking()
        events.extend(commit_events)
        new_connections = set(self.session.world.connections).difference(self._baseline_connections)
        unexpected = new_connections.difference((target,))
        if unexpected:
            names = ", ".join(sorted(str(item) for item in unexpected))
            self._fail(
                f"{action.label}: unexpected connection committed at pivot endpoint: {names}"
            )
            return tuple(events)
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
        elif target in self.session.world.connections:
            self._enter_phase(
                ReconfigurationPhase.DOCKING,
                f"{action.label}: authored pivot endpoint reached; dock requested",
            )
        else:
            self._fail(f"{action.label}: pivot endpoint produced no committed target connection")
        return tuple(events)

    def _apply_pivot(self, fraction: float) -> None:
        route = self._current_route()
        adapter = self._kinematic_adapter()
        reference_pose = self._reference_pose(route)
        rotation = Transform(
            rotation=quat_from_axis_angle(
                route.pivot_axis,
                route.angle_rad * fraction,
            )
        )
        pivot_transform = (
            Transform.from_translation(route.pivot_point_m)
            .compose(rotation)
            .compose(Transform.from_translation(vec_scale(route.pivot_point_m, -1.0)))
        )
        try:
            for module_id in self._moving_modules:
                relative = self._start_relative_poses[module_id]
                adapter.set_module_pose(
                    module_id,
                    reference_pose.compose(pivot_transform).compose(relative),
                )
        except BackendError as error:
            raise ReconfigurationScenarioError(
                f"backend could not apply the authored kinematic pivot: {error}"
            ) from error

    def _step_docking(self) -> tuple[Event, ...]:
        action = self._current_action()
        assert action.dock is not None
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
                f"{action.label}: target face connection committed",
            )
        else:
            self._fail(f"{action.label}: committed target connection was not retained")
        return events

    def _step_connected_hold(self) -> tuple[Event, ...]:
        action = self._current_action()
        assert action.dock is not None
        events = self.session.step(self.config.dt_s)
        if action.dock.connection_id not in self.session.world.connections:
            self._fail(f"{action.label}: target connection was lost during its hold")
        elif self._held_for(self.config.connected_hold_s):
            self._advance_action()
        return events

    def _start_action(self, index: int) -> None:
        action = self.plan.actions[index]
        assert action.undock is not None
        self._action_index = index
        self._moving_modules = ()
        self._start_relative_poses.clear()
        self._baseline_connections = frozenset()
        if action.undock.connection_id not in self.session.world.connections:
            raise ReconfigurationScenarioError(
                f"action {index + 1} '{action.label}' expected active connection "
                f"'{action.undock.connection_id}'"
            )
        self.session.request_undock(action.undock.connection_id)
        self._enter_phase(
            ReconfigurationPhase.UNDOCKING,
            f"{action.label}: face undock requested before kinematic pivot",
        )

    def _advance_action(self) -> None:
        current = self._action_index
        if current is None:  # pragma: no cover - phase invariant
            raise AssertionError("pivot phase requires a current action")
        next_index = current + 1
        if next_index >= len(self.plan.actions):
            self._complete()
        else:
            self._start_action(next_index)

    def _reference_pose(self, route: KinematicPivotRoute) -> Transform:
        if route.reference_module is None:
            return Transform.identity()
        return self.session.world.modules[route.reference_module].pose

    def _kinematic_adapter(self) -> SupportsModuleKinematics:
        adapter = self.session.adapter
        if not isinstance(adapter, SupportsModuleKinematics):
            raise ReconfigurationScenarioError(
                f"backend '{adapter.capabilities().name}' cannot execute kinematic pivot routes"
            )
        return adapter

    def _current_action(self) -> ReconfigurationAction:
        if self._action_index is None:
            raise AssertionError("pivot phase requires a current action")
        return self.plan.actions[self._action_index]

    def _current_route(self) -> KinematicPivotRoute:
        if self._action_index is None:
            raise AssertionError("pivot phase requires a current route")
        return self.routes[self._action_index]

    def _held_for(self, duration_s: float) -> bool:
        elapsed = self.session.world.time_s - self._phase_started_at_s
        return elapsed + _EPSILON >= duration_s

    def _enter_phase(self, phase: ReconfigurationPhase, detail: str) -> None:
        self._phase = phase
        self._phase_started_at_s = self.session.world.time_s
        self._detail = detail

    def _fail(self, detail: str) -> None:
        self._enter_phase(ReconfigurationPhase.FAILED, detail)

    def _complete(self) -> None:
        self._enter_phase(
            ReconfigurationPhase.COMPLETE,
            f"Completed {len(self.plan.actions)} kinematic pivot action(s)",
        )


def _validate_routes_against_plan(
    session: RuntimeSession,
    plan: ReconfigurationPlan,
    routes: tuple[KinematicPivotRoute, ...],
) -> None:
    if len(routes) != len(plan.actions):
        raise ReconfigurationScenarioError(
            f"kinematic pivot plan '{plan.id}' has {len(plan.actions)} action(s) but "
            f"{len(routes)} route(s)"
        )
    plan_modules = set(plan.module_ids)
    for index, (action, route) in enumerate(zip(plan.actions, routes, strict=True)):
        context = f"action {index + 1} '{action.label}'"
        if action.undock is None or action.dock is None:
            raise ReconfigurationScenarioError(
                f"{context} must replace one connection for a kinematic pivot"
            )
        if route.moving_module not in plan_modules:
            raise ReconfigurationScenarioError(
                f"{context} route references unknown moving module '{route.moving_module}'"
            )
        if route.reference_module is not None and route.reference_module not in plan_modules:
            raise ReconfigurationScenarioError(
                f"{context} route references unknown reference module '{route.reference_module}'"
            )
        undock_moving = split_connector_instance_id(action.undock.moving_connector)[0]
        if route.moving_module != undock_moving:
            raise ReconfigurationScenarioError(
                f"{context} route moving module '{route.moving_module}' does not match "
                f"directed undock module '{undock_moving}'"
            )
    adapter = session.adapter
    if not isinstance(adapter, SupportsModuleKinematics):
        raise ReconfigurationScenarioError(
            f"backend '{adapter.capabilities().name}' cannot execute kinematic pivot routes"
        )


def _require_finite_vec3(value: Vec3, name: str) -> None:
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    for index, component in enumerate(value):
        require_finite(component, f"{name}[{index}]")


__all__ = [
    "KinematicPivotConfig",
    "KinematicPivotRoute",
    "KinematicPivotScenario",
]
