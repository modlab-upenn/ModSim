"""Sequential orchestration for physics-driven M-Blocks edge pivots.

This module composes the single-action :mod:`modsim.runtime.momentum_pivot`
primitive without taking ownership of rigid-body motion. Initial topology may
be staged once at simulation time zero. Thereafter the sequence uses only
joint-effort commands and the normal connector dock/undock lifecycle while the
backend integrates gravity, contacts, flywheel reaction torque, and hinges.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from modsim.core.events import DockFailed, Event
from modsim.core.ids import ModuleInstanceId, connection_id, split_connector_instance_id
from modsim.runtime.momentum_pivot import (
    MomentumPivotConfig,
    MomentumPivotScenario,
    MomentumPivotTelemetry,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationPhase,
    ReconfigurationPlan,
    ReconfigurationScenarioError,
    ReconfigurationStatus,
)
from modsim.runtime.session import RuntimeSession

_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class MomentumPivotSequencePlan:
    """One authored topology plan paired with one physical pivot per action."""

    reconfiguration: ReconfigurationPlan
    pivots: tuple[MomentumPivotConfig, ...]

    def __post_init__(self) -> None:
        actions = self.reconfiguration.actions
        if not actions:
            raise ValueError("momentum sequence requires at least one action")
        if len(self.pivots) != len(actions):
            raise ValueError("momentum sequence requires exactly one pivot per action")

        plan_modules = set(self.reconfiguration.module_ids)
        referenced_pairs: list[tuple[str, ConnectorPairRef]] = [
            (f"initial connection {index + 1}", pair)
            for index, pair in enumerate(self.reconfiguration.initial_connections)
        ]
        for index, action in enumerate(actions):
            if action.undock is not None:
                referenced_pairs.append((f"action {index + 1} undock", action.undock))
            if action.dock is not None:
                referenced_pairs.append((f"action {index + 1} dock", action.dock))
        for label, pair in referenced_pairs:
            referenced_modules = {
                split_connector_instance_id(connector)[0] for connector in pair.connectors
            }
            unknown = referenced_modules - plan_modules
            if unknown:
                raise ValueError(
                    f"{label} references module(s) absent from the plan: "
                    f"{', '.join(sorted(unknown))}"
                )

        moving_module: ModuleInstanceId | None = None
        for index, (action, pivot) in enumerate(zip(actions, self.pivots, strict=True)):
            if action.undock is None or action.dock is None:
                raise ValueError(f"momentum action {index + 1} must replace one connection")
            if action.undock != pivot.initial_face:
                raise ValueError(
                    f"momentum action {index + 1} directed undock does not match its initial face"
                )
            if action.dock != pivot.target_face:
                raise ValueError(
                    f"momentum action {index + 1} directed dock does not match its target face"
                )
            if pivot.plan_id != self.reconfiguration.id:
                raise ValueError(
                    f"momentum action {index + 1} plan_id must match the reconfiguration plan"
                )
            if pivot.plan_name != self.reconfiguration.name:
                raise ValueError(
                    f"momentum action {index + 1} plan_name must match the reconfiguration plan"
                )
            if moving_module is None:
                moving_module = pivot.moving_module
            elif pivot.moving_module != moving_module:
                raise ValueError("all momentum actions must move the same module")
            if index > 0:
                previous = self.pivots[index - 1]
                if previous.target_face != pivot.initial_face:
                    raise ValueError(
                        f"momentum action {index + 1} does not continue from the prior "
                        "directed target"
                    )
                if not math.isclose(
                    previous.dt_s,
                    pivot.dt_s,
                    rel_tol=0.0,
                    abs_tol=_EPSILON,
                ):
                    raise ValueError("all momentum actions must use one fixed timestep")

        assert moving_module is not None
        if moving_module not in plan_modules:
            raise ValueError("moving module is absent from the reconfiguration plan")
        for pivot in self.pivots:
            supports = {pivot.fixed_module, pivot.target_module}
            unknown = supports - plan_modules
            if unknown:
                raise ValueError(
                    "momentum pivot references support module(s) absent from the plan: "
                    f"{', '.join(sorted(unknown))}"
                )
        if self.pivots[0].initial_face not in self.reconfiguration.initial_connections:
            raise ValueError("first momentum pivot must start from an initial connection")


@dataclass(slots=True)
class MomentumPivotSequenceScenario:
    """Execute an ordered physical pivot sequence without runtime root control."""

    session: RuntimeSession
    plan: MomentumPivotSequencePlan
    _current: MomentumPivotScenario
    _action_index: int
    _terminal_phase: ReconfigurationPhase | None = None
    _terminal_detail: str = ""
    _completed_telemetry: list[MomentumPivotTelemetry] = field(
        default_factory=list[MomentumPivotTelemetry]
    )

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        plan: MomentumPivotSequencePlan,
    ) -> MomentumPivotSequenceScenario:
        """Stage the initial tree at time zero and activate the first hinge."""
        if abs(session.world.time_s) > _EPSILON:
            raise ReconfigurationScenarioError(
                "momentum sequence must be created at simulation time zero"
            )
        if session.world.connections:
            raise ReconfigurationScenarioError(
                "momentum sequence requires a fresh world with no active connections"
            )
        _commit_initial_connections(session, plan.reconfiguration)
        current, _ = MomentumPivotScenario.create_from_existing(
            session,
            plan.pivots[0],
            action_index=0,
            action_count=len(plan.pivots),
        )
        return cls(
            session=session,
            plan=plan,
            _current=current,
            _action_index=0,
        )

    @property
    def status(self) -> ReconfigurationStatus:
        """Return aggregate progress plus the current physical measurements."""
        if self._terminal_phase is not None:
            phase = self._terminal_phase
            detail = self._terminal_detail
        else:
            phase = self._current.status.phase
            action = self.plan.reconfiguration.actions[self._action_index]
            detail = (
                f"Action {self._action_index + 1}/{len(self.plan.pivots)} "
                f"({action.label}): {self._current.status.detail}"
            )
        return ReconfigurationStatus(
            phase=phase,
            time_s=self.session.world.time_s,
            plan_id=self.plan.reconfiguration.id,
            plan_name=self.plan.reconfiguration.name,
            action_index=self._action_index,
            action_count=len(self.plan.pivots),
            detail=detail,
        )

    @property
    def completed_telemetry(self) -> tuple[MomentumPivotTelemetry, ...]:
        """Return measurements for every fully completed pivot."""
        return tuple(self._completed_telemetry)

    @property
    def current_telemetry(self) -> MomentumPivotTelemetry:
        """Return live measurements for the active or final pivot."""
        return self._current.telemetry

    def step(self) -> tuple[Event, ...]:
        """Advance one backend step and, when ready, activate the next pivot."""
        if self._terminal_phase is not None:
            return self.session.step(
                self.plan.pivots[0].dt_s,
                process_connectors=False,
            )

        events = list(self._current.step())
        phase = self._current.status.phase
        if phase is ReconfigurationPhase.FAILED:
            self._terminal_phase = ReconfigurationPhase.FAILED
            self._terminal_detail = (
                f"Action {self._action_index + 1} failed: {self._current.status.detail}"
            )
            return tuple(events)
        if phase is not ReconfigurationPhase.COMPLETE:
            return tuple(events)

        self._completed_telemetry.append(self._current.telemetry)
        next_index = self._action_index + 1
        if next_index >= len(self.plan.pivots):
            self._terminal_phase = ReconfigurationPhase.COMPLETE
            total_impulse = sum(item.brake_impulse_nms for item in self._completed_telemetry)
            self._terminal_detail = (
                f"Completed {len(self.plan.pivots)} physics-driven pivots; "
                f"total brake impulse {total_impulse:.5g} N m s"
            )
            return tuple(events)

        self._action_index = next_index
        try:
            self._current, activation_events = MomentumPivotScenario.create_from_existing(
                self.session,
                self.plan.pivots[next_index],
                action_index=next_index,
                action_count=len(self.plan.pivots),
            )
        except ReconfigurationScenarioError as error:
            self._terminal_phase = ReconfigurationPhase.FAILED
            self._terminal_detail = f"Could not start action {next_index + 1}: {error}"
        else:
            events.extend(activation_events)
        return tuple(events)


def _commit_initial_connections(
    session: RuntimeSession,
    plan: ReconfigurationPlan,
) -> tuple[Event, ...]:
    """Commit a complete, already-posed initial tree in one docking pass.

    Physical scenario files provide exact grounded placements. Requesting the
    complete tree before processing avoids an auto-latching face being treated
    as unexpected merely because it committed earlier than the pair-by-pair
    kinematic staging helper anticipated.
    """
    actual_modules = set(session.world.modules)
    expected_modules = set(plan.module_ids)
    if actual_modules != expected_modules:
        missing = ", ".join(sorted(expected_modules - actual_modules)) or "none"
        unexpected = ", ".join(sorted(actual_modules - expected_modules)) or "none"
        raise ReconfigurationScenarioError(
            "momentum sequence scene does not match its plan "
            f"(missing modules: {missing}; unexpected modules: {unexpected})"
        )
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
        failure_detail = "; ".join(
            f"{connection_id(event.connector_a, event.connector_b)}: {event.detail}"
            for event in failures
        )
        missing = ", ".join(sorted(expected - actual)) or "none"
        unexpected = ", ".join(sorted(actual - expected)) or "none"
        suffix = f"; failures: {failure_detail}" if failure_detail else ""
        raise ReconfigurationScenarioError(
            "could not commit exact initial momentum topology "
            f"(missing: {missing}; unexpected: {unexpected}{suffix})"
        )
    return events


__all__ = [
    "MomentumPivotSequencePlan",
    "MomentumPivotSequenceScenario",
]
