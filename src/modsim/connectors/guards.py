"""Guards evaluated before a dock or undock is allowed to proceed.

Guards are the policy layer between "the geometry works" and "we should do
it". They are separated from acceptance so that a Studio preview can show a
pair as geometrically valid while the runtime still refuses to latch, and so
that the reason is reportable either way.
"""

from __future__ import annotations

from dataclasses import dataclass

from modsim.core.entities import ConnectionRuntime, ConnectorInstance
from modsim.core.state import WorldState
from modsim.robot_packs.schema import (
    ConnectorTypeSpec,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
)

SUPPORTED_CONSTRAINTS = frozenset(
    {
        PhysicalConstraintType.FIXED,
        PhysicalConstraintType.COMPLIANT,
        PhysicalConstraintType.HINGE,
    }
)
"""Constraint intents the format-0.1 engine can ask a backend to realise.

``ball`` and ``custom`` may be authored, but their parameters are not modelled
yet, so committing them would mean guessing at physics.
"""


@dataclass(frozen=True, slots=True)
class GuardResult:
    """Whether an action is permitted, and what the caller should know."""

    allowed: bool
    code: str | None = None
    reason: str | None = None
    warnings: tuple[str, ...] = ()
    physical_connection: PhysicalConnectionSpec | None = None

    def __bool__(self) -> bool:
        return self.allowed


def _denied(code: str, reason: str) -> GuardResult:
    return GuardResult(allowed=False, code=code, reason=reason)


def resolve_physical_connection(
    type_a: ConnectorTypeSpec,
    type_b: ConnectorTypeSpec,
) -> tuple[PhysicalConnectionSpec | None, str | None]:
    """Return the connection intent two connector types agree on.

    Disagreement is treated as an authoring error rather than silently
    preferring one side: a pair where one end wants a rigid weld and the other
    wants compliance has no defensible default.
    """
    left, right = type_a.physical_connection, type_b.physical_connection
    if left is None and right is None:
        return None, (
            f"neither connector type '{type_a.id}' nor '{type_b.id}' declares a physical connection"
        )
    if left is None:
        return right, None
    if right is None:
        return left, None
    if left.constraint is not right.constraint:
        return None, (
            f"connector types '{type_a.id}' and '{type_b.id}' declare conflicting "
            f"physical connections: '{left.constraint.value}' and '{right.constraint.value}'"
        )
    if left.constraint is PhysicalConstraintType.HINGE and left.hinge != right.hinge:
        return None, (
            f"connector types '{type_a.id}' and '{type_b.id}' declare conflicting hinge parameters"
        )
    return left, None


def evaluate_dock_guards(
    world: WorldState,
    connector_a: ConnectorInstance,
    connector_b: ConnectorInstance,
    *,
    requested: bool,
) -> GuardResult:
    """Return whether these two connectors may attempt to dock now."""
    if connector_a.id == connector_b.id:
        return _denied("self_pair", "a connector cannot dock to itself")
    if connector_a.module_id == connector_b.module_id:
        return _denied(
            "same_module",
            f"connectors '{connector_a.id}' and '{connector_b.id}' belong to the same module",
        )
    for connector in (connector_a, connector_b):
        if connector.is_engaged:
            return _denied(
                "already_engaged",
                f"connector '{connector.id}' is already engaged in connection "
                f"'{connector.connection_id}'",
            )
        if world.time_s < connector.available_at_s:
            return _denied(
                "cooldown",
                f"connector '{connector.id}' is in redock cooldown until "
                f"{connector.available_at_s:.6g}s",
            )

    type_a = world.connector_type(connector_a.id)
    type_b = world.connector_type(connector_b.id)
    physical, conflict = resolve_physical_connection(type_a, type_b)
    if physical is None:
        return _denied("physical_connection_undefined", conflict or "no physical connection")
    if physical.constraint not in SUPPORTED_CONSTRAINTS:
        return _denied(
            "physical_connection_unsupported",
            f"physical connection '{physical.constraint.value}' is not modelled in format 0.1",
        )

    if not requested:
        policy_a = type_a.effective_docking_policy
        policy_b = type_b.effective_docking_policy
        if not (policy_a.auto_latch and policy_b.auto_latch):
            return _denied(
                "command_required",
                f"connector types '{type_a.id}' and '{type_b.id}' are not both auto-latching; "
                "an explicit dock command is required",
            )

    warnings: list[str] = []
    if world.assemblies.assembly_of(connector_a.module_id) == world.assemblies.assembly_of(
        connector_b.module_id
    ):
        warnings.append(
            f"modules '{connector_a.module_id}' and '{connector_b.module_id}' are already in "
            "one assembly; this dock closes a kinematic loop"
        )
    return GuardResult(
        allowed=True,
        warnings=tuple(warnings),
        physical_connection=physical,
    )


def evaluate_undock_guards(
    world: WorldState,
    connection: ConnectionRuntime,
) -> GuardResult:
    """Return whether a connection may be released."""
    for connector_id in connection.connectors:
        connector_type = world.connector_type(connector_id)
        if not connector_type.supports_undocking:
            return _denied(
                "undocking_unsupported",
                f"connector type '{connector_type.id}' declares supports_undocking: false",
            )
    return GuardResult(allowed=True)
