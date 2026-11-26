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
from modsim.core.surface import SurfaceContact, central_angle
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
        PhysicalConstraintType.BALL,
    }
)
"""Constraint intents the format-0.1 engine can ask a backend to realise.

``custom`` may be authored, but its parameters are not modelled, so committing
it would mean guessing at physics. ``ball`` is supported only with its anchor
(``physical_connection.ball``); a bare ``ball`` is treated the same way.
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
    if left.constraint is PhysicalConstraintType.BALL and left.ball != right.ball:
        return None, (
            f"connector types '{type_a.id}' and '{type_b.id}' declare conflicting ball parameters"
        )
    if (
        left.surface_realizations
        and right.surface_realizations
        and left.surface_realizations != right.surface_realizations
    ):
        return None, (
            f"connector types '{type_a.id}' and '{type_b.id}' declare conflicting "
            "surface realizations"
        )
    if not left.surface_realizations and right.surface_realizations:
        return right, None
    return left, None


def evaluate_dock_guards(
    world: WorldState,
    connector_a: ConnectorInstance,
    connector_b: ConnectorInstance,
    *,
    requested: bool,
    surface_contact: SurfaceContact | None = None,
) -> GuardResult:
    """Return whether these two connectors may attempt to dock now.

    For a point-to-surface pair, ``surface_contact`` is where the point side
    meets the surface; it is needed for the separation guard. Without it the
    separation guard is skipped, because there is no candidate normal yet.
    """
    if connector_a.id == connector_b.id:
        return _denied("self_pair", "a connector cannot dock to itself")
    if connector_a.module_id == connector_b.module_id:
        return _denied(
            "same_module",
            f"connectors '{connector_a.id}' and '{connector_b.id}' belong to the same module",
        )
    for connector in (connector_a, connector_b):
        if connector.surface:
            denied = _surface_guard(world, connector, surface_contact)
            if denied is not None:
                return denied
            continue
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
    if physical.constraint not in SUPPORTED_CONSTRAINTS or (
        physical.constraint is PhysicalConstraintType.BALL and physical.ball is None
    ):
        return _denied(
            "physical_connection_unsupported",
            f"physical connection '{physical.constraint.value}' is not modelled in format 0.1",
        )
    if (connector_a.surface or connector_b.surface) and (
        physical.constraint is not PhysicalConstraintType.FIXED
    ):
        return _denied(
            "physical_connection_unsupported",
            f"a point-to-surface connection must be authored as 'fixed', not "
            f"'{physical.constraint.value}'; choose its realisation with surface_realizations",
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


def _surface_guard(
    world: WorldState,
    surface: ConnectorInstance,
    contact: SurfaceContact | None,
) -> GuardResult | None:
    """Return the capacity or separation denial for a surface connector, if any."""
    if not surface.has_capacity:
        return _denied(
            "surface_full",
            f"surface connector '{surface.id}' already holds {len(surface.connection_ids)} of "
            f"{surface.capacity} connections",
        )
    if contact is None or not surface.connection_ids:
        return None
    spec = world.connector_type(surface.id).surface
    if spec is None or spec.min_separation_rad <= 0.0:
        return None
    for held in surface.connection_ids:
        attachment = world.surface_attachment(held)
        if attachment is None:
            continue
        held_normal = surface.world_pose.apply_direction(attachment.normal_local)
        angle = central_angle(contact.normal_world, held_normal)
        if angle < spec.min_separation_rad:
            return _denied(
                "surface_separation",
                f"attachment on '{surface.id}' is {angle:.6g} rad from connection '{held}'; "
                f"the minimum separation is {spec.min_separation_rad:.6g} rad",
            )
    return None


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
