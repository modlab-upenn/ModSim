"""Stable runtime identifiers.

Runtime identifiers are distinct from Robot Pack catalog identifiers. A catalog
identifier names a *type* ("generic_cube", "fixed_face"); a runtime identifier
names an *instance* that exists in one world ("cube_3", "cube_3/front").
"""

from __future__ import annotations

from typing import NewType

ModuleInstanceId = NewType("ModuleInstanceId", str)
"""Identity of one module instance in a world."""

ConnectorInstanceId = NewType("ConnectorInstanceId", str)
"""Identity of one connector on one module instance."""

JointInstanceId = NewType("JointInstanceId", str)
"""Identity of one joint on one module instance."""

ConnectionId = NewType("ConnectionId", str)
"""Identity of one logical connection between two connector instances."""

AssemblyId = NewType("AssemblyId", str)
"""Identity of one physically connected component of modules."""

ConstraintHandle = NewType("ConstraintHandle", str)
"""Opaque backend-owned identity of a physical constraint."""

CONNECTOR_SEPARATOR = "/"
JOINT_SEPARATOR = "/"


def connector_instance_id(
    module_id: ModuleInstanceId,
    connector_id: str,
) -> ConnectorInstanceId:
    """Return the canonical connector instance identifier."""
    return ConnectorInstanceId(f"{module_id}{CONNECTOR_SEPARATOR}{connector_id}")


def split_connector_instance_id(
    connector_instance_id_: ConnectorInstanceId,
) -> tuple[ModuleInstanceId, str]:
    """Return the owning module and the module-local connector identifier."""
    module_id, separator, connector_id = connector_instance_id_.rpartition(CONNECTOR_SEPARATOR)
    if not separator:
        raise ValueError(f"malformed connector instance id: {connector_instance_id_!r}")
    return ModuleInstanceId(module_id), connector_id


def joint_instance_id(
    module_id: ModuleInstanceId,
    joint_id: str,
) -> JointInstanceId:
    """Return the canonical joint instance identifier."""
    return JointInstanceId(f"{module_id}{JOINT_SEPARATOR}{joint_id}")


def split_joint_instance_id(
    joint_instance_id_: JointInstanceId,
) -> tuple[ModuleInstanceId, str]:
    """Return the owning module and module-local joint identifier.

    A joint instance ID always has the form ``<module>/<joint>``. Splitting at
    the final separator leaves room for a future scene namespace in the module
    portion without weakening validation of the two required components.
    """
    module_id, separator, joint_id = joint_instance_id_.rpartition(JOINT_SEPARATOR)
    if not separator or not module_id or not joint_id:
        raise ValueError(f"malformed joint instance id: {joint_instance_id_!r}")
    return ModuleInstanceId(module_id), joint_id


def connection_id(
    connector_a: ConnectorInstanceId,
    connector_b: ConnectorInstanceId,
) -> ConnectionId:
    """Return a deterministic, order-independent connection identifier."""
    first, second = sorted((connector_a, connector_b))
    return ConnectionId(f"{first}<->{second}")


def assembly_id(members: frozenset[ModuleInstanceId]) -> AssemblyId:
    """Return a deterministic assembly identifier for a set of modules.

    The identifier is derived from the lexicographically smallest member rather
    than an incrementing counter, so the same physical configuration always
    produces the same identifier across runs, replays, and processes.
    """
    if not members:
        raise ValueError("an assembly must contain at least one module")
    return AssemblyId(f"assembly:{min(members)}")
