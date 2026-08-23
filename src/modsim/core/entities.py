"""Mutable runtime entities tracked by a ModSim world.

Robot Pack models are frozen Pydantic documents describing *types*. The objects
here are the per-step mutable *instances* those types produce, and they are
plain dataclasses so that a simulation loop can update poses without paying
validation cost on every step.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
)
from modsim.core.transforms import ZERO_VEC3, Transform, Vec3


class ConnectorLifecycleState(StrEnum):
    """Lifecycle of a single connector instance.

    ``ALIGNING`` and ``LOAD_BEARING`` are part of the modelled lifecycle but are
    not driven by the format-0.1 docking engine, because deciding that a
    connector is aligning or load bearing needs controller intent and measured
    constraint forces respectively. They are legal states that an adapter or a
    controller may set.
    """

    FREE = "free"
    CANDIDATE_DETECTED = "candidate_detected"
    ALIGNING = "aligning"
    IN_ACCEPTANCE_REGION = "in_acceptance_region"
    LATCHING = "latching"
    DOCKED = "docked"
    LOAD_BEARING = "load_bearing"
    RELEASING = "releasing"
    FAILED = "failed"

ENGAGED_STATES = frozenset(
    {
        ConnectorLifecycleState.LATCHING,
        ConnectorLifecycleState.DOCKED,
        ConnectorLifecycleState.LOAD_BEARING,
        ConnectorLifecycleState.RELEASING,
    }
)
"""States in which a connector holds or is releasing a logical connection."""


@dataclass(slots=True)
class ModuleInstance:
    """One module instance and its most recently ingested backend state."""

    id: ModuleInstanceId
    module_type_id: str
    pose: Transform = field(default_factory=Transform.identity)
    linear_velocity_m_s: Vec3 = ZERO_VEC3
    angular_velocity_rad_s: Vec3 = ZERO_VEC3
    link_poses: dict[str, Transform] = field(default_factory=dict[str, Transform])


@dataclass(slots=True)
class ConnectorInstance:
    """One connector instance, its lifecycle state, and its resolved world frame."""

    id: ConnectorInstanceId
    module_id: ModuleInstanceId
    connector_id: str
    connector_type_id: str
    parent_link: str
    lifecycle_state: ConnectorLifecycleState = ConnectorLifecycleState.FREE
    connection_id: ConnectionId | None = None
    world_pose: Transform = field(default_factory=Transform.identity)
    world_docking_axis: Vec3 = (1.0, 0.0, 0.0)
    world_approach_axis: Vec3 = (1.0, 0.0, 0.0)
    world_velocity_m_s: Vec3 = ZERO_VEC3
    local_pose: Transform = field(default_factory=Transform.identity)
    resolved: bool = False
    available_at_s: float = 0.0

    @property
    def is_engaged(self) -> bool:
        """Whether this connector currently holds or is releasing a connection."""
        return self.lifecycle_state in ENGAGED_STATES


@dataclass(slots=True)
class ConnectionRuntime:
    """One committed logical connection backed by a backend constraint."""

    id: ConnectionId
    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId
    module_a: ModuleInstanceId
    module_b: ModuleInstanceId
    relative_transform: Transform
    orientation_rad: float
    orientation_index: int | None
    constraint_handle: ConstraintHandle
    created_at_s: float
    measured_force_n: float | None = None

    @property
    def connectors(self) -> tuple[ConnectorInstanceId, ConnectorInstanceId]:
        """Return both connector ends in canonical order."""
        return (self.connector_a, self.connector_b)
