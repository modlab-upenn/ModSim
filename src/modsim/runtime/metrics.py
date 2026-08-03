"""Modular-robot metrics derived from world state and the event log.

These are the statistics a modular robotics dashboard needs and a generic
physics viewer cannot produce, because they are about connectors, assemblies,
and reconfiguration rather than about bodies and contacts.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

from modsim.core.entities import ConnectorLifecycleState
from modsim.core.events import (
    AssemblyMerged,
    AssemblySplit,
    ConnectorOverloaded,
    DockCommitted,
    DockFailed,
    UndockCommitted,
    UndockFailed,
)
from modsim.core.state import WorldState


@dataclass(frozen=True, slots=True)
class DockingMetrics:
    """One snapshot of modular-robot runtime metrics."""

    time_s: float
    module_count_total: int
    module_count_connected: int
    module_count_free: int
    assembly_count: int
    largest_assembly_size: int
    connection_count_active: int
    connector_count_total: int
    connector_count_free: int
    connector_count_candidate: int
    connector_count_docked: int
    docking_success_count: int
    docking_failure_count: int
    docking_attempt_count: int
    undocking_success_count: int
    undocking_failure_count: int
    assembly_merge_count: int
    assembly_split_count: int
    connector_overload_count: int
    event_count_total: int

    @property
    def docking_success_rate(self) -> float:
        """Return successful docks over attempted docks, or zero when idle."""
        if self.docking_attempt_count == 0:
            return 0.0
        return self.docking_success_count / self.docking_attempt_count

    def as_dict(self) -> dict[str, float | int]:
        """Return a flat mapping suitable for logging or a metrics panel."""
        return dict(asdict(self))


def collect_metrics(world: WorldState) -> DockingMetrics:
    """Compute the current metric snapshot for a world."""
    log = world.event_log
    connected = {
        module_id
        for connection in world.connections.values()
        for module_id in (connection.module_a, connection.module_b)
    }
    states = [connector.lifecycle_state for connector in world.connectors.values()]
    successes = log.count_of(DockCommitted)
    failures = log.count_of(DockFailed)
    return DockingMetrics(
        time_s=world.time_s,
        module_count_total=len(world.modules),
        module_count_connected=len(connected),
        module_count_free=len(world.modules) - len(connected),
        assembly_count=world.assemblies.count,
        largest_assembly_size=world.assemblies.largest_size,
        connection_count_active=len(world.connections),
        connector_count_total=len(states),
        connector_count_free=states.count(ConnectorLifecycleState.FREE),
        connector_count_candidate=states.count(ConnectorLifecycleState.CANDIDATE_DETECTED)
        + states.count(ConnectorLifecycleState.IN_ACCEPTANCE_REGION),
        connector_count_docked=states.count(ConnectorLifecycleState.DOCKED)
        + states.count(ConnectorLifecycleState.LOAD_BEARING),
        docking_success_count=successes,
        docking_failure_count=failures,
        docking_attempt_count=successes + failures,
        undocking_success_count=log.count_of(UndockCommitted),
        undocking_failure_count=log.count_of(UndockFailed),
        assembly_merge_count=log.count_of(AssemblyMerged),
        assembly_split_count=log.count_of(AssemblySplit),
        connector_overload_count=log.count_of(ConnectorOverloaded),
        event_count_total=len(log),
    )
