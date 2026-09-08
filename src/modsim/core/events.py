"""Runtime events and the append-only event log.

Every semantic change to a :class:`~modsim.core.state.WorldState` is expressed
as an event. Nothing else mutates the world, which makes docking history
replayable and makes "what changed and why" answerable from the log alone.

Events are keyword-only so that a subclass can declare required fields without
inheriting the base class's default-ordering constraints.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass, replace
from enum import StrEnum

from modsim.core.ids import (
    AssemblyId,
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
)
from modsim.core.transforms import Transform
from modsim.robot_packs.schema import PhysicalConstraintType


class DockFailureReason(StrEnum):
    """Why a dock or undock attempt did not commit."""

    INCOMPATIBLE = "incompatible"
    OUTSIDE_ACCEPTANCE_REGION = "outside_acceptance_region"
    GUARD_REJECTED = "guard_rejected"
    BACKEND_REFUSED = "backend_refused"
    UNKNOWN_CONNECTOR = "unknown_connector"
    UNKNOWN_CONNECTION = "unknown_connection"
    UNDOCKING_UNSUPPORTED = "undocking_unsupported"


@dataclass(frozen=True, slots=True, kw_only=True)
class Event:
    """Base class for every world event."""

    time_s: float
    sequence: int = -1

    @property
    def kind(self) -> str:
        """Return the event class name, which is its stable kind label."""
        return type(self).__name__


@dataclass(frozen=True, slots=True, kw_only=True)
class DockCandidateDetected(Event):
    """A compatible connector pair entered the broad-phase neighbourhood."""

    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId


@dataclass(frozen=True, slots=True, kw_only=True)
class DockCommitted(Event):
    """A logical connection was created after the backend accepted a constraint."""

    connection_id: ConnectionId
    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId
    constraint_handle: ConstraintHandle
    relative_transform: Transform
    constraint: PhysicalConstraintType = PhysicalConstraintType.FIXED
    orientation_rad: float = 0.0
    orientation_index: int | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class DockFailed(Event):
    """A dock attempt was rejected before or by the backend."""

    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId
    reason: DockFailureReason
    detail: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class UndockCommitted(Event):
    """A logical connection was removed after the backend released its constraint."""

    connection_id: ConnectionId
    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId


@dataclass(frozen=True, slots=True, kw_only=True)
class UndockFailed(Event):
    """An undock attempt was rejected before or by the backend."""

    connection_id: ConnectionId
    reason: DockFailureReason
    detail: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class AssemblyMerged(Event):
    """Two assemblies became one physically connected component."""

    assembly_id: AssemblyId
    merged_from: tuple[AssemblyId, ...] = ()


@dataclass(frozen=True, slots=True, kw_only=True)
class AssemblySplit(Event):
    """One assembly separated into two physically connected components."""

    source_assembly_id: AssemblyId
    resulting: tuple[AssemblyId, ...] = ()


@dataclass(frozen=True, slots=True, kw_only=True)
class ConnectorOverloaded(Event):
    """A connection's measured constraint force exceeded a declared limit."""

    connection_id: ConnectionId
    measured_force_n: float
    limit_n: float
    released: bool = False


class EventLog:
    """Append-only, sequence-numbered log of world events."""

    __slots__ = ("_events",)

    def __init__(self, events: Iterable[Event] = ()) -> None:
        self._events: list[Event] = list(events)

    def append(self, event: Event) -> Event:
        """Assign the next sequence number and append the event."""
        numbered = replace(event, sequence=len(self._events))
        self._events.append(numbered)
        return numbered

    def extend(self, events: Iterable[Event]) -> tuple[Event, ...]:
        """Append several events in order and return them with sequence numbers."""
        return tuple(self.append(event) for event in events)

    @property
    def events(self) -> tuple[Event, ...]:
        """Return every recorded event in commit order."""
        return tuple(self._events)

    def of_kind(self, *kinds: type[Event]) -> tuple[Event, ...]:
        """Return every recorded event matching one of the given classes."""
        return tuple(event for event in self._events if isinstance(event, kinds))

    def count_of(self, *kinds: type[Event]) -> int:
        """Return how many recorded events match one of the given classes."""
        return len(self.of_kind(*kinds))

    def since(self, sequence: int) -> tuple[Event, ...]:
        """Return every event recorded at or after ``sequence``."""
        return tuple(event for event in self._events if event.sequence >= sequence)

    def __len__(self) -> int:
        return len(self._events)

    def __iter__(self) -> Iterator[Event]:
        return iter(self._events)
