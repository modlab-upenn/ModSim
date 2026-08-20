"""Immutable transport objects for runtime-inspector clients.

The simulation thread owns ``RuntimeSession`` and its mutable ``WorldState``.
This module copies only generated model views, metrics, and display-ready event
rows across that boundary, so a Qt, web, or native client never has to traverse
live backend state while physics is stepping.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, model_validator

from modsim.core.events import (
    AssemblyMerged,
    AssemblySplit,
    ConnectorOverloaded,
    DockCandidateDetected,
    DockCommitted,
    DockFailed,
    Event,
    UndockCommitted,
    UndockFailed,
)
from modsim.model_views import ModelViewContext, ModelViewFactory, ModuleTopologyGraphView
from modsim.robot_packs.schema import ModelViewSpec
from modsim.runtime.metrics import DockingMetrics
from modsim.runtime.scenarios import DockingPairScenarioStatus
from modsim.runtime.session import RuntimeSession


class RuntimeInspectionError(RuntimeError):
    """Raised when a coherent immutable inspector frame cannot be built."""


class _RuntimeInspectionDTO(BaseModel):
    """Strict immutable base shared by inspector transport objects."""

    model_config = ConfigDict(
        allow_inf_nan=False,
        extra="forbid",
        frozen=True,
        strict=True,
        str_strip_whitespace=True,
        validate_default=True,
    )


class RuntimeEventRow(_RuntimeInspectionDTO):
    """One display-ready canonical world event."""

    sequence: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    kind: str = Field(min_length=1)
    detail: str = ""


class RuntimeInspectorFrame(_RuntimeInspectionDTO):
    """One coherent runtime update safe to publish to a client thread.

    ``events`` is a delta whose sequence interval is
    ``[event_start_sequence, next_event_sequence)``. A client advances its
    cursor to ``next_event_sequence`` only after accepting this frame. Empty
    deltas are valid and keep both cursor values equal.
    """

    backend_name: str = Field(min_length=1)
    view: ModuleTopologyGraphView
    metrics: DockingMetrics
    events: tuple[RuntimeEventRow, ...] = ()
    event_start_sequence: int = Field(ge=0)
    next_event_sequence: int = Field(ge=0)
    scenario: DockingPairScenarioStatus | None = None

    @model_validator(mode="after")
    def require_contiguous_event_delta(self) -> RuntimeInspectorFrame:
        if self.next_event_sequence < self.event_start_sequence:
            raise ValueError("next_event_sequence must not precede event_start_sequence")
        expected = tuple(range(self.event_start_sequence, self.next_event_sequence))
        actual = tuple(row.sequence for row in self.events)
        if actual != expected:
            raise ValueError(
                "event rows must exactly cover the contiguous inspector event interval"
            )
        return self


def event_row(event: Event) -> RuntimeEventRow:
    """Copy one recorded world event into a display-ready immutable row."""
    if event.sequence < 0:
        raise RuntimeInspectionError("cannot publish an event before it is recorded")
    return RuntimeEventRow(
        sequence=event.sequence,
        time_s=event.time_s,
        kind=event.kind,
        detail=format_event_detail(event),
    )


def build_runtime_inspector_frame(
    session: RuntimeSession,
    recipe: ModelViewSpec,
    factory: ModelViewFactory,
    *,
    event_cursor: int = 0,
    scenario_status: DockingPairScenarioStatus | None = None,
) -> RuntimeInspectorFrame:
    """Copy a coherent graph, metrics, and event delta from ``session``.

    Callers should invoke this on the same thread that owns and steps the
    session. The returned object is deeply composed of immutable value objects
    and may then be queued safely to a presentation thread.
    """
    event_count = len(session.world.event_log)
    if isinstance(event_cursor, bool):
        raise TypeError("event_cursor must be an integer")
    if event_cursor < 0 or event_cursor > event_count:
        raise ValueError(f"event_cursor must be between 0 and {event_count}")

    generated = factory.build(
        recipe,
        ModelViewContext(pack=session.world.pack, world=session.world),
    )
    if not isinstance(generated, ModuleTopologyGraphView):
        raise RuntimeInspectionError(
            "the first Runtime Inspector renderer requires a module-topology graph; "
            f"recipe '{recipe.id}' generated '{generated.view_type}'"
        )
    rows = tuple(event_row(event) for event in session.world.event_log.since(event_cursor))
    return RuntimeInspectorFrame(
        backend_name=session.adapter.capabilities().name,
        view=generated,
        metrics=session.metrics(),
        events=rows,
        event_start_sequence=event_cursor,
        next_event_sequence=event_count,
        scenario=scenario_status,
    )


def format_event_detail(event: Event) -> str:
    """Return the shared concise description for one canonical event."""
    if isinstance(event, DockCommitted):
        orientation = (
            "continuous" if event.orientation_index is None else f"index {event.orientation_index}"
        )
        return f"{event.connector_a} <-> {event.connector_b} ({orientation})"
    if isinstance(event, DockCandidateDetected | UndockCommitted):
        return f"{event.connector_a} <-> {event.connector_b}"
    if isinstance(event, DockFailed):
        return f"{event.connector_a} <-> {event.connector_b}: {event.detail}"
    if isinstance(event, UndockFailed):
        return f"{event.connection_id}: {event.detail}"
    if isinstance(event, AssemblyMerged):
        return f"{' + '.join(event.merged_from)} -> {event.assembly_id}"
    if isinstance(event, AssemblySplit):
        return f"{event.source_assembly_id} -> {' + '.join(event.resulting)}"
    if isinstance(event, ConnectorOverloaded):
        return (
            f"{event.connection_id}: {event.measured_force_n:.4g} N exceeded {event.limit_n:.4g} N"
        )
    return ""


__all__ = [
    "RuntimeEventRow",
    "RuntimeInspectionError",
    "RuntimeInspectorFrame",
    "build_runtime_inspector_frame",
    "event_row",
    "format_event_detail",
]
