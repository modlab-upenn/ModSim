"""Qt-free presentation state for the Studio Runtime Inspector.

The runtime worker publishes immutable inspection frames.  This module turns
those frames into stable two-dimensional graph geometry and an accumulated
event table without importing Qt, PyQtGraph, MuJoCo, or a live ``WorldState``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Literal

from modsim.model_views import ModuleTopologyGraphView
from modsim.runtime.inspection import RuntimeEventRow, RuntimeInspectorFrame

SelectionKind = Literal["node", "edge"]
Point2D = tuple[float, float]


class RuntimePresentationError(ValueError):
    """Base exception for inconsistent immutable runtime frames."""


class RuntimeEventSequenceError(RuntimePresentationError):
    """Raised when event deltas contain a gap or conflicting sequence row."""


@dataclass(frozen=True, slots=True)
class GraphSelection:
    """One stable graph entity selected by the user."""

    kind: SelectionKind
    entity_id: str


@dataclass(frozen=True, slots=True)
class PresentedModuleNode:
    """One module node plus renderer-owned logical layout coordinates."""

    id: str
    label: str
    module_type_id: str
    assembly_id: str
    position: Point2D
    selected: bool = False


@dataclass(frozen=True, slots=True)
class PresentedConnectionEdge:
    """One connection edge represented by a selectable polyline."""

    id: str
    source: str
    target: str
    connector_a: str
    connector_b: str
    path: tuple[Point2D, ...]
    selected: bool = False
    state: Literal["committed", "pending", "matched"] = "committed"


@dataclass(frozen=True, slots=True)
class RuntimePresentation:
    """Complete immutable state consumed by Runtime Inspector widgets."""

    nodes: tuple[PresentedModuleNode, ...]
    edges: tuple[PresentedConnectionEdge, ...]
    events: tuple[RuntimeEventRow, ...]
    selection: GraphSelection | None
    source_text: str
    status_text: str
    show_labels: bool = True


class RuntimeInspectorPresenter:
    """Accumulate runtime frames while preserving layout and selection.

    Module positions are presentation state.  Existing nodes do not move when
    the topology changes, and backend pose samples therefore cannot make the
    logical topology graph jitter.  Stable module and connection IDs preserve
    selection until the selected entity actually disappears.
    """

    def __init__(self) -> None:
        self._positions: dict[str, Point2D] = {}
        self._events: dict[int, RuntimeEventRow] = {}
        self._event_cursor: int | None = None
        self._selection: GraphSelection | None = None
        self._frame: RuntimeInspectorFrame | None = None
        self._presentation: RuntimePresentation | None = None
        self._show_labels = True

    @property
    def frame(self) -> RuntimeInspectorFrame | None:
        return self._frame

    def set_show_labels(self, visible: bool) -> RuntimePresentation:
        self._show_labels = visible
        self._presentation = self._build_presentation(self._require_frame())
        return self._presentation

    def target_presentation(self) -> RuntimePresentation | None:
        frame = self._frame
        live = self._presentation
        if frame is None or frame.planning is None or live is None:
            return None
        committed = {frozenset((e.connector_a, e.connector_b)) for e in frame.view.edges}
        edges = tuple(
            PresentedConnectionEdge(
                id=f"target:{bond.a}<->{bond.b}",
                source=bond.a.split("/")[0],
                target=bond.b.split("/")[0],
                connector_a=bond.a,
                connector_b=bond.b,
                path=_edge_path(
                    self._positions[bond.a.split("/")[0]],
                    self._positions[bond.b.split("/")[0]],
                    offset_rank=0,
                    self_loop_index=0,
                ),
                state="matched" if frozenset((bond.a, bond.b)) in committed else "pending",
            )
            for bond in frame.planning.target_bonds
        )
        return replace(live, edges=edges, events=(), status_text="Target topology")

    @property
    def presentation(self) -> RuntimePresentation | None:
        """Return the most recently generated presentation, if any."""
        return self._presentation

    def apply_frame(self, frame: RuntimeInspectorFrame) -> RuntimePresentation:
        """Apply one immutable worker frame and return its presentation.

        Event deltas are merged before graph freshness is considered.  This
        means an already-seen or pose-stale graph frame cannot make a unique
        event disappear.  Regressing source stamps never replace the visible
        graph.
        """
        self._merge_events(frame)
        if self._frame is not None and _source_regresses(
            frame.view,
            self._frame.view,
        ):
            if self._presentation is None:  # pragma: no cover - invariant guard
                raise RuntimePresentationError("presenter has a frame without a presentation")
            self._presentation = self._presentation_with_events(self._presentation)
            return self._presentation

        self._frame = frame
        self._sync_layout(frame.view)
        self._drop_missing_selection(frame.view)
        self._presentation = self._build_presentation(frame)
        return self._presentation

    def select(self, kind: str, entity_id: str) -> RuntimePresentation:
        """Select one visible node or edge by its stable runtime identifier."""
        frame = self._require_frame()
        if kind not in ("node", "edge"):
            raise ValueError("selection kind must be 'node' or 'edge'")
        valid_ids = (
            {node.id for node in frame.view.nodes}
            if kind == "node"
            else {edge.id for edge in frame.view.edges}
        )
        if entity_id not in valid_ids:
            raise KeyError(f"unknown graph {kind} '{entity_id}'")
        self._selection = GraphSelection(kind=kind, entity_id=entity_id)
        self._presentation = self._build_presentation(frame)
        return self._presentation

    def clear_selection(self) -> RuntimePresentation:
        """Clear the current graph selection."""
        frame = self._require_frame()
        self._selection = None
        self._presentation = self._build_presentation(frame)
        return self._presentation

    def _require_frame(self) -> RuntimeInspectorFrame:
        if self._frame is None:
            raise RuntimePresentationError("no runtime frame has been applied")
        return self._frame

    def _merge_events(self, frame: RuntimeInspectorFrame) -> None:
        start = frame.event_start_sequence
        stop = frame.next_event_sequence
        if self._event_cursor is None:
            self._event_cursor = start
        assert self._event_cursor is not None
        if start > self._event_cursor:
            raise RuntimeEventSequenceError(
                f"runtime event delta starts at {start}, expected at most {self._event_cursor}"
            )

        for event in frame.events:
            existing = self._events.get(event.sequence)
            if existing is not None and existing != event:
                raise RuntimeEventSequenceError(
                    f"runtime event sequence {event.sequence} has conflicting contents"
                )
            self._events[event.sequence] = event

        if stop > self._event_cursor:
            expected = set(range(self._event_cursor, stop))
            missing = sorted(expected.difference(self._events))
            if missing:
                raise RuntimeEventSequenceError(
                    f"runtime event delta is missing sequence {missing[0]}"
                )
            self._event_cursor = stop

    def _sync_layout(self, view: ModuleTopologyGraphView) -> None:
        node_ids = tuple(node.id for node in view.nodes)
        live_ids = set(node_ids)
        for removed in tuple(self._positions):
            if removed not in live_ids:
                del self._positions[removed]

        missing = tuple(identifier for identifier in node_ids if identifier not in self._positions)
        if not self._positions:
            self._positions.update(_initial_layout(missing))
            return
        for ordinal, identifier in enumerate(missing):
            self._positions[identifier] = _added_node_position(identifier, ordinal)

    def _drop_missing_selection(self, view: ModuleTopologyGraphView) -> None:
        selection = self._selection
        if selection is None:
            return
        visible = (
            {node.id for node in view.nodes}
            if selection.kind == "node"
            else {edge.id for edge in view.edges}
        )
        if selection.entity_id not in visible:
            self._selection = None

    def _build_presentation(self, frame: RuntimeInspectorFrame) -> RuntimePresentation:
        selection = self._selection
        nodes = tuple(
            PresentedModuleNode(
                id=node.id,
                label=node.label,
                module_type_id=node.module_type_id,
                assembly_id=node.assembly_id,
                position=self._positions[node.id],
                selected=selection == GraphSelection("node", node.id),
            )
            for node in frame.view.nodes
        )
        edge_paths = _edge_paths(frame.view, self._positions)
        edges = tuple(
            PresentedConnectionEdge(
                id=edge.id,
                source=edge.source,
                target=edge.target,
                connector_a=edge.connector_a,
                connector_b=edge.connector_b,
                path=edge_paths[edge.id],
                selected=selection == GraphSelection("edge", edge.id),
            )
            for edge in frame.view.edges
        )
        return RuntimePresentation(
            nodes=nodes,
            edges=edges,
            events=tuple(self._events[index] for index in sorted(self._events)),
            selection=selection,
            source_text=_source_text(frame.view),
            status_text=_status_text(frame),
            show_labels=self._show_labels,
        )

    def _presentation_with_events(
        self,
        presentation: RuntimePresentation,
    ) -> RuntimePresentation:
        events = tuple(self._events[index] for index in sorted(self._events))
        if events == presentation.events:
            return presentation
        return RuntimePresentation(
            nodes=presentation.nodes,
            edges=presentation.edges,
            events=events,
            selection=presentation.selection,
            source_text=presentation.source_text,
            status_text=presentation.status_text,
            show_labels=presentation.show_labels,
        )


def _initial_layout(node_ids: tuple[str, ...]) -> dict[str, Point2D]:
    ordered = tuple(sorted(node_ids))
    if not ordered:
        return {}
    if len(ordered) == 1:
        return {ordered[0]: (0.0, 0.0)}
    if len(ordered) == 2:
        return {ordered[0]: (-1.0, 0.0), ordered[1]: (1.0, 0.0)}
    return {
        identifier: (
            math.cos(-math.pi / 2.0 + 2.0 * math.pi * index / len(ordered)),
            math.sin(-math.pi / 2.0 + 2.0 * math.pi * index / len(ordered)),
        )
        for index, identifier in enumerate(ordered)
    }


def _added_node_position(identifier: str, ordinal: int) -> Point2D:
    # A stable character-weighted angle avoids Python's randomized hash while
    # retaining every existing node coordinate.
    seed = sum((index + 1) * ord(character) for index, character in enumerate(identifier))
    angle = (seed % 360) * math.pi / 180.0 + ordinal * 2.399963229728653
    radius = 1.4 + 0.18 * ordinal
    return radius * math.cos(angle), radius * math.sin(angle)


def _edge_paths(
    view: ModuleTopologyGraphView,
    positions: dict[str, Point2D],
) -> dict[str, tuple[Point2D, ...]]:
    groups: dict[tuple[str, str], list[str]] = {}
    edges_by_id = {edge.id: edge for edge in view.edges}
    for edge in view.edges:
        pair = tuple(sorted((edge.source, edge.target)))
        groups.setdefault((pair[0], pair[1]), []).append(edge.id)

    result: dict[str, tuple[Point2D, ...]] = {}
    for pair in sorted(groups):
        identifiers = sorted(groups[pair])
        for index, identifier in enumerate(identifiers):
            edge = edges_by_id[identifier]
            offset_rank = index - (len(identifiers) - 1) / 2.0
            result[identifier] = _edge_path(
                positions[edge.source],
                positions[edge.target],
                offset_rank=offset_rank,
                self_loop_index=index,
            )
    return result


def _edge_path(
    source: Point2D,
    target: Point2D,
    *,
    offset_rank: float,
    self_loop_index: int,
) -> tuple[Point2D, ...]:
    if source == target:
        radius = 0.34 + self_loop_index * 0.12
        return tuple(
            (
                source[0] + radius * math.cos(2.0 * math.pi * index / 24.0),
                source[1] + radius * math.sin(2.0 * math.pi * index / 24.0),
            )
            for index in range(25)
        )

    dx = target[0] - source[0]
    dy = target[1] - source[1]
    distance = math.hypot(dx, dy)
    normal = (-dy / distance, dx / distance)
    spacing = min(max(distance * 0.22, 0.18), 0.5)
    control = (
        (source[0] + target[0]) / 2.0 + normal[0] * spacing * offset_rank,
        (source[1] + target[1]) / 2.0 + normal[1] * spacing * offset_rank,
    )
    points: list[Point2D] = []
    for index in range(25):
        parameter = index / 24.0
        inverse = 1.0 - parameter
        points.append(
            (
                inverse * inverse * source[0]
                + 2.0 * inverse * parameter * control[0]
                + parameter * parameter * target[0],
                inverse * inverse * source[1]
                + 2.0 * inverse * parameter * control[1]
                + parameter * parameter * target[1],
            )
        )
    return tuple(points)


def _source_regresses(
    incoming: ModuleTopologyGraphView,
    current: ModuleTopologyGraphView,
) -> bool:
    incoming_source = incoming.source
    current_source = current.source
    for name in (
        "sample_sequence",
        "topology_revision",
        "docking_revision",
        "event_revision",
    ):
        incoming_value = getattr(incoming_source, name)
        current_value = getattr(current_source, name)
        if (
            incoming_value is not None
            and current_value is not None
            and incoming_value < current_value
        ):
            return True
    return False


def _source_text(view: ModuleTopologyGraphView) -> str:
    source = view.source
    fields = [f"{source.pack_id}@{source.pack_version}"]
    if source.world_time_s is not None:
        fields.append(f"t={source.world_time_s:.3f} s")
    for label, value in (
        ("sample", source.sample_sequence),
        ("topology", source.topology_revision),
        ("docking", source.docking_revision),
        ("events", source.event_revision),
    ):
        if value is not None:
            fields.append(f"{label}={value}")
    return " | ".join(fields)


def _status_text(frame: RuntimeInspectorFrame) -> str:
    scenario_fields = ["running"]
    scenario = frame.scenario
    if scenario is not None:
        phase_value = scenario.phase
        phase = phase_value.value if hasattr(phase_value, "value") else str(phase_value)
        scenario_fields = [phase]
        scenario_fields.insert(0, scenario.plan_name)
        if scenario.action_index is not None:
            scenario_fields.append(f"action={scenario.action_index + 1}/{scenario.action_count}")
        if scenario.detail:
            scenario_fields.append(scenario.detail)
    metrics = frame.metrics
    return (
        f"{frame.backend_name} | {' | '.join(scenario_fields)} | "
        f"modules={metrics.module_count_total} | "
        f"assemblies={metrics.assembly_count} | "
        f"connections={metrics.connection_count_active} | "
        f"largest={metrics.largest_assembly_size} | "
        f"docks={metrics.docking_success_count}/{metrics.docking_attempt_count} "
        f"(failed={metrics.docking_failure_count}) | "
        f"undocks={metrics.undocking_success_count} "
        f"(failed={metrics.undocking_failure_count}) | "
        f"events={metrics.event_count_total}"
    )


__all__ = [
    "GraphSelection",
    "PresentedConnectionEdge",
    "PresentedModuleNode",
    "RuntimeEventSequenceError",
    "RuntimeInspectorPresenter",
    "RuntimePresentation",
    "RuntimePresentationError",
]
