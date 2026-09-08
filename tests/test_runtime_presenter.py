"""Deterministic, Qt-free Runtime Inspector presentation tests."""

from __future__ import annotations

import pytest

from modsim.model_views import (
    DockedConnectionEdge,
    ModelViewSourceStamp,
    ModuleGraphNode,
    ModuleTopologyGraphView,
)
from modsim.runtime.inspection import (
    RuntimeEventRow,
    RuntimeInspectorFrame,
    RuntimeScenarioStatus,
)
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim.runtime.metrics import DockingMetrics
from modsim.runtime.reconfiguration import ReconfigurationPhase, ReconfigurationStatus
from modsim_studio.runtime_presenter import (
    RuntimeEventSequenceError,
    RuntimeInspectorPresenter,
    RuntimePresentation,
)


def metrics(*, connections: int = 0, events: int = 0) -> DockingMetrics:
    return DockingMetrics(
        time_s=0.0,
        module_count_total=2,
        module_count_connected=2 if connections else 0,
        module_count_free=0 if connections else 2,
        assembly_count=1 if connections else 2,
        largest_assembly_size=2 if connections else 1,
        connection_count_active=connections,
        connector_count_total=4,
        connector_count_free=2 if connections else 4,
        connector_count_candidate=0,
        connector_count_docked=2 if connections else 0,
        docking_success_count=connections,
        docking_failure_count=0,
        docking_attempt_count=connections,
        undocking_success_count=0,
        undocking_failure_count=0,
        assembly_merge_count=connections,
        assembly_split_count=0,
        connector_overload_count=0,
        event_count_total=events,
    )


def node(identifier: str, assembly: str, *, x: float) -> ModuleGraphNode:
    return ModuleGraphNode(
        id=identifier,
        label=identifier,
        module_type_id="generic_cube",
        assembly_id=assembly,
        world_position_m=(x, 0.0, 0.0),
        world_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    )


def edge(identifier: str, first: str = "alpha", second: str = "beta") -> DockedConnectionEdge:
    return DockedConnectionEdge(
        id=identifier,
        connection_id=identifier,
        source=first,
        target=second,
        connector_a=f"{first}/front",
        connector_b=f"{second}/rear",
        orientation_rad=0.0,
        orientation_index=0,
        created_at_s=0.2,
    )


def view(
    *,
    sample: int = 0,
    topology: int = 0,
    docking: int = 0,
    event_revision: int = 0,
    connections: tuple[DockedConnectionEdge, ...] = (),
    alpha_x: float = 0.0,
) -> ModuleTopologyGraphView:
    assembly = "assembly:alpha" if connections else None
    return ModuleTopologyGraphView(
        id="module_topology",
        name="Module Topology",
        builder="module_topology_graph",
        source=ModelViewSourceStamp(
            pack_id="generic_cube",
            pack_version="0.1.0",
            world_time_s=sample * 0.01,
            sample_sequence=sample,
            topology_revision=topology,
            docking_revision=docking,
            event_revision=event_revision,
        ),
        nodes=(
            node("alpha", assembly or "assembly:alpha", x=alpha_x),
            node("beta", assembly or "assembly:beta", x=1.0),
        ),
        edges=connections,
    )


def frame(
    graph: ModuleTopologyGraphView,
    *,
    events: tuple[RuntimeEventRow, ...] = (),
    start: int = 0,
    stop: int = 0,
    scenario: RuntimeScenarioStatus | None = None,
) -> RuntimeInspectorFrame:
    return RuntimeInspectorFrame(
        backend_name="mock",
        view=graph,
        metrics=metrics(connections=len(graph.edges), events=stop),
        events=events,
        event_start_sequence=start,
        next_event_sequence=stop,
        scenario=scenario,
    )


def test_initial_layout_is_deterministic_and_ignores_physical_pose_samples() -> None:
    presenter = RuntimeInspectorPresenter()
    initial = presenter.apply_frame(frame(view()))

    assert isinstance(initial, RuntimePresentation)
    assert {item.id: item.position for item in initial.nodes} == {
        "alpha": (-1.0, 0.0),
        "beta": (1.0, 0.0),
    }
    assert "mock | running" in initial.status_text
    assert initial.status_text == (
        "mock | running | modules=2 | assemblies=2 | connections=0 | largest=1 | "
        "docks=0/0 (failed=0) | undocks=0 (failed=0) | events=0"
    )
    assert initial.source_text == (
        "generic_cube@0.1.0 | t=0.000 s | sample=0 | topology=0 | docking=0 | events=0"
    )

    moved = presenter.apply_frame(frame(view(sample=1, alpha_x=42.0)))
    assert isinstance(moved, RuntimePresentation)
    assert {item.id: item.position for item in moved.nodes} == {
        "alpha": (-1.0, 0.0),
        "beta": (1.0, 0.0),
    }


def test_reconfiguration_status_shows_plan_action_and_metrics() -> None:
    presenter = RuntimeInspectorPresenter()
    scenario = ReconfigurationStatus(
        phase=ReconfigurationPhase.APPROACHING,
        time_s=2.0,
        plan_id="smores_driver_to_snake",
        plan_name="SMORES-EP Driver to Snake",
        action_index=1,
        action_count=4,
        detail="Moving module 7 toward module_6/pan",
    )

    transported = decode_runtime_message(
        encode_runtime_message(RuntimeFrame(frame=frame(view(), scenario=scenario)))
    )
    assert isinstance(transported, RuntimeFrame)
    assert isinstance(transported.frame.scenario, ReconfigurationStatus)

    presented = presenter.apply_frame(transported.frame)

    assert presented.status_text == (
        "mock | SMORES-EP Driver to Snake | approaching | action=2/4 | "
        "Moving module 7 toward module_6/pan | modules=2 | assemblies=2 | "
        "connections=0 | largest=1 | docks=0/0 (failed=0) | "
        "undocks=0 (failed=0) | events=0"
    )


def test_dock_and_undock_preserve_node_layout_and_valid_selection() -> None:
    presenter = RuntimeInspectorPresenter()
    initial = presenter.apply_frame(frame(view()))
    selected_node = presenter.select("node", "alpha")

    docked_edge = edge("connection:alpha-front--beta-rear")
    docked = presenter.apply_frame(
        frame(
            view(
                sample=1,
                topology=1,
                docking=1,
                event_revision=2,
                connections=(docked_edge,),
            )
        )
    )
    assert isinstance(initial, RuntimePresentation)
    assert isinstance(docked, RuntimePresentation)
    assert docked.selection == selected_node.selection
    assert [item.position for item in docked.nodes] == [item.position for item in initial.nodes]
    assert [item.id for item in docked.edges] == [docked_edge.id]

    presenter.select("edge", docked_edge.id)
    released = presenter.apply_frame(frame(view(sample=2, topology=2, docking=2, event_revision=4)))
    assert isinstance(released, RuntimePresentation)
    assert released.selection is None
    assert released.edges == ()
    assert [item.position for item in released.nodes] == [item.position for item in initial.nodes]


def test_parallel_connections_receive_distinct_stable_curves() -> None:
    first = edge("connection:one")
    second = edge("connection:two")
    presenter = RuntimeInspectorPresenter()
    presented = presenter.apply_frame(
        frame(view(topology=2, docking=2, connections=(first, second)))
    )

    assert isinstance(presented, RuntimePresentation)
    paths = {item.id: item.path for item in presented.edges}
    assert paths[first.id] != paths[second.id]
    assert paths[first.id][0] == paths[second.id][0] == (-1.0, 0.0)
    assert paths[first.id][-1] == paths[second.id][-1] == (1.0, 0.0)
    assert paths[first.id][12][1] == pytest.approx(-paths[second.id][12][1])

    repeated = presenter.apply_frame(
        frame(view(sample=1, topology=2, docking=2, connections=(first, second)))
    )
    assert isinstance(repeated, RuntimePresentation)
    assert {item.id: item.path for item in repeated.edges} == paths


def test_event_deltas_append_deduplicate_and_reject_gaps() -> None:
    presenter = RuntimeInspectorPresenter()
    presenter.apply_frame(frame(view()))
    rows = (
        RuntimeEventRow(sequence=0, time_s=0.1, kind="DockCandidateDetected"),
        RuntimeEventRow(sequence=1, time_s=0.2, kind="DockCommitted", detail="docked"),
    )
    updated = presenter.apply_frame(
        frame(view(sample=1, event_revision=2), events=rows, start=0, stop=2)
    )
    assert updated.events == rows

    duplicate = presenter.apply_frame(
        frame(view(sample=1, event_revision=2), events=rows, start=0, stop=2)
    )
    assert duplicate.events == rows

    with pytest.raises(RuntimeEventSequenceError, match="starts at 3"):
        presenter.apply_frame(frame(view(sample=2), start=3, stop=3))


def test_regressing_graph_frame_does_not_replace_current_presentation() -> None:
    presenter = RuntimeInspectorPresenter()
    current = presenter.apply_frame(frame(view(sample=5, topology=1)))
    stale = presenter.apply_frame(frame(view(sample=4, topology=0)))

    assert stale is current
    assert "sample=5" in stale.source_text


def test_selection_rejects_unknown_kinds_and_entities() -> None:
    presenter = RuntimeInspectorPresenter()
    presenter.apply_frame(frame(view()))

    with pytest.raises(ValueError, match="selection kind"):
        presenter.select("connector", "alpha")
    with pytest.raises(KeyError, match="unknown graph node"):
        presenter.select("node", "missing")
