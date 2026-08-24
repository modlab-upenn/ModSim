"""Immutable Runtime Inspector transport and event-delta tests."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import DockCandidateDetected
from modsim.core.ids import ConnectorInstanceId
from modsim.core.scene import SceneSpec
from modsim.model_views import ModelViewFactory
from modsim.robot_packs import RobotPack
from modsim.runtime.inspection import (
    RuntimeInspectionError,
    RuntimeInspectorFrame,
    build_runtime_inspector_frame,
    event_row,
)
from modsim.runtime.reconfiguration import (
    ReconfigurationPhase,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
    connector_pair_plan,
)
from modsim.runtime.session import RuntimeSession


def _docking_scenario(pack: RobotPack) -> ScriptedReconfigurationScenario:
    session = RuntimeSession.create(
        pack,
        SceneSpec.grid("generic_cube", 2, spacing_m=1.0),
        MockBackendAdapter(),
    )
    return ScriptedReconfigurationScenario.create(
        session,
        connector_pair_plan(
            ConnectorInstanceId("generic_cube_0/front"),
            ConnectorInstanceId("generic_cube_1/front"),
            include_undock=False,
        ),
        ScriptedReconfigurationConfig(
            gap_m=0.005,
            approach_speed_m_s=0.0,
            dt_s=0.01,
            initial_hold_s=0.0,
        ),
    )


def test_inspector_frame_copies_initial_graph_metrics_and_status(
    example_pack: RobotPack,
) -> None:
    scenario = _docking_scenario(example_pack)
    frame = build_runtime_inspector_frame(
        scenario.session,
        example_pack.manifest.model_views[0],
        ModelViewFactory(),
        scenario_status=scenario.status,
    )

    assert frame.backend_name == "mock"
    assert len(frame.view.nodes) == 2
    assert frame.view.edges == ()
    assert frame.metrics.module_count_total == 2
    assert frame.events == ()
    assert frame.event_start_sequence == 0
    assert frame.next_event_sequence == 0
    assert frame.scenario is not None
    assert frame.scenario.phase is ReconfigurationPhase.APPROACHING
    assert frame.view.source.sample_sequence == 2


def test_inspector_frame_delivers_each_event_once_as_a_contiguous_delta(
    example_pack: RobotPack,
) -> None:
    scenario = _docking_scenario(example_pack)
    factory = ModelViewFactory()
    recipe = example_pack.manifest.model_views[0]

    initial = build_runtime_inspector_frame(
        scenario.session,
        recipe,
        factory,
        scenario_status=scenario.status,
    )
    scenario.step()
    docked = build_runtime_inspector_frame(
        scenario.session,
        recipe,
        factory,
        event_cursor=initial.next_event_sequence,
        scenario_status=scenario.status,
    )

    assert [row.sequence for row in docked.events] == [0, 1, 2]
    assert [row.kind for row in docked.events] == [
        "DockCandidateDetected",
        "DockCommitted",
        "AssemblyMerged",
    ]
    assert "generic_cube_0/front" in docked.events[1].detail
    assert docked.event_start_sequence == 0
    assert docked.next_event_sequence == 3
    assert len(docked.view.edges) == 1
    assert docked.view.source.topology_revision == 1
    assert docked.scenario is not None
    assert docked.scenario.phase is ReconfigurationPhase.DOCKING

    unchanged = build_runtime_inspector_frame(
        scenario.session,
        recipe,
        factory,
        event_cursor=docked.next_event_sequence,
        scenario_status=scenario.status,
    )
    assert unchanged.events == ()
    assert unchanged.event_start_sequence == unchanged.next_event_sequence == 3


def test_inspector_transport_is_json_safe_and_frozen(example_pack: RobotPack) -> None:
    scenario = _docking_scenario(example_pack)
    frame = build_runtime_inspector_frame(
        scenario.session,
        example_pack.manifest.model_views[0],
        ModelViewFactory(),
        scenario_status=scenario.status,
    )

    document = frame.model_dump(mode="json")
    assert document["view"]["nodes"][0]["kind"] == "module"
    assert document["metrics"]["module_count_total"] == 2
    assert document["scenario"]["phase"] == "approaching"
    with pytest.raises(ValidationError, match="frozen"):
        frame.backend_name = "changed"  # type: ignore[misc]


def test_inspector_frame_rejects_event_cursor_gaps(example_pack: RobotPack) -> None:
    scenario = _docking_scenario(example_pack)
    scenario.step()
    frame = build_runtime_inspector_frame(
        scenario.session,
        example_pack.manifest.model_views[0],
        ModelViewFactory(),
    )

    with pytest.raises(ValidationError, match="contiguous inspector event interval"):
        RuntimeInspectorFrame(
            backend_name=frame.backend_name,
            view=frame.view,
            metrics=frame.metrics,
            events=frame.events[1:],
            event_start_sequence=frame.event_start_sequence,
            next_event_sequence=frame.next_event_sequence,
            scenario=frame.scenario,
        )
    with pytest.raises(ValueError, match="between 0 and 3"):
        build_runtime_inspector_frame(
            scenario.session,
            example_pack.manifest.model_views[0],
            ModelViewFactory(),
            event_cursor=4,
        )


def test_unrecorded_event_cannot_cross_the_inspector_boundary() -> None:
    event = DockCandidateDetected(
        time_s=0.0,
        connector_a=ConnectorInstanceId("generic_cube_0/front"),
        connector_b=ConnectorInstanceId("generic_cube_1/front"),
    )

    with pytest.raises(RuntimeInspectionError, match="before it is recorded"):
        event_row(event)
