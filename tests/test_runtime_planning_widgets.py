"""The shared Studio shell and spatial planning views without a physics engine."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtWidgets import QApplication

from modsim.planning.inspection import (
    ActionInterval,
    PlannedTrace,
    PlannerDecision,
    SpatialAction,
    SpatialPlanningSnapshot,
    TargetBond,
    TargetModule,
)
from modsim.runtime import (
    ReconfigurationPhase,
    ReconfigurationStatus,
    RuntimeInspectorConfig,
    RuntimeInspectorFrame,
    RuntimeInspectorRunner,
)
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim_studio.appearance import THEMES, theme_manager
from modsim_studio.runtime_window import RuntimeInspectorWindow


def planning_frame(
    frame: RuntimeInspectorFrame, a: str, b: str, *, complete: bool = False
) -> RuntimeInspectorFrame:
    source = frame.view.source
    assert source.world_time_s is not None
    assert source.sample_sequence is not None
    assert source.topology_revision is not None
    planning = SpatialPlanningSnapshot(
        time_s=source.world_time_s,
        sample_sequence=source.sample_sequence,
        topology_revision=source.topology_revision,
        phase="complete" if complete else "moving",
        detail="Measured handoff",
        targets=tuple(
            TargetModule(id=n.id, position_m=n.world_position_m) for n in frame.view.nodes
        ),
        target_bonds=(TargetBond(a=a, b=b),),
        traces=(
            PlannedTrace(
                module_id=frame.view.nodes[0].id,
                stage="moving",
                positions_m=((0.0, 0.0, 0.0), (0.0, 0.0, 0.15)),
            ),
        ),
        actions=(
            SpatialAction(
                id="moving",
                label="Lift & capture",
                phase="complete" if complete else "moving",
                detail="Verify measured capture",
                waypoint=1,
                waypoint_count=2,
                intervals=(ActionInterval(phase="moving", start_s=0.0),),
            ),
        ),
        decisions=(
            PlannerDecision(sequence=0, time_s=0.0, kind="Search", detail="Capture before release"),
        ),
        approach_expanded=10,
        approach_rejected=2,
        withdrawal_expanded=8,
        withdrawal_rejected=1,
        target_error_m=0.002,
        joint_error_rad=0.01,
        peak_effort_nm=0.6,
        peak_penetration_m=0.0005,
    )
    return frame.model_copy(
        update={
            "planning": planning,
            "scenario": ReconfigurationStatus(
                phase=ReconfigurationPhase.COMPLETE
                if complete
                else ReconfigurationPhase.APPROACHING,
                time_s=frame.metrics.time_s,
                plan_id="smores_spatial_handoff",
                plan_name="Supported 3D handoff",
                action_index=0,
                action_count=2,
                detail="Measured state",
            ),
        }
    )


def test_runtime_matches_planar_layout_with_live_target_planning_and_event_tabs(
    example_pack_dir: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = QApplication.instance()
    if not isinstance(app, QApplication):
        app = QApplication([])
    monkeypatch.setattr(RuntimeInspectorWindow, "start", lambda _: None)
    config = RuntimeInspectorConfig(
        pack_path=example_pack_dir, backend="mock", connector_gap_m=0.005
    )
    runner = RuntimeInspectorRunner.create(config)
    window = RuntimeInspectorWindow(config)
    original_theme = theme_manager().theme.id
    try:
        window.show()
        a = f"generic_cube_0/{runner.fixed_connector}"
        b = f"generic_cube_1/{runner.moving_connector}"
        initial = planning_frame(runner.frame(), a, b)
        message = RuntimeFrame(frame=initial)
        assert decode_runtime_message(encode_runtime_message(message)) == message
        window._receive_frame(initial)
        app.processEvents()
        assert [window.workspace_tabs.tabText(i) for i in range(3)] == [
            "Runtime state",
            "Planning",
            "Event log",
        ]
        assert window.target_panel.isVisible()
        assert window.workspace_tabs.isTabVisible(1)
        assert window.graph.displayed_node_ids == window.target_graph.displayed_node_ids
        assert len(window.graph.displayed_edge_ids) == 0
        assert len(window.target_graph.displayed_edge_ids) == 1
        assert "0/1" in window.target_title.text()
        for theme in THEMES:
            theme_manager().set_theme(theme, persist=False)
            window.workspace_tabs.setCurrentIndex(1)
            app.processEvents()
            assert window.planning.action_table.rowCount() == 1
            assert window.planning.decisions.rowCount() == 1
            assert window.planning.history.plot.getViewBox().state["mouseEnabled"] == [True, False]
            window.planning.projection.setCurrentIndex(1)
            assert window.planning.project((0.0, 0.0, 0.15)) == (0.0, 0.15)
            window.planning.workspace.setXRange(-0.1, 0.1, padding=0)
            before = window.planning.workspace.viewRange()
            window.planning.set_frame(initial)
            assert window.planning.workspace.viewRange() == before
        window.labels_button.setChecked(False)
        assert window.graph.displayed_label_count == window.target_graph.displayed_label_count == 0
        for _ in range(200):
            runner.step()
        final = planning_frame(runner.frame(), a, b, complete=True)
        window._receive_frame(final)
        assert "1/1" in window.target_title.text()
        assert "Target reached" in window.run_state.text()
        assert not window.pause_button.isEnabled()
        assert window.events.model.rowCount() > 0
        stale = initial.planning.model_copy(update={"sample_sequence": 999})
        bad = initial.model_copy(update={"planning": stale})
        with pytest.raises(ValueError, match="world sample"):
            RuntimeInspectorFrame.model_validate_json(bad.model_dump_json())
    finally:
        window.close()
        runner.shutdown()
        theme_manager().set_theme(original_theme, persist=False)
        app.processEvents()
