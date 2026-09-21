"""Native planner UI consumes immutable intent and preserves display controls."""

from __future__ import annotations

import os
from dataclasses import replace
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtWidgets import QApplication

from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.model_views import ModelViewFactory
from modsim.planning import plan_assembly
from modsim.planning.models import ActionInterval, ActionObservation, PlanningSnapshot
from modsim.planning.smores import mobile_manipulator_goal, paper_initial_poses
from modsim.robot_packs import RobotPackLoader
from modsim.runtime import RuntimeSession
from modsim.runtime.inspection import RuntimeInspectorFrame, build_runtime_inspector_frame
from modsim.runtime.inspector_runner import RuntimeInspectorConfig, resolve_runtime_recipe
from modsim.runtime.reconfiguration import ReconfigurationPhase, ReconfigurationStatus
from modsim_studio.appearance import THEMES, theme_manager
from modsim_studio.runtime_planning import PlanningWorkspace
from modsim_studio.runtime_window import RuntimeInspectorWindow


@pytest.fixture
def planning_frame() -> RuntimeInspectorFrame:
    loaded = RobotPackLoader().load(
        Path(__file__).resolve().parents[1] / "examples/robot_packs/smores_ep"
    )
    poses = paper_initial_poses()
    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=ModuleInstanceId(m),
            module_type_id="smores_ep",
            pose=Transform.from_translation((p.x, p.y, 0.05)),
        )
        for m, p in poses.items()
    )
    session = RuntimeSession.create(loaded, scene, "mock")
    try:
        plan = plan_assembly(mobile_manipulator_goal(), poses)
        snapshot = PlanningSnapshot(
            time_s=session.world.time_s,
            sample_sequence=session.world.revision.sample_sequence,
            topology_revision=session.world.revision.topology_revision,
            plan_revision=1,
            plan=plan,
            actions=tuple(
                ActionObservation(
                    action=a,
                    phase="waiting",
                    reason="Waiting for corridor",
                    intervals=(ActionInterval(phase="waiting", start_s=0.0),),
                )
                for a in plan.actions
            ),
        )
        return build_runtime_inspector_frame(
            session,
            resolve_runtime_recipe(loaded.pack, None),
            ModelViewFactory(),
            planning=snapshot,
        )
    finally:
        session.shutdown()


def test_planner_workspace_displays_goal_actions_timeline_and_theme(
    planning_frame: RuntimeInspectorFrame,
) -> None:
    existing = QApplication.instance()
    app = existing if isinstance(existing, QApplication) else QApplication([])
    widget = PlanningWorkspace()
    frame = planning_frame
    manager = theme_manager()
    original = manager.theme.id
    try:
        widget.set_frame(frame)
        assert widget.action_table.rowCount() == 6
        assert "module_1" in widget.summary.text()
        widget.action_table.selectRow(0)
        assert "Waiting for corridor" in widget.detail.text()
        widget.workspace.setRange(xRange=(-1.0, 1.0), yRange=(-1.0, 1.0), padding=0)
        bounds = widget.workspace.viewRange()
        for theme in THEMES:
            manager.set_theme(theme, persist=False)
            widget.set_frame(frame)
            assert widget.workspace.viewRange() == bounds
            assert widget.action_table.currentRow() == 0
            assert widget.workspace.backgroundBrush().color().name() == manager.theme.viewport
        assert widget.history.plot.getViewBox().state["mouseEnabled"] == [True, False]
        widget.history.plot.setXRange(4.0, 8.0, padding=0)
        widget.history.plot.getViewBox().sigRangeChangedManually.emit([True, False])
        assert not widget.history.follow.isChecked()
        history_bounds = widget.history.plot.viewRange()
        assert frame.planning is not None
        widget.history.set_snapshot(frame.planning.model_copy(update={"time_s": 100.0}))
        assert widget.history.plot.viewRange() == history_bounds
        widget.history.fit_button.click()
        assert widget.history.plot.viewRange()[0][1] >= 100.0
        assert "Duration:" in next(iter(widget.history._bars.values())).toolTip()
        assert "Solid outline: measured module" in widget.legend.toggle.toolTip()
        widget.legend.toggle.setChecked(True)
        assert not widget.legend.description.isHidden()
        widget.legend.toggle.setChecked(False)
        assert widget.legend.description.isHidden()
        widget.paths.setChecked(False)
        widget.goals.setChecked(False)
        app.processEvents()
    finally:
        manager.set_theme(original, persist=False)
        widget.close()


@pytest.mark.parametrize(
    ("phase", "time_s", "expected"),
    (
        (ReconfigurationPhase.COMPLETE, 12.0, "Target reached · Simulation complete"),
        (ReconfigurationPhase.FAILED, 12.0, "Simulation failed"),
        (ReconfigurationPhase.APPROACHING, 20.0, "Time limit reached · Target not reached"),
        (ReconfigurationPhase.APPROACHING, 12.0, "Simulation stopped"),
    ),
)
def test_window_shows_topologies_event_log_and_persistent_terminal_state(
    planning_frame: RuntimeInspectorFrame,
    monkeypatch: pytest.MonkeyPatch,
    phase: ReconfigurationPhase,
    time_s: float,
    expected: str,
) -> None:
    existing = QApplication.instance()
    app = existing if isinstance(existing, QApplication) else QApplication([])
    monkeypatch.setattr(RuntimeInspectorWindow, "start", lambda _self: None)
    window = RuntimeInspectorWindow(
        RuntimeInspectorConfig(pack_path=Path("unused"), duration_s=20.0)
    )
    assert planning_frame.planning is not None
    frame = planning_frame.model_copy(
        update={
            "view": planning_frame.view.model_copy(
                update={
                    "source": planning_frame.view.source.model_copy(update={"world_time_s": time_s})
                }
            ),
            "planning": planning_frame.planning.model_copy(update={"time_s": time_s}),
            "metrics": replace(planning_frame.metrics, time_s=time_s),
            "scenario": ReconfigurationStatus(
                phase=phase,
                time_s=time_s,
                plan_id="test",
                plan_name="Test",
                action_index=None,
                action_count=6,
                detail="Test result",
            ),
        }
    )
    try:
        window._receive_frame(frame)
        window._flush_pending_frames()
        window.show()
        app.processEvents()
        assert window.workspace_tabs.tabText(2) == "Event log"
        assert window.workspace_tabs.isTabVisible(1)
        assert window.target_panel.isVisible()
        assert window.target_graph.displayed_node_ids == window.graph.displayed_node_ids
        assert len(window.target_graph.displayed_edge_ids) == 6
        assert window.graph.displayed_edge_ids == ()  # Intent never becomes live topology.
        assert "0/6 connections reached" in window.target_title.text()
        window.workspace_tabs.setCurrentIndex(2)
        assert window.events.isVisible()
        assert window.planning.decisions.isVisible()
        if expected == "Simulation stopped":
            window._runtime_finished()
        assert expected in window.run_state.text()
        window._receive_playback_state(False)
        window._receive_status("Late runtime status")
        window._runtime_finished()
        window.labels_button.click()
        assert expected in window.run_state.text()
        assert not window.pause_button.isEnabled()
        assert window.target_graph.displayed_label_count == 0
    finally:
        window.close()
