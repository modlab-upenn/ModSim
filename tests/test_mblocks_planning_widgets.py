"""M-Blocks planner presentation stays separate from canonical measured state."""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtWidgets import QApplication

from modsim.model_views import ModelViewFactory
from modsim.planning.mblocks import LatticePlanningSnapshot, plan_reconfiguration
from modsim.robot_packs import RobotPackLoader
from modsim.runtime.inspection import RuntimeInspectorFrame, build_runtime_inspector_frame
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim.runtime.inspector_runner import RuntimeInspectorConfig, resolve_runtime_recipe
from modsim.runtime.mblocks_lattice import line_goal, rectangle_state, tabletop_scene
from modsim.runtime.reconfiguration import ReconfigurationPhase, ReconfigurationStatus
from modsim.runtime.session import RuntimeSession
from modsim_studio.appearance import THEMES, theme_manager
from modsim_studio.runtime_mblocks_planning import LatticePlanningWorkspace, target_lattice_view
from modsim_studio.runtime_window import RuntimeInspectorWindow


@pytest.fixture
def lattice_frame() -> RuntimeInspectorFrame:
    loaded = RobotPackLoader().load(Path("examples/robot_packs/mblocks_3d"))
    initial = rectangle_state()
    plan = plan_reconfiguration(initial, line_goal(initial))
    session = RuntimeSession.create(loaded, tabletop_scene(initial), "mock")
    try:
        snapshot = LatticePlanningSnapshot(
            time_s=0.0,
            sample_sequence=session.world.revision.sample_sequence,
            topology_revision=session.world.revision.topology_revision,
            plan_revision=1,
            goal=plan.goal,
            anchor=plan.anchor,
            origin_world_m=(0.0, 0.0, 0.025),
            frame_yaw_rad=0.0,
            blocks=initial.blocks,
            active=plan.actions[0],
            remaining=plan.actions[1:],
        )
        return build_runtime_inspector_frame(
            session,
            resolve_runtime_recipe(loaded.pack, "mblocks_physics_lattice"),
            ModelViewFactory(),
            lattice_planning=snapshot,
        )
    finally:
        session.shutdown()


def test_lattice_protocol_roundtrip_and_sample_coherence(
    lattice_frame: RuntimeInspectorFrame,
) -> None:
    message = RuntimeFrame(frame=lattice_frame)
    assert decode_runtime_message(encode_runtime_message(message)) == message
    assert lattice_frame.lattice_planning is not None
    bad = lattice_frame.model_dump()
    bad["lattice_planning"]["sample_sequence"] += 1
    with pytest.raises(ValueError, match="sample"):
        RuntimeInspectorFrame.model_validate_json(json.dumps(bad))


def test_workspace_retains_items_camera_theme_and_explanations(
    lattice_frame: RuntimeInspectorFrame,
) -> None:
    app = QApplication.instance() or QApplication([])
    workspace = LatticePlanningWorkspace()
    manager = theme_manager()
    original = manager.theme.id
    try:
        workspace.set_frame(lattice_frame)
        assert workspace.moves.rowCount() == 19
        assert workspace.candidates.rowCount() == 6
        assert len(workspace._modules) == 6
        assert len(workspace._goals) == 6
        retained = dict(workspace._modules)
        workspace.plot.setRange(xRange=(-5, 8), yRange=(-5, 8), padding=0)
        bounds = workspace.plot.viewRange()
        for theme in THEMES:
            manager.set_theme(theme, persist=False)
            workspace.set_frame(lattice_frame)
            assert workspace._modules == retained
            assert workspace.plot.viewRange() == bounds
        assert workspace.history.plot.getViewBox().state["mouseEnabled"] == [True, False]
        assert "Swept cells must be empty" in workspace.legend.toggle.toolTip()
        workspace.goals.setChecked(False)
        assert all(not item.isVisible() for item in workspace._goals.values())
        workspace.sweep.setChecked(False)
        assert all(not item.isVisible() for item in workspace._clearance)
        app.processEvents()
    finally:
        manager.set_theme(original, persist=False)
        workspace.close()


def test_runtime_target_uses_live_lattice_renderer_and_terminal_banner(
    lattice_frame: RuntimeInspectorFrame,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(RuntimeInspectorWindow, "start", lambda _self: None)
    window = RuntimeInspectorWindow(RuntimeInspectorConfig(pack_path=Path("unused")))
    target = target_lattice_view(lattice_frame)
    assert target is not None and len(target.nodes) == 6
    assert not target.edges and not lattice_frame.view.edges
    frame = lattice_frame.model_copy(
        update={
            "scenario": ReconfigurationStatus(
                phase=ReconfigurationPhase.COMPLETE,
                time_s=0.0,
                plan_id="test",
                plan_name="Test",
                action_index=19,
                action_count=19,
                detail="Target reached",
            )
        }
    )
    try:
        window._receive_frame(frame)
        window._flush_pending_frames()
        window.show()
        app.processEvents()
        assert window.planning_stack.currentWidget() is window.lattice_planning
        assert window.target_view_stack.currentWidget() is window.target_lattice
        assert len(window.target_lattice.displayed_node_ids) == 6
        assert window.workspace_tabs.tabText(2) == "Event log"
        window.workspace_tabs.setCurrentIndex(2)
        assert window.lattice_planning.decisions.isVisible()
        assert "Target reached" in window.run_state.text()
        window._runtime_finished()
        assert "Target reached" in window.run_state.text()
    finally:
        window.close()
