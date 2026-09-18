"""Native planner UI consumes immutable intent and preserves display controls."""

from __future__ import annotations

import os
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
from modsim.runtime.inspection import build_runtime_inspector_frame
from modsim.runtime.inspector_runner import resolve_runtime_recipe
from modsim_studio.appearance import THEMES, theme_manager
from modsim_studio.runtime_planning import PlanningWorkspace


def test_planner_workspace_displays_goal_actions_timeline_and_theme() -> None:
    existing = QApplication.instance()
    app = existing if isinstance(existing, QApplication) else QApplication([])
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
    widget = PlanningWorkspace()
    manager = theme_manager()
    original = manager.theme.id
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
        frame = build_runtime_inspector_frame(
            session,
            resolve_runtime_recipe(loaded.pack, None),
            ModelViewFactory(),
            planning=snapshot,
        )
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
        widget.paths.setChecked(False)
        widget.goals.setChecked(False)
        app.processEvents()
    finally:
        manager.set_theme(original, persist=False)
        widget.close()
        session.shutdown()
