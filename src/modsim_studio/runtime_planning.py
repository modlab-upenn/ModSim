# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false, reportMissingTypeStubs=false
"""Planar workspace, intent overlays, and execution history from immutable frames."""

from __future__ import annotations

import math
from typing import Any, cast

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QPen
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from modsim.planning.models import PlanningSnapshot, Pose2
from modsim.runtime.inspection import RuntimeInspectorFrame
from modsim_studio.appearance import theme_manager


class PlanningWorkspace(QWidget):
    """Read-only planner observability. Pause/stop remain authoritative runtime controls."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._appearance = theme_manager()
        self._frame: RuntimeInspectorFrame | None = None
        self._trails: dict[str, list[tuple[float, float]]] = {}
        self._last_sample: int | None = None
        self._selected: str | None = None
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.paths = QCheckBox("Paths and clearance")
        self.paths.setChecked(True)
        self.goals = QCheckBox("Goal poses")
        self.goals.setChecked(True)
        controls = QHBoxLayout()
        controls.addWidget(self.summary, 1)
        controls.addWidget(self.paths)
        controls.addWidget(self.goals)
        self.workspace: Any = pg.PlotWidget()
        self.workspace.setAspectLocked(True)
        self.workspace.setLabel("bottom", "World X", units="m")
        self.workspace.setLabel("left", "World Y", units="m")
        self.workspace.showGrid(x=True, y=True, alpha=0.15)
        self.topology: Any = pg.PlotWidget()
        self.topology.setAspectLocked(True)
        self.topology.hideAxis("left")
        self.topology.hideAxis("bottom")
        self.topology.setTitle("Target topology · solid edges are committed")
        self.topology.getViewBox().setDefaultPadding(0.2)
        self.action_table = QTableWidget(0, 4)
        self.action_table.setHorizontalHeaderLabels(["Module → parent", "Stage", "State", "Reason"])
        header = self.action_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        for column, width in enumerate((185, 60, 95)):
            header.resizeSection(column, width)
        header.setStretchLastSection(True)
        self.action_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.action_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.action_table.itemSelectionChanged.connect(self._select_action)
        self.detail = QLabel("Select an action to inspect its connector errors and dependencies.")
        self.detail.setWordWrap(True)
        self.timeline: Any = pg.PlotWidget()
        self.timeline.setLabel("bottom", "Simulation time", units="s")
        self.timeline.setMaximumHeight(180)
        self.timeline.getAxis("left").setWidth(95)
        self.decisions = QTableWidget(0, 3)
        self.decisions.setHorizontalHeaderLabels(["Time (s)", "Decision", "Explanation"])
        self.decisions.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.decisions.horizontalHeader().setStretchLastSection(True)
        self.decisions.setMaximumHeight(140)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(self.topology, 2)
        right_layout.addWidget(self.action_table, 2)
        right_layout.addWidget(self.detail)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.workspace)
        split.addWidget(right)
        split.setSizes([650, 420])
        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(split, 1)
        layout.addWidget(self.timeline)
        layout.addWidget(self.decisions)
        self.paths.toggled.connect(self._redraw)
        self.goals.toggled.connect(self._redraw)
        self._appearance.changed.connect(self._redraw)

    def set_frame(self, frame: RuntimeInspectorFrame) -> None:
        if frame.planning is None:
            return
        self._frame = frame
        sample = frame.view.source.sample_sequence
        if self._last_sample != sample:
            for node in frame.view.nodes:
                trail = self._trails.setdefault(node.id, [])
                position = (node.world_position_m[0], node.world_position_m[1])
                if not trail or math.dist(trail[-1], position) > 0.003:
                    trail.append(position)
                    del trail[:-1500]
            self._last_sample = sample
        self._redraw()

    def _redraw(self) -> None:
        frame = self._frame
        if frame is None or frame.planning is None:
            return
        snapshot = frame.planning
        theme = self._appearance.theme
        for plot in (self.workspace, self.topology, self.timeline):
            plot.setBackground(theme.viewport)
            for name in ("left", "bottom"):
                plot.getAxis(name).setTextPen(theme.muted)
                plot.getAxis(name).setPen(theme.border)
            plot.clear()
        self.summary.setText(
            f"Plan {snapshot.plan_revision} · Root {snapshot.plan.root_module} · "
            f"{snapshot.replans} replans · Peak planning {snapshot.planning_ms:.1f} ms · "
            f"Travel {snapshot.path_length_m:.2f} m"
        )
        colors = {
            "complete": theme.success,
            "failed": theme.danger,
            "waiting": theme.warning,
            "pending": theme.muted,
            "navigating": theme.accent,
            "aligning": theme.warning,
            "approaching": theme.accent,
            "holding": theme.success,
            "retreating": theme.warning,
        }
        assignments = {a.goal_node: a for a in snapshot.plan.assignments}
        committed = {
            tuple(sorted((edge.connector_a, edge.connector_b))) for edge in frame.view.edges
        }
        for edge in snapshot.plan.goal.edges:
            a, b = assignments[edge.a], assignments[edge.b]
            key = tuple(sorted((f"{a.module_id}/{edge.face_a}", f"{b.module_id}/{edge.face_b}")))
            pen = cast(
                QPen,
                pg.mkPen(
                    theme.success if key in committed else theme.muted,
                    width=2,
                    style=Qt.PenStyle.SolidLine if key in committed else Qt.PenStyle.DashLine,
                ),
            )
            self.topology.plot([a.target.x, b.target.x], [a.target.y, b.target.y], pen=pen)
        for assignment in snapshot.plan.assignments:
            pose = assignment.target
            self._label(self.topology, pose.x, pose.y, assignment.module_id, theme.text)
            if self.goals.isChecked():
                self._rectangle(pose, snapshot, theme.muted, dashed=True)
        module_poses: dict[str, Pose2] = {}
        for node in frame.view.nodes:
            w, x, y, z = node.world_orientation_wxyz
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            pose = Pose2(x=node.world_position_m[0], y=node.world_position_m[1], yaw=yaw)
            module_poses[node.id] = pose
            self._rectangle(pose, snapshot, theme.text)
            self._label(self.workspace, pose.x, pose.y, node.id, theme.text)
            self.workspace.plot(
                [pose.x, pose.x + 0.065 * math.cos(yaw)],
                [pose.y, pose.y + 0.065 * math.sin(yaw)],
                pen=pg.mkPen(theme.accent, width=2),
            )
            trail = self._trails.get(node.id, [])
            if len(trail) > 1:
                self.workspace.plot(
                    [p[0] for p in trail], [p[1] for p in trail], pen=pg.mkPen(theme.muted)
                )
        self.action_table.blockSignals(True)
        self.action_table.setRowCount(len(snapshot.actions))
        ticks: list[tuple[int, str]] = []
        for row, observation in enumerate(snapshot.actions):
            action = observation.action
            color = colors[observation.phase]
            for column, text in enumerate(
                (
                    f"{action.moving} → {action.parent}",
                    str(action.batch),
                    observation.phase,
                    observation.reason,
                )
            ):
                item = QTableWidgetItem(text)
                item.setToolTip(text)
                self.action_table.setItem(row, column, item)
            if self._selected == action.id:
                self.action_table.selectRow(row)
            if self.paths.isChecked() and observation.path:
                path = observation.path[observation.waypoint_index :]
                self.workspace.plot(
                    [p.pose.x for p in path],
                    [p.pose.y for p in path],
                    pen=pg.mkPen(color, width=2),
                    symbol="o",
                    symbolSize=4,
                )
                root = module_poses.get(action.moving)
                relatives = (
                    tuple(
                        module_poses[m].relative_to(root)
                        for m in observation.members
                        if m in module_poses
                    )
                    if root is not None
                    else ()
                )
                for point in path:
                    for relative in relatives or (Pose2(x=0.0, y=0.0),):
                        self._rectangle(point.pose.compose(relative), snapshot, color, dashed=True)
            ticks.append((row, action.moving))
            for interval in observation.intervals:
                end = snapshot.time_s if interval.end_s is None else interval.end_s
                bar = pg.BarGraphItem(
                    x0=interval.start_s,
                    width=max(0.01, end - interval.start_s),
                    y0=row - 0.3,
                    height=0.6,
                    brush=colors[interval.phase],
                    pen=None,
                )
                self.timeline.addItem(bar)
        self.action_table.blockSignals(False)
        self.timeline.getAxis("left").setTicks([ticks])
        self.timeline.setYRange(-1, len(snapshot.actions))
        decisions = snapshot.decisions[-100:]
        self.decisions.setRowCount(len(decisions))
        for row, decision in enumerate(decisions):
            for column, text in enumerate(
                (f"{decision.time_s:.2f}", decision.kind, decision.detail)
            ):
                self.decisions.setItem(row, column, QTableWidgetItem(text))
        self.decisions.scrollToBottom()
        self._update_detail(snapshot)

    def _rectangle(
        self, pose: Pose2, snapshot: PlanningSnapshot, color: str, *, dashed: bool = False
    ) -> None:
        hx, hy = snapshot.footprint_half_m
        cx = snapshot.footprint_center_x_m
        corners = [
            pose.compose(Pose2(x=cx + x, y=y))
            for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy))
        ]
        self.workspace.plot(
            [p.x for p in corners],
            [p.y for p in corners],
            pen=pg.mkPen(
                color, width=1.2, style=Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine
            ),
        )

    @staticmethod
    def _label(plot: Any, x: float, y: float, label: str, color: str) -> None:
        item = pg.TextItem(label, color=color, anchor=(0.5, 1.0))
        item.setPos(x, y)
        plot.addItem(item)

    def _select_action(self) -> None:
        frame = self._frame
        row = self.action_table.currentRow()
        if frame is None or frame.planning is None or row < 0:
            return
        self._selected = frame.planning.actions[row].action.id
        self._update_detail(frame.planning)

    def _update_detail(self, snapshot: PlanningSnapshot) -> None:
        for observation in snapshot.actions:
            if observation.action.id != self._selected:
                continue
            action = observation.action
            position = (
                "—"
                if observation.position_error_m is None
                else f"{observation.position_error_m * 1000:.1f} mm"
            )
            angle = (
                "—"
                if observation.orientation_error_rad is None
                else f"{math.degrees(observation.orientation_error_rad):.1f}°"
            )
            velocity = (
                "—"
                if observation.relative_velocity_m_s is None
                else f"{observation.relative_velocity_m_s:.3f} m/s"
            )
            self.detail.setText(
                f"{action.moving}/{action.moving_face} → {action.parent}/{action.parent_face}\n"
                f"Position: {position} · Axis: {angle} · Relative speed: {velocity}\n"
                f"Dependencies: {', '.join(action.dependencies) or 'none'}\n{observation.reason}"
            )
