# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false, reportMissingTypeStubs=false
"""Planar workspace, intent overlays, and execution history from immutable frames."""

from __future__ import annotations

import math
from typing import Any

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPen
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QGraphicsRectItem,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QScrollBar,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from modsim.planning.models import PlanningSnapshot, Pose2
from modsim.runtime.inspection import RuntimeInspectorFrame
from modsim_studio.appearance import theme_manager
from modsim_studio.chrome import LegendWidget
from modsim_studio.visual_colors import action_colors, visual_colors


def phase_colors() -> dict[str, str]:
    return action_colors(theme_manager().theme.id)


class ActionHistory(QWidget):
    """Categorical action rows with time-only navigation and stable hoverable intervals."""

    def __init__(self) -> None:
        super().__init__()
        self._snapshot: PlanningSnapshot | None = None
        self._bars: dict[tuple[str, int], QGraphicsRectItem] = {}
        self._row_count = 0
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("Action history · time spent in each state"), 1)
        self.follow = QCheckBox("Follow time")
        self.follow.setChecked(True)
        self.follow.setToolTip(
            "Keep the whole run visible. Manual time navigation switches this off."
        )
        self.fit_button = QPushButton("Fit all")
        controls.addWidget(self.follow)
        controls.addWidget(self.fit_button)
        layout.addLayout(controls)
        self.legend = LegendWidget("History legend and controls")
        layout.addWidget(self.legend)
        self.plot: Any = pg.PlotWidget()
        self.plot.setAspectLocked(False)
        self.plot.setMouseEnabled(x=True, y=False)
        self.plot.setMenuEnabled(False)
        self.plot.enableAutoRange(x=False, y=False)
        self.plot.setLabel("bottom", "Simulation time", units="s")
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot.getAxis("left").setWidth(105)
        self.plot.getViewBox().invertY(True)
        self.plot.setToolTip(
            "One row per docking action. Wheel zooms time only; drag pans time. "
            "Hover a segment for its state and duration."
        )
        self.plot.setMinimumHeight(170)
        self.row_scroll = QScrollBar(Qt.Orientation.Vertical)
        self.row_scroll.setToolTip("Scroll action rows; the time axis stays visible")
        chart = QHBoxLayout()
        chart.addWidget(self.plot, 1)
        chart.addWidget(self.row_scroll)
        layout.addLayout(chart, 1)
        self.row_scroll.valueChanged.connect(self._fit_rows)
        self.plot.getViewBox().sigResized.connect(self._fit_rows)
        self.plot.getViewBox().sigRangeChangedManually.connect(self._manual_range)
        self.follow.toggled.connect(self._follow_changed)
        self.fit_button.clicked.connect(self.fit_all)

    def _manual_range(self, _axes: object) -> None:
        self.follow.setChecked(False)

    def _follow_changed(self, checked: bool) -> None:
        if checked:
            self.fit_all()

    def fit_all(self) -> None:
        end = max(1.0, self._snapshot.time_s if self._snapshot is not None else 0.0)
        self.plot.setXRange(0.0, end, padding=0.02)

    def _fit_rows(self) -> None:
        visible = min(max(1, int(self.plot.getViewBox().height() / 26)), max(1, self._row_count))
        maximum = max(0, self._row_count - visible)
        self.row_scroll.setRange(0, maximum)
        self.row_scroll.setPageStep(visible)
        self.row_scroll.setVisible(maximum > 0)
        first = self.row_scroll.value()
        self.plot.setYRange(first - 0.5, first + visible - 0.5, padding=0)

    def set_snapshot(self, snapshot: PlanningSnapshot) -> None:
        self._snapshot = snapshot
        theme = theme_manager().theme
        colors = phase_colors()
        self.plot.setBackground(theme.viewport)
        for axis in ("left", "bottom"):
            self.plot.getAxis(axis).setTextPen(theme.muted)
            self.plot.getAxis(axis).setPen(theme.border)
        self.plot.setLabel("bottom", "Simulation time", units="s", color=theme.muted)
        self.legend.set_entries(
            tuple(("■", colors[p], p.capitalize()) for p in colors),
            "Each row is one module's docking action. Segment width is elapsed simulation time, "
            "not distance. Wheel/drag changes time only; scroll the row list vertically. "
            "Hover for exact state and duration.",
        )
        ticks: list[tuple[int, str]] = []
        live_keys: set[tuple[str, int]] = set()
        for row, observation in enumerate(snapshot.actions):
            ticks.append((row, observation.action.moving))
            for index, interval in enumerate(observation.intervals):
                key = (observation.action.id, index)
                live_keys.add(key)
                bar = self._bars.get(key)
                if bar is None:
                    bar = QGraphicsRectItem()
                    self._bars[key] = bar
                    self.plot.addItem(bar)
                end = snapshot.time_s if interval.end_s is None else interval.end_s
                bar.setRect(interval.start_s, row - 0.32, max(0.0, end - interval.start_s), 0.64)
                bar.setBrush(QColor(colors[interval.phase]))
                bar.setPen(QPen(Qt.PenStyle.NoPen))
                bar.setToolTip(
                    f"{observation.action.moving} → {observation.action.parent}\n"
                    f"{interval.phase.capitalize()}: {interval.start_s:.2f}-{end:.2f} s\n"
                    f"Duration: {end - interval.start_s:.2f} s"
                )
        for key in self._bars.keys() - live_keys:
            self.plot.removeItem(self._bars.pop(key))
        self.plot.getAxis("left").setTicks([ticks])
        if self._row_count != len(ticks):
            self._row_count = len(ticks)
            self._fit_rows()
        if self.follow.isChecked():
            self.fit_all()


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
        self.action_table = QTableWidget(0, 4)
        self.action_table.setHorizontalHeaderLabels(["Module → parent", "Stage", "State", "Reason"])
        stage_header = self.action_table.horizontalHeaderItem(1)
        assert stage_header is not None
        stage_header.setToolTip(
            "Assembly order group. Actions in the same group may run in parallel; "
            "earlier groups finish before dependent groups start."
        )
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
        self.history = ActionHistory()
        self.timeline = self.history.plot
        self.legend = LegendWidget("Workspace legend")
        self.decisions = QTableWidget(0, 3, self)
        self.decisions.setHorizontalHeaderLabels(["Time (s)", "Decision", "Explanation"])
        self.decisions.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.decisions.horizontalHeader().setStretchLastSection(True)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(self.action_table, 2)
        right_layout.addWidget(self.detail)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.workspace)
        split.addWidget(right)
        split.setSizes([650, 420])
        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.legend)
        vertical = QSplitter(Qt.Orientation.Vertical)
        vertical.addWidget(split)
        vertical.addWidget(self.history)
        vertical.setSizes([450, 230])
        layout.addWidget(vertical, 1)
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
        for plot in (self.workspace,):
            plot.setBackground(theme.viewport)
            for name in ("left", "bottom"):
                plot.getAxis(name).setTextPen(theme.muted)
                plot.getAxis(name).setPen(theme.border)
            plot.clear()
        self.workspace.setLabel("bottom", "World X", units="m", color=theme.muted)
        self.workspace.setLabel("left", "World Y", units="m", color=theme.muted)
        self.summary.setText(
            f"Plan {snapshot.plan_revision} · Root {snapshot.plan.root_module} · "
            f"{snapshot.replans} replans · Peak planning {snapshot.planning_ms:.1f} ms · "
            f"Travel {snapshot.path_length_m:.2f} m"
        )
        colors = phase_colors()
        overlay = visual_colors(theme.id)
        self.legend.set_entries(
            (
                ("□", theme.text, "Solid outline: measured module"),
                ("┄", overlay["violet"], "Dashed outline: goal pose"),
                ("━", overlay["cyan"], "Short line: module heading"),
                ("─", overlay["orange"], "Thin line: travelled trail"),
                ("─●─", colors["navigating"], "Route and waypoints: action state color"),
                ("┄", colors["navigating"], "Route rectangles: predicted assembly footprint"),
            ),
            "XY is a physical map in metres. Drag to pan; wheel to zoom. "
            "Paths are predictions, not committed connections.",
        )
        for assignment in snapshot.plan.assignments:
            if self.goals.isChecked():
                self._rectangle(assignment.target, snapshot, overlay["violet"], dashed=True)
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
                pen=pg.mkPen(overlay["cyan"], width=2),
            )
            trail = self._trails.get(node.id, [])
            if len(trail) > 1:
                self.workspace.plot(
                    [p[0] for p in trail], [p[1] for p in trail], pen=pg.mkPen(overlay["orange"])
                )
        self.action_table.blockSignals(True)
        self.action_table.setRowCount(len(snapshot.actions))
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
                if column == 2:
                    item.setForeground(QColor(color))
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
        self.action_table.blockSignals(False)
        self.history.set_snapshot(snapshot)
        decisions = snapshot.decisions[-100:]
        scrollbar = self.decisions.verticalScrollBar()
        follow_tail = scrollbar.value() >= scrollbar.maximum() - 1
        self.decisions.setRowCount(len(decisions))
        for row, decision in enumerate(decisions):
            for column, text in enumerate(
                (f"{decision.time_s:.2f}", decision.kind, decision.detail)
            ):
                self.decisions.setItem(row, column, QTableWidgetItem(text))
        if follow_tail:
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
