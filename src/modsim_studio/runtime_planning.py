# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false, reportMissingTypeStubs=false
"""Spatial workspace and shared categorical history from immutable planner frames."""

from __future__ import annotations

import math
from typing import Any

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QPen, QShowEvent
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
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

from modsim.planning.inspection import SpatialPlanningSnapshot
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
        self._snapshot: SpatialPlanningSnapshot | None = None
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
        self.plot.getAxis("left").setWidth(145)
        self.plot.getAxis("left").setStyle(textFillLimits=[(0, 1.0)])
        self.plot.getViewBox().invertY(True)
        self.plot.setToolTip(
            "One row per docking action. Wheel zooms time only; drag pans time. "
            "Hover a segment for its state and duration."
        )
        self.plot.setMinimumHeight(120)
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

    def set_snapshot(self, snapshot: SpatialPlanningSnapshot) -> None:
        self._snapshot = snapshot
        theme = theme_manager().theme
        colors = phase_colors()
        colors.update(
            moving=colors["navigating"],
            transfer=colors["holding"],
            retreat=colors["retreating"],
            verify=colors["aligning"],
        )
        self.plot.setBackground(theme.viewport)
        for axis in ("left", "bottom"):
            self.plot.getAxis(axis).setTextPen(theme.muted)
            self.plot.getAxis(axis).setPen(theme.border)
        self.plot.setLabel("bottom", "Simulation time", units="s", color=theme.muted)
        self.legend.set_entries(
            tuple(
                ("■", colors[p], p.capitalize())
                for p in ("moving", "transfer", "retreat", "verify")
            ),
            "Each row is one stage of the handoff. Segment width is elapsed simulation time, "
            "not distance. Wheel/drag changes time only; scroll the row list vertically. "
            "Hover for exact state and duration.",
        )
        ticks: list[tuple[int, str]] = []
        live_keys: set[tuple[str, int]] = set()
        for row, observation in enumerate(snapshot.actions):
            ticks.append((row, observation.label))
            for index, interval in enumerate(observation.intervals):
                key = (observation.id, index)
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
                    f"{observation.label} → {observation.detail}\n"
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
    """Spatial planner intent in the planar demo's workspace/table/history layout."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._appearance = theme_manager()
        self._frame: RuntimeInspectorFrame | None = None
        self._last_sample: int | None = None
        self._trails: dict[str, list[tuple[float, float, float]]] = {}
        self._fit_pending = True
        self._has_been_shown = False
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.projection = QComboBox()
        self.projection.addItems(["3D overview", "Front · X/Z", "Side · Y/Z", "Top · X/Y"])
        self.projection.setToolTip(
            "Project the same spatial plan; front and side views show height."
        )
        self.paths = QCheckBox("Planned paths")
        self.paths.setChecked(True)
        self.goals = QCheckBox("Goal poses")
        self.goals.setChecked(True)
        self.fit_button = QPushButton("Fit scene")
        controls = QHBoxLayout()
        controls.addWidget(self.summary, 1)
        controls.addWidget(self.projection)
        controls.addWidget(self.paths)
        controls.addWidget(self.goals)
        controls.addWidget(self.fit_button)
        self.workspace: Any = pg.PlotWidget()
        self.workspace.setMinimumHeight(240)
        self.workspace.setAspectLocked(True)
        self.workspace.setMenuEnabled(False)
        self.workspace.enableAutoRange(x=False, y=False)
        self.workspace.showGrid(x=True, y=True, alpha=0.15)
        self.workspace.setToolTip(
            "Module root positions, not collision volumes. Dashed paths are the searched plan; "
            "dotted trails are measured motion. Drag to pan; wheel to zoom."
        )
        self.action_table = QTableWidget(0, 4)
        self.action_table.setMinimumHeight(165)
        self.action_table.verticalHeader().hide()
        self.action_table.setHorizontalHeaderLabels(["Action", "Progress", "State", "Reason"])
        self.action_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.action_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.action_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Interactive
        )
        for column, width in enumerate((155, 75, 85)):
            self.action_table.horizontalHeader().resizeSection(column, width)
        self.action_table.horizontalHeader().setStretchLastSection(True)
        self.action_table.itemSelectionChanged.connect(self._update_detail)
        self.detail = QLabel("Select an action to inspect its progress and conditions.")
        self.detail.setWordWrap(True)
        self.diagnostics = QLabel()
        self.diagnostics.setWordWrap(True)
        self.diagnostics.setObjectName("Muted")
        self.history = ActionHistory()
        self.history.setMaximumHeight(245)
        self.timeline = self.history.plot
        self.legend = LegendWidget("Workspace legend")
        self.decisions = QTableWidget(0, 3, self)
        self.decisions.setHorizontalHeaderLabels(["Time (s)", "Decision", "Explanation"])
        self.decisions.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.decisions.horizontalHeader().setStretchLastSection(True)
        self.decisions.setColumnWidth(1, 165)
        self._decision_count = 0
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(self.action_table, 2)
        right_layout.addWidget(self.detail)
        right_layout.addWidget(self.diagnostics)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.workspace)
        split.addWidget(right)
        split.setSizes([650, 420])
        vertical = QSplitter(Qt.Orientation.Vertical)
        vertical.addWidget(split)
        vertical.addWidget(self.history)
        vertical.setSizes([450, 220])
        vertical.setStretchFactor(0, 3)
        vertical.setStretchFactor(1, 1)
        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.legend)
        layout.addWidget(vertical, 1)
        self.paths.toggled.connect(self._redraw)
        self.goals.toggled.connect(self._redraw)
        self.projection.currentIndexChanged.connect(self.fit_scene)
        self.fit_button.clicked.connect(self.fit_scene)
        self._appearance.changed.connect(self._redraw)

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        if not self._has_been_shown:
            self._has_been_shown = True
            QTimer.singleShot(0, self.fit_scene)

    def set_frame(self, frame: RuntimeInspectorFrame) -> None:
        if frame.planning is None:
            return
        self._frame = frame
        if self._last_sample != frame.view.source.sample_sequence:
            for node in frame.view.nodes:
                point = node.world_position_m
                trail = self._trails.setdefault(node.id, [])
                if not trail or math.dist(trail[-1], point) > 0.001:
                    trail.append(point)
                    del trail[:-1500]
            self._last_sample = frame.view.source.sample_sequence
        self._redraw()

    def project(self, point: tuple[float, float, float]) -> tuple[float, float]:
        x, y, z = point
        index = self.projection.currentIndex()
        if index == 1:
            return x, z
        if index == 2:
            return y, z
        if index == 3:
            return x, y
        return (x - y) / math.sqrt(2), z - (x + y) / math.sqrt(6)

    def fit_scene(self) -> None:
        self._fit_pending = True
        self._redraw()

    def _redraw(self) -> None:
        frame = self._frame
        if frame is None or frame.planning is None:
            return
        snapshot = frame.planning
        theme = self._appearance.theme
        colors = visual_colors(theme.id)
        module_colors = {
            "helper": colors["orange"],
            "payload": colors["blue"],
            "receiver": colors["violet"],
            "arm": colors["green"],
            "foot": colors["gold"],
            "upper": colors["gold"],
        }
        plot = self.workspace
        plot.setBackground(theme.viewport)
        plot.clear()
        labels = (
            ("Projected horizontal", "Projected height"),
            ("World X", "World Z"),
            ("World Y", "World Z"),
            ("World X", "World Y"),
        )[self.projection.currentIndex()]
        for axis, label in zip(("bottom", "left"), labels, strict=True):
            plot.setLabel(axis, label, units="m", color=theme.muted)
            plot.getAxis(axis).setPen(theme.border)
            plot.getAxis(axis).setTextPen(theme.muted)
        positions = {n.id: n.world_position_m for n in frame.view.nodes}
        targets = {n.id: n.position_m for n in snapshot.targets}
        fit_points = [self.project(p) for p in (*positions.values(), *targets.values())]
        if self.paths.isChecked():
            for trace in snapshot.traces:
                if all(math.dist(trace.positions_m[0], p) < 1e-6 for p in trace.positions_m):
                    continue
                points = [self.project(p) for p in trace.positions_m]
                fit_points.extend(points)
                color = colors["cyan"] if trace.stage == "moving" else colors["magenta"]
                curve = plot.plot(
                    [p[0] for p in points],
                    [p[1] for p in points],
                    pen=pg.mkPen(color, width=2, style=Qt.PenStyle.DashLine),
                )
                curve.setToolTip(f"{trace.module_id}: searched {trace.stage} path")
                if trace.module_id == "payload":
                    action = next(a for a in snapshot.actions if a.id == trace.stage)
                    if action.phase == trace.stage:
                        point = points[min(action.waypoint, len(points) - 1)]
                        plot.plot(
                            [point[0]],
                            [point[1]],
                            pen=None,
                            symbol="star",
                            symbolSize=18,
                            symbolBrush=color,
                            symbolPen=theme.text,
                        )
            for module, trail in self._trails.items():
                points = [self.project(p) for p in trail]
                plot.plot(
                    [p[0] for p in points],
                    [p[1] for p in points],
                    pen=pg.mkPen(
                        module_colors.get(module, theme.text), width=1, style=Qt.PenStyle.DotLine
                    ),
                )
        if self.goals.isChecked():
            for bond in snapshot.target_bonds:
                self._line(
                    targets[bond.a.split("/")[0]],
                    targets[bond.b.split("/")[0]],
                    colors["violet"],
                    Qt.PenStyle.DashLine,
                )
            for module, position in targets.items():
                x, y = self.project(position)
                plot.plot(
                    [x],
                    [y],
                    pen=None,
                    symbol="s",
                    symbolSize=23,
                    symbolBrush=None,
                    symbolPen=pg.mkPen(module_colors.get(module, theme.text), width=2),
                )
        for edge in frame.view.edges:
            self._line(positions[edge.source], positions[edge.target], colors["neutral"])
        for node in snapshot.targets:
            if node.anchored:
                x, y = self.project(positions[node.id])
                plot.plot(
                    [x],
                    [y],
                    pen=None,
                    symbol="t",
                    symbolSize=32,
                    symbolBrush=None,
                    symbolPen=pg.mkPen(colors["gold"], width=2),
                )
        for module, position in positions.items():
            x, y = self.project(position)
            plot.plot(
                [x],
                [y],
                pen=None,
                symbol="o",
                symbolSize=15,
                symbolBrush=module_colors.get(module, theme.text),
                symbolPen=theme.text,
            )
            label = pg.TextItem(text=module, color=theme.text, anchor=(0.5, -0.65))
            label.setPos(x, y)
            plot.addItem(label)
        self.legend.set_entries(
            (
                *(("●", c, m.capitalize()) for m, c in module_colors.items()),
                ("┄", colors["cyan"], "Approach plan"),
                ("┄", colors["magenta"], "Withdrawal plan"),
                ("□", theme.text, "Goal roots (module colors)"),
                ("┄", colors["violet"], "Target bonds"),
                ("━", colors["neutral"], "Committed bonds"),
                ("△", colors["gold"], "Fixed supports"),
                ("★", theme.text, "Active waypoint"),
                ("···", colors["neutral"], "Measured trail"),
            ),
            "Positions are module roots. Solid links are committed connections. "
            "The 3D overview is an isometric projection; front/side views show world height. "
            "Search checks collision proxies with fixed supports; load capacity is not certified.",
        )
        if self._fit_pending and fit_points:
            plot.setXRange(
                min(p[0] for p in fit_points) - 0.04,
                max(p[0] for p in fit_points) + 0.04,
                padding=0,
            )
            plot.setYRange(
                min(p[1] for p in fit_points) - 0.04,
                max(p[1] for p in fit_points) + 0.04,
                padding=0,
            )
            self._fit_pending = False
        self.summary.setText(
            f"Supported 3D handoff · {snapshot.phase.capitalize()}\n"
            f"Target error {snapshot.target_error_m * 1000:.2f} mm · two fixed supports"
        )
        self.diagnostics.setText(
            f"BFS: capture → release\n"
            f"A* approach: {snapshot.approach_expanded} expanded / "
            f"{snapshot.approach_rejected} rejected\n"
            f"A* withdrawal: {snapshot.withdrawal_expanded} expanded / "
            f"{snapshot.withdrawal_rejected} rejected\n"
            f"Joint error: {snapshot.joint_error_rad:.3f} rad · "
            f"peak effort: {snapshot.peak_effort_nm:.2f} Nm\n"
            f"Peak contact penetration: {snapshot.peak_penetration_m * 1000:.2f} mm"
        )
        previous = self.action_table.blockSignals(True)
        self.action_table.setRowCount(len(snapshot.actions))
        for row, action in enumerate(snapshot.actions):
            for column, value in enumerate(
                (
                    action.label,
                    f"{action.waypoint}/{action.waypoint_count}",
                    action.phase.capitalize(),
                    action.detail,
                )
            ):
                item = self.action_table.item(row, column)
                if item is None:
                    item = QTableWidgetItem()
                    self.action_table.setItem(row, column, item)
                item.setText(value)
                item.setToolTip(value)
        self.action_table.blockSignals(previous)
        self.history.set_snapshot(snapshot)
        self.decisions.setRowCount(len(snapshot.decisions))
        for row in range(self._decision_count, len(snapshot.decisions)):
            decision = snapshot.decisions[row]
            for column, value in enumerate(
                (f"{decision.time_s:.3f}", decision.kind, decision.detail)
            ):
                item = QTableWidgetItem(value)
                item.setToolTip(value)
                self.decisions.setItem(row, column, item)
        self._decision_count = len(snapshot.decisions)
        self._update_detail()

    def _line(
        self,
        first: tuple[float, float, float],
        second: tuple[float, float, float],
        color: str,
        style: Qt.PenStyle = Qt.PenStyle.SolidLine,
    ) -> None:
        a, b = self.project(first), self.project(second)
        self.workspace.plot([a[0], b[0]], [a[1], b[1]], pen=pg.mkPen(color, width=2, style=style))

    def _update_detail(self) -> None:
        if self._frame is None or self._frame.planning is None:
            return
        row = self.action_table.currentRow()
        actions = self._frame.planning.actions
        if 0 <= row < len(actions):
            action = actions[row]
            self.detail.setText(
                f"{action.label} · {action.phase}\n{action.detail}\n"
                f"Progress: {action.waypoint}/{action.waypoint_count}"
            )
