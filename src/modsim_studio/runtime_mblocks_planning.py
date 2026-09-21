# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false, reportMissingTypeStubs=false
"""Retained planner graphics for the one-plane M-Blocks demonstration."""

from __future__ import annotations

import math
from typing import Any

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QGraphicsRectItem,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from modsim.core.transforms import (
    Transform,
    quat_conjugate,
    quat_from_rpy,
    quat_multiply,
    quat_rotate,
)
from modsim.model_views import CubicLatticeView
from modsim.model_views.models import CubicLatticePoseResidual
from modsim.planning.mblocks.geometry import connected, pivots
from modsim.planning.mblocks.models import LatticeState
from modsim.runtime.inspection import RuntimeInspectorFrame
from modsim_studio.appearance import theme_manager
from modsim_studio.chrome import LegendWidget
from modsim_studio.runtime_planning import ActionHistory
from modsim_studio.visual_colors import visual_colors


def target_lattice_view(frame: RuntimeInspectorFrame) -> CubicLatticeView | None:
    """Create presentation-only goal cubes; never insert planned world connections."""
    snapshot = frame.lattice_planning
    live = frame.view
    if snapshot is None or not isinstance(live, CubicLatticeView) or not live.nodes:
        return None
    origin = Transform(
        translation=snapshot.origin_world_m,
        rotation=quat_from_rpy((0.0, 0.0, snapshot.frame_yaw_rad)),
    )
    orientation = live.orientation_catalog[0]
    template = live.nodes[0]
    nodes = tuple(
        template.model_copy(
            update={
                "id": f"goal_{x}_{y}",
                "label": f"({x}, {y})",
                "assembly_id": "target_shape",
                "cell": (x, y, 0),
                "world_position_m": origin.apply_point(
                    (x * snapshot.goal.pitch_m, y * snapshot.goal.pitch_m, 0.0)
                ),
                "world_orientation_wxyz": quat_multiply(
                    origin.rotation, orientation.lattice_orientation_wxyz
                ),
                "orientation_index": orientation.index,
                "orientation_id": orientation.id,
                "pose_residual": CubicLatticePoseResidual(
                    translation_m=(0.0, 0.0, 0.0),
                    position_m=0.0,
                    orientation_rad=0.0,
                ),
                "position_within_tolerance": True,
                "orientation_within_tolerance": True,
                "off_lattice": False,
                "occupancy_conflict": False,
            }
        )
        for x, y in snapshot.goal.cells
    )
    return live.model_copy(
        update={
            "id": "mblocks_target_shape",
            "name": "Target shape (planner intent)",
            "nodes": nodes,
            "edges": (),
            "occupancy_conflicts": (),
            "origin_world_m": origin.translation,
            "orientation_world_wxyz": origin.rotation,
        }
    )


class LatticePlanningWorkspace(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._frame: RuntimeInspectorFrame | None = None
        self._modules: dict[str, tuple[QGraphicsRectItem, Any]] = {}
        self._goals: dict[tuple[int, int], QGraphicsRectItem] = {}
        self._clearance: list[QGraphicsRectItem] = []
        self._fit_pending = True
        self._table_key: object = None
        self._decision_sequence = -1
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.detail = QLabel()
        self.detail.setWordWrap(True)
        self.goals = QCheckBox("Target cells")
        self.goals.setChecked(True)
        self.sweep = QCheckBox("Swept cells")
        self.sweep.setChecked(True)
        self.fit_button = QPushButton("Fit")
        controls = QHBoxLayout()
        controls.addWidget(self.summary, 1)
        controls.addWidget(self.goals)
        controls.addWidget(self.sweep)
        controls.addWidget(self.fit_button)
        self.plot: Any = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.enableAutoRange(x=False, y=False)
        self.plot.setLabel("bottom", "Lattice X", units="cells")
        self.plot.setLabel("left", "Lattice Y", units="cells")
        self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.arc: Any = self.plot.plot()
        self.pivot_marker: Any = pg.ScatterPlotItem(symbol="d", size=12)
        self.plot.addItem(self.pivot_marker)
        self.legend = LegendWidget("Planner legend")
        self.moves = QTableWidget(0, 4)
        self.moves.setHorizontalHeaderLabels(["Module", "Destination", "Turn", "Reason"])
        self.moves.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.moves.horizontalHeader().setStretchLastSection(True)
        self.candidates = QTableWidget(0, 2)
        self.candidates.setHorizontalHeaderLabels(["Module", "Eligibility at last settled state"])
        self.candidates.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.candidates.horizontalHeader().setStretchLastSection(True)
        right = QWidget()
        side = QVBoxLayout(right)
        side.addWidget(QLabel("Generated pivots · active move followed by remaining route"))
        side.addWidget(self.moves, 2)
        side.addWidget(self.detail)
        side.addWidget(self.candidates, 1)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.plot)
        split.addWidget(right)
        split.setSizes([650, 440])
        self.history = ActionHistory()
        self.history.plot.setMinimumHeight(100)
        vertical = QSplitter(Qt.Orientation.Vertical)
        vertical.addWidget(split)
        vertical.addWidget(self.history)
        vertical.setSizes([440, 210])
        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.legend)
        layout.addWidget(vertical)
        self.decisions = QTableWidget(0, 3)
        self.decisions.setHorizontalHeaderLabels(["Time (s)", "Decision", "Explanation"])
        self.decisions.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.decisions.horizontalHeader().setStretchLastSection(True)
        self.goals.toggled.connect(self._redraw)
        self.sweep.toggled.connect(self._redraw)
        self.fit_button.clicked.connect(self.fit)
        theme_manager().changed.connect(self._redraw)

    def set_frame(self, frame: RuntimeInspectorFrame) -> None:
        if frame.lattice_planning is None:
            return
        self._frame = frame
        self._redraw()

    def fit(self) -> None:
        if self._frame is None or self._frame.lattice_planning is None:
            return
        snapshot = self._frame.lattice_planning
        cells = [*snapshot.goal.cells, *(b.cell for b in snapshot.blocks)]
        self.plot.setXRange(
            min(c[0] for c in cells) - 1.5, max(c[0] for c in cells) + 1.5, padding=0
        )
        self.plot.setYRange(
            min(c[1] for c in cells) - 1.5, max(c[1] for c in cells) + 1.5, padding=0
        )
        self._fit_pending = False

    def _redraw(self) -> None:
        frame = self._frame
        if frame is None or frame.lattice_planning is None:
            return
        snapshot = frame.lattice_planning
        theme = theme_manager().theme
        colors = visual_colors(theme.id)
        self.plot.setBackground(theme.viewport)
        for name in ("left", "bottom"):
            self.plot.getAxis(name).setPen(theme.border)
            self.plot.getAxis(name).setTextPen(theme.muted)
        self.legend.set_entries(
            (
                ("■", colors["blue"], "Measured block"),
                ("■", colors["orange"], "Moving block"),
                ("■", colors["cyan"], "Supporting block"),
                ("┄", colors["violet"], "Target cell"),
                ("□", colors["gold"], "Swept cells must be empty"),
                ("◆", colors["magenta"], "Hinge axis / pivot point"),
            ),
            "Coordinates follow the stationary anchor. Solid squares use measured poses. "
            "The arc and target cells are planner intent. Attachments use ideal welds/hinges; "
            "no magnetic attraction force is simulated. Hover blocks and move rows for details.",
        )
        active = snapshot.active
        total = snapshot.completed_actions + len(snapshot.remaining) + int(active is not None)
        self.summary.setText(
            f"Sung 2015 · {snapshot.goal.id} · "
            f"{snapshot.completed_actions}/{total} pivots verified\n"
            f"Plan {snapshot.plan_revision} · {snapshot.replans} replans · "
            f"Peak planning {snapshot.planning_ms:.1f} ms"
        )
        self.detail.setText(
            f"{snapshot.detail}\nFlywheel {snapshot.flywheel_rpm:.0f} RPM · "
            f"Pivot {snapshot.pivot_angle_deg:.1f}° · "
            f"Landing effort {snapshot.landing_effort_nm:.4f} N m · "
            f"Last settled position error {snapshot.landing_error_m * 1000:.2f} mm"
        )
        origin = Transform(
            translation=snapshot.origin_world_m,
            rotation=quat_from_rpy((0.0, 0.0, snapshot.frame_yaw_rad)),
        )
        for node in frame.view.nodes:
            pair = self._modules.get(node.id)
            if pair is None:
                rect = QGraphicsRectItem(-0.43, -0.43, 0.86, 0.86)
                label: Any = pg.TextItem(anchor=(0.5, 0.5))
                self.plot.addItem(rect)
                self.plot.addItem(label)
                pair = self._modules[node.id] = (rect, label)
            rect, label = pair
            point = origin.inverse().apply_point(node.world_position_m)
            x, y = point[0] / snapshot.goal.pitch_m, point[1] / snapshot.goal.pitch_m
            axis = quat_rotate(
                quat_multiply(quat_conjugate(origin.rotation), node.world_orientation_wxyz),
                (1.0, 0.0, 0.0),
            )
            rect.setPos(x, y)
            rect.setRotation(math.degrees(math.atan2(axis[1], axis[0])))
            color = (
                colors["orange"]
                if active and node.id == active.moving
                else (colors["cyan"] if active and node.id == active.support else colors["blue"])
            )
            fill = QColor(color)
            fill.setAlpha(95)
            rect.setBrush(fill)
            rect.setPen(pg.mkPen(color, width=2))
            rect.setToolTip(f"{node.id}\nMeasured cell position ({x:.3f}, {y:.3f})")
            label.setText(node.id, color=theme.text)
            label.setPos(x, y)
        for cell in snapshot.goal.cells:
            rect = self._goals.get(cell)
            if rect is None:
                rect = QGraphicsRectItem(cell[0] - 0.49, cell[1] - 0.49, 0.98, 0.98)
                self._goals[cell] = rect
                self.plot.addItem(rect)
                rect.setToolTip(f"Target cell {cell}; any module may occupy it")
            rect.setPen(pg.mkPen(colors["violet"], width=2, style=Qt.PenStyle.DashLine))
            rect.setVisible(self.goals.isChecked())
        clearance = active.clearance_cells if active else ()
        while len(self._clearance) < len(clearance):
            rect = QGraphicsRectItem()
            self.plot.addItem(rect)
            self._clearance.append(rect)
        for index, rect in enumerate(self._clearance):
            visible = index < len(clearance) and self.sweep.isChecked()
            rect.setVisible(visible)
            if visible:
                x, y = clearance[index]
                rect.setRect(x - 0.5, y - 0.5, 1, 1)
                rect.setPen(pg.mkPen(colors["gold"], style=Qt.PenStyle.DotLine))
                rect.setToolTip("This cell intersects the moving cube's swept volume")
        if active:
            px, py = active.pivot_twice[0] / 2, active.pivot_twice[1] / 2
            dx, dy = active.source[0] - px, active.source[1] - py
            points = [
                (px + dx * math.cos(a) - dy * math.sin(a), py + dx * math.sin(a) + dy * math.cos(a))
                for a in (active.turns * math.pi / 2 * i / 40 for i in range(41))
            ]
            self.arc.setData(
                [p[0] for p in points],
                [p[1] for p in points],
                pen=pg.mkPen(colors["orange"], width=3),
            )
            self.pivot_marker.setData([px], [py], brush=colors["magenta"], pen=colors["magenta"])
        else:
            self.arc.setData([], [])
            self.pivot_marker.setData([], [])
        key = (snapshot.plan_revision, snapshot.completed_actions, active, snapshot.blocks)
        if key != self._table_key:
            route = (*((active,) if active else ()), *snapshot.remaining)
            self.moves.setRowCount(len(route))
            for row, action in enumerate(route):
                for column, value in enumerate(
                    (action.moving, str(action.destination), f"{action.turns * 90}°", action.reason)
                ):
                    item = QTableWidgetItem(value)
                    item.setToolTip(
                        f"{action.source} → {action.destination}; support {action.support}"
                    )
                    self.moves.setItem(row, column, item)
            state = LatticeState(blocks=snapshot.blocks)
            self.candidates.setRowCount(len(state.blocks))
            for row, block in enumerate(state.blocks):
                if block.id == snapshot.anchor:
                    reason = "Preserved anchor"
                elif not connected(state.cells - {block.cell}):
                    reason = "Moving this block would disconnect the remainder"
                elif not pivots(state, block.id):
                    reason = "No supported pivot with clear swept cells"
                else:
                    reason = "Mobile; selection follows boundary / target construction"
                self.candidates.setItem(row, 0, QTableWidgetItem(block.id))
                self.candidates.setItem(row, 1, QTableWidgetItem(reason))
            self._table_key = key
        if snapshot.decisions and snapshot.decisions[-1].sequence != self._decision_sequence:
            self.decisions.setRowCount(len(snapshot.decisions))
            for row, decision in enumerate(snapshot.decisions):
                for col, value in enumerate(
                    (f"{decision.time_s:.3f}", decision.kind, decision.detail)
                ):
                    self.decisions.setItem(row, col, QTableWidgetItem(value))
            self._decision_sequence = snapshot.decisions[-1].sequence
        self.history.set_snapshot(snapshot)
        if self._fit_pending:
            self.fit()
