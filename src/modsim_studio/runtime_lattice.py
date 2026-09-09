# pyright: reportMissingTypeStubs=false, reportUnknownArgumentType=false
# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false
"""PyQtGraph renderer for immutable cubic-lattice presentations."""

from __future__ import annotations

import math
from functools import partial
from typing import Any

import pyqtgraph as pg
from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPen, QPolygonF
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGraphicsPolygonItem,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from modsim_studio.runtime_lattice_presenter import LatticeProjection, PresentedLatticeModule
from modsim_studio.runtime_presenter import CubicLatticePresentation

_BACKGROUND = "#111820"
_GRID_COLOR = "#34424f"
_GRID_MAJOR_COLOR = "#526574"
_EDGE_COLOR = "#26c6da"
_NODE_OUTLINE = "#0b1117"
_SNAP_COLOR = "#8093a2"
_OFF_LATTICE_COLOR = "#f4b942"
_OFF_LATTICE_OUTLINE = "#ffe0a3"
_CONFLICT_COLOR = "#ef5350"
_CONFLICT_OUTLINE = "#ffcdd2"
_SELECTED_COLOR = "#ffd166"
_AXIS_COLORS = {"x": "#ef5350", "y": "#66bb6a", "z": "#42a5f5"}
_ORBIT_RADIANS_PER_PIXEL = math.radians(0.4)
_ASSEMBLY_PALETTE = (
    "#4fc3f7",
    "#ce93d8",
    "#80cbc4",
    "#ffb74d",
    "#81c784",
    "#ef9a9a",
    "#9fa8da",
    "#f48fb1",
)


class LatticeViewBox(pg.ViewBox):
    """Keep ordinary pan/wheel zoom while assigning right-drag to orbit."""

    orbit_dragged = Signal(float, float)

    def mouseDragEvent(self, event: Any, axis: int | None = None) -> None:
        if event.button() == Qt.MouseButton.RightButton:
            delta = event.pos() - event.lastPos()
            event.accept()
            if not delta.isNull():
                self.orbit_dragged.emit(float(delta.x()), float(delta.y()))
            return
        super().mouseDragEvent(event, axis=axis)


class CubicLatticeWidget(QWidget):
    """Render measured cubes, snap cells, connections, and a finite grid."""

    entity_selected = Signal(str, str)
    projection_changed = Signal(str)
    layer_changed = Signal(object)
    snap_cells_changed = Signal(bool)
    orientation_axes_changed = Signal(bool)
    orbit_requested = Signal(float, float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        controls = QHBoxLayout()
        controls.setContentsMargins(8, 5, 8, 3)
        controls.addWidget(QLabel("Projection"))
        self.projection_combo = QComboBox()
        for label, projection in (
            ("Isometric / orbit", LatticeProjection.ISOMETRIC),
            ("XY", LatticeProjection.XY),
            ("XZ", LatticeProjection.XZ),
            ("YZ", LatticeProjection.YZ),
        ):
            self.projection_combo.addItem(label, projection.value)
        controls.addWidget(self.projection_combo)

        controls.addWidget(QLabel("Z layer"))
        self.layer_combo = QComboBox()
        self.layer_combo.addItem("All", None)
        controls.addWidget(self.layer_combo)

        self.snap_cells_checkbox = QCheckBox("Snap cells")
        self.snap_cells_checkbox.setChecked(True)
        controls.addWidget(self.snap_cells_checkbox)
        self.orientation_axes_checkbox = QCheckBox("Local axes")
        self.orientation_axes_checkbox.setChecked(True)
        controls.addWidget(self.orientation_axes_checkbox)
        controls.addStretch(1)
        self.fit_button = QPushButton("Fit")
        controls.addWidget(self.fit_button)
        root.addLayout(controls)

        self._view_box = LatticeViewBox()
        self.plot = pg.PlotWidget(background=_BACKGROUND, viewBox=self._view_box)
        root.addWidget(self.plot, 1)
        self.legend_label = QLabel(
            "Dashed cube: nearest cell  ·  amber: off lattice  ·  red: occupancy conflict  "
            "·  local axes: X / Y / Z  ·  left-drag: pan  ·  right-drag: orbit  "
            "·  wheel: zoom"
        )
        self.legend_label.setWordWrap(True)
        self.legend_label.setStyleSheet(
            "background: #1a242d; color: #aab8c2; padding: 5px 8px 2px 8px;"
        )
        root.addWidget(self.legend_label)
        self.detail_label = QLabel("No lattice sample received")
        self.detail_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.detail_label.setStyleSheet(
            "background: #1a242d; color: #dce6ef; padding: 2px 8px 6px 8px;"
        )
        root.addWidget(self.detail_label)

        self._plot_item: Any = self.plot.getPlotItem()
        self._plot_item.hideAxis("left")
        self._plot_item.hideAxis("bottom")
        self._plot_item.setAspectLocked(True)
        self._plot_item.setMenuEnabled(False)
        self._plot_item.setMouseEnabled(x=True, y=True)
        self.plot.setToolTip(
            "Measured cubes move continuously. Dashed cubes mark nearest lattice cells. "
            "Click to select; left-drag pans, right-drag rotates, and the wheel zooms."
        )

        self._fingerprint: object | None = None
        self._fit_key: (
            tuple[
                tuple[str, ...],
                LatticeProjection,
                int | None,
                tuple[int, int, int, int, int, int],
            ]
            | None
        ) = None
        self._presentation: CubicLatticePresentation | None = None
        self._node_ids: tuple[str, ...] = ()
        self._edge_ids: tuple[str, ...] = ()
        self._cell_ids: tuple[str, ...] = ()
        self._selection: tuple[str, str] | None = None
        self._grid_items: list[Any] = []
        self._cell_outline_items: dict[str, list[Any]] = {}
        self._cell_badges: dict[str, Any] = {}
        self._cell_item: Any | None = None
        self._tether_items: dict[str, Any] = {}
        self._connection_items: dict[str, Any] = {}
        self._connection_marker_item: Any | None = None
        self._module_face_items: dict[str, list[QGraphicsPolygonItem]] = {}
        self._module_outline_items: dict[str, list[Any]] = {}
        self._module_axis_items: dict[tuple[str, str], Any] = {}
        self._module_label_items: dict[str, Any] = {}
        self._node_item: Any | None = None
        self._lattice_axis_items: dict[str, Any] = {}
        self._lattice_axis_labels: dict[str, Any] = {}

        self.projection_combo.currentIndexChanged.connect(self._projection_selected)
        self.layer_combo.currentIndexChanged.connect(self._layer_selected)
        self.snap_cells_checkbox.toggled.connect(self.snap_cells_changed.emit)
        self.orientation_axes_checkbox.toggled.connect(self.orientation_axes_changed.emit)
        self._view_box.orbit_dragged.connect(self._orbit_dragged)
        self.fit_button.clicked.connect(self.fit_presentation)

    @property
    def displayed_node_ids(self) -> tuple[str, ...]:
        """Return currently drawn module IDs."""
        return self._node_ids

    @property
    def displayed_edge_ids(self) -> tuple[str, ...]:
        """Return currently drawn connection IDs."""
        return self._edge_ids

    @property
    def displayed_cell_ids(self) -> tuple[str, ...]:
        """Return occupied nearest-cell IDs represented by the view."""
        return self._cell_ids

    @property
    def displayed_selection(self) -> tuple[str, str] | None:
        """Return the selected lattice entity currently styled by the widget."""
        return self._selection

    @property
    def displayed_label_count(self) -> int:
        """Return the number of currently drawn module labels."""
        return sum(item.isVisible() for item in self._module_label_items.values())

    def set_presentation(self, presentation: CubicLatticePresentation) -> None:
        """Draw ``presentation`` unless its visible lattice geometry is unchanged."""
        self._presentation = presentation
        geometry = presentation.geometry
        self._sync_controls(presentation)
        self.detail_label.setText(_selection_text(presentation))
        fingerprint = (
            geometry,
            presentation.selection,
            presentation.show_snap_cells,
            presentation.show_orientation_axes,
            presentation.show_labels,
        )
        if fingerprint == self._fingerprint:
            return

        self._fingerprint = fingerprint
        self._node_ids = tuple(node.id for node in geometry.nodes)
        self._edge_ids = tuple(edge.id for edge in geometry.edges)
        self._cell_ids = tuple(cell.id for cell in geometry.cells)
        self._selection = (
            (presentation.selection.kind, presentation.selection.entity_id)
            if presentation.selection is not None
            else None
        )
        self._draw_grid(presentation)
        self._draw_snap_cells(presentation)
        self._draw_connections(presentation)
        self._draw_modules(presentation)
        self._draw_lattice_axes(presentation)

        fit_key = (
            self._node_ids,
            geometry.projection,
            geometry.layer_z,
            geometry.bounds,
        )
        if self._fit_key is None or fit_key[:3] != self._fit_key[:3]:
            self.fit_presentation()
        self._fit_key = fit_key

    def fit_presentation(self) -> None:
        """Fit the retained finite grid and visible modules in the plot."""
        if self._presentation is None:
            return
        geometry = self._presentation.geometry
        points = [node.center for node in geometry.nodes]
        points.extend(line.start for line in geometry.grid_lines)
        points.extend(line.end for line in geometry.grid_lines)
        if not points:
            self._plot_item.setXRange(-1.0, 1.0, padding=0.0)
            self._plot_item.setYRange(-1.0, 1.0, padding=0.0)
            return
        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]
        x_span = max(max(x_values) - min(x_values), 1.0)
        y_span = max(max(y_values) - min(y_values), 1.0)
        padding = max(x_span, y_span) * 0.08 + 0.25
        self._plot_item.setXRange(min(x_values) - padding, max(x_values) + padding, padding=0.0)
        self._plot_item.setYRange(min(y_values) - padding, max(y_values) + padding, padding=0.0)

    def _sync_controls(self, presentation: CubicLatticePresentation) -> None:
        geometry = presentation.geometry
        projection_index = self.projection_combo.findData(geometry.projection.value)
        self.projection_combo.blockSignals(True)
        self.projection_combo.setCurrentIndex(projection_index)
        self.projection_combo.blockSignals(False)

        desired_layers = (None, *geometry.available_layers_z)
        current_layers = tuple(
            self.layer_combo.itemData(index) for index in range(self.layer_combo.count())
        )
        self.layer_combo.blockSignals(True)
        if current_layers != desired_layers:
            self.layer_combo.clear()
            self.layer_combo.addItem("All", None)
            for layer in geometry.available_layers_z:
                self.layer_combo.addItem(str(layer), layer)
        layer_index = self.layer_combo.findData(geometry.layer_z)
        self.layer_combo.setCurrentIndex(max(layer_index, 0))
        self.layer_combo.blockSignals(False)

        self.snap_cells_checkbox.blockSignals(True)
        self.snap_cells_checkbox.setChecked(presentation.show_snap_cells)
        self.snap_cells_checkbox.blockSignals(False)
        self.orientation_axes_checkbox.blockSignals(True)
        self.orientation_axes_checkbox.setChecked(presentation.show_orientation_axes)
        self.orientation_axes_checkbox.blockSignals(False)

    def _draw_grid(self, presentation: CubicLatticePresentation) -> None:
        lines = presentation.geometry.grid_lines
        _resize_curve_items(self._plot_item, self._grid_items, len(lines))
        for item, line in zip(self._grid_items, lines, strict=True):
            item.setData(
                x=[line.start[0], line.end[0]],
                y=[line.start[1], line.end[1]],
            )
            item.setPen(
                pg.mkPen(
                    _GRID_MAJOR_COLOR if line.major else _GRID_COLOR,
                    width=1.4 if line.major else 0.8,
                )
            )
            item.setZValue(-100)

    def _draw_snap_cells(self, presentation: CubicLatticePresentation) -> None:
        geometry = presentation.geometry
        off_lattice_cells = {
            f"cell:{node.cell[0]},{node.cell[1]},{node.cell[2]}"
            for node in geometry.nodes
            if node.off_lattice
        }
        live_cell_ids = {cell.id for cell in geometry.cells}
        for identifier in set(self._cell_outline_items).difference(live_cell_ids):
            _remove_items(self._plot_item, self._cell_outline_items.pop(identifier))
        for identifier in set(self._cell_badges).difference(live_cell_ids):
            self._plot_item.removeItem(self._cell_badges.pop(identifier))

        cell_spots: list[dict[str, object]] = []
        for cell in geometry.cells:
            color = (
                _SELECTED_COLOR
                if cell.selected
                else _CONFLICT_COLOR
                if cell.occupancy_conflict
                else _OFF_LATTICE_COLOR
                if cell.id in off_lattice_cells
                else _SNAP_COLOR
            )
            width = 2.6 if cell.selected or cell.occupancy_conflict else 1.1
            outline_items = self._cell_outline_items.setdefault(cell.id, [])
            _resize_curve_items(
                self._plot_item,
                outline_items,
                len(cell.outline_segments),
            )
            for item, (start, end) in zip(
                outline_items,
                cell.outline_segments,
                strict=True,
            ):
                item.setData(
                    x=[start[0], end[0]],
                    y=[start[1], end[1]],
                )
                item.setPen(pg.mkPen(color, width=width, style=Qt.PenStyle.DashLine))
                item.setZValue(-25)
            if cell.occupancy_conflict or cell.id in off_lattice_cells:
                cell_spots.append(
                    {
                        "pos": cell.center,
                        "data": cell.id,
                        "size": 17 if cell.occupancy_conflict else 13,
                        "symbol": "x" if cell.occupancy_conflict else "s",
                        "brush": pg.mkBrush(QColor(0, 0, 0, 0)),
                        "pen": pg.mkPen(color, width=2.2),
                    }
                )
            if cell.occupancy_conflict:
                badge = self._cell_badges.get(cell.id)
                if badge is None:
                    badge = pg.TextItem(color=_CONFLICT_COLOR, anchor=(-0.3, 1.2))
                    badge.setZValue(80)
                    self._plot_item.addItem(badge)
                    self._cell_badges[cell.id] = badge
                badge.setText(str(len(cell.occupant_ids)))
                badge.setPos(*cell.center)
                badge.setToolTip(
                    f"Occupancy conflict at {cell.cell}: {', '.join(cell.occupant_ids)}"
                )
            elif cell.id in self._cell_badges:
                self._plot_item.removeItem(self._cell_badges.pop(cell.id))
        if self._cell_item is None:
            self._cell_item = pg.ScatterPlotItem(pxMode=True, hoverable=True)
            self._cell_item.setZValue(75)
            self._cell_item.sigClicked.connect(self._cells_clicked)
            self._plot_item.addItem(self._cell_item)
        self._cell_item.setData(spots=cell_spots)

        live_tethers = {node.id for node in geometry.nodes if node.tether is not None}
        for identifier in set(self._tether_items).difference(live_tethers):
            self._plot_item.removeItem(self._tether_items.pop(identifier))
        for node in geometry.nodes:
            if node.tether is None:
                continue
            start, end = node.tether
            tether = self._tether_items.get(node.id)
            if tether is None:
                tether = pg.PlotCurveItem(antialias=True)
                self._plot_item.addItem(tether)
                self._tether_items[node.id] = tether
            tether.setData(x=[start[0], end[0]], y=[start[1], end[1]])
            tether.setPen(pg.mkPen(_OFF_LATTICE_COLOR, width=1.7, style=Qt.PenStyle.DotLine))
            tether.setZValue(-15)

    def _draw_connections(self, presentation: CubicLatticePresentation) -> None:
        live_edge_ids = {edge.id for edge in presentation.geometry.edges}
        for identifier in set(self._connection_items).difference(live_edge_ids):
            self._plot_item.removeItem(self._connection_items.pop(identifier))

        marker_spots: list[dict[str, object]] = []
        for edge in presentation.geometry.edges:
            color = _SELECTED_COLOR if edge.selected else _EDGE_COLOR
            curve = self._connection_items.get(edge.id)
            if curve is None:
                curve = pg.PlotCurveItem(antialias=True, clickable=True)
                curve.setClickable(True, width=12)
                curve.sigClicked.connect(partial(self._edge_clicked, edge.id))
                self._plot_item.addItem(curve)
                self._connection_items[edge.id] = curve
            curve.setData(
                x=[point[0] for point in edge.path],
                y=[point[1] for point in edge.path],
            )
            curve.setPen(pg.mkPen(color, width=4.0 if edge.selected else 2.2))
            curve.setClickable(True, width=12)
            curve.setZValue(45)
            curve.setToolTip(
                f"{edge.id}\n{edge.connector_a} ({edge.source_face}) ↔ "
                f"{edge.connector_b} ({edge.target_face})"
            )
            marker_spots.append(
                {
                    "pos": edge.marker,
                    "data": edge.id,
                    "size": 11 if edge.selected else 8,
                    "symbol": "d",
                    "brush": pg.mkBrush(color),
                    "pen": pg.mkPen(_BACKGROUND, width=1.2),
                }
            )
        if self._connection_marker_item is None:
            self._connection_marker_item = pg.ScatterPlotItem(pxMode=True, hoverable=True)
            self._connection_marker_item.setZValue(65)
            self._connection_marker_item.sigClicked.connect(self._edges_clicked)
            self._plot_item.addItem(self._connection_marker_item)
        self._connection_marker_item.setData(spots=marker_spots)

    def _draw_modules(self, presentation: CubicLatticePresentation) -> None:
        geometry = presentation.geometry
        live_node_ids = {node.id for node in geometry.nodes}
        for identifier in set(self._module_face_items).difference(live_node_ids):
            _remove_items(self._plot_item, self._module_face_items.pop(identifier))
        for identifier in set(self._module_outline_items).difference(live_node_ids):
            _remove_items(self._plot_item, self._module_outline_items.pop(identifier))
        for identifier in set(self._module_label_items).difference(live_node_ids):
            self._plot_item.removeItem(self._module_label_items.pop(identifier))

        for node in geometry.nodes:
            face_items = self._module_face_items.setdefault(node.id, [])
            _resize_polygon_items(self._plot_item, face_items, len(node.faces))

        ordered_faces = sorted(
            ((face.depth, node, face) for node in geometry.nodes for face in node.faces),
            key=lambda item: (item[0], item[1].id),
        )
        face_indices: dict[str, int] = {}
        for index, (_depth, node, face) in enumerate(ordered_faces):
            face_index = face_indices.get(node.id, 0)
            face_indices[node.id] = face_index + 1
            fill_name = (
                _CONFLICT_COLOR
                if node.occupancy_conflict
                else _OFF_LATTICE_COLOR
                if node.off_lattice
                else _assembly_color(node.assembly_id)
            )
            fill = _shaded_color(fill_name, face.brightness, alpha=205)
            outline = (
                _SELECTED_COLOR
                if node.selected
                else _CONFLICT_OUTLINE
                if node.occupancy_conflict
                else _OFF_LATTICE_OUTLINE
                if node.off_lattice
                else _NODE_OUTLINE
            )
            polygon = self._module_face_items[node.id][face_index]
            polygon.setPolygon(QPolygonF([QPointF(point[0], point[1]) for point in face.points]))
            polygon.setBrush(QBrush(fill))
            pen = QPen(QColor(outline))
            pen.setWidthF(1.0)
            pen.setCosmetic(True)
            polygon.setPen(pen)
            polygon.setZValue(10.0 + index * 0.01)
            polygon.setToolTip(_module_tooltip(node))

        for node in geometry.nodes:
            outline = (
                _SELECTED_COLOR
                if node.selected
                else _CONFLICT_OUTLINE
                if node.occupancy_conflict
                else _OFF_LATTICE_OUTLINE
                if node.off_lattice
                else _NODE_OUTLINE
            )
            outline_items = self._module_outline_items.setdefault(node.id, [])
            _resize_curve_items(
                self._plot_item,
                outline_items,
                len(node.outline_segments),
            )
            for edge_item, (start, end) in zip(
                outline_items,
                node.outline_segments,
                strict=True,
            ):
                edge_item.setData(
                    x=[start[0], end[0]],
                    y=[start[1], end[1]],
                )
                edge_item.setPen(pg.mkPen(outline, width=3.0 if node.selected else 1.45))
                edge_item.setZValue(60)

        node_spots = [
            {
                "pos": node.center,
                "data": node.id,
                "size": 34,
                "symbol": "s",
                "brush": pg.mkBrush(QColor(0, 0, 0, 0)),
                "pen": pg.mkPen(QColor(0, 0, 0, 0)),
            }
            for node in geometry.nodes
        ]
        if self._node_item is None:
            self._node_item = pg.ScatterPlotItem(pxMode=True, hoverable=True)
            self._node_item.setZValue(70)
            self._node_item.sigClicked.connect(self._nodes_clicked)
            self._plot_item.addItem(self._node_item)
        self._node_item.setData(spots=node_spots)

        desired_axes = {
            (node.id, axis.axis) for node in geometry.nodes for axis in node.orientation_axes
        }
        for key in set(self._module_axis_items).difference(desired_axes):
            self._plot_item.removeItem(self._module_axis_items.pop(key))
        for node in geometry.nodes:
            for axis in node.orientation_axes:
                key = (node.id, axis.axis)
                item = self._module_axis_items.get(key)
                if item is None:
                    item = pg.PlotCurveItem(antialias=True)
                    self._plot_item.addItem(item)
                    self._module_axis_items[key] = item
                item.setData(
                    x=[axis.start[0], axis.end[0]],
                    y=[axis.start[1], axis.end[1]],
                )
                item.setPen(pg.mkPen(_AXIS_COLORS[axis.axis], width=1.7))
                item.setZValue(72)
            label = self._module_label_items.get(node.id)
            if label is None:
                label = pg.TextItem(
                    color="#eef5fb",
                    anchor=(0.5, -0.18),
                    border=None,
                    fill=pg.mkBrush(17, 24, 32, 150),
                )
                label.setZValue(90)
                self._plot_item.addItem(label)
                self._module_label_items[node.id] = label
            label.setText(f"{node.label}\n{node.cell}")
            label.setPos(*node.center)
            label.setToolTip(_module_tooltip(node))
            label.setVisible(presentation.show_labels)

    def _draw_lattice_axes(self, presentation: CubicLatticePresentation) -> None:
        axes = presentation.geometry.lattice_axes
        live_axes = {axis.axis for axis in axes}
        for name in set(self._lattice_axis_items).difference(live_axes):
            self._plot_item.removeItem(self._lattice_axis_items.pop(name))
        for name in set(self._lattice_axis_labels).difference(live_axes):
            self._plot_item.removeItem(self._lattice_axis_labels.pop(name))
        for axis in axes:
            item = self._lattice_axis_items.get(axis.axis)
            if item is None:
                item = pg.PlotCurveItem(antialias=True)
                self._plot_item.addItem(item)
                self._lattice_axis_items[axis.axis] = item
            item.setData(
                x=[axis.start[0], axis.end[0]],
                y=[axis.start[1], axis.end[1]],
            )
            item.setPen(pg.mkPen(_AXIS_COLORS[axis.axis], width=3.0))
            item.setZValue(100)
            label = self._lattice_axis_labels.get(axis.axis)
            if label is None:
                label = pg.TextItem(
                    text=axis.axis.upper(),
                    color=_AXIS_COLORS[axis.axis],
                    anchor=(0.5, 0.5),
                )
                label.setZValue(101)
                self._plot_item.addItem(label)
                self._lattice_axis_labels[axis.axis] = label
            label.setPos(*axis.end)

    def _projection_selected(self, index: int) -> None:
        value = self.projection_combo.itemData(index)
        if isinstance(value, str):
            self.projection_changed.emit(value)

    def _layer_selected(self, index: int) -> None:
        self.layer_changed.emit(self.layer_combo.itemData(index))

    def _nodes_clicked(self, _item: Any, points: list[Any], _event: Any) -> None:
        if points and isinstance(points[0].data(), str):
            self.entity_selected.emit("node", points[0].data())

    def _cells_clicked(self, _item: Any, points: list[Any], _event: Any) -> None:
        if points and isinstance(points[0].data(), str):
            self.entity_selected.emit("cell", points[0].data())

    def _edges_clicked(self, _item: Any, points: list[Any], _event: Any) -> None:
        if points and isinstance(points[0].data(), str):
            self.entity_selected.emit("edge", points[0].data())

    def _edge_clicked(self, identifier: str, _item: Any, _event: Any) -> None:
        self.entity_selected.emit("edge", identifier)

    def _orbit_dragged(self, delta_x: float, delta_y: float) -> None:
        self.orbit_requested.emit(
            -delta_x * _ORBIT_RADIANS_PER_PIXEL,
            delta_y * _ORBIT_RADIANS_PER_PIXEL,
        )


def _resize_curve_items(plot_item: Any, items: list[Any], count: int) -> None:
    """Reuse curve objects so live frames do not rebuild the graphics scene."""
    while len(items) < count:
        item = pg.PlotCurveItem(antialias=True)
        plot_item.addItem(item)
        items.append(item)
    while len(items) > count:
        plot_item.removeItem(items.pop())


def _resize_polygon_items(
    plot_item: Any,
    items: list[QGraphicsPolygonItem],
    count: int,
) -> None:
    """Reuse cube-face polygon objects across measured-pose samples."""
    while len(items) < count:
        item = QGraphicsPolygonItem()
        plot_item.addItem(item)
        items.append(item)
    while len(items) > count:
        plot_item.removeItem(items.pop())


def _remove_items(plot_item: Any, items: list[Any]) -> None:
    for item in items:
        plot_item.removeItem(item)


def _module_tooltip(node: PresentedLatticeModule) -> str:
    return (
        f"{node.id}\ncell={node.cell}\n"
        f"measured=({node.measured_lattice_position[0]:.3f}, "
        f"{node.measured_lattice_position[1]:.3f}, "
        f"{node.measured_lattice_position[2]:.3f}) cells\n"
        f"position residual={node.position_residual_m:.4g} m\n"
        f"orientation residual={node.orientation_residual_rad:.4g} rad\n"
        f"assembly={node.assembly_id}"
    )


def _selection_text(presentation: CubicLatticePresentation) -> str:
    geometry = presentation.geometry
    selection = presentation.selection
    if selection is None:
        off_lattice = sum(node.off_lattice for node in geometry.nodes)
        conflicts = sum(cell.occupancy_conflict for cell in geometry.cells)
        layer = "all Z layers" if geometry.layer_z is None else f"Z={geometry.layer_z}"
        return (
            f"{len(geometry.nodes)} modules · {len(geometry.edges)} connections · "
            f"{layer} · off lattice={off_lattice} · conflicts={conflicts}"
        )
    if selection.kind == "node":
        node = next((item for item in geometry.nodes if item.id == selection.entity_id), None)
        if node is not None:
            return (
                f"Module {node.id} · cell={node.cell} · "
                f"position residual={node.position_residual_m:.4g} m · "
                f"orientation residual={node.orientation_residual_rad:.4g} rad · "
                f"assembly={node.assembly_id}"
            )
    elif selection.kind == "edge":
        edge = next((item for item in geometry.edges if item.id == selection.entity_id), None)
        if edge is not None:
            return (
                f"Connection {edge.id} · {edge.connector_a} [{edge.source_face}] ↔ "
                f"{edge.connector_b} [{edge.target_face}]"
            )
    else:
        cell = next((item for item in geometry.cells if item.id == selection.entity_id), None)
        if cell is not None:
            return f"Cell {cell.cell} · occupants={', '.join(cell.occupant_ids)}"
    return f"Selected {selection.kind}: {selection.entity_id} (hidden by the active layer)"


def _shaded_color(name: str, brightness: float, *, alpha: int) -> QColor:
    color = QColor(name)
    result = QColor(
        min(round(color.red() * brightness), 255),
        min(round(color.green() * brightness), 255),
        min(round(color.blue() * brightness), 255),
        alpha,
    )
    return result


def _assembly_color(identifier: str) -> str:
    seed = sum((index + 1) * ord(character) for index, character in enumerate(identifier))
    return _ASSEMBLY_PALETTE[seed % len(_ASSEMBLY_PALETTE)]


__all__ = ["CubicLatticeWidget", "LatticeViewBox"]
