# pyright: reportMissingTypeStubs=false, reportUnknownArgumentType=false
# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false
"""PyQtGraph renderer for immutable cubic-lattice presentations."""

from __future__ import annotations

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


class CubicLatticeWidget(QWidget):
    """Render measured cubes, snap cells, connections, and a finite grid."""

    entity_selected = Signal(str, str)
    projection_changed = Signal(str)
    layer_changed = Signal(object)
    snap_cells_changed = Signal(bool)
    orientation_axes_changed = Signal(bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        controls = QHBoxLayout()
        controls.setContentsMargins(8, 5, 8, 3)
        controls.addWidget(QLabel("Projection"))
        self.projection_combo = QComboBox()
        for label, projection in (
            ("Isometric", LatticeProjection.ISOMETRIC),
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

        self.plot = pg.PlotWidget(background=_BACKGROUND)
        root.addWidget(self.plot, 1)
        self.legend_label = QLabel(
            "Dashed cube: nearest cell  ·  amber: off lattice  ·  red: occupancy conflict  "
            "·  local axes: X / Y / Z"
        )
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
            "Click a module, docking marker, or warning cell; drag to pan and wheel to zoom."
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
        self._graphics_items: list[Any] = []

        self.projection_combo.currentIndexChanged.connect(self._projection_selected)
        self.layer_combo.currentIndexChanged.connect(self._layer_selected)
        self.snap_cells_checkbox.toggled.connect(self.snap_cells_changed.emit)
        self.orientation_axes_checkbox.toggled.connect(self.orientation_axes_changed.emit)
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
        self._plot_item.clear()
        self._graphics_items.clear()

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

        self.layer_combo.blockSignals(True)
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
        for line in presentation.geometry.grid_lines:
            item = pg.PlotCurveItem(
                x=[line.start[0], line.end[0]],
                y=[line.start[1], line.end[1]],
                pen=pg.mkPen(
                    _GRID_MAJOR_COLOR if line.major else _GRID_COLOR,
                    width=1.4 if line.major else 0.8,
                ),
                antialias=True,
            )
            item.setZValue(-100)
            self._plot_item.addItem(item)

    def _draw_snap_cells(self, presentation: CubicLatticePresentation) -> None:
        off_lattice_cells = {
            f"cell:{node.cell[0]},{node.cell[1]},{node.cell[2]}"
            for node in presentation.geometry.nodes
            if node.off_lattice
        }
        cell_spots: list[dict[str, object]] = []
        for cell in presentation.geometry.cells:
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
            for start, end in cell.outline_segments:
                item = pg.PlotCurveItem(
                    x=[start[0], end[0]],
                    y=[start[1], end[1]],
                    pen=pg.mkPen(color, width=width, style=Qt.PenStyle.DashLine),
                    antialias=True,
                )
                item.setZValue(-25)
                self._plot_item.addItem(item)
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
                badge = pg.TextItem(
                    text=str(len(cell.occupant_ids)),
                    color=_CONFLICT_COLOR,
                    anchor=(-0.3, 1.2),
                )
                badge.setPos(*cell.center)
                badge.setToolTip(
                    f"Occupancy conflict at {cell.cell}: {', '.join(cell.occupant_ids)}"
                )
                badge.setZValue(80)
                self._plot_item.addItem(badge)
        if cell_spots:
            cell_item = pg.ScatterPlotItem(pxMode=True, hoverable=True)
            cell_item.setData(spots=cell_spots)
            cell_item.setZValue(75)
            cell_item.sigClicked.connect(self._cells_clicked)
            self._plot_item.addItem(cell_item)

        for node in presentation.geometry.nodes:
            if node.tether is None:
                continue
            start, end = node.tether
            tether = pg.PlotCurveItem(
                x=[start[0], end[0]],
                y=[start[1], end[1]],
                pen=pg.mkPen(_OFF_LATTICE_COLOR, width=1.7, style=Qt.PenStyle.DotLine),
                antialias=True,
            )
            tether.setZValue(-15)
            self._plot_item.addItem(tether)

    def _draw_connections(self, presentation: CubicLatticePresentation) -> None:
        marker_spots: list[dict[str, object]] = []
        for edge in presentation.geometry.edges:
            color = _SELECTED_COLOR if edge.selected else _EDGE_COLOR
            curve = pg.PlotCurveItem(
                x=[point[0] for point in edge.path],
                y=[point[1] for point in edge.path],
                pen=pg.mkPen(color, width=4.0 if edge.selected else 2.2),
                antialias=True,
                clickable=True,
            )
            curve.setClickable(True, width=12)
            curve.setZValue(45)
            curve.setToolTip(
                f"{edge.id}\n{edge.connector_a} ({edge.source_face}) ↔ "
                f"{edge.connector_b} ({edge.target_face})"
            )
            curve.sigClicked.connect(partial(self._edge_clicked, edge.id))
            self._plot_item.addItem(curve)
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
        if marker_spots:
            marker_item = pg.ScatterPlotItem(pxMode=True, hoverable=True)
            marker_item.setData(spots=marker_spots)
            marker_item.setZValue(65)
            marker_item.sigClicked.connect(self._edges_clicked)
            self._plot_item.addItem(marker_item)

    def _draw_modules(self, presentation: CubicLatticePresentation) -> None:
        ordered_faces = sorted(
            (
                (face.depth, node, face)
                for node in presentation.geometry.nodes
                for face in node.faces
            ),
            key=lambda item: (item[0], item[1].id),
        )
        for index, (_depth, node, face) in enumerate(ordered_faces):
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
            polygon = QGraphicsPolygonItem(
                QPolygonF([QPointF(point[0], point[1]) for point in face.points])
            )
            polygon.setBrush(QBrush(fill))
            pen = QPen(QColor(outline))
            pen.setWidthF(1.0)
            pen.setCosmetic(True)
            polygon.setPen(pen)
            polygon.setZValue(10.0 + index * 0.01)
            polygon.setToolTip(_module_tooltip(node))
            self._plot_item.addItem(polygon)
            self._graphics_items.append(polygon)

        for node in presentation.geometry.nodes:
            outline = (
                _SELECTED_COLOR
                if node.selected
                else _CONFLICT_OUTLINE
                if node.occupancy_conflict
                else _OFF_LATTICE_OUTLINE
                if node.off_lattice
                else _NODE_OUTLINE
            )
            for start, end in node.outline_segments:
                edge_item = pg.PlotCurveItem(
                    x=[start[0], end[0]],
                    y=[start[1], end[1]],
                    pen=pg.mkPen(outline, width=3.0 if node.selected else 1.45),
                    antialias=True,
                )
                edge_item.setZValue(60)
                self._plot_item.addItem(edge_item)

        node_spots = [
            {
                "pos": node.center,
                "data": node.id,
                "size": 34,
                "symbol": "s",
                "brush": pg.mkBrush(QColor(0, 0, 0, 0)),
                "pen": pg.mkPen(QColor(0, 0, 0, 0)),
            }
            for node in presentation.geometry.nodes
        ]
        if node_spots:
            node_item = pg.ScatterPlotItem(pxMode=True, hoverable=True)
            node_item.setData(spots=node_spots)
            node_item.setZValue(70)
            node_item.sigClicked.connect(self._nodes_clicked)
            self._plot_item.addItem(node_item)

        for node in presentation.geometry.nodes:
            for axis in node.orientation_axes:
                item = pg.PlotCurveItem(
                    x=[axis.start[0], axis.end[0]],
                    y=[axis.start[1], axis.end[1]],
                    pen=pg.mkPen(_AXIS_COLORS[axis.axis], width=1.7),
                    antialias=True,
                )
                item.setZValue(72)
                self._plot_item.addItem(item)
            label = pg.TextItem(
                text=f"{node.label}\n{node.cell}",
                color="#eef5fb",
                anchor=(0.5, -0.18),
                border=None,
                fill=pg.mkBrush(17, 24, 32, 150),
            )
            label.setPos(*node.center)
            label.setToolTip(_module_tooltip(node))
            label.setZValue(90)
            self._plot_item.addItem(label)

    def _draw_lattice_axes(self, presentation: CubicLatticePresentation) -> None:
        for axis in presentation.geometry.lattice_axes:
            item = pg.PlotCurveItem(
                x=[axis.start[0], axis.end[0]],
                y=[axis.start[1], axis.end[1]],
                pen=pg.mkPen(_AXIS_COLORS[axis.axis], width=3.0),
                antialias=True,
            )
            item.setZValue(100)
            self._plot_item.addItem(item)
            label = pg.TextItem(
                text=axis.axis.upper(),
                color=_AXIS_COLORS[axis.axis],
                anchor=(0.5, 0.5),
            )
            label.setPos(*axis.end)
            label.setZValue(101)
            self._plot_item.addItem(label)

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


__all__ = ["CubicLatticeWidget"]
