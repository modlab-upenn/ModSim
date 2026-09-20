# pyright: reportMissingTypeStubs=false, reportUnknownArgumentType=false
# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false
"""PyQtGraph renderer for immutable runtime graph presentations."""

from __future__ import annotations

from functools import partial
from typing import Any

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QVBoxLayout, QWidget

from modsim_studio.appearance import theme_manager
from modsim_studio.chrome import LegendWidget
from modsim_studio.runtime_presenter import RuntimePresentation

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


class TopologyGraphWidget(QWidget):
    """Render stable module nodes and selectable connection curves."""

    entity_selected = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._appearance = theme_manager()
        self.plot = pg.PlotWidget(background=self._appearance.theme.viewport)
        layout.addWidget(self.plot)
        self.legend = LegendWidget()
        layout.addWidget(self.legend)

        self._plot_item: Any = self.plot.getPlotItem()
        self._plot_item.hideAxis("left")
        self._plot_item.hideAxis("bottom")
        self._plot_item.setAspectLocked(True)
        self._plot_item.setMenuEnabled(False)
        self._plot_item.setMouseEnabled(x=True, y=True)
        self.plot.setToolTip(
            "Click a module or docking edge to select it. Drag to pan and use the wheel to zoom."
        )

        self._fingerprint: object | None = None
        self._node_ids: tuple[str, ...] = ()
        self._edge_ids: tuple[str, ...] = ()
        self._selection: tuple[str, str] | None = None
        self._node_item: Any | None = None
        self._edge_items: dict[str, Any] = {}
        self._labels: list[Any] = []
        self._presentation: RuntimePresentation | None = None
        self._appearance.changed.connect(self._apply_theme)

    def _apply_theme(self) -> None:
        self.plot.setBackground(self._appearance.theme.viewport)
        self._fingerprint = None
        if self._presentation is not None:
            self.set_presentation(self._presentation)

    @property
    def displayed_node_ids(self) -> tuple[str, ...]:
        """Return node IDs currently drawn, primarily for UI diagnostics."""
        return self._node_ids

    @property
    def displayed_edge_ids(self) -> tuple[str, ...]:
        """Return connection IDs currently drawn."""
        return self._edge_ids

    @property
    def displayed_selection(self) -> tuple[str, str] | None:
        """Return the selected graph entity currently styled by the widget."""
        return self._selection

    @property
    def displayed_label_count(self) -> int:
        """Return the number of currently drawn module labels."""
        return len(self._labels)

    def set_presentation(self, presentation: RuntimePresentation) -> None:
        """Draw ``presentation`` unless its visible graph is unchanged."""
        self._presentation = presentation
        theme = self._appearance.theme
        fingerprint = _presentation_fingerprint(presentation)
        if fingerprint == self._fingerprint:
            return

        previous_node_ids = self._node_ids
        self._fingerprint = fingerprint
        self._node_ids = tuple(node.id for node in presentation.nodes)
        self._edge_ids = tuple(edge.id for edge in presentation.edges)
        self._selection = (
            (presentation.selection.kind, presentation.selection.entity_id)
            if presentation.selection is not None
            else None
        )

        self._plot_item.clear()
        self._edge_items.clear()
        self._labels.clear()

        entries = [
            ("●", _assembly_color(identifier), f"Assembly {identifier}")
            for identifier in sorted({n.assembly_id for n in presentation.nodes})
        ]
        entries.extend(
            (
                ("━", theme.grid_major, "Committed connection"),
                ("┄", theme.muted, "Target connection still needed"),
                ("━", theme.success, "Target connection reached"),
                ("●", theme.warning, "Selected module or connection"),
            )
        )
        self.legend.set_entries(
            tuple(entries), "Node positions are logical, not physical. Drag to pan; wheel to zoom."
        )

        for edge in presentation.edges:
            x_values = [point[0] for point in edge.path]
            y_values = [point[1] for point in edge.path]
            curve = pg.PlotCurveItem(
                x=x_values,
                y=y_values,
                pen=pg.mkPen(
                    theme.warning
                    if edge.selected
                    else theme.success
                    if edge.state == "matched"
                    else theme.muted
                    if edge.state == "pending"
                    else theme.grid_major,
                    width=4.0 if edge.selected else 2.2,
                    style=Qt.PenStyle.DashLine
                    if edge.state == "pending"
                    else Qt.PenStyle.SolidLine,
                ),
                antialias=True,
                clickable=True,
            )
            curve.setClickable(True, width=12)
            curve.setZValue(5 if edge.selected else 1)
            curve.setToolTip(f"{edge.connector_a} ↔ {edge.connector_b}\n{edge.state.capitalize()}")
            curve.sigClicked.connect(partial(self._edge_clicked, edge.id))
            self._plot_item.addItem(curve)
            self._edge_items[edge.id] = curve

        spots = []
        for node in presentation.nodes:
            spots.append(
                {
                    "pos": node.position,
                    "data": node.id,
                    "size": 28 if node.selected else 23,
                    "symbol": "o",
                    "brush": pg.mkBrush(_assembly_color(node.assembly_id)),
                    "pen": pg.mkPen(
                        theme.warning if node.selected else theme.text,
                        width=4.0 if node.selected else 1.6,
                    ),
                }
            )
        self._node_item = pg.ScatterPlotItem(pxMode=True, hoverable=True)
        self._node_item.setData(spots=spots)
        self._node_item.setZValue(10)
        self._node_item.sigClicked.connect(self._nodes_clicked)
        self._plot_item.addItem(self._node_item)

        if presentation.show_labels:
            for node in presentation.nodes:
                label = pg.TextItem(
                    text=node.label,
                    color=theme.text,
                    anchor=(0.5, -0.65),
                    border=None,
                    fill=None,
                )
                label.setPos(*node.position)
                label.setZValue(11)
                self._plot_item.addItem(label)
                self._labels.append(label)

        if previous_node_ids != self._node_ids:
            self._fit_nodes(tuple(node.position for node in presentation.nodes))

    def _nodes_clicked(self, _item: Any, points: list[Any], _event: Any) -> None:
        if not points:
            return
        identifier = points[0].data()
        if isinstance(identifier, str):
            self.entity_selected.emit("node", identifier)

    def _edge_clicked(self, identifier: str, _item: Any, _event: Any) -> None:
        self.entity_selected.emit("edge", identifier)

    def _fit_nodes(self, positions: tuple[tuple[float, float], ...]) -> None:
        if not positions:
            self._plot_item.setXRange(-1.0, 1.0, padding=0.0)
            self._plot_item.setYRange(-1.0, 1.0, padding=0.0)
            return
        x_values = [position[0] for position in positions]
        y_values = [position[1] for position in positions]
        x_span = max(max(x_values) - min(x_values), 1.0)
        y_span = max(max(y_values) - min(y_values), 1.0)
        padding = max(x_span, y_span) * 0.35
        self._plot_item.setXRange(
            min(x_values) - padding,
            max(x_values) + padding,
            padding=0.0,
        )
        self._plot_item.setYRange(
            min(y_values) - padding,
            max(y_values) + padding,
            padding=0.0,
        )


def _presentation_fingerprint(presentation: RuntimePresentation) -> object:
    return (
        presentation.show_labels,
        tuple(
            (
                node.id,
                node.label,
                node.module_type_id,
                node.assembly_id,
                node.position,
                node.selected,
            )
            for node in presentation.nodes
        ),
        tuple(
            (
                edge.id,
                edge.source,
                edge.target,
                edge.connector_a,
                edge.connector_b,
                edge.path,
                edge.selected,
                edge.state,
            )
            for edge in presentation.edges
        ),
    )


def _assembly_color(identifier: str) -> str:
    seed = sum((index + 1) * ord(character) for index, character in enumerate(identifier))
    return _ASSEMBLY_PALETTE[seed % len(_ASSEMBLY_PALETTE)]


__all__ = ["TopologyGraphWidget"]
