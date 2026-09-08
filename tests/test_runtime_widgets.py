"""Focused native widget tests for the Runtime Inspector graph and event table."""

from __future__ import annotations

import os
from collections.abc import Iterator
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from modsim.runtime.inspection import RuntimeEventRow
from modsim.runtime.inspector_runner import RuntimeInspectorConfig
from modsim_studio.runtime_events import (
    RuntimeEventLogWidget,
    RuntimeEventTableModel,
)
from modsim_studio.runtime_graph import TopologyGraphWidget
from modsim_studio.runtime_lattice import CubicLatticeWidget
from modsim_studio.runtime_lattice_presenter import (
    CubicLatticeGeometry,
    LatticeProjection,
    PresentedGridLine,
    PresentedLatticeAxis,
    PresentedLatticeCell,
    PresentedLatticeConnection,
    PresentedLatticeFace,
    PresentedLatticeModule,
    PresentedOrientationAxis,
)
from modsim_studio.runtime_presenter import (
    CubicLatticePresentation,
    GraphSelection,
    PresentedConnectionEdge,
    PresentedModuleNode,
    RuntimePresentation,
)
from modsim_studio.runtime_window import RuntimeInspectorWindow


@pytest.fixture(scope="module")
def application() -> Iterator[QApplication]:
    existing = QApplication.instance()
    app = existing if isinstance(existing, QApplication) else QApplication([])
    yield app
    app.processEvents()


def presentation(*, selected: bool = False, edge: bool = True) -> RuntimePresentation:
    selection = GraphSelection("edge", "connection:one") if selected and edge else None
    return RuntimePresentation(
        nodes=(
            PresentedModuleNode(
                id="alpha",
                label="alpha",
                module_type_id="generic_cube",
                assembly_id="assembly:alpha",
                position=(-1.0, 0.0),
            ),
            PresentedModuleNode(
                id="beta",
                label="beta",
                module_type_id="generic_cube",
                assembly_id="assembly:alpha" if edge else "assembly:beta",
                position=(1.0, 0.0),
            ),
        ),
        edges=(
            PresentedConnectionEdge(
                id="connection:one",
                source="alpha",
                target="beta",
                connector_a="alpha/front",
                connector_b="beta/rear",
                path=((-1.0, 0.0), (0.0, 0.0), (1.0, 0.0)),
                selected=selected,
            ),
        )
        if edge
        else (),
        events=(),
        selection=selection,
        source_text="generic_cube@0.1.0 | sample=1",
        status_text="mock | docked",
    )


def lattice_presentation(
    *,
    projection: LatticeProjection = LatticeProjection.ISOMETRIC,
    layer_z: int | None = None,
) -> CubicLatticePresentation:
    """Return representative immutable geometry without a live runtime."""
    geometry = CubicLatticeGeometry(
        projection=projection,
        layer_z=layer_z,
        available_layers_z=(0, 1),
        grid_lines=(
            PresentedGridLine(start=(-1.0, 0.0), end=(2.0, 0.0), axis="x", major=True),
            PresentedGridLine(start=(0.0, -1.0), end=(0.0, 2.0), axis="y"),
        ),
        lattice_axes=(
            PresentedLatticeAxis(axis="x", start=(-1.0, -1.0), end=(-0.2, -1.0)),
            PresentedLatticeAxis(axis="y", start=(-1.0, -1.0), end=(-1.0, -0.2)),
            PresentedLatticeAxis(axis="z", start=(-1.0, -1.0), end=(-0.6, -0.6)),
        ),
        cells=(
            PresentedLatticeCell(
                id="cell:0,0,0",
                cell=(0, 0, 0),
                center=(0.0, 0.0),
                outline_segments=(
                    ((-0.45, -0.45), (0.45, -0.45)),
                    ((0.45, -0.45), (0.45, 0.45)),
                    ((0.45, 0.45), (-0.45, 0.45)),
                    ((-0.45, 0.45), (-0.45, -0.45)),
                ),
                occupant_ids=("alpha",),
                occupancy_conflict=False,
            ),
            PresentedLatticeCell(
                id="cell:1,0,1",
                cell=(1, 0, 1),
                center=(1.0, 1.0),
                outline_segments=(),
                occupant_ids=("beta",),
                occupancy_conflict=False,
            ),
        ),
        nodes=(
            PresentedLatticeModule(
                id="alpha",
                label="alpha",
                module_type_id="mblock",
                assembly_id="assembly:alpha",
                cell=(0, 0, 0),
                measured_lattice_position=(0.0, 0.0, 0.0),
                center=(0.0, 0.0),
                faces=(
                    PresentedLatticeFace(
                        points=((-0.4, -0.4), (0.4, -0.4), (0.4, 0.4), (-0.4, 0.4)),
                        depth=0.0,
                        brightness=0.8,
                    ),
                ),
                orientation_axes=(
                    PresentedOrientationAxis(axis="x", start=(0.0, 0.0), end=(0.3, 0.0)),
                    PresentedOrientationAxis(axis="y", start=(0.0, 0.0), end=(0.0, 0.3)),
                    PresentedOrientationAxis(axis="z", start=(0.0, 0.0), end=(0.2, 0.2)),
                ),
                tether=None,
                position_residual_m=0.0,
                orientation_residual_rad=0.0,
                off_lattice=False,
                occupancy_conflict=False,
            ),
            PresentedLatticeModule(
                id="beta",
                label="beta",
                module_type_id="mblock",
                assembly_id="assembly:alpha",
                cell=(1, 0, 1),
                measured_lattice_position=(1.15, 0.0, 1.0),
                center=(1.15, 1.0),
                faces=(
                    PresentedLatticeFace(
                        points=((0.75, 0.6), (1.55, 0.6), (1.55, 1.4), (0.75, 1.4)),
                        depth=1.0,
                        brightness=0.65,
                    ),
                ),
                orientation_axes=(),
                tether=((1.15, 1.0), (1.0, 1.0)),
                position_residual_m=0.015,
                orientation_residual_rad=0.0,
                off_lattice=True,
                occupancy_conflict=False,
            ),
        ),
        edges=(
            PresentedLatticeConnection(
                id="connection:one",
                source="alpha",
                target="beta",
                connector_a="alpha/positive_x",
                connector_b="beta/negative_x",
                source_face="positive_x",
                target_face="negative_x",
                source_face_residual_rad=0.0,
                target_face_residual_rad=0.0,
                path=((0.4, 0.0), (0.6, 0.5), (0.75, 1.0)),
                marker=(0.6, 0.5),
            ),
        ),
        bounds=(-1, 2, -1, 1, -1, 2),
    )
    return CubicLatticePresentation(
        geometry=geometry,
        events=(),
        selection=None,
        source_text="mblocks_3d@0.1.0 | sample=1",
        status_text="mock | pivoting",
        show_snap_cells=True,
        show_orientation_axes=True,
    )


def test_graph_widget_updates_topology_and_selection_without_a_window(
    application: QApplication,
) -> None:
    widget = TopologyGraphWidget()
    try:
        widget.set_presentation(presentation())
        application.processEvents()
        assert widget.displayed_node_ids == ("alpha", "beta")
        assert widget.displayed_edge_ids == ("connection:one",)
        assert widget.displayed_selection is None

        widget.set_presentation(presentation(selected=True))
        application.processEvents()
        assert widget.displayed_selection == ("edge", "connection:one")

        widget.set_presentation(presentation(edge=False))
        application.processEvents()
        assert widget.displayed_node_ids == ("alpha", "beta")
        assert widget.displayed_edge_ids == ()
        assert widget.displayed_selection is None
    finally:
        widget.close()


def test_lattice_widget_draws_entities_and_emits_view_controls(
    application: QApplication,
) -> None:
    widget = CubicLatticeWidget()
    projections: list[str] = []
    layers: list[object] = []
    snap_cells: list[bool] = []
    orientation_axes: list[bool] = []
    widget.projection_changed.connect(projections.append)
    widget.layer_changed.connect(layers.append)
    widget.snap_cells_changed.connect(snap_cells.append)
    widget.orientation_axes_changed.connect(orientation_axes.append)
    try:
        widget.set_presentation(lattice_presentation())
        application.processEvents()

        assert widget.displayed_node_ids == ("alpha", "beta")
        assert widget.displayed_edge_ids == ("connection:one",)
        assert widget.displayed_cell_ids == ("cell:0,0,0", "cell:1,0,1")
        assert widget.displayed_selection is None
        assert widget.projection_combo.currentData() == "isometric"
        assert tuple(widget.layer_combo.itemData(index) for index in range(3)) == (
            None,
            0,
            1,
        )
        assert widget.snap_cells_checkbox.isChecked()
        assert widget.orientation_axes_checkbox.isChecked()

        widget.projection_combo.setCurrentIndex(widget.projection_combo.findData("xz"))
        widget.layer_combo.setCurrentIndex(widget.layer_combo.findData(1))
        widget.snap_cells_checkbox.setChecked(False)
        widget.orientation_axes_checkbox.setChecked(False)
        application.processEvents()

        assert projections == ["xz"]
        assert layers == [1]
        assert snap_cells == [False]
        assert orientation_axes == [False]
    finally:
        widget.close()


def test_event_table_model_appends_and_resets_rows(application: QApplication) -> None:
    model = RuntimeEventTableModel()
    first = RuntimeEventRow(
        sequence=0,
        time_s=0.125,
        kind="DockCandidateDetected",
        detail="alpha/front <-> beta/rear",
    )
    second = RuntimeEventRow(
        sequence=1,
        time_s=0.25,
        kind="DockCommitted",
        detail="alpha/front <-> beta/rear (index 0)",
    )
    model.set_events((first,))
    model.set_events((first, second))
    application.processEvents()

    assert model.rowCount() == 2
    assert model.columnCount() == 4
    assert model.data(model.index(0, 0)) == "0"
    assert model.data(model.index(0, 1)) == "0.1250"
    assert model.data(model.index(1, 2)) == "DockCommitted"
    assert model.data(model.index(1, 3)) == second.detail
    assert (
        model.headerData(2, Qt.Orientation.Horizontal, int(Qt.ItemDataRole.DisplayRole)) == "Event"
    )

    model.set_events((second,))
    assert model.rowCount() == 1
    assert model.events == (second,)


def test_event_log_widget_exposes_accumulated_model(application: QApplication) -> None:
    widget = RuntimeEventLogWidget()
    rows = (
        RuntimeEventRow(sequence=0, time_s=0.1, kind="DockCandidateDetected"),
        RuntimeEventRow(sequence=1, time_s=0.2, kind="DockCommitted", detail="docked"),
    )
    try:
        widget.set_events(rows)
        application.processEvents()
        assert widget.model.events == rows
        assert widget.table.model() is widget.model
    finally:
        widget.close()


def test_event_table_rejects_unsorted_or_duplicate_sequences() -> None:
    model = RuntimeEventTableModel()
    duplicate = RuntimeEventRow(sequence=0, time_s=0.1, kind="Duplicate")
    with pytest.raises(ValueError, match="unique ascending"):
        model.set_events((duplicate, duplicate))


def test_runtime_window_keeps_target_speed_visible_across_presentations(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    # The window normally auto-starts on the next event-loop turn. This focused
    # presentation test deliberately keeps its controller dormant.
    monkeypatch.setattr(RuntimeInspectorWindow, "start", lambda _self: None)
    window = RuntimeInspectorWindow(
        RuntimeInspectorConfig(
            pack_path=tmp_path / "pack",
            backend="mock",
            real_time_factor=4.0,
        )
    )
    try:
        assert window.speed_label.text() == "Target speed: 4x"
        assert "motor speeds and physics are unchanged" in window.speed_label.toolTip()

        window._apply_presentation(presentation())
        application.processEvents()

        assert window.speed_label.text() == "Target speed: 4x"
        assert window.source_label.text() == "generic_cube@0.1.0 | sample=1"
    finally:
        window.close()


def test_runtime_window_switches_between_topology_and_lattice_renderers(
    application: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(RuntimeInspectorWindow, "start", lambda _self: None)
    window = RuntimeInspectorWindow(
        RuntimeInspectorConfig(
            pack_path=tmp_path / "pack",
            backend="mock",
        )
    )
    try:
        window._apply_presentation(presentation())
        application.processEvents()
        assert window.view_stack.currentWidget() is window.graph
        assert window.graph.displayed_node_ids == ("alpha", "beta")

        window._apply_presentation(lattice_presentation())
        application.processEvents()
        assert window.view_stack.currentWidget() is window.lattice
        assert window.lattice.displayed_node_ids == ("alpha", "beta")
        assert window.lattice.displayed_edge_ids == ("connection:one",)
        assert window.source_label.text() == "mblocks_3d@0.1.0 | sample=1"
        assert window.status_label.text() == "mock | pivoting"

        window._apply_presentation(presentation(edge=False))
        application.processEvents()
        assert window.view_stack.currentWidget() is window.graph
        assert window.graph.displayed_edge_ids == ()
    finally:
        window.close()
