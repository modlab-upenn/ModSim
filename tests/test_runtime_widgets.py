"""Focused native widget tests for the Runtime Inspector graph and event table."""

from __future__ import annotations

import os
from collections.abc import Iterator

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from modsim.runtime.inspection import RuntimeEventRow
from modsim_studio.runtime_events import (
    RuntimeEventLogWidget,
    RuntimeEventTableModel,
)
from modsim_studio.runtime_graph import TopologyGraphWidget
from modsim_studio.runtime_presenter import (
    GraphSelection,
    PresentedConnectionEdge,
    PresentedModuleNode,
    RuntimePresentation,
)


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
