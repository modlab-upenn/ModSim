"""Deterministic, Qt-free cubic-lattice Runtime Inspector presentation tests."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.state import WorldState
from modsim.core.transforms import Transform
from modsim.model_views import CubicLatticeView, ModelViewContext, ModelViewFactory
from modsim.robot_packs import RobotPack, RobotPackLoader
from modsim.runtime.inspection import RuntimeEventRow, RuntimeInspectorFrame
from modsim.runtime.metrics import collect_metrics
from modsim_studio.runtime_lattice_presenter import (
    CubicLatticeProjector,
    LatticeProjection,
    project_lattice_point,
)
from modsim_studio.runtime_presenter import (
    CubicLatticePresentation,
    GraphSelection,
    RuntimeInspectorPresenter,
    RuntimePresentationError,
)

_ROOT = Path(__file__).resolve().parents[1]
_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"
_MODULE_TYPE = "mblocks_3d"


@pytest.fixture(scope="module")
def mblocks_pack() -> RobotPack:
    return RobotPackLoader().load(_PACK_PATH).pack


def _world_and_view(
    pack: RobotPack,
    *positions: tuple[float, float, float],
) -> tuple[WorldState, CubicLatticeView]:
    world = WorldState.from_scene(
        pack,
        SceneSpec(
            placements=tuple(
                ModulePlacement(
                    instance_id=ModuleInstanceId(f"block_{index + 1}"),
                    module_type_id=_MODULE_TYPE,
                    pose=Transform.from_translation(position),
                )
                for index, position in enumerate(positions)
            )
        ),
    )
    recipe = next(recipe for recipe in pack.manifest.model_views if recipe.id == "mblocks_lattice")
    generated = ModelViewFactory().build(recipe, ModelViewContext(pack=pack, world=world))
    assert isinstance(generated, CubicLatticeView)
    return world, generated


def _frame(
    world: WorldState,
    view: CubicLatticeView,
    *,
    events: tuple[RuntimeEventRow, ...] = (),
    start: int = 0,
    stop: int = 0,
) -> RuntimeInspectorFrame:
    return RuntimeInspectorFrame(
        backend_name="mock",
        view=view,
        metrics=collect_metrics(world),
        events=events,
        event_start_sequence=start,
        next_event_sequence=stop,
    )


@pytest.mark.parametrize(
    ("projection", "expected"),
    (
        (LatticeProjection.XY, (1.0, 2.0, 3.0)),
        (LatticeProjection.XZ, (1.0, 3.0, 2.0)),
        (LatticeProjection.YZ, (2.0, 3.0, 1.0)),
        (
            LatticeProjection.ISOMETRIC,
            (-math.sqrt(3.0) / 2.0, 1.5, 2.0 * math.sqrt(3.0)),
        ),
    ),
)
def test_project_lattice_point_supports_all_spatial_projections(
    projection: LatticeProjection,
    expected: tuple[float, float, float],
) -> None:
    assert project_lattice_point((1.0, 2.0, 3.0), projection) == pytest.approx(expected)


def test_projector_draws_measured_residual_motion_separately_from_snap_cell(
    mblocks_pack: RobotPack,
) -> None:
    world, view = _world_and_view(mblocks_pack, (0.01, 0.0, 0.0))
    node = view.nodes[0]
    assert node.cell == (0, 0, 0)
    assert node.pose_residual.translation_m == pytest.approx((0.01, 0.0, 0.0))
    assert node.off_lattice

    geometry = CubicLatticeProjector().project(view, projection=LatticeProjection.XY)

    assert len(geometry.nodes) == len(geometry.cells) == 1
    presented = geometry.nodes[0]
    snap_cell = geometry.cells[0]
    assert presented.measured_lattice_position == pytest.approx((0.2, 0.0, 0.0))
    assert presented.center == pytest.approx((0.2, 0.0))
    assert snap_cell.center == pytest.approx((0.0, 0.0))
    assert presented.tether is not None
    assert presented.tether[0] == pytest.approx((0.2, 0.0))
    assert presented.tether[1] == pytest.approx((0.0, 0.0))
    assert presented.off_lattice
    assert snap_cell.outline_segments
    assert collect_metrics(world).module_count_total == 1


def test_projector_exposes_conflicts_and_filters_integer_z_layers(
    mblocks_pack: RobotPack,
) -> None:
    _world, conflict_view = _world_and_view(
        mblocks_pack,
        (0.0, 0.0, 0.0),
        (0.001, 0.0, 0.0),
    )
    conflict = CubicLatticeProjector().project(conflict_view)

    assert len(conflict.cells) == 1
    assert conflict.cells[0].occupant_ids == ("block_1", "block_2")
    assert conflict.cells[0].occupancy_conflict
    assert all(node.occupancy_conflict for node in conflict.nodes)

    _world, layered_view = _world_and_view(
        mblocks_pack,
        (0.0, 0.0, 0.0),
        (0.05, 0.0, 0.05),
    )
    layer_zero = CubicLatticeProjector().project(
        layered_view,
        projection=LatticeProjection.XY,
        layer_z=0,
    )

    assert layer_zero.available_layers_z == (0, 1)
    assert layer_zero.layer_z == 0
    assert [node.id for node in layer_zero.nodes] == ["block_1"]
    assert [cell.cell for cell in layer_zero.cells] == [(0, 0, 0)]


def test_lattice_presenter_keeps_events_selection_controls_and_overlays_in_sync(
    mblocks_pack: RobotPack,
) -> None:
    world, view = _world_and_view(
        mblocks_pack,
        (0.0, 0.0, 0.0),
        (0.05, 0.0, 0.05),
    )
    rows = (
        RuntimeEventRow(sequence=0, time_s=0.1, kind="UndockCommitted", detail="released"),
        RuntimeEventRow(sequence=1, time_s=0.2, kind="DockCommitted", detail="captured"),
    )
    presenter = RuntimeInspectorPresenter()
    presented = presenter.apply_frame(_frame(world, view, events=rows, start=0, stop=2))

    assert isinstance(presented, CubicLatticePresentation)
    assert presented.events == rows
    assert presented.geometry.projection is LatticeProjection.ISOMETRIC
    assert presented.geometry.available_layers_z == (0, 1)
    assert "mblocks_3d@0.1.0" in presented.source_text
    assert "modules=2" in presented.status_text

    projected = presenter.set_lattice_projection(LatticeProjection.XZ)
    assert projected.geometry.projection is LatticeProjection.XZ
    assert projected.events == rows

    layered = presenter.set_lattice_layer(1)
    assert [node.id for node in layered.geometry.nodes] == ["block_2"]
    with pytest.raises(KeyError, match="Z layer 7"):
        presenter.set_lattice_layer(7)

    presenter.set_lattice_layer(None)
    selected_node = presenter.select("node", "block_1")
    assert isinstance(selected_node, CubicLatticePresentation)
    assert selected_node.selection == GraphSelection("node", "block_1")
    assert next(node for node in selected_node.geometry.nodes if node.id == "block_1").selected

    selected_cell = presenter.select("cell", "cell:1,0,1")
    assert isinstance(selected_cell, CubicLatticePresentation)
    assert selected_cell.selection == GraphSelection("cell", "cell:1,0,1")
    assert next(cell for cell in selected_cell.geometry.cells if cell.cell == (1, 0, 1)).selected

    overlays = presenter.set_lattice_overlays(
        show_snap_cells=False,
        show_orientation_axes=False,
    )
    assert not overlays.show_snap_cells
    assert not overlays.show_orientation_axes
    assert all(not cell.outline_segments for cell in overlays.geometry.cells)
    assert all(not node.orientation_axes for node in overlays.geometry.nodes)


def test_lattice_controls_reject_a_topology_presentation() -> None:
    presenter = RuntimeInspectorPresenter()

    with pytest.raises(RuntimePresentationError, match="no runtime frame"):
        presenter.set_lattice_projection(LatticeProjection.XY)
