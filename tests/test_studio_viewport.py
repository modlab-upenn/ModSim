"""Retained Builder scenes use real VTK actors without requiring GPU rendering."""

from __future__ import annotations

import os
from collections.abc import Iterator
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyvistaqt")

import pyvista as pv
from PySide6.QtWidgets import QApplication, QWidget

from modsim.importers import ImportedGeometry
from modsim.importers.urdf import GeometryKind
from modsim_studio import viewport as viewport_module
from modsim_studio.main_window import MainWindow
from modsim_studio.project import StudioProject


class _HeadlessInteractor(pv.Plotter):
    """Exercise VTK pipelines/cameras/actors, counting draws without calling OpenGL."""

    def __init__(self, parent: QWidget, **_kwargs: Any) -> None:
        self.draws = 0
        super().__init__(off_screen=True)
        self.interactor = QWidget(parent)

    def render(self) -> None:
        if not self.suppress_rendering:
            self.draws += 1


@pytest.fixture(scope="module")
def application() -> QApplication:
    existing = QApplication.instance()
    return existing if isinstance(existing, QApplication) else QApplication([])


@pytest.fixture(scope="module")
def project() -> StudioProject:
    return StudioProject.open(
        Path(__file__).resolve().parents[1] / "examples/robot_packs/smores_ep"
    )


@pytest.fixture
def builder(
    application: QApplication, project: StudioProject, monkeypatch: pytest.MonkeyPatch
) -> Iterator[MainWindow]:
    monkeypatch.setattr(viewport_module, "QtInteractor", _HeadlessInteractor)
    window = MainWindow()
    window._set_project(project)
    try:
        yield window
    finally:
        window.project = None
        window.close()
        window.deleteLater()
        application.processEvents()


def _camera(viewport: Any) -> tuple[object, ...]:
    camera = viewport.plotter.camera
    return (
        camera.position,
        camera.focal_point,
        camera.up,
        camera.parallel_scale,
        camera.view_angle,
        camera.parallel_projection,
    )


def _actors(viewport: Any) -> dict[str, tuple[Any, ...]]:
    return {key: tuple(actors) for key, actors in viewport._layer_actors.items()}


def _forbid_scene_work(monkeypatch: pytest.MonkeyPatch, viewport: Any) -> None:
    def unexpected(*_args: Any, **_kwargs: Any) -> None:
        pytest.fail("An interaction reloaded or rebuilt the retained scene")

    monkeypatch.setattr(viewport, "_rebuild", unexpected)
    monkeypatch.setattr(viewport_module, "_read_mesh", unexpected)


def test_tree_selections_retain_meshes_materials_camera_and_draw_once(
    builder: MainWindow, project: StudioProject, monkeypatch: pytest.MonkeyPatch
) -> None:
    viewport = builder.viewport
    module = project.pack.hardware_catalog.module_types["smores_ep"]
    asset = project.imported_assets[module.asset_ref]
    actors = _actors(viewport)
    materials = [actor.prop.color for actor in actors["visuals"]]
    viewport.plotter.camera.azimuth += 25
    viewport.plotter.camera.zoom(1.7)
    camera = _camera(viewport)
    _forbid_scene_work(monkeypatch, viewport)
    for kind, identifier in (
        ("link", asset.links[0].name),
        ("link", asset.links[1].name),
        ("joint", module.joints[0].id),
    ):
        draws = viewport.plotter.draws
        assert builder._select_tree_entity((kind, identifier, module.id))
        assert viewport.plotter.draws == draws + 1
        assert _actors(viewport) == actors
        assert _camera(viewport) == camera
        assert [actor.prop.color for actor in actors["visuals"]] == materials
        assert viewport._selected_link == (identifier if kind == "link" else None)
    assert builder.project is project and not project.dirty


@pytest.mark.parametrize(
    "layer", ("visuals", "collisions", "frames", "joint_axes", "connectors", "ground")
)
def test_layer_toggles_preserve_actors_camera_and_selection(
    builder: MainWindow, project: StudioProject, monkeypatch: pytest.MonkeyPatch, layer: str
) -> None:
    viewport = builder.viewport
    module = project.pack.hardware_catalog.module_types["smores_ep"]
    link = project.imported_assets[module.asset_ref].links[0].name
    viewport.select_entity("link", link)
    viewport.plotter.camera.azimuth += 17
    camera = _camera(viewport)
    actors = _actors(viewport)
    assert actors[layer]
    _forbid_scene_work(monkeypatch, viewport)
    initial = getattr(viewport, f"_show_{layer}")
    for visible in (not initial, initial):
        draws = viewport.plotter.draws
        viewport.set_layer_visibility(**{layer: visible})
        assert viewport.plotter.draws == draws + 1
        assert all(actor.visibility == visible for actor in actors[layer])
        assert _actors(viewport) == actors
        assert _camera(viewport) == camera
        assert viewport._selected_link == link
        assert all(a.visibility == viewport._show_visuals for a in viewport._link_outlines[link])
        viewport.set_layer_visibility(**{layer: visible})
        assert viewport.plotter.draws == draws + 1  # Setting the same state is a no-op.


def test_connector_edits_refresh_only_overlays_and_keep_hidden_layer_hidden(
    builder: MainWindow, project: StudioProject, monkeypatch: pytest.MonkeyPatch
) -> None:
    viewport = builder.viewport
    module = project.pack.hardware_catalog.module_types["smores_ep"]
    asset = project.imported_assets[module.asset_ref]
    viewport.set_layer_visibility(connectors=False)
    viewport.select_entity("link", asset.links[0].name)
    camera = _camera(viewport)
    actors = _actors(viewport)
    _forbid_scene_work(monkeypatch, viewport)
    connector = module.connectors[0]
    assert connector.local_pose is not None
    moved = connector.model_copy(
        update={"local_pose": connector.local_pose.model_copy(update={"xyz_m": (0.1, 0.2, 0.3)})}
    )
    updated = module.model_copy(update={"connectors": (moved, *module.connectors[1:])})
    draws = viewport.plotter.draws
    viewport.render_module(asset, updated)
    assert viewport.plotter.draws == draws + 1
    assert tuple(viewport._layer_actors["connectors"]) != actors["connectors"]
    assert all(not a.visibility for a in viewport._layer_actors["connectors"])
    for layer, previous in actors.items():
        if layer != "connectors":
            assert tuple(viewport._layer_actors[layer]) == previous
    assert _camera(viewport) == camera
    assert viewport._selected_link == asset.links[0].name
    # Non-geometric metadata edits do not replace any graphics.
    actors = _actors(viewport)
    viewport.render_module(asset, updated.model_copy(update={"name": "Edited module name"}))
    assert _actors(viewport) == actors
    assert viewport.plotter.draws == draws + 1


def test_mesh_cache_copies_before_scaling_and_reloads_after_reimport(
    builder: MainWindow, project: StudioProject, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "shared.stl"
    pv.Cube().save(path)
    geometry = ImportedGeometry(
        kind=GeometryKind.MESH, resolved_mesh_path=path, scale=(2.0, 1.0, 1.0)
    )
    cache: dict[Path, pv.DataSet | None] = {}
    first = viewport_module._geometry_dataset(geometry, mesh_cache=cache)
    second = viewport_module._geometry_dataset(geometry, mesh_cache=cache)
    assert first is not None and second is not None and cache[path] is not None
    assert first.bounds == second.bounds
    assert cache[path].bounds[1] == pytest.approx(0.5)
    assert first.bounds[1] == pytest.approx(1.0)

    module = project.pack.hardware_catalog.module_types["smores_ep"]
    original = project.imported_assets[module.asset_ref]
    link = original.links[0]
    visual = replace(link.visuals[0], geometry=geometry)
    asset = replace(original, links=(replace(link, visuals=(visual,), collisions=()),), joints=())
    viewport = builder.viewport
    viewport.render_module(asset, module)
    old_actors = _actors(viewport)
    pv.Cube(x_length=3.0).save(path)
    reads: list[Path] = []
    read_mesh = viewport_module._read_mesh

    def track_read(mesh_path: Path) -> pv.DataSet | None:
        reads.append(mesh_path)
        return read_mesh(mesh_path)

    monkeypatch.setattr(viewport_module, "_read_mesh", track_read)
    viewport.render_module(asset, module)
    assert reads == []
    viewport.render_module(replace(asset), module)
    assert reads == [path]
    assert _actors(viewport) != old_actors
    assert viewport._mesh_cache[path].bounds[1] == pytest.approx(1.5)


def test_new_module_scene_is_batched_into_one_draw(builder: MainWindow) -> None:
    project = StudioProject.open(
        Path(__file__).resolve().parents[1] / "examples/robot_packs/generic_cube"
    )
    module = project.pack.hardware_catalog.module_types["generic_cube"]
    viewport = builder.viewport
    viewport.set_layer_visibility(visuals=False, collisions=True, frames=True)
    old_actors = _actors(viewport)
    draws = viewport.plotter.draws
    viewport.render_module(project.imported_assets[module.asset_ref], module)
    assert viewport.plotter.draws == draws + 1
    assert _actors(viewport) != old_actors
    assert all(not a.visibility for a in viewport._layer_actors["visuals"])
    assert all(a.visibility for a in viewport._layer_actors["collisions"])
    assert all(a.visibility for a in viewport._layer_actors["frames"])
