# pyright: reportMissingTypeStubs=false, reportUnknownArgumentType=false
# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false
"""Embedded PyVistaQt viewport for imported URDF and Robot Pack overlays."""

from __future__ import annotations

from collections.abc import Generator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

import numpy as np
import pyvista as pv
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QVBoxLayout, QWidget
from pyvistaqt import QtInteractor

from modsim.importers import (
    ImportedGeometry,
    ImportedRobotAsset,
    ImportedVisual,
)
from modsim.importers.urdf import GeometryKind
from modsim.robot_packs import ModuleType
from modsim_studio.appearance import theme_manager
from modsim_studio.visual_colors import categorical_color

_GROUND_SIZE_MULTIPLIER = 16.0
_GROUND_RESOLUTION = 128
_CAMERA_FOOTPRINT_MULTIPLIER = 1.8
_SELECTION_COLOR = "#ffd166"


class RobotViewport(QWidget):
    """Interactive 3D view of one imported module type."""

    entity_selected = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.plotter = QtInteractor(self, auto_update=False)
        layout.addWidget(self.plotter.interactor)
        self._appearance = theme_manager()
        self._ground_actor: Any | None = None
        self._axes_actor: Any = self.plotter.add_axes(color=self._appearance.theme.text)
        self.plotter.enable_anti_aliasing("fxaa")
        self._link_actors: dict[str, list[Any]] = {}
        self._link_outlines: dict[str, list[Any]] = {}
        self._actor_entities: dict[str, tuple[str, str]] = {}
        self._layer_actors: dict[str, list[Any]] = {}
        self._mesh_cache: dict[Path, pv.DataSet | None] = {}
        self._selected_link: str | None = None
        self._asset: ImportedRobotAsset | None = None
        self._module: ModuleType | None = None
        self._link_transforms: dict[str, np.ndarray[Any, Any]] = {}
        self._show_visuals = True
        self._show_collisions = False
        self._show_frames = False
        self._show_joint_axes = False
        self._show_connectors = True
        self._show_ground = True
        self._apply_theme()
        self._appearance.changed.connect(self._apply_theme)

    def _apply_theme(self) -> None:
        theme = self._appearance.theme
        self.plotter.set_background(theme.viewport, top=theme.viewport_top)
        if self._ground_actor is not None:
            self._ground_actor.prop.color = theme.viewport_top
            self._ground_actor.prop.edge_color = theme.grid
        for caption in (
            self._axes_actor.GetXAxisCaptionActor2D(),
            self._axes_actor.GetYAxisCaptionActor2D(),
            self._axes_actor.GetZAxisCaptionActor2D(),
        ):
            caption.GetCaptionTextProperty().SetColor(pv.Color(theme.text).float_rgb)
        self.plotter.render()

    def fit_module(self) -> None:
        """Fit visible module geometry without changing layers or selection."""
        self.plotter.reset_camera(render=False)
        self.plotter.render()

    def render_module(self, asset: ImportedRobotAsset, module: ModuleType) -> None:
        """Load a new mechanical scene, or update an existing module's overlays."""
        if self._asset is asset and self._module is not None and self._module.id == module.id:
            previous = self._module
            self._module = module
            if previous.connectors != module.connectors:
                with self._scene_update():
                    for actor in self._layer_actors.pop("connectors", []):
                        self.plotter.remove_actor(actor, reset_camera=False, render=False)
                    self._add_connectors()
                    self._apply_layer_visibility()
            return
        if self._asset is not asset:
            # A reopened/reimported asset must reload any changed mesh files.
            self._mesh_cache.clear()
        self._asset = asset
        self._module = module
        self._selected_link = None
        self._link_transforms = _link_transforms(asset)
        self._rebuild()

    def set_layer_visibility(
        self,
        *,
        visuals: bool | None = None,
        collisions: bool | None = None,
        frames: bool | None = None,
        joint_axes: bool | None = None,
        connectors: bool | None = None,
        ground: bool | None = None,
    ) -> None:
        """Show/hide retained graphics without reloading meshes or moving the camera."""
        changed = False
        for layer, visible in (
            ("visuals", visuals),
            ("collisions", collisions),
            ("frames", frames),
            ("joint_axes", joint_axes),
            ("connectors", connectors),
            ("ground", ground),
        ):
            attribute = f"_show_{layer}"
            if visible is not None and visible != getattr(self, attribute):
                setattr(self, attribute, visible)
                changed = True
        if changed:
            self._apply_layer_visibility()
            self.plotter.render()

    def select_entity(self, kind: str, entity_id: str) -> None:
        """Highlight a selected link without changing document state."""
        selected_link = entity_id if kind == "link" else None
        if selected_link == self._selected_link:
            return
        self._selected_link = selected_link
        self._apply_selection_highlight()
        self.plotter.render()

    def close(self) -> bool:
        self._appearance.changed.disconnect(self._apply_theme)
        self._release_shadow_resources()
        self.plotter.close()
        self._mesh_cache.clear()
        return super().close()

    @contextmanager
    def _scene_update(self) -> Generator[None, None, None]:
        """Keep intermediate scene changes off screen and draw the result once."""
        suppressed = self.plotter.suppress_rendering
        self.plotter.suppress_rendering = True
        try:
            yield
        finally:
            self.plotter.suppress_rendering = suppressed
            if not suppressed:
                self.plotter.render()

    def _apply_layer_visibility(self) -> None:
        for layer, actors in self._layer_actors.items():
            visible = getattr(self, f"_show_{layer}")
            for actor in actors:
                actor.visibility = visible
        self._apply_selection_highlight()

    def _release_shadow_resources(self) -> None:
        """Free shadow-map GPU resources before the GL context is torn down.

        PyVista's teardown drops its Python references to the shadow render
        passes without calling ``ReleaseGraphicsResources``, so VTK destroys
        ``vtkShadowMapBakerPass`` after the OpenGL context is already gone and
        logs FBO / ShadowMap / LightCamera errors. Releasing explicitly while
        the context is still current avoids that. Best-effort only: this reaches
        into PyVista internals, so any failure must not block window close.
        """
        try:
            render_passes = self.plotter.renderer._render_passes
            shadow_pass = getattr(render_passes, "_shadow_map_pass", None)
            render_window = self.plotter.render_window
        except AttributeError:
            return
        if shadow_pass is None or render_window is None:
            return
        try:
            render_window.MakeCurrent()
            shadow_pass.ReleaseGraphicsResources(render_window)
            shadow_pass.GetShadowMapBakerPass().ReleaseGraphicsResources(render_window)
        except Exception:
            pass

    def _rebuild(self) -> None:
        with self._scene_update():
            self._build_scene()

    def _build_scene(self) -> None:
        self.plotter.disable_picking()
        self._release_shadow_resources()
        self.plotter.disable_shadows()
        self.plotter.clear()
        self._ground_actor = None
        self._axes_actor = self.plotter.add_axes(color=self._appearance.theme.text)
        self._link_actors.clear()
        self._link_outlines.clear()
        self._actor_entities.clear()
        self._layer_actors.clear()
        if self._asset is None or self._module is None:
            self._configure_lighting(None)
            return

        # Prepare each layer once. Later toggles only change actor visibility.
        scene_bounds: list[tuple[float, float, float, float, float, float]] = []
        for link_index, link in enumerate(self._asset.links):
            transform = self._link_transforms.get(link.name, np.eye(4))
            scene_bounds.extend(
                self._add_geometries(
                    link.name,
                    link.visuals,
                    transform,
                    color=_palette_color(link_index),
                    opacity=1.0,
                    layer="visual",
                )
            )
            scene_bounds.extend(
                self._add_geometries(
                    link.name,
                    link.collisions,
                    transform,
                    color="#ef476f",
                    opacity=0.28,
                    layer="collision",
                    style="wireframe",
                )
            )
            self._layer_actors.setdefault("frames", []).extend(
                _add_frame(self.plotter, transform, scale=0.04)
            )

        bounds = _merge_bounds(scene_bounds)
        self._configure_lighting(bounds)
        if bounds is not None:
            self._add_ground(bounds)

        for joint in self._asset.joints:
            if joint.axis is None:
                continue
            transform = self._link_transforms.get(joint.child_link)
            if transform is None:
                continue
            origin = transform[:3, 3]
            direction = transform[:3, :3] @ np.asarray(joint.axis)
            actor = self.plotter.add_arrows(
                np.asarray([origin]),
                np.asarray([direction]),
                mag=0.075,
                color="#f4d35e",
                pickable=False,
                reset_camera=False,
                render=False,
            )
            actor.use_bounds = False
            self._layer_actors.setdefault("joint_axes", []).append(actor)
        self._add_connectors()

        self.plotter.enable_mesh_picking(
            callback=self._picked_actor,
            show=False,
            show_message=False,
            use_actor=True,
            left_clicking=True,
        )
        if bounds is not None:
            self.plotter.enable_shadows()
        self._apply_layer_visibility()
        camera_bounds = _camera_bounds(bounds) if bounds is not None else None
        self.plotter.view_isometric(bounds=camera_bounds, render=False)
        self.plotter.camera.zoom(1.08)

    def _add_connectors(self) -> None:
        if self._module is None:
            return
        actors = self._layer_actors.setdefault("connectors", [])
        for connector in self._module.connectors:
            parent_transform = self._link_transforms.get(connector.parent_link)
            if parent_transform is None or connector.local_pose is None:
                continue
            connector_transform = parent_transform @ _pose_matrix(
                connector.local_pose.xyz_m,
                connector.local_pose.rpy_rad,
            )
            actors.extend(_add_frame(self.plotter, connector_transform, scale=0.055))
            origin = connector_transform[:3, 3]
            for axis, magnitude, color in (
                (connector.docking_axis, 0.1, "#06d6a0"),
                (connector.approach_axis, 0.085, "#118ab2"),
            ):
                if axis is None:
                    continue
                direction = parent_transform[:3, :3] @ np.asarray(axis)
                actor = self.plotter.add_arrows(
                    np.asarray([origin]),
                    np.asarray([direction]),
                    mag=magnitude,
                    color=color,
                    pickable=False,
                    reset_camera=False,
                    render=False,
                )
                actor.use_bounds = False
                actors.append(actor)

    def _add_geometries(
        self,
        link_name: str,
        geometries: tuple[ImportedVisual, ...],
        link_transform: np.ndarray[Any, Any],
        *,
        color: str,
        opacity: float,
        layer: str,
        style: str = "surface",
    ) -> list[tuple[float, float, float, float, float, float]]:
        bounds: list[tuple[float, float, float, float, float, float]] = []
        for index, visual in enumerate(geometries):
            dataset = _geometry_dataset(visual.geometry, mesh_cache=self._mesh_cache)
            if dataset is None:
                continue
            transform = link_transform @ _pose_matrix(
                visual.origin_xyz_m,
                visual.origin_rpy_rad,
            )
            dataset.transform(transform, inplace=True)
            dataset_bounds = dataset.bounds
            bounds.append(
                (
                    float(dataset_bounds[0]),
                    float(dataset_bounds[1]),
                    float(dataset_bounds[2]),
                    float(dataset_bounds[3]),
                    float(dataset_bounds[4]),
                    float(dataset_bounds[5]),
                )
            )
            actor_name = f"{layer}:{link_name}:{index}"
            actor_color, actor_opacity = _visual_appearance(visual, color, opacity)
            if style == "surface":
                actor = self.plotter.add_mesh(
                    dataset,
                    name=actor_name,
                    color=actor_color,
                    opacity=actor_opacity,
                    style=style,
                    pickable=True,
                    smooth_shading=True,
                    split_sharp_edges=True,
                    pbr=True,
                    metallic=0.04,
                    roughness=0.56,
                    reset_camera=False,
                    render=False,
                )
            else:
                actor = self.plotter.add_mesh(
                    dataset,
                    name=actor_name,
                    color=color,
                    opacity=opacity,
                    style=style,
                    pickable=True,
                    line_width=1.5,
                    reset_camera=False,
                    render=False,
                    lighting=False,
                )
            self._link_actors.setdefault(link_name, []).append(actor)
            layer_key = "visuals" if layer == "visual" else "collisions"
            self._layer_actors.setdefault(layer_key, []).append(actor)
            self._actor_entities[actor_name] = ("link", link_name)
            if style == "surface":
                outline_actor = self.plotter.add_mesh(
                    dataset.outline(),
                    name=f"selection:{link_name}:{index}",
                    color=_SELECTION_COLOR,
                    line_width=3.0,
                    render_lines_as_tubes=True,
                    lighting=False,
                    pickable=False,
                    reset_camera=False,
                    render=False,
                )
                outline_actor.use_bounds = False
                outline_actor.visibility = False
                self._link_outlines.setdefault(link_name, []).append(outline_actor)
        return bounds

    def _picked_actor(self, actor: Any) -> None:
        actor_name = str(getattr(actor, "name", ""))
        entity = self._actor_entities.get(actor_name)
        if entity is not None:
            self.entity_selected.emit(*entity)

    def _apply_selection_highlight(self) -> None:
        for link_name, actors in self._link_outlines.items():
            selected = self._show_visuals and link_name == self._selected_link
            for actor in actors:
                actor.visibility = selected

    def _configure_lighting(
        self,
        bounds: tuple[float, float, float, float, float, float] | None,
    ) -> None:
        self.plotter.remove_all_lights()
        if bounds is None:
            self.plotter.add_light(pv.Light(light_type="headlight", intensity=0.8, color="#ffffff"))
            return
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        center = (
            (x_min + x_max) / 2.0,
            (y_min + y_max) / 2.0,
            (z_min + z_max) / 2.0,
        )
        scale = max(x_max - x_min, y_max - y_min, z_max - z_min, 0.1)
        self.plotter.add_light(
            pv.Light(
                position=(center[0] + scale, center[1] - scale, center[2] + scale * 1.5),
                focal_point=center,
                color="#fff3df",
                light_type="scene light",
                intensity=0.9,
            )
        )
        self.plotter.add_light(
            pv.Light(
                position=(center[0] - scale, center[1] + scale, center[2] + scale * 0.5),
                focal_point=center,
                color="#dbe9ff",
                light_type="scene light",
                intensity=0.45,
            )
        )
        self.plotter.add_light(pv.Light(light_type="headlight", intensity=0.25, color="#ffffff"))

    def _add_ground(self, bounds: tuple[float, float, float, float, float, float]) -> None:
        x_min, x_max, y_min, y_max, z_min, _z_max = bounds
        x_size = x_max - x_min
        y_size = y_max - y_min
        size = max(x_size, y_size, 0.1)
        plane = pv.Plane(
            center=(
                (x_min + x_max) / 2.0,
                (y_min + y_max) / 2.0,
                z_min - max(size * 0.015, 0.0005),
            ),
            direction=(0.0, 0.0, 1.0),
            i_size=size * _GROUND_SIZE_MULTIPLIER,
            j_size=size * _GROUND_SIZE_MULTIPLIER,
            i_resolution=_GROUND_RESOLUTION,
            j_resolution=_GROUND_RESOLUTION,
        )
        ground_actor = self.plotter.add_mesh(
            plane,
            name="viewport:ground",
            color=self._appearance.theme.viewport_top,
            show_edges=True,
            edge_color=self._appearance.theme.grid,
            edge_opacity=0.55,
            line_width=1.0,
            pickable=False,
            pbr=True,
            metallic=0.0,
            roughness=0.92,
            reset_camera=False,
            render=False,
        )
        # Exclude the decorative floor from automatic camera fitting. This is
        # also needed when Qt performs its first render after the window opens.
        ground_actor.use_bounds = False
        self._ground_actor = ground_actor
        self._layer_actors.setdefault("ground", []).append(ground_actor)


def _visual_appearance(
    visual: ImportedVisual,
    fallback_color: str,
    fallback_opacity: float,
) -> tuple[str | tuple[float, float, float], float]:
    material = visual.material
    if material is None or material.color_rgba is None:
        return fallback_color, fallback_opacity
    red, green, blue, alpha = material.color_rgba
    return (red, green, blue), fallback_opacity * alpha


def _merge_bounds(
    bounds: list[tuple[float, float, float, float, float, float]],
) -> tuple[float, float, float, float, float, float] | None:
    if not bounds:
        return None
    return (
        min(item[0] for item in bounds),
        max(item[1] for item in bounds),
        min(item[2] for item in bounds),
        max(item[3] for item in bounds),
        min(item[4] for item in bounds),
        max(item[5] for item in bounds),
    )


def _camera_bounds(
    bounds: tuple[float, float, float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    """Pad robot-derived bounds without considering decorative scene actors."""
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    size = max(x_max - x_min, y_max - y_min, 0.1)
    half_footprint = size * _CAMERA_FOOTPRINT_MULTIPLIER / 2.0
    x_center = (x_min + x_max) / 2.0
    y_center = (y_min + y_max) / 2.0
    return (
        x_center - half_footprint,
        x_center + half_footprint,
        y_center - half_footprint,
        y_center + half_footprint,
        z_min - max(size * 0.015, 0.0005),
        z_max,
    )


def _geometry_dataset(
    geometry: ImportedGeometry, *, mesh_cache: dict[Path, pv.DataSet | None] | None = None
) -> pv.DataSet | None:
    if geometry.kind is GeometryKind.BOX and geometry.size_m is not None:
        x_size, y_size, z_size = geometry.size_m
        return pv.Box(
            bounds=(
                -x_size / 2,
                x_size / 2,
                -y_size / 2,
                y_size / 2,
                -z_size / 2,
                z_size / 2,
            )
        )
    if geometry.kind is GeometryKind.CYLINDER:
        if geometry.radius_m is None or geometry.length_m is None:
            return None
        return pv.Cylinder(
            center=(0.0, 0.0, 0.0),
            direction=(0.0, 0.0, 1.0),
            radius=geometry.radius_m,
            height=geometry.length_m,
            resolution=48,
        )
    if geometry.kind is GeometryKind.SPHERE and geometry.radius_m is not None:
        return pv.Sphere(radius=geometry.radius_m)
    if geometry.kind is GeometryKind.MESH and geometry.resolved_mesh_path is not None:
        path = geometry.resolved_mesh_path
        if mesh_cache is None:
            dataset = _read_mesh(path)
        else:
            if path not in mesh_cache:
                mesh_cache[path] = _read_mesh(path)
            cached = mesh_cache[path]
            # Scaling/placement below must never mutate the shared source mesh.
            dataset = None if cached is None else cached.copy(deep=True)
        if dataset is None:
            return None
        dataset.scale(geometry.scale, inplace=True)
        return dataset
    return None


def _read_mesh(path: Path) -> pv.DataSet | None:
    loaded = pv.read(path)
    if isinstance(loaded, pv.MultiBlock):
        combined = loaded.combine()
        return combined if combined.n_points else None
    return loaded


def _link_transforms(
    asset: ImportedRobotAsset,
) -> dict[str, np.ndarray[Any, Any]]:
    transforms: dict[str, np.ndarray[Any, Any]] = {root: np.eye(4) for root in asset.root_links}
    remaining = list(asset.joints)
    while remaining:
        next_remaining = []
        progress = False
        for joint in remaining:
            parent = transforms.get(joint.parent_link)
            if parent is None:
                next_remaining.append(joint)
                continue
            transforms[joint.child_link] = parent @ _pose_matrix(
                joint.origin_xyz_m,
                joint.origin_rpy_rad,
            )
            progress = True
        if not progress:
            break
        remaining = next_remaining
    return transforms


def _pose_matrix(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> np.ndarray[Any, Any]:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation = np.asarray(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=float,
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz
    return transform


def _add_frame(
    plotter: QtInteractor,
    transform: np.ndarray[Any, Any],
    *,
    scale: float,
) -> list[Any]:
    actors: list[Any] = []
    origin = transform[:3, 3]
    colors = ("#ff5252", "#69f0ae", "#448aff")
    for column, color in enumerate(colors):
        direction = transform[:3, column]
        actor = plotter.add_arrows(
            np.asarray([origin]),
            np.asarray([direction]),
            mag=scale,
            color=color,
            pickable=False,
            reset_camera=False,
            render=False,
        )
        actor.use_bounds = False
        actors.append(actor)
    return actors


def _palette_color(index: int) -> str:
    # Fallback materials stay fixed when the UI theme changes, like URDF materials.
    return categorical_color(index, "graphite")
