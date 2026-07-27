# pyright: reportMissingTypeStubs=false, reportUnknownArgumentType=false
# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false
"""Embedded PyVistaQt viewport for imported URDF and Robot Pack overlays."""

from __future__ import annotations

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


class RobotViewport(QWidget):
    """Interactive 3D view of one imported module type."""

    entity_selected = Signal(str, str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)
        self.plotter.set_background("#20252b")
        self.plotter.add_axes()
        self.plotter.enable_anti_aliasing("fxaa")
        self._link_actors: dict[str, list[Any]] = {}
        self._actor_entities: dict[str, tuple[str, str]] = {}
        self._asset: ImportedRobotAsset | None = None
        self._module: ModuleType | None = None
        self._link_transforms: dict[str, np.ndarray[Any, Any]] = {}
        self._show_visuals = True
        self._show_collisions = False
        self._show_frames = True
        self._show_joint_axes = True
        self._show_connectors = True

    def render_module(self, asset: ImportedRobotAsset, module: ModuleType) -> None:
        """Replace the scene with a module at its zero joint configuration."""
        self._asset = asset
        self._module = module
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
    ) -> None:
        """Update viewport overlay visibility and redraw."""
        if visuals is not None:
            self._show_visuals = visuals
        if collisions is not None:
            self._show_collisions = collisions
        if frames is not None:
            self._show_frames = frames
        if joint_axes is not None:
            self._show_joint_axes = joint_axes
        if connectors is not None:
            self._show_connectors = connectors
        self._rebuild()

    def select_entity(self, kind: str, entity_id: str) -> None:
        """Highlight a selected link without changing document state."""
        if kind != "link":
            self._reset_actor_colors()
            self.plotter.render()
            return
        self._reset_actor_colors()
        for actor in self._link_actors.get(entity_id, []):
            actor.prop.color = "#ffd166"
        self.plotter.render()

    def close(self) -> bool:
        self.plotter.close()
        return super().close()

    def _rebuild(self) -> None:
        self.plotter.disable_picking()
        self.plotter.clear()
        self.plotter.add_axes()
        self._link_actors.clear()
        self._actor_entities.clear()
        if self._asset is None or self._module is None:
            self.plotter.render()
            return

        for link_index, link in enumerate(self._asset.links):
            transform = self._link_transforms.get(link.name, np.eye(4))
            color = _palette_color(link_index)
            if self._show_visuals:
                self._add_geometries(
                    link.name,
                    link.visuals,
                    transform,
                    color=color,
                    opacity=1.0,
                    layer="visual",
                )
            if self._show_collisions:
                self._add_geometries(
                    link.name,
                    link.collisions,
                    transform,
                    color="#ef476f",
                    opacity=0.28,
                    layer="collision",
                    style="wireframe",
                )
            if self._show_frames:
                _add_frame(self.plotter, transform, scale=0.04)

        if self._show_joint_axes:
            for joint in self._asset.joints:
                if joint.axis is None:
                    continue
                transform = self._link_transforms.get(joint.child_link)
                if transform is None:
                    continue
                origin = transform[:3, 3]
                direction = transform[:3, :3] @ np.asarray(joint.axis)
                self.plotter.add_arrows(
                    np.asarray([origin]),
                    np.asarray([direction]),
                    mag=0.075,
                    color="#f4d35e",
                )

        if self._show_connectors:
            for connector in self._module.connectors:
                parent_transform = self._link_transforms.get(connector.parent_link)
                if parent_transform is None or connector.local_pose is None:
                    continue
                connector_transform = parent_transform @ _pose_matrix(
                    connector.local_pose.xyz_m,
                    connector.local_pose.rpy_rad,
                )
                _add_frame(self.plotter, connector_transform, scale=0.055)
                origin = connector_transform[:3, 3]
                if connector.docking_axis is not None:
                    direction = parent_transform[:3, :3] @ np.asarray(connector.docking_axis)
                    self.plotter.add_arrows(
                        np.asarray([origin]),
                        np.asarray([direction]),
                        mag=0.1,
                        color="#06d6a0",
                    )
                if connector.approach_axis is not None:
                    direction = parent_transform[:3, :3] @ np.asarray(connector.approach_axis)
                    self.plotter.add_arrows(
                        np.asarray([origin]),
                        np.asarray([direction]),
                        mag=0.085,
                        color="#118ab2",
                    )

        self.plotter.enable_mesh_picking(
            callback=self._picked_actor,
            show=False,
            show_message=False,
            use_actor=True,
            left_clicking=True,
        )
        self.plotter.reset_camera()
        self.plotter.render()

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
    ) -> None:
        for index, visual in enumerate(geometries):
            dataset = _geometry_dataset(visual.geometry)
            if dataset is None:
                continue
            transform = link_transform @ _pose_matrix(
                visual.origin_xyz_m,
                visual.origin_rpy_rad,
            )
            dataset.transform(transform, inplace=True)
            actor_name = f"{layer}:{link_name}:{index}"
            actor = self.plotter.add_mesh(
                dataset,
                name=actor_name,
                color=color,
                opacity=opacity,
                style=style,
                pickable=True,
            )
            self._link_actors.setdefault(link_name, []).append(actor)
            self._actor_entities[actor_name] = ("link", link_name)

    def _picked_actor(self, actor: Any) -> None:
        actor_name = str(getattr(actor, "name", ""))
        entity = self._actor_entities.get(actor_name)
        if entity is not None:
            self.entity_selected.emit(*entity)

    def _reset_actor_colors(self) -> None:
        if self._asset is None:
            return
        colors = {link.name: _palette_color(index) for index, link in enumerate(self._asset.links)}
        for link_name, actors in self._link_actors.items():
            for actor in actors:
                actor.prop.color = colors.get(link_name, "#90caf9")


def _geometry_dataset(geometry: ImportedGeometry) -> pv.DataSet | None:
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
        dataset = _read_mesh(geometry.resolved_mesh_path)
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
) -> None:
    origin = transform[:3, 3]
    colors = ("#ff5252", "#69f0ae", "#448aff")
    for column, color in enumerate(colors):
        direction = transform[:3, column]
        plotter.add_arrows(
            np.asarray([origin]),
            np.asarray([direction]),
            mag=scale,
            color=color,
        )


def _palette_color(index: int) -> str:
    palette = (
        "#90caf9",
        "#ce93d8",
        "#80cbc4",
        "#ffcc80",
        "#a5d6a7",
        "#ef9a9a",
    )
    return palette[index % len(palette)]
