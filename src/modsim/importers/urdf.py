"""Typed, dependency-light URDF import for Robot Pack authoring."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree

_MAX_URDF_BYTES = 32 * 1024 * 1024


class URDFImportError(ValueError):
    """Raised when a URDF cannot be imported safely."""


class GeometryKind(StrEnum):
    """URDF visual or collision geometry categories."""

    MESH = "mesh"
    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"


@dataclass(frozen=True, slots=True)
class ImportedGeometry:
    """One typed URDF geometry declaration."""

    kind: GeometryKind
    mesh_filename: str | None = None
    resolved_mesh_path: Path | None = None
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    size_m: tuple[float, float, float] | None = None
    radius_m: float | None = None
    length_m: float | None = None


@dataclass(frozen=True, slots=True)
class ImportedVisual:
    """Geometry attached to a link at a local pose."""

    name: str | None
    origin_xyz_m: tuple[float, float, float]
    origin_rpy_rad: tuple[float, float, float]
    geometry: ImportedGeometry


@dataclass(frozen=True, slots=True)
class ImportedLink:
    """Imported URDF link data used by authoring and visualization."""

    name: str
    visuals: tuple[ImportedVisual, ...]
    collisions: tuple[ImportedVisual, ...]
    mass_kg: float | None


@dataclass(frozen=True, slots=True)
class ImportedJointLimits:
    """Raw scalar limits from a URDF joint."""

    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None


@dataclass(frozen=True, slots=True)
class ImportedJoint:
    """Imported URDF joint and its zero-configuration transform."""

    name: str
    type: str
    parent_link: str
    child_link: str
    origin_xyz_m: tuple[float, float, float]
    origin_rpy_rad: tuple[float, float, float]
    axis: tuple[float, float, float] | None
    limits: ImportedJointLimits | None


@dataclass(frozen=True, slots=True)
class ImportedRobotAsset:
    """A parsed URDF with explicit filesystem context."""

    source_path: Path
    name: str
    links: tuple[ImportedLink, ...]
    joints: tuple[ImportedJoint, ...]
    root_links: tuple[str, ...]
    warnings: tuple[str, ...]

    def link(self, name: str) -> ImportedLink:
        """Return a link by source name."""
        for link in self.links:
            if link.name == name:
                return link
        raise KeyError(name)


class URDFImporter:
    """Parse the URDF subset needed for Robot Pack authoring.

    The importer intentionally does not expand Xacro or fetch network assets.
    """

    def load(
        self,
        path: str | Path,
        *,
        asset_roots: tuple[str | Path, ...] = (),
    ) -> ImportedRobotAsset:
        """Load a local URDF and resolve local mesh references where possible."""
        source_path = Path(os.path.abspath(Path(path)))
        if source_path.is_symlink():
            raise URDFImportError(f"URDF must not be a symbolic link: {source_path}")
        if not source_path.is_file():
            raise URDFImportError(f"URDF does not exist or is not a file: {source_path}")
        try:
            size = source_path.stat().st_size
        except OSError as error:
            raise URDFImportError(f"cannot inspect URDF: {error}") from error
        if size > _MAX_URDF_BYTES:
            raise URDFImportError(
                f"URDF exceeds the {_MAX_URDF_BYTES // (1024 * 1024)} MiB authoring limit"
            )
        try:
            content = source_path.read_bytes()
        except OSError as error:
            raise URDFImportError(f"cannot read URDF: {error}") from error
        upper_prefix = content[:4096].upper()
        if b"<!DOCTYPE" in upper_prefix or b"<!ENTITY" in upper_prefix:
            raise URDFImportError("URDF document type and entity declarations are not supported")
        try:
            root = ElementTree.fromstring(content)
        except ElementTree.ParseError as error:
            raise URDFImportError(f"invalid URDF XML: {error}") from error
        if self._local_name(root.tag) != "robot":
            raise URDFImportError("URDF root element must be <robot>")

        resolved_roots = tuple(Path(os.path.abspath(Path(root_path))) for root_path in asset_roots)
        warnings: list[str] = []
        links = tuple(
            self._parse_link(
                element,
                source_path=source_path,
                asset_roots=resolved_roots,
                warnings=warnings,
            )
            for element in self._children(root, "link")
        )
        joints = tuple(self._parse_joint(element) for element in self._children(root, "joint"))
        if not links:
            raise URDFImportError("URDF must contain at least one link")

        link_names = [link.name for link in links]
        if len(set(link_names)) != len(link_names):
            raise URDFImportError("URDF link names must be unique")
        joint_names = [joint.name for joint in joints]
        if len(set(joint_names)) != len(joint_names):
            raise URDFImportError("URDF joint names must be unique")
        known_links = set(link_names)
        child_links: set[str] = set()
        for joint in joints:
            if joint.parent_link not in known_links:
                raise URDFImportError(
                    f"joint '{joint.name}' references unknown parent link '{joint.parent_link}'"
                )
            if joint.child_link not in known_links:
                raise URDFImportError(
                    f"joint '{joint.name}' references unknown child link '{joint.child_link}'"
                )
            if joint.child_link in child_links:
                raise URDFImportError(f"link '{joint.child_link}' has more than one parent joint")
            child_links.add(joint.child_link)
        root_links = tuple(name for name in link_names if name not in child_links)
        if not root_links:
            raise URDFImportError("URDF joint graph has no root link")
        if len(root_links) > 1:
            warnings.append("URDF has multiple root links: " + ", ".join(root_links))
        return ImportedRobotAsset(
            source_path=source_path,
            name=root.attrib.get("name", "").strip() or source_path.stem,
            links=links,
            joints=joints,
            root_links=root_links,
            warnings=tuple(warnings),
        )

    def _parse_link(
        self,
        element: ElementTree.Element,
        *,
        source_path: Path,
        asset_roots: tuple[Path, ...],
        warnings: list[str],
    ) -> ImportedLink:
        name = self._required_attribute(element, "name", context="link")
        visuals = tuple(
            self._parse_visual(
                visual,
                source_path=source_path,
                asset_roots=asset_roots,
                warnings=warnings,
                context=f"link '{name}' visual",
            )
            for visual in self._children(element, "visual")
        )
        collisions = tuple(
            self._parse_visual(
                collision,
                source_path=source_path,
                asset_roots=asset_roots,
                warnings=warnings,
                context=f"link '{name}' collision",
            )
            for collision in self._children(element, "collision")
        )
        mass_kg: float | None = None
        inertial = self._child(element, "inertial")
        if inertial is not None:
            mass = self._child(inertial, "mass")
            if mass is not None and mass.attrib.get("value"):
                mass_kg = self._float(mass.attrib["value"], context=f"link '{name}' mass")
                if mass_kg <= 0.0:
                    raise URDFImportError(f"link '{name}' mass must be positive")
        return ImportedLink(
            name=name,
            visuals=visuals,
            collisions=collisions,
            mass_kg=mass_kg,
        )

    def _parse_visual(
        self,
        element: ElementTree.Element,
        *,
        source_path: Path,
        asset_roots: tuple[Path, ...],
        warnings: list[str],
        context: str,
    ) -> ImportedVisual:
        xyz, rpy = self._parse_origin(element, context=context)
        geometry_element = self._child(element, "geometry")
        if geometry_element is None:
            raise URDFImportError(f"{context} is missing <geometry>")
        geometry_children = list(geometry_element)
        if len(geometry_children) != 1:
            raise URDFImportError(f"{context} geometry must contain exactly one shape")
        shape = geometry_children[0]
        kind = self._local_name(shape.tag)
        if kind == GeometryKind.MESH:
            filename = self._required_attribute(shape, "filename", context=f"{context} mesh")
            scale = self._vector(
                shape.attrib.get("scale", "1 1 1"),
                size=3,
                context=f"{context} mesh scale",
            )
            resolved = self._resolve_mesh(
                filename,
                urdf_path=source_path,
                asset_roots=asset_roots,
            )
            if resolved is None:
                warnings.append(f"{context}: mesh could not be resolved: {filename}")
            geometry = ImportedGeometry(
                kind=GeometryKind.MESH,
                mesh_filename=filename,
                resolved_mesh_path=resolved,
                scale=scale,
            )
        elif kind == GeometryKind.BOX:
            size = self._vector(
                self._required_attribute(shape, "size", context=f"{context} box"),
                size=3,
                context=f"{context} box size",
            )
            if any(component <= 0.0 for component in size):
                raise URDFImportError(f"{context} box dimensions must be positive")
            geometry = ImportedGeometry(kind=GeometryKind.BOX, size_m=size)
        elif kind == GeometryKind.CYLINDER:
            radius = self._positive_float_attribute(shape, "radius", context=context)
            length = self._positive_float_attribute(shape, "length", context=context)
            geometry = ImportedGeometry(
                kind=GeometryKind.CYLINDER,
                radius_m=radius,
                length_m=length,
            )
        elif kind == GeometryKind.SPHERE:
            radius = self._positive_float_attribute(shape, "radius", context=context)
            geometry = ImportedGeometry(kind=GeometryKind.SPHERE, radius_m=radius)
        else:
            raise URDFImportError(f"{context} uses unsupported geometry <{kind}>")
        return ImportedVisual(
            name=element.attrib.get("name"),
            origin_xyz_m=xyz,
            origin_rpy_rad=rpy,
            geometry=geometry,
        )

    def _parse_joint(self, element: ElementTree.Element) -> ImportedJoint:
        name = self._required_attribute(element, "name", context="joint")
        joint_type = self._required_attribute(element, "type", context=f"joint '{name}'")
        supported = {"fixed", "revolute", "continuous", "prismatic", "floating", "planar"}
        if joint_type not in supported:
            raise URDFImportError(f"joint '{name}' uses unsupported type '{joint_type}'")
        parent_element = self._child(element, "parent")
        child_element = self._child(element, "child")
        if parent_element is None or child_element is None:
            raise URDFImportError(f"joint '{name}' requires parent and child links")
        parent = self._required_attribute(parent_element, "link", context=f"joint '{name}' parent")
        child = self._required_attribute(child_element, "link", context=f"joint '{name}' child")
        xyz, rpy = self._parse_origin(element, context=f"joint '{name}'")

        axis: tuple[float, float, float] | None = None
        if joint_type not in {"fixed", "floating"}:
            axis_element = self._child(element, "axis")
            raw_axis = (
                axis_element.attrib.get("xyz", "1 0 0") if axis_element is not None else "1 0 0"
            )
            parsed_axis = self._vector(raw_axis, size=3, context=f"joint '{name}' axis")
            magnitude = math.sqrt(sum(component * component for component in parsed_axis))
            if magnitude <= 1e-12:
                raise URDFImportError(f"joint '{name}' axis must not be zero")
            axis = (
                parsed_axis[0] / magnitude,
                parsed_axis[1] / magnitude,
                parsed_axis[2] / magnitude,
            )

        limit_element = self._child(element, "limit")
        limits: ImportedJointLimits | None = None
        if limit_element is not None:
            limits = ImportedJointLimits(
                lower=self._optional_float_attribute(limit_element, "lower", context=name),
                upper=self._optional_float_attribute(limit_element, "upper", context=name),
                effort=self._optional_float_attribute(limit_element, "effort", context=name),
                velocity=self._optional_float_attribute(limit_element, "velocity", context=name),
            )
        return ImportedJoint(
            name=name,
            type=joint_type,
            parent_link=parent,
            child_link=child,
            origin_xyz_m=xyz,
            origin_rpy_rad=rpy,
            axis=axis,
            limits=limits,
        )

    @classmethod
    def _resolve_mesh(
        cls,
        filename: str,
        *,
        urdf_path: Path,
        asset_roots: tuple[Path, ...],
    ) -> Path | None:
        parsed = urlparse(filename)
        candidates: list[Path] = []
        if parsed.scheme in {"http", "https"}:
            return None
        if parsed.scheme == "package":
            relative = Path(unquote(parsed.netloc + parsed.path))
            package_relative = Path(*relative.parts[1:]) if len(relative.parts) > 1 else relative
            for root in asset_roots:
                candidates.extend((root / relative, root / package_relative))
            candidates.extend(
                (
                    urdf_path.parent / relative,
                    urdf_path.parent / package_relative,
                )
            )
        elif parsed.scheme == "file":
            candidates.append(Path(unquote(parsed.path)))
        elif parsed.scheme:
            return None
        else:
            raw_path = Path(unquote(filename))
            if raw_path.is_absolute():
                candidates.append(raw_path)
            else:
                candidates.append(urdf_path.parent / raw_path)
                candidates.extend(root / raw_path for root in asset_roots)
        for candidate in candidates:
            try:
                resolved = candidate.resolve(strict=True)
            except (OSError, RuntimeError):
                continue
            if resolved.is_file() and not candidate.is_symlink():
                return resolved
        return None

    @classmethod
    def _parse_origin(
        cls,
        element: ElementTree.Element,
        *,
        context: str,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        origin = cls._child(element, "origin")
        if origin is None:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        xyz = cls._vector(
            origin.attrib.get("xyz", "0 0 0"),
            size=3,
            context=f"{context} origin xyz",
        )
        rpy = cls._vector(
            origin.attrib.get("rpy", "0 0 0"),
            size=3,
            context=f"{context} origin rpy",
        )
        return xyz, rpy

    @staticmethod
    def _required_attribute(
        element: ElementTree.Element,
        name: str,
        *,
        context: str,
    ) -> str:
        value = element.attrib.get(name, "").strip()
        if not value:
            raise URDFImportError(f"{context} requires attribute '{name}'")
        return value

    @classmethod
    def _positive_float_attribute(
        cls,
        element: ElementTree.Element,
        name: str,
        *,
        context: str,
    ) -> float:
        value = cls._float(
            cls._required_attribute(element, name, context=context),
            context=f"{context} {name}",
        )
        if value <= 0.0:
            raise URDFImportError(f"{context} {name} must be positive")
        return value

    @classmethod
    def _optional_float_attribute(
        cls,
        element: ElementTree.Element,
        name: str,
        *,
        context: str,
    ) -> float | None:
        raw_value = element.attrib.get(name)
        if raw_value is None:
            return None
        return cls._float(raw_value, context=f"joint '{context}' limit {name}")

    @staticmethod
    def _float(value: str, *, context: str) -> float:
        try:
            result = float(value)
        except ValueError as error:
            raise URDFImportError(f"{context} must be numeric") from error
        if not math.isfinite(result):
            raise URDFImportError(f"{context} must be finite")
        return result

    @classmethod
    def _vector(
        cls,
        value: str,
        *,
        size: int,
        context: str,
    ) -> tuple[float, float, float]:
        parts = value.replace(",", " ").split()
        if len(parts) != size:
            raise URDFImportError(f"{context} must have {size} components")
        parsed = tuple(cls._float(part, context=context) for part in parts)
        return parsed[0], parsed[1], parsed[2]

    @staticmethod
    def _local_name(tag: str) -> str:
        return tag.rsplit("}", maxsplit=1)[-1]

    @classmethod
    def _children(
        cls,
        element: ElementTree.Element,
        name: str,
    ) -> tuple[ElementTree.Element, ...]:
        return tuple(child for child in element if cls._local_name(child.tag) == name)

    @classmethod
    def _child(
        cls,
        element: ElementTree.Element,
        name: str,
    ) -> ElementTree.Element | None:
        return next(
            (child for child in element if cls._local_name(child.tag) == name),
            None,
        )
