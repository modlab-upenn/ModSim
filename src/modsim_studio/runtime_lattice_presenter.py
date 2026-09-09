"""Qt-free projection geometry for cubic-lattice Runtime Inspector views.

The model-view builder owns quantization and semantic diagnostics.  This
module owns only bounded, renderer-neutral 2-D geometry.  Keeping projection
here makes the same immutable presentation usable by Qt today and another
frontend later without importing a GUI toolkit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import Literal

from modsim.core.transforms import (
    Quat,
    Vec3,
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    quat_rotate,
    vec_dot,
    vec_normalize,
)
from modsim.model_views import CubicLatticeNode, CubicLatticeView, LatticeFace

Point2D = tuple[float, float]
Point3D = tuple[float, float, float]
Segment2D = tuple[Point2D, Point2D]
AxisName = Literal["x", "y", "z"]

_SQRT_THREE = math.sqrt(3.0)
_MAX_GRID_INTERVALS = 48
_CUBE_HALF_EXTENT = 0.39
_SNAP_HALF_EXTENT = 0.48
_FACE_OFFSET = 0.5
_ORIENTATION_AXIS_LENGTH = 0.28
_ZERO_TOLERANCE = 1e-10
_DEFAULT_AZIMUTH_RAD = math.pi / 4.0
_DEFAULT_ELEVATION_RAD = math.asin(1.0 / _SQRT_THREE)
_MAX_ORBIT_ELEVATION_RAD = math.radians(85.0)
_ORBIT_PROJECTION_SCALE = math.sqrt(3.0 / 2.0)


class LatticeProjection(StrEnum):
    """Supported orthographic projections for a cubic-lattice view."""

    ISOMETRIC = "isometric"
    XY = "xy"
    XZ = "xz"
    YZ = "yz"


@dataclass(frozen=True, slots=True)
class LatticeCamera:
    """Renderer-owned orbit camera used by the isometric projection."""

    azimuth_rad: float = _DEFAULT_AZIMUTH_RAD
    elevation_rad: float = _DEFAULT_ELEVATION_RAD

    def __post_init__(self) -> None:
        if not math.isfinite(self.azimuth_rad) or not math.isfinite(self.elevation_rad):
            raise ValueError("lattice camera angles must be finite")
        if abs(self.elevation_rad) > _MAX_ORBIT_ELEVATION_RAD:
            raise ValueError("lattice camera elevation must stay between -85 and 85 degrees")

    def orbited(self, azimuth_delta_rad: float, elevation_delta_rad: float) -> LatticeCamera:
        """Return a yawed/pitched camera without permitting a pole flip."""
        if not math.isfinite(azimuth_delta_rad) or not math.isfinite(elevation_delta_rad):
            raise ValueError("lattice orbit deltas must be finite")
        azimuth = math.remainder(self.azimuth_rad + azimuth_delta_rad, 2.0 * math.pi)
        elevation = min(
            max(
                self.elevation_rad + elevation_delta_rad,
                -_MAX_ORBIT_ELEVATION_RAD,
            ),
            _MAX_ORBIT_ELEVATION_RAD,
        )
        return LatticeCamera(azimuth_rad=azimuth, elevation_rad=elevation)


DEFAULT_LATTICE_CAMERA = LatticeCamera()


@dataclass(frozen=True, slots=True)
class PresentedGridLine:
    """One finite lattice-grid segment."""

    start: Point2D
    end: Point2D
    axis: AxisName
    major: bool = False


@dataclass(frozen=True, slots=True)
class PresentedLatticeAxis:
    """One labelled global lattice-axis arrow."""

    axis: AxisName
    start: Point2D
    end: Point2D


@dataclass(frozen=True, slots=True)
class PresentedOrientationAxis:
    """One projected measured local-axis indicator for a module."""

    axis: AxisName
    start: Point2D
    end: Point2D


@dataclass(frozen=True, slots=True)
class PresentedLatticeFace:
    """One visible face of a measured module cube."""

    points: tuple[Point2D, Point2D, Point2D, Point2D]
    depth: float
    brightness: float


@dataclass(frozen=True, slots=True)
class PresentedLatticeCell:
    """One occupied nearest lattice cell and its ghost-cube wireframe."""

    id: str
    cell: tuple[int, int, int]
    center: Point2D
    outline_segments: tuple[Segment2D, ...]
    occupant_ids: tuple[str, ...]
    occupancy_conflict: bool
    selected: bool = False


@dataclass(frozen=True, slots=True)
class PresentedLatticeModule:
    """One measured module pose projected independently of its snap cell."""

    id: str
    label: str
    module_type_id: str
    assembly_id: str
    cell: tuple[int, int, int]
    measured_lattice_position: Point3D
    center: Point2D
    faces: tuple[PresentedLatticeFace, ...]
    orientation_axes: tuple[PresentedOrientationAxis, ...]
    tether: Segment2D | None
    position_residual_m: float
    orientation_residual_rad: float
    off_lattice: bool
    occupancy_conflict: bool
    selected: bool = False
    outline_segments: tuple[Segment2D, ...] = ()


@dataclass(frozen=True, slots=True)
class PresentedLatticeConnection:
    """One face-labelled connection and always-visible docking marker."""

    id: str
    source: str
    target: str
    connector_a: str
    connector_b: str
    source_face: LatticeFace
    target_face: LatticeFace
    source_face_residual_rad: float
    target_face_residual_rad: float
    path: tuple[Point2D, ...]
    marker: Point2D
    selected: bool = False


@dataclass(frozen=True, slots=True)
class CubicLatticeGeometry:
    """Complete finite geometry generated from one cubic-lattice result."""

    projection: LatticeProjection
    layer_z: int | None
    available_layers_z: tuple[int, ...]
    grid_lines: tuple[PresentedGridLine, ...]
    lattice_axes: tuple[PresentedLatticeAxis, ...]
    cells: tuple[PresentedLatticeCell, ...]
    nodes: tuple[PresentedLatticeModule, ...]
    edges: tuple[PresentedLatticeConnection, ...]
    bounds: tuple[int, int, int, int, int, int]
    camera: LatticeCamera = DEFAULT_LATTICE_CAMERA


class CubicLatticeProjector:
    """Project lattice views while retaining non-shrinking grid bounds."""

    def __init__(self) -> None:
        self._bounds: tuple[int, int, int, int, int, int] | None = None
        self._view_key: tuple[str, str] | None = None

    @property
    def bounds(self) -> tuple[int, int, int, int, int, int] | None:
        """Return retained lattice bounds, primarily for diagnostics."""
        return self._bounds

    def project(
        self,
        view: CubicLatticeView,
        *,
        projection: LatticeProjection = LatticeProjection.ISOMETRIC,
        layer_z: int | None = None,
        selection: tuple[str, str] | None = None,
        show_snap_cells: bool = True,
        show_orientation_axes: bool = True,
        camera: LatticeCamera = DEFAULT_LATTICE_CAMERA,
    ) -> CubicLatticeGeometry:
        """Return deterministic 2-D geometry for one immutable lattice view."""
        key = (view.source.pack_id, view.id)
        if key != self._view_key:
            self._view_key = key
            self._bounds = None

        measured_by_id = {
            node.id: _measured_lattice_position(
                node.cell,
                node.pose_residual.translation_m,
                view.pitch_m,
            )
            for node in view.nodes
        }
        self._expand_bounds(tuple(node.cell for node in view.nodes), tuple(measured_by_id.values()))
        bounds = self._bounds or (-1, 1, -1, 1, -1, 1)
        available_layers = tuple(sorted({node.cell[2] for node in view.nodes}))
        visible_nodes = tuple(
            node for node in view.nodes if layer_z is None or node.cell[2] == layer_z
        )
        visible_ids = {node.id for node in visible_nodes}

        cells_by_coordinate: dict[tuple[int, int, int], list[str]] = {}
        for node in visible_nodes:
            cells_by_coordinate.setdefault(node.cell, []).append(node.id)
        cells = tuple(
            _present_cell(
                cell,
                tuple(sorted(occupants)),
                projection,
                camera,
                selected=selection == ("cell", _cell_id(cell)),
                show_outline=show_snap_cells
                or any(
                    node.occupancy_conflict or node.off_lattice
                    for node in visible_nodes
                    if node.id in occupants
                ),
                conflict=any(
                    node.occupancy_conflict for node in visible_nodes if node.id in occupants
                ),
            )
            for cell, occupants in sorted(cells_by_coordinate.items())
        )

        lattice_inverse = quat_conjugate(view.orientation_world_wxyz)
        nodes = tuple(
            _present_module(
                node=node,
                measured_position=measured_by_id[node.id],
                relative_orientation=quat_normalize(
                    quat_multiply(lattice_inverse, node.world_orientation_wxyz)
                ),
                projection=projection,
                camera=camera,
                selected=selection == ("node", node.id),
                show_orientation_axes=show_orientation_axes,
            )
            for node in visible_nodes
        )
        edges = _present_connections(
            view,
            measured_by_id,
            visible_ids,
            projection,
            camera,
            selection,
        )
        return CubicLatticeGeometry(
            projection=projection,
            layer_z=layer_z,
            available_layers_z=available_layers,
            grid_lines=_grid_lines(bounds, projection, layer_z, camera),
            lattice_axes=_lattice_axes(bounds, projection, layer_z, camera),
            cells=cells,
            nodes=nodes,
            edges=edges,
            bounds=bounds,
            camera=camera,
        )

    def _expand_bounds(
        self,
        cells: tuple[tuple[int, int, int], ...],
        measured: tuple[Point3D, ...],
    ) -> None:
        coordinates: tuple[Point3D, ...] = (
            tuple((float(cell[0]), float(cell[1]), float(cell[2])) for cell in cells) + measured
        )
        if not coordinates:
            candidate = (-1, 1, -1, 1, -1, 1)
        else:
            candidate = (
                math.floor(min(point[0] for point in coordinates)) - 1,
                math.ceil(max(point[0] for point in coordinates)) + 1,
                math.floor(min(point[1] for point in coordinates)) - 1,
                math.ceil(max(point[1] for point in coordinates)) + 1,
                math.floor(min(point[2] for point in coordinates)) - 1,
                math.ceil(max(point[2] for point in coordinates)) + 1,
            )
        if self._bounds is None:
            self._bounds = candidate
            return
        current = self._bounds
        self._bounds = (
            min(current[0], candidate[0]),
            max(current[1], candidate[1]),
            min(current[2], candidate[2]),
            max(current[3], candidate[3]),
            min(current[4], candidate[4]),
            max(current[5], candidate[5]),
        )


def project_lattice_point(
    point: Point3D,
    projection: LatticeProjection,
    camera: LatticeCamera = DEFAULT_LATTICE_CAMERA,
) -> tuple[float, float, float]:
    """Project one lattice-space point to screen x, screen y, and depth."""
    x_value, y_value, z_value = point
    if projection is LatticeProjection.ISOMETRIC:
        screen_x, screen_y, camera_direction = _camera_basis(camera)
        return (
            _ORBIT_PROJECTION_SCALE * vec_dot(point, screen_x),
            _ORBIT_PROJECTION_SCALE * vec_dot(point, screen_y),
            vec_dot(point, camera_direction),
        )
    if projection is LatticeProjection.XY:
        return x_value, y_value, z_value
    if projection is LatticeProjection.XZ:
        return x_value, z_value, y_value
    return y_value, z_value, x_value


def _present_cell(
    cell: tuple[int, int, int],
    occupants: tuple[str, ...],
    projection: LatticeProjection,
    camera: LatticeCamera,
    *,
    selected: bool,
    show_outline: bool,
    conflict: bool,
) -> PresentedLatticeCell:
    coordinate = (float(cell[0]), float(cell[1]), float(cell[2]))
    center = project_lattice_point(coordinate, projection, camera)[:2]
    outline = _projected_cube_segments(
        coordinate,
        _IDENTITY_QUATERNION,
        projection,
        camera,
        _SNAP_HALF_EXTENT,
    )
    return PresentedLatticeCell(
        id=_cell_id(cell),
        cell=cell,
        center=center,
        outline_segments=outline if show_outline else (),
        occupant_ids=occupants,
        occupancy_conflict=conflict,
        selected=selected,
    )


def _present_module(
    *,
    node: CubicLatticeNode,
    measured_position: Point3D,
    relative_orientation: Quat,
    projection: LatticeProjection,
    camera: LatticeCamera,
    selected: bool,
    show_orientation_axes: bool,
) -> PresentedLatticeModule:
    cell = node.cell
    snap_center = project_lattice_point(
        (float(cell[0]), float(cell[1]), float(cell[2])), projection, camera
    )[:2]
    center = project_lattice_point(measured_position, projection, camera)[:2]
    tether = None if _points_close(center, snap_center) else (center, snap_center)
    return PresentedLatticeModule(
        id=node.id,
        label=node.label,
        module_type_id=node.module_type_id,
        assembly_id=node.assembly_id,
        cell=cell,
        measured_lattice_position=measured_position,
        center=center,
        faces=_projected_cube_faces(
            measured_position,
            relative_orientation,
            projection,
            camera,
        ),
        orientation_axes=(
            _orientation_axes(measured_position, relative_orientation, projection, camera)
            if show_orientation_axes
            else ()
        ),
        tether=tether,
        position_residual_m=node.pose_residual.position_m,
        orientation_residual_rad=node.pose_residual.orientation_rad,
        off_lattice=node.off_lattice,
        occupancy_conflict=node.occupancy_conflict,
        selected=selected,
        outline_segments=_projected_cube_segments(
            measured_position,
            relative_orientation,
            projection,
            camera,
            _CUBE_HALF_EXTENT,
        ),
    )


def _present_connections(
    view: CubicLatticeView,
    measured_by_id: dict[str, Point3D],
    visible_ids: set[str],
    projection: LatticeProjection,
    camera: LatticeCamera,
    selection: tuple[str, str] | None,
) -> tuple[PresentedLatticeConnection, ...]:
    visible_edges = tuple(
        edge for edge in view.edges if edge.source in visible_ids and edge.target in visible_ids
    )
    groups: dict[tuple[str, str], list[str]] = {}
    for edge in visible_edges:
        pair = tuple(sorted((edge.source, edge.target)))
        groups.setdefault((pair[0], pair[1]), []).append(edge.id)
    rank_by_id: dict[str, float] = {}
    for identifiers in groups.values():
        ordered = sorted(identifiers)
        for index, identifier in enumerate(ordered):
            rank_by_id[identifier] = index - (len(ordered) - 1) / 2.0

    result: list[PresentedLatticeConnection] = []
    for edge in visible_edges:
        source_position = _offset(
            measured_by_id[edge.source],
            _face_vector(edge.source_face),
            _FACE_OFFSET,
        )
        target_position = _offset(
            measured_by_id[edge.target],
            _face_vector(edge.target_face),
            _FACE_OFFSET,
        )
        source = project_lattice_point(source_position, projection, camera)[:2]
        target = project_lattice_point(target_position, projection, camera)[:2]
        marker = ((source[0] + target[0]) / 2.0, (source[1] + target[1]) / 2.0)
        rank = rank_by_id[edge.id]
        dx = target[0] - source[0]
        dy = target[1] - source[1]
        length = math.hypot(dx, dy)
        normal = (0.0, 1.0) if length <= _ZERO_TOLERANCE else (-dy / length, dx / length)
        control = (
            marker[0] + normal[0] * 0.10 * rank,
            marker[1] + normal[1] * 0.10 * rank,
        )
        result.append(
            PresentedLatticeConnection(
                id=edge.id,
                source=edge.source,
                target=edge.target,
                connector_a=edge.connector_a,
                connector_b=edge.connector_b,
                source_face=edge.source_face,
                target_face=edge.target_face,
                source_face_residual_rad=edge.source_face_residual_rad,
                target_face_residual_rad=edge.target_face_residual_rad,
                path=(source, control, target),
                marker=marker,
                selected=selection == ("edge", edge.id),
            )
        )
    return tuple(result)


def _projected_cube_faces(
    center: Point3D,
    orientation: Quat,
    projection: LatticeProjection,
    camera: LatticeCamera,
) -> tuple[PresentedLatticeFace, ...]:
    camera_direction = _camera_direction(projection, camera)
    light = vec_normalize((0.25, -0.4, 1.0))
    faces: list[PresentedLatticeFace] = []
    for vertices, local_normal in _LOCAL_FACES:
        normal = quat_rotate(orientation, local_normal)
        if vec_dot(normal, camera_direction) <= _ZERO_TOLERANCE:
            continue
        transformed = tuple(
            _offset(center, quat_rotate(orientation, vertex), 1.0) for vertex in vertices
        )
        projected = tuple(project_lattice_point(point, projection, camera) for point in transformed)
        brightness = 0.48 + 0.42 * max(0.0, vec_dot(normal, light))
        faces.append(
            PresentedLatticeFace(
                points=(
                    projected[0][:2],
                    projected[1][:2],
                    projected[2][:2],
                    projected[3][:2],
                ),
                depth=sum(point[2] for point in projected) / 4.0,
                brightness=min(brightness, 1.0),
            )
        )
    return tuple(sorted(faces, key=lambda face: face.depth))


def _projected_cube_segments(
    center: Point3D,
    orientation: Quat,
    projection: LatticeProjection,
    camera: LatticeCamera,
    half_extent: float,
) -> tuple[Segment2D, ...]:
    corners = tuple(
        _offset(center, quat_rotate(orientation, corner), 1.0)
        for corner in _cube_corners(half_extent)
    )
    projected = tuple(project_lattice_point(point, projection, camera)[:2] for point in corners)
    return tuple((projected[first], projected[second]) for first, second in _CUBE_EDGES)


def _orientation_axes(
    center: Point3D,
    orientation: Quat,
    projection: LatticeProjection,
    camera: LatticeCamera,
) -> tuple[PresentedOrientationAxis, ...]:
    start = project_lattice_point(center, projection, camera)[:2]
    result: list[PresentedOrientationAxis] = []
    directions: tuple[tuple[AxisName, Vec3], ...] = (
        ("x", (1.0, 0.0, 0.0)),
        ("y", (0.0, 1.0, 0.0)),
        ("z", (0.0, 0.0, 1.0)),
    )
    for axis, direction in directions:
        endpoint = _offset(center, quat_rotate(orientation, direction), _ORIENTATION_AXIS_LENGTH)
        result.append(
            PresentedOrientationAxis(
                axis=axis,
                start=start,
                end=project_lattice_point(endpoint, projection, camera)[:2],
            )
        )
    return tuple(result)


def _grid_lines(
    bounds: tuple[int, int, int, int, int, int],
    projection: LatticeProjection,
    layer_z: int | None,
    camera: LatticeCamera,
) -> tuple[PresentedGridLine, ...]:
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    x_values = _bounded_axis_values(x_min, x_max)
    y_values = _bounded_axis_values(y_min, y_max)
    z_values = _bounded_axis_values(z_min, z_max)
    floor_z = z_min if layer_z is None else layer_z
    lines: list[PresentedGridLine] = []

    if projection in (LatticeProjection.ISOMETRIC, LatticeProjection.XY):
        for x_value in x_values:
            lines.append(
                _grid_line(
                    (float(x_value), float(y_min), float(floor_z)),
                    (float(x_value), float(y_max), float(floor_z)),
                    "y",
                    projection,
                    camera,
                    major=x_value == 0,
                )
            )
        for y_value in y_values:
            lines.append(
                _grid_line(
                    (float(x_min), float(y_value), float(floor_z)),
                    (float(x_max), float(y_value), float(floor_z)),
                    "x",
                    projection,
                    camera,
                    major=y_value == 0,
                )
            )
    elif projection is LatticeProjection.XZ:
        for x_value in x_values:
            lines.append(
                _grid_line(
                    (float(x_value), float(y_min), float(z_min)),
                    (float(x_value), float(y_min), float(z_max)),
                    "z",
                    projection,
                    camera,
                    major=x_value == 0,
                )
            )
        for z_value in z_values:
            lines.append(
                _grid_line(
                    (float(x_min), float(y_min), float(z_value)),
                    (float(x_max), float(y_min), float(z_value)),
                    "x",
                    projection,
                    camera,
                    major=z_value == 0,
                )
            )
    else:
        for y_value in y_values:
            lines.append(
                _grid_line(
                    (float(x_min), float(y_value), float(z_min)),
                    (float(x_min), float(y_value), float(z_max)),
                    "z",
                    projection,
                    camera,
                    major=y_value == 0,
                )
            )
        for z_value in z_values:
            lines.append(
                _grid_line(
                    (float(x_min), float(y_min), float(z_value)),
                    (float(x_min), float(y_max), float(z_value)),
                    "y",
                    projection,
                    camera,
                    major=z_value == 0,
                )
            )

    if projection is LatticeProjection.ISOMETRIC:
        for z_value in z_values:
            corners = (
                (float(x_min), float(y_min), float(z_value)),
                (float(x_max), float(y_min), float(z_value)),
                (float(x_max), float(y_max), float(z_value)),
                (float(x_min), float(y_max), float(z_value)),
            )
            for index in range(4):
                lines.append(
                    _grid_line(
                        corners[index],
                        corners[(index + 1) % 4],
                        "x" if index % 2 == 0 else "y",
                        projection,
                        camera,
                        major=z_value == 0,
                    )
                )
        for x_value, y_value in (
            (x_min, y_min),
            (x_min, y_max),
            (x_max, y_min),
            (x_max, y_max),
        ):
            lines.append(
                _grid_line(
                    (float(x_value), float(y_value), float(z_min)),
                    (float(x_value), float(y_value), float(z_max)),
                    "z",
                    projection,
                    camera,
                )
            )
    return tuple(lines)


def _lattice_axes(
    bounds: tuple[int, int, int, int, int, int],
    projection: LatticeProjection,
    layer_z: int | None,
    camera: LatticeCamera,
) -> tuple[PresentedLatticeAxis, ...]:
    x_min, _x_max, y_min, _y_max, z_min, _z_max = bounds
    anchor = (float(x_min), float(y_min), float(z_min if layer_z is None else layer_z))
    directions: tuple[tuple[AxisName, Vec3], ...] = (
        ("x", (1.0, 0.0, 0.0)),
        ("y", (0.0, 1.0, 0.0)),
        ("z", (0.0, 0.0, 1.0)),
    )
    return tuple(
        PresentedLatticeAxis(
            axis=axis,
            start=project_lattice_point(anchor, projection, camera)[:2],
            end=project_lattice_point(
                _offset(anchor, direction, 0.8),
                projection,
                camera,
            )[:2],
        )
        for axis, direction in directions
    )


def _grid_line(
    start: Point3D,
    end: Point3D,
    axis: AxisName,
    projection: LatticeProjection,
    camera: LatticeCamera,
    *,
    major: bool = False,
) -> PresentedGridLine:
    return PresentedGridLine(
        start=project_lattice_point(start, projection, camera)[:2],
        end=project_lattice_point(end, projection, camera)[:2],
        axis=axis,
        major=major,
    )


def _bounded_axis_values(low: int, high: int) -> tuple[int, ...]:
    span = max(high - low, 0)
    stride = max(1, math.ceil(span / _MAX_GRID_INTERVALS))
    values = list(range(low, high + 1, stride))
    if not values or values[-1] != high:
        values.append(high)
    return tuple(values)


def _measured_lattice_position(
    cell: tuple[int, int, int],
    residual_m: Vec3,
    pitch_m: float,
) -> Point3D:
    return (
        cell[0] + residual_m[0] / pitch_m,
        cell[1] + residual_m[1] / pitch_m,
        cell[2] + residual_m[2] / pitch_m,
    )


def _face_vector(face: LatticeFace) -> Vec3:
    return {
        "positive_x": (1.0, 0.0, 0.0),
        "negative_x": (-1.0, 0.0, 0.0),
        "positive_y": (0.0, 1.0, 0.0),
        "negative_y": (0.0, -1.0, 0.0),
        "positive_z": (0.0, 0.0, 1.0),
        "negative_z": (0.0, 0.0, -1.0),
    }[face]


def _camera_direction(projection: LatticeProjection, camera: LatticeCamera) -> Vec3:
    if projection is LatticeProjection.ISOMETRIC:
        return _camera_basis(camera)[2]
    if projection is LatticeProjection.XY:
        return (0.0, 0.0, 1.0)
    if projection is LatticeProjection.XZ:
        return (0.0, 1.0, 0.0)
    return (1.0, 0.0, 0.0)


def _camera_basis(camera: LatticeCamera) -> tuple[Vec3, Vec3, Vec3]:
    """Return right, up, and view-direction unit vectors for one orbit camera."""
    cosine_azimuth = math.cos(camera.azimuth_rad)
    sine_azimuth = math.sin(camera.azimuth_rad)
    cosine_elevation = math.cos(camera.elevation_rad)
    sine_elevation = math.sin(camera.elevation_rad)
    right = (sine_azimuth, -cosine_azimuth, 0.0)
    up = (
        -sine_elevation * cosine_azimuth,
        -sine_elevation * sine_azimuth,
        cosine_elevation,
    )
    direction = (
        cosine_elevation * cosine_azimuth,
        cosine_elevation * sine_azimuth,
        sine_elevation,
    )
    return right, up, direction


def _offset(origin: Point3D, direction: Vec3, scale: float) -> Point3D:
    return (
        origin[0] + direction[0] * scale,
        origin[1] + direction[1] * scale,
        origin[2] + direction[2] * scale,
    )


def _points_close(first: Point2D, second: Point2D) -> bool:
    return math.isclose(first[0], second[0], abs_tol=_ZERO_TOLERANCE) and math.isclose(
        first[1], second[1], abs_tol=_ZERO_TOLERANCE
    )


def _cell_id(cell: tuple[int, int, int]) -> str:
    return f"cell:{cell[0]},{cell[1]},{cell[2]}"


def _cube_corners(half_extent: float) -> tuple[Point3D, ...]:
    return tuple(
        (x_value, y_value, z_value)
        for x_value in (-half_extent, half_extent)
        for y_value in (-half_extent, half_extent)
        for z_value in (-half_extent, half_extent)
    )


_IDENTITY_QUATERNION: Quat = (1.0, 0.0, 0.0, 0.0)
_CUBE_EDGES = (
    (0, 1),
    (0, 2),
    (0, 4),
    (1, 3),
    (1, 5),
    (2, 3),
    (2, 6),
    (3, 7),
    (4, 5),
    (4, 6),
    (5, 7),
    (6, 7),
)
_H = _CUBE_HALF_EXTENT
_LOCAL_FACES: tuple[tuple[tuple[Point3D, Point3D, Point3D, Point3D], Vec3], ...] = (
    (((_H, -_H, -_H), (_H, _H, -_H), (_H, _H, _H), (_H, -_H, _H)), (1.0, 0.0, 0.0)),
    (((-_H, -_H, -_H), (-_H, -_H, _H), (-_H, _H, _H), (-_H, _H, -_H)), (-1.0, 0.0, 0.0)),
    (((-_H, _H, -_H), (-_H, _H, _H), (_H, _H, _H), (_H, _H, -_H)), (0.0, 1.0, 0.0)),
    (((-_H, -_H, -_H), (_H, -_H, -_H), (_H, -_H, _H), (-_H, -_H, _H)), (0.0, -1.0, 0.0)),
    (((-_H, -_H, _H), (_H, -_H, _H), (_H, _H, _H), (-_H, _H, _H)), (0.0, 0.0, 1.0)),
    (((-_H, -_H, -_H), (-_H, _H, -_H), (_H, _H, -_H), (_H, -_H, -_H)), (0.0, 0.0, -1.0)),
)


__all__ = [
    "DEFAULT_LATTICE_CAMERA",
    "CubicLatticeGeometry",
    "CubicLatticeProjector",
    "LatticeCamera",
    "LatticeProjection",
    "PresentedGridLine",
    "PresentedLatticeAxis",
    "PresentedLatticeCell",
    "PresentedLatticeConnection",
    "PresentedLatticeFace",
    "PresentedLatticeModule",
    "PresentedOrientationAxis",
    "project_lattice_point",
]
