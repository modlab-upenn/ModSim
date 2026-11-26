"""Point-to-surface docking geometry.

A surface connector (for example a steel sphere) can hold point connectors
anywhere on it. For one point connector the core builds a *virtual attachment
frame* A_q at the closest point q on the surface: its z axis is the outward
normal, and its x axis is the point connector's own x axis projected onto the
tangent plane, so the measured spin is kept. A_q then behaves exactly like an
ordinary point connector mating with the point side, which lets acceptance,
nominal snapping and the backend request reuse the point-to-point machinery
unchanged. See ``docs/continuous_docking.md`` section 4.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable

from modsim.connectors.acceptance import AcceptanceResult, evaluate_acceptance
from modsim.core.entities import ConnectorInstance
from modsim.core.ids import ConnectorInstanceId
from modsim.core.surface import SurfaceContact, surface_contact
from modsim.core.transforms import Transform, Vec3, vec_norm, vec_sub
from modsim.robot_packs.schema import ConnectorTypeSpec


def virtual_attachment_connector(
    surface: ConnectorInstance,
    contact: SurfaceContact,
    velocity_m_s: Vec3,
) -> ConnectorInstance:
    """Return a detached point-connector stand-in for the surface at ``contact``.

    It carries the surface connector's identity, type and module, but its
    frame is A_q, its docking axis is the outward normal, and its velocity is
    the surface body's velocity at q. It is never bound to a world clock, so
    its values are fixed for the sample it was built from.
    """
    return ConnectorInstance(
        id=surface.id,
        module_id=surface.module_id,
        connector_id=surface.connector_id,
        connector_type_id=surface.connector_type_id,
        parent_link=surface.parent_link,
        world_pose=contact.frame_world,
        world_docking_axis=contact.normal_world,
        world_approach_axis=contact.normal_world,
        world_velocity_m_s=velocity_m_s,
        local_pose=contact.frame_link,
        resolved=True,
    )


Cell = tuple[int, int, int]


def candidate_surface_pairs(
    points: Iterable[tuple[ConnectorInstanceId, Vec3]],
    surfaces: Iterable[tuple[ConnectorInstanceId, Vec3, float]],
    tolerance_m: float,
) -> tuple[tuple[ConnectorInstanceId, ConnectorInstanceId], ...]:
    """Return every (point, surface) pair whose point lies within the surface shell.

    Surface centres are hashed into a uniform grid with cell size
    ``max(radius) + tolerance``. Each point then inspects only the 27 cells
    around it and keeps a pair when ``|p - c| <= radius + tolerance``. The
    expected cost is O(points + surfaces). Pairs are returned sorted so a
    docking pass over the same state always evaluates them in one order.
    """
    surface_list = list(surfaces)
    if not surface_list:
        return ()
    cell_size = max(radius for _, _, radius in surface_list) + max(tolerance_m, 0.0)
    if cell_size <= 0.0:
        return ()

    def cell(point: Vec3) -> Cell:
        return (
            math.floor(point[0] / cell_size),
            math.floor(point[1] / cell_size),
            math.floor(point[2] / cell_size),
        )

    grid: dict[Cell, list[tuple[ConnectorInstanceId, Vec3, float]]] = {}
    for item in surface_list:
        grid.setdefault(cell(item[1]), []).append(item)

    pairs: set[tuple[ConnectorInstanceId, ConnectorInstanceId]] = set()
    for point_id, position in points:
        base = cell(position)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for surface_id, center, radius in grid.get(
                        (base[0] + dx, base[1] + dy, base[2] + dz), ()
                    ):
                        if vec_norm(vec_sub(position, center)) <= radius + tolerance_m:
                            pairs.add((point_id, surface_id))
    return tuple(sorted(pairs))


def evaluate_surface_acceptance(
    point: ConnectorInstance,
    surface: ConnectorInstance,
    point_type: ConnectorTypeSpec,
    surface_type: ConnectorTypeSpec,
    *,
    surface_link_pose: Transform | None,
    surface_velocity_at: Callable[[Vec3], Vec3],
) -> tuple[AcceptanceResult, SurfaceContact | None]:
    """Evaluate a point connector against a surface connector.

    The surface is replaced by its virtual attachment connector at the closest
    point, and the ordinary four criteria are evaluated with the point side as
    side A. The position criterion therefore measures the seat gap, the axis
    criterion measures tilt from the inward normal, orientation reports the
    roll for the record with zero error (both sides are continuous), and the
    relative velocity uses the surface body's velocity at the attachment point.
    """
    spec = surface_type.surface
    if spec is None:  # pragma: no cover - guaranteed by ConnectorTypeSpec validation
        raise ValueError(f"connector type '{surface_type.id}' has no surface geometry")
    if not point.resolved or not surface.resolved or surface_link_pose is None:
        return evaluate_acceptance(point, surface, point_type, surface_type), None
    contact = surface_contact(point.world_pose, surface.world_pose, surface_link_pose, spec)
    if contact is None:
        return (
            AcceptanceResult(
                satisfied=False,
                criteria=(),
                relative_transform=Transform.identity(),
                orientation_rad=0.0,
                orientation_index=None,
                detail="point connector sits at the surface centre; no attachment normal exists",
            ),
            None,
        )
    virtual = virtual_attachment_connector(
        surface, contact, surface_velocity_at(contact.point_world)
    )
    return evaluate_acceptance(point, virtual, point_type, surface_type), contact
