"""Analytic surface geometry for continuous docking.

Pure geometry with no docking policy: where a point connector meets a sphere,
the virtual attachment frame there, and the attachment record kept as
measured state. See ``docs/continuous_docking.md`` sections 4 and 6.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modsim.core.transforms import (
    Quat,
    Transform,
    Vec3,
    minimal_rotation,
    project_onto_plane,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    quat_rotate,
    signed_angle_about,
    vec_add,
    vec_norm,
    vec_scale,
    vec_sub,
)
from modsim.robot_packs.schema import SurfaceSpec

_DEGENERATE_REFERENCE = 0.5
_MIN_RADIAL_DISTANCE_M = 1e-12
_Z: Vec3 = (0.0, 0.0, 1.0)


@dataclass(frozen=True, slots=True)
class SurfaceContact:
    """Where one point connector meets one surface right now."""

    center_world: Vec3
    normal_world: Vec3
    """Outward unit normal at the attachment point."""
    point_world: Vec3
    """The attachment point q on the surface."""
    seat_gap_m: float
    """Signed distance from the surface to the point connector origin."""
    frame_world: Transform
    """The virtual attachment frame A_q in the world."""
    frame_link: Transform
    """A_q expressed in the surface connector's parent-link frame."""


def surface_center(surface_pose: Transform, spec: SurfaceSpec) -> Vec3:
    """Return the sphere centre in the world, given the surface connector frame."""
    return surface_pose.apply_point(spec.center_m)


def _tangent_reference(pose: Transform, normal: Vec3) -> Vec3:
    """Return the point connector's x axis on the tangent plane, falling back to y, z."""
    for cardinal in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        projected = project_onto_plane(pose.apply_direction(cardinal), normal)
        if vec_norm(projected) > _DEGENERATE_REFERENCE:
            return projected
    raise ValueError("point connector frame has no direction tangent to the surface")


def frame_with_z(normal: Vec3, x_reference: Vec3, origin: Vec3) -> Transform:
    """Return a frame at ``origin`` whose z is ``normal`` and whose x follows ``x_reference``."""
    align = minimal_rotation(_Z, normal)
    roll = signed_angle_about(quat_rotate(align, (1.0, 0.0, 0.0)), x_reference, normal)
    rotation: Quat = quat_normalize(quat_multiply(quat_from_axis_angle(normal, roll), align))
    return Transform(translation=origin, rotation=rotation)


def surface_contact(
    point_pose: Transform,
    surface_pose: Transform,
    surface_link_pose: Transform,
    spec: SurfaceSpec,
) -> SurfaceContact | None:
    """Return the attachment of a point connector to a sphere, or ``None`` if undefined.

    The contact is undefined only when the point connector sits exactly at the
    sphere centre, where no outward normal exists.
    """
    center = surface_center(surface_pose, spec)
    offset = vec_sub(point_pose.translation, center)
    distance = vec_norm(offset)
    if distance < _MIN_RADIAL_DISTANCE_M:
        return None
    normal = vec_scale(offset, 1.0 / distance)
    point = vec_add(center, vec_scale(normal, spec.radius_m))
    try:
        reference = _tangent_reference(point_pose, normal)
    except ValueError:
        return None
    frame = frame_with_z(normal, reference, point)
    return SurfaceContact(
        center_world=center,
        normal_world=normal,
        point_world=point,
        seat_gap_m=distance - spec.radius_m,
        frame_world=frame,
        frame_link=surface_link_pose.inverse().compose(frame),
    )


def central_angle(normal_a: Vec3, normal_b: Vec3) -> float:
    """Return the angle between two unit surface normals."""
    dot = sum(a * b for a, b in zip(normal_a, normal_b, strict=True))
    return math.acos(max(-1.0, min(1.0, dot)))


_SPIN_DEGENERATE_REFERENCE = 0.5


@dataclass(frozen=True, slots=True)
class SurfaceAttachment:
    """Where a point connector currently sits on a surface connector.

    Canonical attachment state is the outward unit normal and the attachment
    frame A_q, both in the surface connector frame. A normal plus a frame has
    no coordinate singularity. The polar, azimuth and spin angles below are
    derived display coordinates: polar and azimuth are singular at the
    surface frame's poles, and no spin reference can be continuous over the
    whole sphere, so :attr:`spin_reference_degenerate` flags where the
    documented reference falls back.
    """

    normal_local: Vec3
    frame_local: Transform
    seat_gap_m: float
    magnet_height_m: float | None = None

    @property
    def polar_rad(self) -> float:
        """Angle from the surface frame's +z axis to the normal, in [0, pi]."""
        return math.acos(max(-1.0, min(1.0, self.normal_local[2])))

    @property
    def azimuth_rad(self) -> float:
        """Angle of the normal about the surface frame's z axis, in (-pi, pi]."""
        return math.atan2(self.normal_local[1], self.normal_local[0])

    def _spin_reference(self) -> tuple[Vec3, bool]:
        reference = project_onto_plane((1.0, 0.0, 0.0), self.normal_local)
        if vec_norm(reference) > _SPIN_DEGENERATE_REFERENCE:
            return reference, False
        return project_onto_plane((0.0, 1.0, 0.0), self.normal_local), True

    @property
    def spin_rad(self) -> float:
        """Spin of A_q about the normal, measured from the surface frame's x (or y) axis."""
        reference, _ = self._spin_reference()
        x_axis = self.frame_local.apply_direction((1.0, 0.0, 0.0))
        return signed_angle_about(reference, x_axis, self.normal_local)

    @property
    def spin_reference_degenerate(self) -> bool:
        """True near the surface frame's +-x poles, where spin is measured from y."""
        return self._spin_reference()[1]


def attachment_from_contact(
    contact: SurfaceContact,
    surface_pose: Transform,
    magnet_height_m: float | None = None,
) -> SurfaceAttachment:
    """Express a world-frame contact in the surface connector frame."""
    to_surface = surface_pose.inverse()
    return SurfaceAttachment(
        normal_local=to_surface.apply_direction(contact.normal_world),
        frame_local=to_surface.compose(contact.frame_world),
        seat_gap_m=contact.seat_gap_m,
        magnet_height_m=magnet_height_m,
    )
