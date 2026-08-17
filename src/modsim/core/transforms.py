"""Rigid-body transforms for the ModSim core.

The core package deliberately has no NumPy or SciPy dependency, so this module
implements the small amount of rotation algebra that connector geometry needs
using the standard library only. Quaternions are ``(w, x, y, z)`` and always
represent rotations of vectors in a fixed world frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modsim.robot_packs.schema import PoseSpec

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

ZERO_VEC3: Vec3 = (0.0, 0.0, 0.0)
IDENTITY_QUAT: Quat = (1.0, 0.0, 0.0, 0.0)

_EPSILON = 1e-12


def vec_add(a: Vec3, b: Vec3) -> Vec3:
    """Return ``a + b``."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    """Return ``a - b``."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(a: Vec3, factor: float) -> Vec3:
    """Return ``a * factor``."""
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def vec_dot(a: Vec3, b: Vec3) -> float:
    """Return the dot product of ``a`` and ``b``."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a: Vec3, b: Vec3) -> Vec3:
    """Return the cross product ``a x b``."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vec_norm(a: Vec3) -> float:
    """Return the Euclidean length of ``a``."""
    return math.sqrt(vec_dot(a, a))


def vec_normalize(a: Vec3) -> Vec3:
    """Return ``a`` scaled to unit length."""
    magnitude = vec_norm(a)
    if magnitude < _EPSILON:
        raise ValueError("cannot normalize a zero-length vector")
    return vec_scale(a, 1.0 / magnitude)


def angle_between(a: Vec3, b: Vec3) -> float:
    """Return the unsigned angle in radians between two non-zero vectors.

    ``atan2`` of the cross and dot products stays accurate near 0 and pi, where
    an ``acos`` of the dot product loses most of its significant digits. Docking
    alignment checks live exactly in that near-zero regime.
    """
    cross = vec_norm(vec_cross(a, b))
    dot = vec_dot(a, b)
    return math.atan2(cross, dot)


def quat_normalize(q: Quat) -> Quat:
    """Return ``q`` scaled to unit length."""
    magnitude = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if magnitude < _EPSILON:
        raise ValueError("cannot normalize a zero-length quaternion")
    return (q[0] / magnitude, q[1] / magnitude, q[2] / magnitude, q[3] / magnitude)


def quat_multiply(a: Quat, b: Quat) -> Quat:
    """Return the Hamilton product ``a * b``."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q: Quat) -> Quat:
    """Return the conjugate of a unit quaternion, which is its inverse."""
    return (q[0], -q[1], -q[2], -q[3])


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate vector ``v`` by unit quaternion ``q``."""
    w, x, y, z = q
    axis: Vec3 = (x, y, z)
    t = vec_scale(vec_cross(axis, v), 2.0)
    return vec_add(vec_add(v, vec_scale(t, w)), vec_cross(axis, t))


def quat_from_axis_angle(axis: Vec3, angle_rad: float) -> Quat:
    """Return the rotation of ``angle_rad`` about ``axis``."""
    unit = vec_normalize(axis)
    half = angle_rad * 0.5
    sine = math.sin(half)
    return (math.cos(half), unit[0] * sine, unit[1] * sine, unit[2] * sine)


def quat_from_rpy(rpy_rad: Vec3) -> Quat:
    """Return the URDF fixed-axis roll-pitch-yaw rotation ``Rz(y) Ry(p) Rx(r)``."""
    roll, pitch, yaw = rpy_rad
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    sr, cr = math.sin(half_roll), math.cos(half_roll)
    sp, cp = math.sin(half_pitch), math.cos(half_pitch)
    sy, cy = math.sin(half_yaw), math.cos(half_yaw)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_angle(q: Quat) -> float:
    """Return the magnitude in radians of the rotation encoded by ``q``."""
    unit = quat_normalize(q)
    vector_norm = vec_norm((unit[1], unit[2], unit[3]))
    return 2.0 * math.atan2(vector_norm, abs(unit[0]))


def minimal_rotation(source: Vec3, target: Vec3) -> Quat:
    """Return the smallest rotation taking unit vector ``source`` onto ``target``."""
    a = vec_normalize(source)
    b = vec_normalize(target)
    dot = max(-1.0, min(1.0, vec_dot(a, b)))
    if dot > 1.0 - 1e-12:
        return IDENTITY_QUAT
    if dot < -1.0 + 1e-12:
        return quat_from_axis_angle(perpendicular(a), math.pi)
    return quat_from_axis_angle(vec_cross(a, b), math.acos(dot))


def perpendicular(axis: Vec3) -> Vec3:
    """Return a deterministic unit vector perpendicular to ``axis``."""
    unit = vec_normalize(axis)
    # Cross with whichever cardinal axis is least aligned with ``unit`` so the
    # result never degenerates toward zero length.
    smallest = min(range(3), key=lambda index: abs(unit[index]))
    cardinal: Vec3 = (
        1.0 if smallest == 0 else 0.0,
        1.0 if smallest == 1 else 0.0,
        1.0 if smallest == 2 else 0.0,
    )
    return vec_normalize(vec_cross(unit, cardinal))


def project_onto_plane(vector: Vec3, normal: Vec3) -> Vec3:
    """Return the component of ``vector`` perpendicular to ``normal``."""
    unit_normal = vec_normalize(normal)
    return vec_sub(vector, vec_scale(unit_normal, vec_dot(vector, unit_normal)))


def signed_angle_about(reference: Vec3, target: Vec3, axis: Vec3) -> float:
    """Return the signed angle from ``reference`` to ``target`` about ``axis``."""
    unit_axis = vec_normalize(axis)
    flat_reference = project_onto_plane(reference, unit_axis)
    flat_target = project_onto_plane(target, unit_axis)
    if vec_norm(flat_reference) < _EPSILON or vec_norm(flat_target) < _EPSILON:
        raise ValueError("cannot measure a roll angle about a parallel reference direction")
    sine = vec_dot(unit_axis, vec_cross(flat_reference, flat_target))
    cosine = vec_dot(flat_reference, flat_target)
    return math.atan2(sine, cosine)


def wrap_angle(angle_rad: float) -> float:
    """Return ``angle_rad`` wrapped into ``(-pi, pi]``."""
    wrapped = math.remainder(angle_rad, math.tau)
    return math.pi if wrapped == -math.pi else wrapped


@dataclass(frozen=True, slots=True)
class Transform:
    """A rigid transform expressed as a translation and a unit quaternion."""

    translation: Vec3 = ZERO_VEC3
    rotation: Quat = IDENTITY_QUAT

    @classmethod
    def identity(cls) -> Transform:
        """Return the identity transform."""
        return cls()

    @classmethod
    def from_pose_spec(cls, pose: PoseSpec) -> Transform:
        """Return the transform described by a Robot Pack pose."""
        return cls(translation=pose.xyz_m, rotation=quat_from_rpy(pose.rpy_rad))

    @classmethod
    def from_translation(cls, translation: Vec3) -> Transform:
        """Return a pure translation."""
        return cls(translation=translation)

    def compose(self, other: Transform) -> Transform:
        """Return ``self * other``, applying ``other`` first."""
        return Transform(
            translation=vec_add(self.translation, quat_rotate(self.rotation, other.translation)),
            rotation=quat_normalize(quat_multiply(self.rotation, other.rotation)),
        )

    def inverse(self) -> Transform:
        """Return the inverse transform."""
        inverse_rotation = quat_conjugate(self.rotation)
        return Transform(
            translation=vec_scale(quat_rotate(inverse_rotation, self.translation), -1.0),
            rotation=inverse_rotation,
        )

    def apply_point(self, point: Vec3) -> Vec3:
        """Return ``point`` expressed in the parent frame."""
        return vec_add(self.translation, quat_rotate(self.rotation, point))

    def apply_direction(self, direction: Vec3) -> Vec3:
        """Return ``direction`` rotated into the parent frame."""
        return quat_rotate(self.rotation, direction)

    def relative_to(self, other: Transform) -> Transform:
        """Return this transform expressed in ``other``'s frame."""
        return other.inverse().compose(self)

    def is_close(
        self,
        other: Transform,
        *,
        position_tolerance_m: float = 1e-9,
        orientation_tolerance_rad: float = 1e-9,
    ) -> bool:
        """Return whether two transforms agree within the given tolerances."""
        if vec_norm(vec_sub(self.translation, other.translation)) > position_tolerance_m:
            return False
        delta = quat_multiply(quat_conjugate(self.rotation), other.rotation)
        return quat_angle(delta) <= orientation_tolerance_rad
