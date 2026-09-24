"""Backend-neutral differential-drive kinematics and effort controllers.

The geometry converts a desired planar body twist into wheel angular-velocity
targets.  The small controllers then turn measured joint error into bounded
effort commands; a physics backend remains responsible for rigid-body,
contact, and actuator dynamics.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from modsim.core.ids import ModuleInstanceId
from modsim.core.transforms import (
    Transform,
    Vec3,
    quat_rotate,
    vec_dot,
    vec_norm,
    vec_normalize,
    wrap_angle,
)
from modsim.core.validation import (
    require_finite,
    require_finite_nonnegative,
    require_finite_positive,
)

_PLANAR_EPSILON = 1e-12
_MODULE_FORWARD: Vec3 = (1.0, 0.0, 0.0)


@dataclass(frozen=True, slots=True)
class DifferentialDriveGeometry:
    """Wheel geometry and source-joint direction conventions."""

    wheel_radius_m: float
    track_width_m: float
    left_direction: int = 1
    right_direction: int = 1

    def __post_init__(self) -> None:
        require_finite_positive(self.wheel_radius_m, "wheel_radius_m")
        require_finite_positive(self.track_width_m, "track_width_m")
        if self.left_direction not in {-1, 1}:
            raise ValueError("left_direction must be -1 or 1")
        if self.right_direction not in {-1, 1}:
            raise ValueError("right_direction must be -1 or 1")

    def wheel_velocity_targets(
        self,
        linear_m_s: float,
        yaw_rate_rad_s: float,
    ) -> tuple[float, float]:
        """Return left/right source-joint velocity targets in radians/s."""
        require_finite(linear_m_s, "linear_m_s")
        require_finite(yaw_rate_rad_s, "yaw_rate_rad_s")
        half_differential = 0.5 * self.track_width_m * yaw_rate_rad_s
        left = (linear_m_s - half_differential) / self.wheel_radius_m
        right = (linear_m_s + half_differential) / self.wheel_radius_m
        return self.left_direction * left, self.right_direction * right


@dataclass(frozen=True, slots=True)
class RigidAssemblyModuleTarget:
    """Wheel targets and nonholonomic residual for one assembly module.

    ``desired_center_velocity_m_s`` is the velocity induced at the module root
    by the requested world-frame rigid-body twist.  A differential drive can
    reproduce only its projection onto the module's planar forward axis.  The
    signed lateral remainder is retained as a diagnostic instead of being
    silently discarded.
    """

    module_id: ModuleInstanceId
    left_rad_s: float
    right_rad_s: float
    desired_center_velocity_m_s: Vec3
    longitudinal_velocity_m_s: float
    lateral_residual_m_s: float


@dataclass(frozen=True, slots=True)
class RigidAssemblyWheelAllocation:
    """Deterministic wheel allocation and its worst lateral residual."""

    targets: tuple[RigidAssemblyModuleTarget, ...]
    maximum_lateral_residual_m_s: float
    lateral_residual_limit_m_s: float | None

    @property
    def within_lateral_residual_limit(self) -> bool:
        """Return whether the allocation satisfies its optional residual bound."""
        return (
            self.lateral_residual_limit_m_s is None
            or self.maximum_lateral_residual_m_s
            <= self.lateral_residual_limit_m_s + _PLANAR_EPSILON
        )


def allocate_rigid_assembly_wheel_targets(
    geometry: DifferentialDriveGeometry,
    module_poses: Mapping[ModuleInstanceId, Transform],
    *,
    reference_position_m: Vec3,
    reference_linear_velocity_m_s: Vec3,
    yaw_rate_rad_s: float,
    lateral_residual_limit_m_s: float | None = None,
) -> RigidAssemblyWheelAllocation:
    """Project a world planar rigid-body twist into module wheel targets.

    The reference linear velocity is defined at ``reference_position_m``. For
    each module center ``p_i``, the desired velocity is

    ``v_i = v_reference + yaw_rate * world_up x (p_i - p_reference)``.

    Its longitudinal projection and the shared yaw rate pass through the
    ordinary differential-drive geometry. The orthogonal planar component is
    reported as a signed residual because a rigid assembly of differently
    oriented differential drives may not be able to realize an arbitrary
    twist without tire slip or internal constraint forces.
    """
    if not module_poses:
        raise ValueError("module_poses must contain at least one module")
    _require_finite_vec3(reference_position_m, "reference_position_m")
    _require_finite_vec3(
        reference_linear_velocity_m_s,
        "reference_linear_velocity_m_s",
    )
    require_finite(yaw_rate_rad_s, "yaw_rate_rad_s")
    if abs(reference_linear_velocity_m_s[2]) > _PLANAR_EPSILON:
        raise ValueError("reference_linear_velocity_m_s must be planar (z must be zero)")
    if lateral_residual_limit_m_s is not None:
        require_finite_nonnegative(
            lateral_residual_limit_m_s,
            "lateral_residual_limit_m_s",
        )

    targets: list[RigidAssemblyModuleTarget] = []
    maximum_residual = 0.0
    for module_id in sorted(module_poses):
        pose = module_poses[module_id]
        _require_finite_vec3(pose.translation, f"module_poses[{module_id!s}].translation")
        forward = quat_rotate(pose.rotation, _MODULE_FORWARD)
        planar_forward = (forward[0], forward[1], 0.0)
        if vec_norm(planar_forward) <= _PLANAR_EPSILON:
            raise ValueError(f"module '{module_id}' forward axis has no planar component")
        planar_forward = vec_normalize(planar_forward)
        planar_left: Vec3 = (-planar_forward[1], planar_forward[0], 0.0)

        offset_x = pose.translation[0] - reference_position_m[0]
        offset_y = pose.translation[1] - reference_position_m[1]
        desired_center_velocity: Vec3 = (
            reference_linear_velocity_m_s[0] - yaw_rate_rad_s * offset_y,
            reference_linear_velocity_m_s[1] + yaw_rate_rad_s * offset_x,
            0.0,
        )
        longitudinal = vec_dot(desired_center_velocity, planar_forward)
        lateral_residual = vec_dot(desired_center_velocity, planar_left)
        left, right = geometry.wheel_velocity_targets(longitudinal, yaw_rate_rad_s)
        targets.append(
            RigidAssemblyModuleTarget(
                module_id=module_id,
                left_rad_s=left,
                right_rad_s=right,
                desired_center_velocity_m_s=desired_center_velocity,
                longitudinal_velocity_m_s=longitudinal,
                lateral_residual_m_s=lateral_residual,
            )
        )
        maximum_residual = max(maximum_residual, abs(lateral_residual))

    return RigidAssemblyWheelAllocation(
        targets=tuple(targets),
        maximum_lateral_residual_m_s=maximum_residual,
        lateral_residual_limit_m_s=lateral_residual_limit_m_s,
    )


def _require_finite_vec3(value: Vec3, name: str) -> None:
    """Require every component of a three-dimensional vector to be finite."""
    for index, component in enumerate(value):
        require_finite(component, f"{name}[{index}]")


@dataclass(frozen=True, slots=True)
class VelocityEffortController:
    """A bounded proportional wheel-velocity servo."""

    gain_nm_per_rad_s: float
    max_effort_nm: float

    def __post_init__(self) -> None:
        require_finite_positive(self.gain_nm_per_rad_s, "gain_nm_per_rad_s")
        require_finite_positive(self.max_effort_nm, "max_effort_nm")

    def effort_nm(self, target_rad_s: float, measured_rad_s: float) -> float:
        """Return a bounded torque from one velocity error."""
        require_finite(target_rad_s, "target_rad_s")
        require_finite(measured_rad_s, "measured_rad_s")
        raw = self.gain_nm_per_rad_s * (target_rad_s - measured_rad_s)
        return max(-self.max_effort_nm, min(self.max_effort_nm, raw))


@dataclass(frozen=True, slots=True)
class PositionEffortController:
    """A bounded scalar-joint proportional-derivative hold controller."""

    position_gain_nm_per_rad: float
    velocity_gain_nm_per_rad_s: float
    max_effort_nm: float

    def __post_init__(self) -> None:
        require_finite_positive(self.position_gain_nm_per_rad, "position_gain_nm_per_rad")
        require_finite_positive(self.velocity_gain_nm_per_rad_s, "velocity_gain_nm_per_rad_s")
        require_finite_positive(self.max_effort_nm, "max_effort_nm")

    def effort_nm(
        self,
        target_rad: float,
        measured_rad: float,
        measured_rad_s: float,
        *,
        continuous: bool = False,
    ) -> float:
        """Return a bounded hold torque for a revolute or continuous joint."""
        require_finite(target_rad, "target_rad")
        require_finite(measured_rad, "measured_rad")
        require_finite(measured_rad_s, "measured_rad_s")
        error = target_rad - measured_rad
        if continuous:
            error = wrap_angle(error)
        raw = (
            self.position_gain_nm_per_rad * error - self.velocity_gain_nm_per_rad_s * measured_rad_s
        )
        return max(-self.max_effort_nm, min(self.max_effort_nm, raw))


__all__ = [
    "DifferentialDriveGeometry",
    "PositionEffortController",
    "RigidAssemblyModuleTarget",
    "RigidAssemblyWheelAllocation",
    "VelocityEffortController",
    "allocate_rigid_assembly_wheel_targets",
]
