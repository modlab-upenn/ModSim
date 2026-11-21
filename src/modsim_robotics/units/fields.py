"""Declare physical quantities for typed fields and scalar joint commands.

This is an explicit contract registry, not a heuristic that walks arbitrary
metadata. Axis vectors, quaternion components, lattice cells, and unknown
extension fields deliberately have no physical-unit mapping here.
"""

from __future__ import annotations

from types import MappingProxyType

from modsim_robotics.units.conversion import to_si
from modsim_robotics.units.definitions import QuantityKind, UnitError

_FIELDS = MappingProxyType(
    {
        **dict.fromkeys(
            (
                "xyz_m",
                "origin_xyz_m",
                "origin_world_m",
                "pitch_m",
                "position_tolerance_m",
                "anchor_separation_m",
                "lower_position_m",
                "upper_position_m",
                "spacing_m",
                "initial_height_m",
                "height_m",
                "connector_gap_m",
                "landing_error_m",
                "position_error_m",
                "position_m",
                "size_m",
                "radius_m",
                "length_m",
            ),
            QuantityKind.LENGTH,
        ),
        **dict.fromkeys(
            (
                "rpy_rad",
                "origin_rpy_rad",
                "orientation_rad",
                "orientation_tolerance_rad",
                "values_rad",
                "lower_position_rad",
                "upper_position_rad",
                "frame_yaw_rad",
                "pivot_angle_rad",
                "angular_error_rad",
                "orientation_error_rad",
            ),
            QuantityKind.ANGLE,
        ),
        **dict.fromkeys(
            (
                "linear_velocity_m_s",
                "max_relative_velocity_m_s",
                "max_velocity_m_per_s",
                "approach_speed_m_s",
                "retract_speed_m_s",
            ),
            QuantityKind.LINEAR_VELOCITY,
        ),
        **dict.fromkeys(
            ("angular_velocity_rad_s", "max_velocity_rad_per_s", "flywheel_angular_velocity_rad_s"),
            QuantityKind.ANGULAR_VELOCITY,
        ),
        **dict.fromkeys(
            (
                "time_s",
                "duration_s",
                "dt_s",
                "timestep_s",
                "undock_at_s",
                "redock_cooldown_s",
                "planning_time_s",
                "started_at_s",
                "ended_at_s",
                "cooldown_s",
            ),
            QuantityKind.TIME,
        ),
        **dict.fromkeys(
            ("max_normal_force_n", "max_shear_force_n", "max_effort_n", "break_force_n"),
            QuantityKind.FORCE,
        ),
        **dict.fromkeys(
            ("max_bending_moment_nm", "max_effort_nm", "landing_effort_nm"), QuantityKind.TORQUE
        ),
        "mass_kg": QuantityKind.MASS,
        "inertia_kg_m2": QuantityKind.INERTIA,
        "gravity_m_s2": QuantityKind.LINEAR_ACCELERATION,
        "translational_stiffness_n_per_m": QuantityKind.LINEAR_STIFFNESS,
        "rotational_stiffness_nm_per_rad": QuantityKind.ANGULAR_STIFFNESS,
        "linear_damping_n_s_per_m": QuantityKind.LINEAR_DAMPING,
        "angular_damping_nm_s_per_rad": QuantityKind.ANGULAR_DAMPING,
        "publish_hz": QuantityKind.FREQUENCY,
        **dict.fromkeys(
            (
                "assignment_cost_m",
                "axle_offset_m",
                "capture_distance_m",
                "error_m",
                "footprint_center_x_m",
                "footprint_half_m",
                "forward_m",
                "gap_m",
                "initial_gap_m",
                "latch_distance_m",
                "left_m",
                "path_length_m",
                "pivot_point_m",
                "resolution_m",
                "retract_distance_m",
                "track_width_m",
                "tracking_error_m",
                "translation_m",
                "wheel_radius_m",
                "world_position_m",
            ),
            QuantityKind.LENGTH,
        ),
        **dict.fromkeys(
            (
                "angle_rad",
                "capture_angle_tolerance_rad",
                "desired_orientation_rad",
                "heading_rad",
                "heading_tolerance_rad",
                "maximum_pivot_angle_rad",
                "source_face_residual_rad",
                "target_face_residual_rad",
                "target_pivot_angle_rad",
                "target_rad",
                "yaw_rad",
            ),
            QuantityKind.ANGLE,
        ),
        **dict.fromkeys(
            (
                "approach_m_s",
                "desired_center_velocity_m_s",
                "lateral_residual_limit_m_s",
                "lateral_residual_m_s",
                "longitudinal_velocity_m_s",
                "maximum_lateral_residual_m_s",
                "navigation_speed_m_s",
                "relative_velocity_m_s",
                "retract_m_s",
                "speed_m_s",
                "velocity_m_s",
                "world_velocity_m_s",
            ),
            QuantityKind.LINEAR_VELOCITY,
        ),
        **dict.fromkeys(
            (
                "brake_end_flywheel_speed_rad_s",
                "flywheel_speed_rad_s",
                "flywheel_speeds_rad_s",
                "flywheel_stop_speed_rad_s",
                "half_speed_rad_s",
                "left_rad_s",
                "maximum_wheel_speed_rad_s",
                "maximum_yaw_rate_rad_s",
                "quarter_speed_rad_s",
                "right_rad_s",
                "spinup_speed_tolerance_rad_s",
                "target_flywheel_speed_rad_s",
                "yaw_rate_rad_s",
            ),
            QuantityKind.ANGULAR_VELOCITY,
        ),
        **dict.fromkeys(
            (
                "action_timeout_s",
                "approach_timeout_s",
                "available_at_s",
                "brake_ended_at_s",
                "brake_started_at_s",
                "brake_timeout_s",
                "capture_time_s",
                "connected_hold_s",
                "controller_period_s",
                "created_at_s",
                "end_s",
                "finished_s",
                "initial_hold_s",
                "last_progress_s",
                "next_route_s",
                "pivot_timeout_s",
                "planning_period_s",
                "recovery_until_s",
                "release_after_s",
                "release_delay_s",
                "separated_hold_s",
                "settle_s",
                "spinup_timeout_s",
                "start_s",
                "started_s",
                "world_time_s",
            ),
            QuantityKind.TIME,
        ),
        **dict.fromkeys(
            ("constraint_forces_n", "measured_force_n", "force_limit_n", "limit_n"),
            QuantityKind.FORCE,
        ),
        **dict.fromkeys(
            (
                "brake_effort_nm",
                "commanded_effort_nm",
                "maximum_brake_effort_nm",
                "spinup_effort_nm",
            ),
            QuantityKind.TORQUE,
        ),
        "flywheel_axial_inertia_kg_m2": QuantityKind.INERTIA,
        "position_gain_nm_per_rad": QuantityKind.ANGULAR_STIFFNESS,
        "velocity_gain_nm_per_rad_s": QuantityKind.ANGULAR_DAMPING,
        "gain_nm_per_rad_s": QuantityKind.ANGULAR_DAMPING,
    }
)


def joint_quantity_kind(joint_type: str, control_mode: str) -> QuantityKind:
    """Resolve scalar joint values using mechanical type and command mode.

    Args:
        joint_type: ``revolute``, ``continuous``, or ``prismatic``.
        control_mode: ``position``, ``velocity``, or ``effort``.

    Returns:
        Angle/angular velocity/torque for rotating joints, or
        length/linear velocity/force for sliding joints.

    Raises:
        UnitError: A joint type or command mode has no scalar quantity contract.
    """
    if joint_type not in {"revolute", "continuous", "prismatic"}:
        raise UnitError(f"Joint type {joint_type!r} has no scalar unit contract.")
    kinds = (
        {
            "position": QuantityKind.LENGTH,
            "velocity": QuantityKind.LINEAR_VELOCITY,
            "effort": QuantityKind.FORCE,
        }
        if joint_type == "prismatic"
        else {
            "position": QuantityKind.ANGLE,
            "velocity": QuantityKind.ANGULAR_VELOCITY,
            "effort": QuantityKind.TORQUE,
        }
    )
    try:
        return kinds[control_mode]
    except KeyError as error:
        raise UnitError(f"Unknown scalar control mode {control_mode!r}.") from error


def joint_value_to_si(value: float, unit: str, *, joint_type: str, control_mode: str) -> float:
    """Normalize an explicit joint input using its contextual quantity.

    Args:
        value: Finite magnitude supplied by an author or controller.
        unit: Explicit supported unit compatible with this joint command.
        joint_type: Scalar mechanical joint type.
        control_mode: Position, velocity, or effort command.

    Returns:
        The canonical scalar suitable for existing JointCommand or JointLimits APIs.

    Raises:
        UnitError: The joint/mode/unit is incompatible or the magnitude is non-finite.
    """
    return to_si(value, unit, kind=joint_quantity_kind(joint_type, control_mode))


def quantity_for_field(
    field_name: str, *, joint_type: str | None = None, control_mode: str | None = None
) -> QuantityKind | None:
    """Read an explicitly registered field's physical quantity.

    Args:
        field_name: Canonical typed field name, without a metadata path prefix.
        joint_type: Required mechanical type for scalar joint state/command values.
        control_mode: Required mode for a generic joint command ``value`` field.

    Returns:
        Declared physical quantity, or None for unknown or dimensionless fields.
        This function does not authorize conversion of arbitrary JSON properties.

    Raises:
        UnitError: A contextual joint value lacks its type/mode or is unsupported.
    """
    if field_name in {"position", "velocity", "effort", "value"}:
        mode = control_mode if field_name == "value" else field_name
        if joint_type is None or mode is None:
            raise UnitError("Scalar joint values require a mechanical type and control mode.")
        return joint_quantity_kind(joint_type, mode)
    return _FIELDS.get(field_name)
