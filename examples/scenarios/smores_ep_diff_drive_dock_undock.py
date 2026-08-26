"""SMORES-EP parameters for the physical two-module docking demonstration.

The wheel radius, track, joint names, and speed cap come from the committed
Fusion-derived model and published SMORES-EP descriptions.  The effort gains,
effort limits, tire friction, and rear skid are provisional MuJoCo bootstrap
parameters, not measured motor or tire specifications.
"""

from __future__ import annotations

import math

from modsim.core.ids import ModuleInstanceId, connector_instance_id
from modsim.runtime.differential_drive import (
    DifferentialDriveGeometry,
    PositionEffortController,
    VelocityEffortController,
)
from modsim.runtime.physics_docking import (
    DifferentialDriveDockingConfig,
    JointHoldTarget,
)


def build_config(
    fixed_module: ModuleInstanceId,
    moving_module: ModuleInstanceId,
    *,
    dt_s: float,
    initial_gap_m: float,
    approach_speed_m_s: float,
    retract_speed_m_s: float,
) -> DifferentialDriveDockingConfig:
    """Return the source-checkout SMORES-EP physical docking configuration."""
    return DifferentialDriveDockingConfig(
        fixed_connector=connector_instance_id(fixed_module, "bottom"),
        moving_connector=connector_instance_id(moving_module, "pan"),
        left_wheel_joint="joint_left_wheel",
        right_wheel_joint="joint_right_wheel",
        hold_joints=(
            JointHoldTarget("joint_tilt"),
            JointHoldTarget("joint_pan", continuous=True),
        ),
        geometry=DifferentialDriveGeometry(
            wheel_radius_m=0.04,
            track_width_m=0.0672,
        ),
        wheel_controller=VelocityEffortController(
            gain_nm_per_rad_s=0.1,
            max_effort_nm=0.04,
        ),
        hold_controller=PositionEffortController(
            position_gain_nm_per_rad=1.0,
            velocity_gain_nm_per_rad_s=0.02,
            max_effort_nm=0.1,
        ),
        dt_s=dt_s,
        # This first-pass direct-effort plant has no identified motor/gearbox
        # dynamics, so solver-rate feedback avoids aliasing the very small
        # wheel inertias. A hardware-fidelity 20--40 Hz outer loop belongs with
        # a calibrated actuator model in the next iteration.
        controller_period_s=dt_s,
        initial_gap_m=initial_gap_m,
        approach_speed_m_s=approach_speed_m_s,
        retract_speed_m_s=retract_speed_m_s,
        maximum_wheel_speed_rad_s=math.pi / 2.0,
        settle_s=0.5,
        connected_hold_s=1.0,
        release_delay_s=0.08,
        retract_distance_m=0.04,
        latch_distance_m=0.001,
        approach_timeout_s=15.0,
        plan_id="smores_diff_drive_dock_undock",
        plan_name="SMORES-EP physical differential-drive dock and undock",
    )


__all__ = ["build_config"]
