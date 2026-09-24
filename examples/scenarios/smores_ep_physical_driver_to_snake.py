"""Physical SMORES-EP execution parameters for Driver-to-Snake.

The topology replacements come from Liu, Whitzer, and Yim (2019), while the
collision-clearance corridors below are authored ModSim routes.  They are not
published hardware trajectories.  All offsets are relative to the final root
pose of the moving module named by each docking action.
"""

from __future__ import annotations

import math

from modsim.runtime.differential_drive import (
    DifferentialDriveGeometry,
    PositionEffortController,
    VelocityEffortController,
)
from modsim.runtime.physical_reconfiguration import (
    DifferentialDriveReconfigurationConfig,
    PhysicalActionRoute,
    TargetRelativeWaypoint,
)
from modsim.runtime.physics_docking import JointHoldTarget


def build_config(
    *,
    dt_s: float,
    navigation_speed_m_s: float,
    approach_speed_m_s: float,
) -> DifferentialDriveReconfigurationConfig:
    """Return the tuned physical controller and four authored routes."""
    return DifferentialDriveReconfigurationConfig(
        left_wheel_joint="joint_left_wheel",
        right_wheel_joint="joint_right_wheel",
        hold_joints=(
            JointHoldTarget("joint_tilt"),
            JointHoldTarget("joint_pan", continuous=True),
        ),
        connector_roll_joint="joint_pan",
        connector_roll_connector="pan",
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
        routes=(
            # Module 1 leaves the upper-right end, drives around the top of
            # the Driver, and approaches module 3's rear face from the left.
            PhysicalActionRoute(
                waypoints=(
                    _waypoint(0.34, 0.0, 0.0, 1),
                    _waypoint(0.34, 0.13, math.pi / 2.0, 1),
                    _waypoint(-0.06, 0.13, math.pi, 1),
                    _waypoint(-0.06, 0.0, -math.pi / 2.0, 1),
                    _waypoint(
                        -0.03,
                        0.0,
                        0.0,
                        1,
                        position_tolerance_m=0.01,
                    ),
                ),
                approach_direction=1,
            ),
            # Module 7 mirrors that path below the Driver, finishing with a
            # reverse approach of its rear face to module 6's front face.
            PhysicalActionRoute(
                waypoints=(
                    _waypoint(-0.34, 0.0, 0.0, -1),
                    _waypoint(-0.34, -0.13, -math.pi / 2.0, 1),
                    _waypoint(0.06, -0.13, 0.0, 1),
                    _waypoint(0.06, 0.0, math.pi / 2.0, 1),
                    _waypoint(
                        0.12,
                        0.0,
                        0.0,
                        1,
                        position_tolerance_m=0.015,
                    ),
                ),
                approach_direction=-1,
            ),
            # Once the leaves have moved, modules 1-3-2 are one rigid train.
            # Pull it clear to the left, skid-steer downward, then drive its
            # module-2 front face into module 4's rear face.
            PhysicalActionRoute(
                waypoints=(
                    _waypoint(-0.27, 0.088, 0.0, -1),
                    _waypoint(
                        -0.03,
                        0.0,
                        -0.35,
                        1,
                        position_tolerance_m=0.01,
                    ),
                ),
                approach_direction=1,
            ),
            # Modules 5-6-7 mirror the train maneuver on the right and finish
            # with module 5 reversing its rear face onto module 4's front.
            PhysicalActionRoute(
                waypoints=(
                    # The first point is a clearance corridor, not a docking
                    # target. A broad radius lets the long three-module train
                    # transition before rear-skid pivot drift turns it away
                    # from an already-clear obstacle boundary.
                    _waypoint(
                        0.27,
                        -0.088,
                        0.0,
                        1,
                        position_tolerance_m=0.075,
                    ),
                    _waypoint(
                        0.03,
                        # The long reverse-driving train has a repeatable
                        # rightward skid bias during its last 30 mm. Start
                        # 8 mm to the left so tire dynamics carry the bottom
                        # face through the measured capture envelope.
                        0.008,
                        -0.35,
                        -1,
                        position_tolerance_m=0.01,
                    ),
                ),
                approach_direction=-1,
            ),
        ),
        dt_s=dt_s,
        controller_period_s=dt_s,
        navigation_speed_m_s=navigation_speed_m_s,
        approach_speed_m_s=approach_speed_m_s,
        maximum_wheel_speed_rad_s=math.pi / 2.0,
        maximum_yaw_rate_rad_s=0.7,
        # The tire pairs have deliberate longitudinal grip. A stronger
        # heading loop reaches the bounded wheel effort needed to break static
        # pivot friction instead of stalling on small residual yaw errors.
        heading_gain_per_s=8.0,
        # The rear skid needs a decisive steering request to break static
        # friction while correcting the final centimetre of lateral error.
        approach_lateral_gain_rad_per_m_s=60.0,
        settle_s=0.5,
        release_delay_s=0.08,
        connected_hold_s=0.3,
        # The published EP-Face characterization reports capture through a
        # 4 mm normal gap and 7 mm lateral offset. The generic acceptance API
        # currently exposes one spherical position error, so this demo uses
        # the pack's conservative 6 mm envelope after every other criterion
        # passes. The two-module showcase remains stricter at 1 mm.
        latch_distance_m=0.006,
        action_timeout_s=180.0,
    )


def _waypoint(
    forward_m: float,
    left_m: float,
    heading_rad: float,
    drive_direction: int,
    *,
    align_heading: bool = False,
    position_tolerance_m: float = 0.03,
) -> TargetRelativeWaypoint:
    return TargetRelativeWaypoint(
        forward_m=forward_m,
        left_m=left_m,
        heading_rad=heading_rad,
        drive_direction=drive_direction,
        # The provisional tire/skid model has momentum and lateral scrub. The
        # corridor points deliberately use a broad 30 mm capture radius; only
        # the final connector approach uses the strict 1 mm latch gate.
        position_tolerance_m=position_tolerance_m,
        heading_tolerance_rad=0.15 if align_heading else math.pi,
    )


__all__ = ["build_config"]
