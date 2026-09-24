from __future__ import annotations

import math
from collections.abc import Callable

import pytest

from modsim.core.ids import ModuleInstanceId
from modsim.core.transforms import Transform, quat_from_axis_angle
from modsim.runtime.differential_drive import (
    DifferentialDriveGeometry,
    PositionEffortController,
    RigidAssemblyWheelAllocation,
    VelocityEffortController,
    allocate_rigid_assembly_wheel_targets,
)


def test_differential_drive_converts_forward_and_yaw_targets() -> None:
    geometry = DifferentialDriveGeometry(wheel_radius_m=0.04, track_width_m=0.0672)

    assert geometry.wheel_velocity_targets(0.03, 0.0) == pytest.approx((0.75, 0.75))
    left, right = geometry.wheel_velocity_targets(0.0, 0.5)
    assert left == pytest.approx(-0.42)
    assert right == pytest.approx(0.42)


def test_differential_drive_applies_source_joint_signs() -> None:
    geometry = DifferentialDriveGeometry(
        wheel_radius_m=0.04,
        track_width_m=0.0672,
        left_direction=-1,
        right_direction=1,
    )

    assert geometry.wheel_velocity_targets(0.02, 0.0) == pytest.approx((-0.5, 0.5))


def test_rigid_assembly_allocation_projects_translation_in_deterministic_order() -> None:
    geometry = DifferentialDriveGeometry(wheel_radius_m=0.1, track_width_m=0.4)
    module_a = ModuleInstanceId("module_a")
    module_b = ModuleInstanceId("module_b")

    allocation = allocate_rigid_assembly_wheel_targets(
        geometry,
        {
            module_b: Transform.from_translation((2.0, -1.0, 0.04)),
            module_a: Transform.from_translation((-1.0, 3.0, 0.04)),
        },
        reference_position_m=(0.0, 0.0, 0.04),
        reference_linear_velocity_m_s=(1.0, 0.0, 0.0),
        yaw_rate_rad_s=0.0,
    )

    assert isinstance(allocation, RigidAssemblyWheelAllocation)
    assert tuple(target.module_id for target in allocation.targets) == (module_a, module_b)
    for target in allocation.targets:
        assert target.desired_center_velocity_m_s == pytest.approx((1.0, 0.0, 0.0))
        assert target.longitudinal_velocity_m_s == pytest.approx(1.0)
        assert target.lateral_residual_m_s == pytest.approx(0.0)
        assert (target.left_rad_s, target.right_rad_s) == pytest.approx((10.0, 10.0))
    assert allocation.maximum_lateral_residual_m_s == pytest.approx(0.0)
    assert allocation.lateral_residual_limit_m_s is None
    assert allocation.within_lateral_residual_limit


def test_rigid_assembly_allocation_accounts_for_center_offset_and_heading() -> None:
    geometry = DifferentialDriveGeometry(wheel_radius_m=0.1, track_width_m=0.4)
    west = ModuleInstanceId("west")
    north = ModuleInstanceId("north")

    allocation = allocate_rigid_assembly_wheel_targets(
        geometry,
        {
            west: Transform.from_translation((0.0, 1.0, 0.0)),
            north: Transform(
                translation=(1.0, 0.0, 0.0),
                rotation=quat_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0),
            ),
        },
        reference_position_m=(0.0, 0.0, 0.0),
        reference_linear_velocity_m_s=(0.0, 0.0, 0.0),
        yaw_rate_rad_s=1.0,
        lateral_residual_limit_m_s=1e-9,
    )

    by_module = {target.module_id: target for target in allocation.targets}
    assert by_module[west].desired_center_velocity_m_s == pytest.approx((-1.0, 0.0, 0.0))
    assert by_module[west].longitudinal_velocity_m_s == pytest.approx(-1.0)
    assert (by_module[west].left_rad_s, by_module[west].right_rad_s) == pytest.approx((-12.0, -8.0))
    assert by_module[north].desired_center_velocity_m_s == pytest.approx((0.0, 1.0, 0.0))
    assert by_module[north].longitudinal_velocity_m_s == pytest.approx(1.0)
    assert (by_module[north].left_rad_s, by_module[north].right_rad_s) == pytest.approx((8.0, 12.0))
    assert all(target.lateral_residual_m_s == pytest.approx(0.0) for target in allocation.targets)
    assert allocation.within_lateral_residual_limit


def test_rigid_assembly_allocation_reports_unachievable_lateral_motion() -> None:
    geometry = DifferentialDriveGeometry(wheel_radius_m=0.1, track_width_m=0.4)
    module = ModuleInstanceId("module")

    allocation = allocate_rigid_assembly_wheel_targets(
        geometry,
        {module: Transform.from_translation((1.0, 0.0, 0.0))},
        reference_position_m=(0.0, 0.0, 0.0),
        reference_linear_velocity_m_s=(0.0, 0.0, 0.0),
        yaw_rate_rad_s=1.0,
        lateral_residual_limit_m_s=0.5,
    )

    target = allocation.targets[0]
    assert target.desired_center_velocity_m_s == pytest.approx((0.0, 1.0, 0.0))
    assert target.longitudinal_velocity_m_s == pytest.approx(0.0)
    assert target.lateral_residual_m_s == pytest.approx(1.0)
    assert (target.left_rad_s, target.right_rad_s) == pytest.approx((-2.0, 2.0))
    assert allocation.maximum_lateral_residual_m_s == pytest.approx(1.0)
    assert not allocation.within_lateral_residual_limit


@pytest.mark.parametrize(
    ("module_poses", "linear_velocity", "limit", "message"),
    (
        ({}, (0.0, 0.0, 0.0), None, "module_poses must contain at least one module"),
        (
            {ModuleInstanceId("module"): Transform.identity()},
            (0.0, 0.0, 0.01),
            None,
            "reference_linear_velocity_m_s must be planar",
        ),
        (
            {ModuleInstanceId("module"): Transform.identity()},
            (0.0, 0.0, 0.0),
            -0.01,
            "lateral_residual_limit_m_s must not be negative",
        ),
    ),
)
def test_rigid_assembly_allocation_rejects_invalid_inputs(
    module_poses: dict[ModuleInstanceId, Transform],
    linear_velocity: tuple[float, float, float],
    limit: float | None,
    message: str,
) -> None:
    geometry = DifferentialDriveGeometry(wheel_radius_m=0.1, track_width_m=0.4)

    with pytest.raises(ValueError, match=message):
        allocate_rigid_assembly_wheel_targets(
            geometry,
            module_poses,
            reference_position_m=(0.0, 0.0, 0.0),
            reference_linear_velocity_m_s=linear_velocity,
            yaw_rate_rad_s=0.0,
            lateral_residual_limit_m_s=limit,
        )


def test_rigid_assembly_allocation_rejects_vertical_module_forward_axis() -> None:
    geometry = DifferentialDriveGeometry(wheel_radius_m=0.1, track_width_m=0.4)
    module = ModuleInstanceId("module")
    vertical = Transform(rotation=quat_from_axis_angle((0.0, 1.0, 0.0), math.pi / 2.0))

    with pytest.raises(ValueError, match="forward axis has no planar component"):
        allocate_rigid_assembly_wheel_targets(
            geometry,
            {module: vertical},
            reference_position_m=(0.0, 0.0, 0.0),
            reference_linear_velocity_m_s=(0.0, 0.0, 0.0),
            yaw_rate_rad_s=0.0,
        )


def test_velocity_controller_is_bounded() -> None:
    controller = VelocityEffortController(gain_nm_per_rad_s=0.02, max_effort_nm=0.04)

    assert controller.effort_nm(0.75, 0.25) == pytest.approx(0.01)
    assert controller.effort_nm(100.0, 0.0) == pytest.approx(0.04)
    assert controller.effort_nm(-100.0, 0.0) == pytest.approx(-0.04)


def test_position_controller_wraps_continuous_error_and_damps() -> None:
    controller = PositionEffortController(
        position_gain_nm_per_rad=1.0,
        velocity_gain_nm_per_rad_s=0.02,
        max_effort_nm=0.1,
    )

    effort = controller.effort_nm(
        -math.pi + 0.05,
        math.pi - 0.05,
        0.5,
        continuous=True,
    )
    assert effort == pytest.approx(0.09)
    assert controller.effort_nm(1.0, 0.0, 0.0) == pytest.approx(0.1)


@pytest.mark.parametrize(
    ("factory", "message"),
    (
        (
            lambda: DifferentialDriveGeometry(wheel_radius_m=0.0, track_width_m=0.1),
            "wheel_radius_m must be greater than zero",
        ),
        (
            lambda: DifferentialDriveGeometry(
                wheel_radius_m=0.04,
                track_width_m=0.1,
                left_direction=0,
            ),
            "left_direction must be -1 or 1",
        ),
        (
            lambda: VelocityEffortController(
                gain_nm_per_rad_s=math.nan,
                max_effort_nm=0.1,
            ),
            "gain_nm_per_rad_s must be finite",
        ),
    ),
)
def test_controller_configuration_rejects_invalid_values(
    factory: Callable[[], object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        factory()
