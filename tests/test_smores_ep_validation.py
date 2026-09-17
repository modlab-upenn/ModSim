"""Pose-error and repeatability validation on the committed SMORES-EP pack.

These are the same quantitative measurements as ``test_pose_error`` and
``test_repeatability``, but run on real published hardware -- the SMORES-EP V4.2
Fusion export in ``examples/robot_packs/smores_ep`` -- rather than the generic
cube. Two things make SMORES-EP the more demanding case:

* Three of its four EP faces (``pan``, ``left``, ``right``) sit on *articulated*
  links (the wheels and tilt body), so a committed weld on those connectors
  exercises the connector-frame-on-an-articulated-child conversion that a
  root-link connector never reaches. ``pan``-``pan`` is included for exactly
  this reason.
* Because the mock backend has no articulated kinematics, there is no kinematic
  reference to compare against for those connectors, so these tests use the
  deterministic staging path (``stage_docking_assembly_pair`` +
  ``process_docking``) that the committed SMORES-EP regression already relies
  on, and measure the realised weld against the pose the core committed.
"""

from __future__ import annotations

from collections.abc import Callable

import pytest

from modsim.core.events import DockCommitted, Event
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.transforms import (
    Transform,
    angle_between,
    quat_angle,
    quat_conjugate,
    quat_multiply,
    vec_norm,
    vec_scale,
    vec_sub,
)
from modsim.robot_packs import LoadedRobotPack
from modsim.runtime.reconfiguration import stage_docking_assembly_pair
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "smores_ep"
BOTTOM_0 = ConnectorInstanceId("smores_ep_0/bottom")
BOTTOM_1 = ConnectorInstanceId("smores_ep_1/bottom")
PAN_0 = ConnectorInstanceId("smores_ep_0/pan")
PAN_1 = ConnectorInstanceId("smores_ep_1/pan")

SPACING_M = 0.3
STEP_S = 0.002
SETTLE_STEPS = 500
PHYSICAL_REPEATS = 3

# The EP face declares a 6 mm / 0.175 rad capture region; a weld that holds
# tighter than a fresh dock would be accepted at is holding correctly.
HOLD_POSITION_TOLERANCE_M = 0.006
HOLD_ORIENTATION_TOLERANCE_RAD = 0.18
# A staged, nominally-aligned latch places the connector frames coincident; the
# residual is expected near machine precision.
LATCH_RESIDUAL_TOLERANCE_M = 1e-3
# MuJoCo is deterministic for a fixed model and single-threaded stepping.
POSE_REPEATABILITY_TOLERANCE_M = 1e-9


def _report(
    record_property: Callable[[str, object], None],
    name: str,
    value: float,
    unit: str,
) -> None:
    """Attach a measured value to the test record and print it for ``-s`` runs."""
    record_property(name, value)
    print(f"[smores-ep] {name} = {value:.3e} {unit}")


def _rotation_error_rad(measured: Transform, requested: Transform) -> float:
    """Return the geodesic angle between two orientations, in radians."""
    delta = quat_multiply(quat_conjugate(measured.rotation), requested.rotation)
    return quat_angle(delta)


def _session(pack: LoadedRobotPack) -> RuntimeSession:
    """Two SMORES-EP modules a short distance apart under real, weightless physics."""
    return RuntimeSession.create(
        pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=SPACING_M),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
        timestep_s=STEP_S,
    )


def _staged_dock(
    session: RuntimeSession,
    fixed: ConnectorInstanceId,
    moving: ConnectorInstanceId,
) -> tuple[Event, ...]:
    """Exact-align a connector pair and commit the weld before any physics step."""
    stage_docking_assembly_pair(session, fixed, moving, gap_m=0.0)
    session.request_dock(fixed, moving)
    events = session.process_docking()
    assert any(isinstance(event, DockCommitted) for event in events), (
        f"the {fixed} / {moving} pair did not commit"
    )
    return events


def _measure_weld_hold(session: RuntimeSession) -> tuple[float, float]:
    """Settle the current single connection and return its translation/rotation drift."""
    connection = next(iter(session.world.connections.values()))
    requested = connection.relative_transform
    for _ in range(SETTLE_STEPS):
        session.step(STEP_S)
    first = session.world.connector(connection.connector_a)
    second = session.world.connector(connection.connector_b)
    measured = second.world_pose.relative_to(first.world_pose)
    translation_error_m = vec_norm(vec_sub(measured.translation, requested.translation))
    rotation_error_rad = _rotation_error_rad(measured, requested)
    return translation_error_m, rotation_error_rad


# ----------------------------------------------------------------------
# pose error -- root-link connector
# ----------------------------------------------------------------------


@pytest.mark.mujoco
def test_smores_bottom_weld_hold_pose_error(
    smores_loaded_pack: LoadedRobotPack,
    record_property: Callable[[str, object], None],
) -> None:
    """A rear-base (root-link) weld holds the committed relative pose."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    session = _session(smores_loaded_pack)
    try:
        _staged_dock(session, BOTTOM_0, BOTTOM_1)
        translation_error_m, rotation_error_rad = _measure_weld_hold(session)
    finally:
        session.shutdown()

    _report(record_property, "bottom_weld_translation_error_m", translation_error_m, "m")
    _report(record_property, "bottom_weld_rotation_error_rad", rotation_error_rad, "rad")

    assert translation_error_m < HOLD_POSITION_TOLERANCE_M
    assert rotation_error_rad < HOLD_ORIENTATION_TOLERANCE_RAD


@pytest.mark.mujoco
def test_smores_staged_latch_places_connectors_coincident(
    smores_loaded_pack: LoadedRobotPack,
    record_property: Callable[[str, object], None],
) -> None:
    """A staged, nominally-aligned latch leaves the two EP faces coincident."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    session = _session(smores_loaded_pack)
    try:
        _staged_dock(session, BOTTOM_0, BOTTOM_1)
        connection = next(iter(session.world.connections.values()))
        first = session.world.connector(connection.connector_a)
        second = session.world.connector(connection.connector_b)
        origin_gap_m = vec_norm(
            vec_sub(first.world_pose.translation, second.world_pose.translation)
        )
        axis_residual_rad = angle_between(
            first.world_docking_axis, vec_scale(second.world_docking_axis, -1.0)
        )
    finally:
        session.shutdown()

    _report(record_property, "latch_connector_origin_gap_m", origin_gap_m, "m")
    _report(record_property, "latch_axis_residual_rad", axis_residual_rad, "rad")

    assert origin_gap_m < LATCH_RESIDUAL_TOLERANCE_M
    assert axis_residual_rad < HOLD_ORIENTATION_TOLERANCE_RAD


# ----------------------------------------------------------------------
# pose error -- articulated-child connector (difficulty 3)
# ----------------------------------------------------------------------


@pytest.mark.mujoco
def test_smores_pan_articulated_weld_hold_pose_error(
    smores_loaded_pack: LoadedRobotPack,
    record_property: Callable[[str, object], None],
) -> None:
    """A weld on the pan face -- which sits on an articulated wheel link -- holds.

    The pan connector is carried on ``front_wheel_1``, itself behind the tilt and
    pan joints, so committing this weld exercises the articulated-child to
    root-free-joint conversion rather than a simple root-link frame.
    """
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    session = _session(smores_loaded_pack)
    try:
        _staged_dock(session, PAN_0, PAN_1)
        translation_error_m, rotation_error_rad = _measure_weld_hold(session)
    finally:
        session.shutdown()

    _report(record_property, "pan_weld_translation_error_m", translation_error_m, "m")
    _report(record_property, "pan_weld_rotation_error_rad", rotation_error_rad, "rad")

    assert translation_error_m < HOLD_POSITION_TOLERANCE_M
    assert rotation_error_rad < HOLD_ORIENTATION_TOLERANCE_RAD


# ----------------------------------------------------------------------
# repeatability -- real hardware, real physics
# ----------------------------------------------------------------------


def _staged_dock_run(pack: LoadedRobotPack) -> tuple[tuple[str, ...], dict[str, Transform]]:
    """Stage and commit a bottom-bottom dock, settle, return event kinds and poses."""
    session = _session(pack)
    try:
        events = list(_staged_dock(session, BOTTOM_0, BOTTOM_1))
        for _ in range(200):
            events.extend(session.step(STEP_S))
        kinds = tuple(event.kind for event in events)
        poses = {
            str(module_id): module.pose for module_id, module in session.world.modules.items()
        }
    finally:
        session.shutdown()
    return kinds, poses


@pytest.mark.mujoco
def test_smores_staged_dock_is_repeatable(smores_loaded_pack: LoadedRobotPack) -> None:
    """The SMORES-EP dock repeats with an identical event sequence and final pose."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

    reference_kinds: tuple[str, ...] | None = None
    reference_poses: dict[str, Transform] | None = None
    for _ in range(PHYSICAL_REPEATS):
        kinds, poses = _staged_dock_run(smores_loaded_pack)
        if reference_kinds is None:
            reference_kinds, reference_poses = kinds, poses
            continue
        assert kinds == reference_kinds, "event-kind sequence changed between runs"
        assert reference_poses is not None
        assert set(poses) == set(reference_poses), "module set changed between runs"
        max_offset = max(
            vec_norm(vec_sub(pose.translation, reference_poses[name].translation))
            for name, pose in poses.items()
        )
        assert max_offset <= POSE_REPEATABILITY_TOLERANCE_M, (
            f"final poses differ by {max_offset:.3e} m between runs"
        )


if __name__ == "__main__":  # pragma: no cover - manual measurement entry point
    import sys

    sys.exit(pytest.main([__file__, "-s", "-p", "no:cov"]))
