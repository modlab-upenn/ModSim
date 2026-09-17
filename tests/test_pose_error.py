"""Quantitative pose-error measurement for the paper's validation section.

Two numbers answer the reviewer's "connector pose errors" request, and they
live at two different layers:

* **Adapter agreement** -- given one Robot Pack and one scene, the MuJoCo
  adapter and the kinematic mock must resolve every connector to the *same*
  world frame. Any disagreement is an adapter geometry bug, so the residual is
  a direct measure of connector-frame conversion error. It is expected to sit
  at machine precision.
* **Physical hold error** -- once a weld is committed under real dynamics, the
  realised connector-frame relative pose drifts from the pose the core
  committed. That drift, measured after the pair settles, is the physical
  accuracy of a dock.

Every test both asserts a bound and *reports* the measured value through
``record_property`` and stdout (visible with ``pytest -s``), so the exact
figures can be lifted straight into the paper rather than only the tolerance.

All scenes disable gravity, which isolates connector geometry and the weld
constraint from ground contact and fall-over.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

import pytest

from conftest import with_connector_policy
from modsim.backends.base import SupportsModuleKinematics
from modsim.backends.registry import installed_backends
from modsim.core.events import DockCommitted, Event
from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
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
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime.session import RuntimeSession

pytestmark = pytest.mark.mujoco

MODULE_TYPE = "generic_cube"
CONNECTOR_TYPE = "fixed_face"
GRID_COUNT = 4
GRID_SPACING_M = 0.1

CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")

APPROACH_SPEED_M_S = 0.03
START_GAP_M = 0.15
STEP_S = 0.002
APPROACH_TIMEOUT_S = 20.0
SETTLE_STEPS = 500
REDOCK_COOLDOWN_S = 2.0

# Acceptance thresholds. Adapter agreement is a pure geometry comparison and is
# held to near machine precision; the physical hold bar is the connector type's
# own position tolerance (0.006 m) and orientation tolerance (0.14 rad), since a
# weld that drifts further than a fresh dock would be accepted at is not holding.
ADAPTER_POSITION_TOLERANCE_M = 1e-6
ADAPTER_ANGLE_TOLERANCE_RAD = 1e-6
HOLD_POSITION_TOLERANCE_M = 0.006
HOLD_ORIENTATION_TOLERANCE_RAD = 0.14
# The acceptance region is a box, so a within-tolerance mate can sit up to
# sqrt(3) * tol away in Euclidean distance at the corner.
LATCH_GAP_TOLERANCE_M = math.sqrt(3.0) * HOLD_POSITION_TOLERANCE_M


def _report(
    record_property: Callable[[str, object], None],
    name: str,
    value: float,
    unit: str,
) -> None:
    """Attach a measured value to the test record and print it for ``-s`` runs."""
    record_property(name, value)
    print(f"[pose-error] {name} = {value:.3e} {unit}")


@pytest.fixture
def loaded_pack(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


@pytest.fixture
def latching_pack(loaded_pack: LoadedRobotPack) -> LoadedRobotPack:
    """The example pack with its connector type made auto-latching."""
    return loaded_pack.with_pack(
        with_connector_policy(
            loaded_pack.pack,
            CONNECTOR_TYPE,
            auto_latch=True,
            redock_cooldown_s=REDOCK_COOLDOWN_S,
        )
    )


def _quiescent_session(loaded: LoadedRobotPack, backend: str) -> RuntimeSession:
    scene = SceneSpec.grid(MODULE_TYPE, GRID_COUNT, spacing_m=GRID_SPACING_M)
    if backend == "mock":
        return RuntimeSession.create(loaded, scene, backend)
    return RuntimeSession.create(loaded, scene, backend, gravity=(0.0, 0.0, 0.0))


def _approach_session(loaded: LoadedRobotPack) -> RuntimeSession:
    """Two cubes a short distance apart, the second closing in below the limit."""
    placements = (
        ModulePlacement(instance_id=CUBE_0, module_type_id=MODULE_TYPE),
        ModulePlacement(
            instance_id=CUBE_1,
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((START_GAP_M, 0.0, 0.0)),
        ),
    )
    session = RuntimeSession.create(
        loaded, SceneSpec(placements=placements), "mujoco", gravity=(0.0, 0.0, 0.0)
    )
    adapter = session.adapter
    assert isinstance(adapter, SupportsModuleKinematics)
    adapter.set_module_twist(CUBE_1, linear_m_s=(-APPROACH_SPEED_M_S, 0.0, 0.0))
    return session


def _run_until(
    session: RuntimeSession,
    predicate: Callable[[RuntimeSession], bool],
    *,
    timeout_s: float = APPROACH_TIMEOUT_S,
) -> tuple[Event, ...]:
    collected: list[Event] = []
    deadline = session.world.time_s + timeout_s
    while session.world.time_s < deadline:
        collected.extend(session.step(STEP_S))
        if predicate(session):
            break
    return tuple(collected)


def _rotation_error_rad(measured: Transform, requested: Transform) -> float:
    """Return the geodesic angle between two orientations, in radians."""
    delta = quat_multiply(quat_conjugate(measured.rotation), requested.rotation)
    return quat_angle(delta)


# ----------------------------------------------------------------------
# adapter agreement -- connector-frame conversion error
# ----------------------------------------------------------------------


def test_adapter_connector_pose_error_is_machine_precision(
    loaded_pack: LoadedRobotPack,
    record_property: Callable[[str, object], None],
) -> None:
    """MuJoCo and the mock must resolve identical connector world frames.

    The maximum translation offset and axis misalignment across every connector
    are the connector-frame conversion error of the MuJoCo adapter, since the
    mock is an exact kinematic reference.
    """
    if "mujoco" not in installed_backends():
        pytest.skip("needs the MuJoCo backend to compare against the mock reference")

    reference = _quiescent_session(loaded_pack, "mock")
    subject = _quiescent_session(loaded_pack, "mujoco")

    reference_poses = {
        str(connector.id): connector.world_pose
        for connector in reference.world.connectors.values()
    }
    reference_axes = {
        str(connector.id): connector.world_docking_axis
        for connector in reference.world.connectors.values()
    }

    max_offset_m = 0.0
    max_angle_rad = 0.0
    try:
        for connector in subject.world.connectors.values():
            name = str(connector.id)
            offset = vec_norm(
                vec_sub(connector.world_pose.translation, reference_poses[name].translation)
            )
            angle = angle_between(connector.world_docking_axis, reference_axes[name])
            max_offset_m = max(max_offset_m, offset)
            max_angle_rad = max(max_angle_rad, angle)
    finally:
        subject.shutdown()

    _report(record_property, "adapter_max_connector_offset_m", max_offset_m, "m")
    _report(record_property, "adapter_max_axis_angle_rad", max_angle_rad, "rad")

    assert max_offset_m <= ADAPTER_POSITION_TOLERANCE_M, (
        f"MuJoCo places a connector {max_offset_m:.3e} m from the mock reference"
    )
    assert max_angle_rad <= ADAPTER_ANGLE_TOLERANCE_RAD, (
        f"MuJoCo points a connector {max_angle_rad:.3e} rad from the mock reference"
    )


# ----------------------------------------------------------------------
# physical hold error -- realised weld pose vs committed pose
# ----------------------------------------------------------------------


def test_weld_hold_pose_error_under_physics(
    latching_pack: LoadedRobotPack,
    record_property: Callable[[str, object], None],
) -> None:
    """Measure how far a settled weld drifts from the committed relative pose."""
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    session = _approach_session(latching_pack)
    try:
        _run_until(session, lambda active: bool(active.world.connections))
        assert session.world.connections, "the modules never latched"

        connection = next(iter(session.world.connections.values()))
        requested = connection.relative_transform

        for _ in range(SETTLE_STEPS):
            session.step(STEP_S)

        first = session.world.connector(connection.connector_a)
        second = session.world.connector(connection.connector_b)
        measured = second.world_pose.relative_to(first.world_pose)

        translation_error_m = vec_norm(vec_sub(measured.translation, requested.translation))
        rotation_error_rad = _rotation_error_rad(measured, requested)
    finally:
        session.shutdown()

    _report(record_property, "weld_hold_translation_error_m", translation_error_m, "m")
    _report(record_property, "weld_hold_rotation_error_rad", rotation_error_rad, "rad")

    assert translation_error_m < HOLD_POSITION_TOLERANCE_M, (
        f"weld drifted {translation_error_m:.4g} m from the committed pose"
    )
    assert rotation_error_rad < HOLD_ORIENTATION_TOLERANCE_RAD, (
        f"weld rotated {rotation_error_rad:.4g} rad from the committed pose"
    )


def test_committed_connector_frames_are_coincident_at_latch(
    latching_pack: LoadedRobotPack,
    record_property: Callable[[str, object], None],
) -> None:
    """At the instant of commit, the two mated connector frames should coincide.

    This is the connector-frame residual of a dock: the distance between the two
    connector origins the moment the constraint is realised, before any further
    integration. It complements the settled-drift figure above.
    """
    pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")
    session = _approach_session(latching_pack)
    try:
        events = _run_until(session, lambda active: bool(active.world.connections))
        assert any(isinstance(event, DockCommitted) for event in events)

        connection = next(iter(session.world.connections.values()))
        first = session.world.connector(connection.connector_a)
        second = session.world.connector(connection.connector_b)

        origin_gap_m = vec_norm(
            vec_sub(first.world_pose.translation, second.world_pose.translation)
        )
        # A mate aligns the two docking axes antiparallel; residual is the angle
        # away from exactly opposed.
        axis_residual_rad = angle_between(
            first.world_docking_axis, vec_scale(second.world_docking_axis, -1.0)
        )
    finally:
        session.shutdown()

    _report(record_property, "latch_connector_origin_gap_m", origin_gap_m, "m")
    _report(record_property, "latch_axis_residual_rad", axis_residual_rad, "rad")

    assert origin_gap_m < LATCH_GAP_TOLERANCE_M
    assert axis_residual_rad < HOLD_ORIENTATION_TOLERANCE_RAD


if __name__ == "__main__":  # pragma: no cover - manual measurement entry point
    import sys

    sys.exit(pytest.main([__file__, "-s", "-p", "no:cov"]))
