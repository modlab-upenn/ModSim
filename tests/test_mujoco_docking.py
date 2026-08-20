"""The MuJoCo docking prototype, end to end under real physics.

This module is the executable definition of "docking works on MuJoCo": two
modules coast together, latch, move as one rigid body, then separate when
released. Everything here runs with gravity disabled, which isolates the
docking constraint from friction and ground contact — a failure is then
attributable to the weld, not to the modules falling over.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from conftest import with_connector_policy
from modsim.core.events import (
    DockCommitted,
    DockFailed,
    Event,
    UndockCommitted,
)
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import (
    Transform,
    quat_from_axis_angle,
    vec_norm,
    vec_sub,
)
from modsim.model_views import ModelViewFactory
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime.inspection import build_runtime_inspector_frame
from modsim.runtime.scenarios import DockingPairScenario, DockingPairScenarioConfig
from modsim.runtime.session import RuntimeSession
from modsim_backend_mujoco.adapter import MuJoCoBackendAdapter

pytestmark = pytest.mark.mujoco

MODULE_TYPE = "generic_cube"
CONNECTOR_TYPE = "fixed_face"
CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")

APPROACH_SPEED_M_S = 0.03
"""Below the pack's 0.05 m/s acceptance limit, with headroom."""

START_GAP_M = 0.15
STEP_S = 0.002
APPROACH_TIMEOUT_S = 20.0


REDOCK_COOLDOWN_S = 2.0
"""Long enough that a released pair stays released.

An auto-latching connector still satisfies acceptance the instant after it is
released, because the two halves are still touching. A docking pass runs
releases before detection, so with no cooldown the pair re-latches in the very
same step. The cooldown is the mechanism the schema provides for that, and
setting it here exercises it under real physics.
"""


@pytest.fixture
def latching_pack(example_pack_dir: Path) -> LoadedRobotPack:
    """The example pack with its connector type made auto-latching."""
    loaded = RobotPackLoader().load(example_pack_dir)
    return loaded.with_pack(
        with_connector_policy(
            loaded.pack,
            CONNECTOR_TYPE,
            auto_latch=True,
            redock_cooldown_s=REDOCK_COOLDOWN_S,
        )
    )


def approach_session(
    loaded: LoadedRobotPack,
    *,
    second_rotation: Transform | None = None,
) -> RuntimeSession:
    """Return two modules a short distance apart, with the second closing in."""
    placements = (
        ModulePlacement(instance_id=CUBE_0, module_type_id=MODULE_TYPE),
        ModulePlacement(
            instance_id=CUBE_1,
            module_type_id=MODULE_TYPE,
            pose=(
                second_rotation
                if second_rotation is not None
                else Transform.from_translation((START_GAP_M, 0.0, 0.0))
            ),
        ),
    )
    session = RuntimeSession.create(
        loaded, SceneSpec(placements=placements), "mujoco", gravity=(0.0, 0.0, 0.0)
    )
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.set_module_twist(CUBE_1, linear_m_s=(-APPROACH_SPEED_M_S, 0.0, 0.0))
    return session


def run_until(
    session: RuntimeSession,
    predicate: Callable[[RuntimeSession], bool],
    *,
    timeout_s: float = APPROACH_TIMEOUT_S,
) -> tuple[Event, ...]:
    """Step until ``predicate`` holds or the timeout elapses, returning events."""
    collected: list[Event] = []
    deadline = session.world.time_s + timeout_s
    while session.world.time_s < deadline:
        collected.extend(session.step(STEP_S))
        if predicate(session):
            break
    return tuple(collected)


def docked(session: RuntimeSession) -> bool:
    return bool(session.world.connections)


def relative_translation(session: RuntimeSession) -> float:
    """Return the distance between the two module origins."""
    first = session.world.modules[CUBE_0].pose.translation
    second = session.world.modules[CUBE_1].pose.translation
    return vec_norm(vec_sub(second, first))


# ----------------------------------------------------------------------
# the prototype
# ----------------------------------------------------------------------


def test_modules_coast_together_and_dock(latching_pack: LoadedRobotPack) -> None:
    session = approach_session(latching_pack)
    assert not session.world.connections

    events = run_until(session, docked)

    assert any(isinstance(event, DockCommitted) for event in events), (
        "the modules never latched; "
        f"final connector gap was {relative_translation(session) - 0.1:.4g} m"
    )
    assert session.world.assemblies.count == 1
    assert session.world.assemblies.largest_size == 2


def test_runtime_inspector_frames_follow_a_real_mujoco_dock(
    example_pack_dir: Path,
) -> None:
    """The graph/event transport observes the same session MuJoCo constrains."""
    loaded = RobotPackLoader().load(example_pack_dir)
    session = RuntimeSession.create(
        loaded,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=1.0),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
    )
    scenario = DockingPairScenario.create(
        session,
        DockingPairScenarioConfig(
            fixed_connector=FRONT_0,
            moving_connector=ConnectorInstanceId("generic_cube_1/front"),
            gap_m=0.02,
            approach_speed_m_s=APPROACH_SPEED_M_S,
            dt_s=STEP_S,
        ),
    )
    factory = ModelViewFactory()
    recipe = loaded.pack.manifest.model_views[0]
    initial = build_runtime_inspector_frame(
        session,
        recipe,
        factory,
        scenario_status=scenario.status,
    )

    try:
        while session.world.time_s < 2.0 and not session.world.connections:
            scenario.step()
        docked_frame = build_runtime_inspector_frame(
            session,
            recipe,
            factory,
            event_cursor=initial.next_event_sequence,
            scenario_status=scenario.status,
        )
    finally:
        session.shutdown()

    assert initial.view.edges == ()
    assert [event.kind for event in docked_frame.events] == [
        "DockCandidateDetected",
        "DockCommitted",
        "AssemblyMerged",
    ]
    assert len(docked_frame.view.edges) == 1
    edge = docked_frame.view.edges[0]
    assert {edge.source, edge.target} == {"generic_cube_0", "generic_cube_1"}
    assert docked_frame.metrics.connection_count_active == 1


def test_the_weld_holds_the_requested_relative_pose(latching_pack: LoadedRobotPack) -> None:
    session = approach_session(latching_pack)
    run_until(session, docked)
    assert session.world.connections

    connection = next(iter(session.world.connections.values()))
    requested = connection.relative_transform

    for _ in range(500):
        session.step(STEP_S)

    first = session.world.connector(connection.connector_a)
    second = session.world.connector(connection.connector_b)
    measured = second.world_pose.relative_to(first.world_pose)
    drift = vec_norm(vec_sub(measured.translation, requested.translation))

    # The connector type's own position tolerance is the natural bar: a weld
    # that drifts further than a dock would be accepted at is not holding.
    tolerance = 0.006
    assert drift < tolerance, f"weld drifted {drift:.4g} m from the committed pose"


def test_a_docked_pair_moves_as_one_rigid_body(latching_pack: LoadedRobotPack) -> None:
    session = approach_session(latching_pack)
    run_until(session, docked)
    assert session.world.connections

    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    separation_before = relative_translation(session)
    adapter.set_module_twist(CUBE_0, linear_m_s=(0.0, 0.0, 0.2))
    start = session.world.modules[CUBE_0].pose.translation

    for _ in range(250):
        session.step(STEP_S)

    travelled = vec_norm(vec_sub(session.world.modules[CUBE_0].pose.translation, start))
    assert travelled > 0.02, "the driven module did not move"
    assert relative_translation(session) == pytest.approx(separation_before, abs=1e-3), (
        "the welded pair did not hold together while moving"
    )


def test_undocking_lets_the_modules_separate(latching_pack: LoadedRobotPack) -> None:
    session = approach_session(latching_pack)
    run_until(session, docked)
    connection = next(iter(session.world.connections))

    session.request_undock(connection)
    events = tuple(session.step(STEP_S))
    assert any(isinstance(event, UndockCommitted) for event in events)
    # The cooldown is what stops an auto-latching pair re-forming immediately.
    assert not session.world.connections
    assert session.world.assemblies.count == 2

    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.set_module_twist(CUBE_0, linear_m_s=(0.0, 0.0, 0.3))
    separation_before = relative_translation(session)

    for _ in range(250):
        session.step(STEP_S)

    assert relative_translation(session) > separation_before + 0.02, (
        "the modules stayed together after the constraint was released"
    )


def test_without_a_cooldown_a_released_pair_relatches_immediately(
    example_pack_dir: Path,
) -> None:
    """Document the behaviour the cooldown exists to control.

    Two auto-latching connectors that are still touching satisfy acceptance the
    instant the constraint is removed, and a docking pass evaluates releases
    before detection. Re-latching in the same step is therefore correct, not a
    bug, and authoring a cooldown is how a pack opts out.
    """
    loaded = RobotPackLoader().load(example_pack_dir)
    no_cooldown = loaded.with_pack(
        with_connector_policy(loaded.pack, CONNECTOR_TYPE, auto_latch=True)
    )
    session = approach_session(no_cooldown)
    run_until(session, docked)
    first = next(iter(session.world.connections))

    session.request_undock(first)
    events = tuple(session.step(STEP_S))

    assert any(isinstance(event, UndockCommitted) for event in events)
    assert any(isinstance(event, DockCommitted) for event in events)
    assert session.world.connections


def test_the_weld_pool_is_returned_on_release(latching_pack: LoadedRobotPack) -> None:
    session = approach_session(latching_pack)
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    capacity = adapter.weld_pool.capacity
    assert capacity > 0

    run_until(session, docked)
    assert adapter.weld_pool.in_use == 1

    session.request_undock(next(iter(session.world.connections)))
    session.step(STEP_S)
    assert adapter.weld_pool.in_use == 0
    assert adapter.weld_pool.available == capacity


def test_an_exhausted_weld_pool_refuses_rather_than_faking(
    latching_pack: LoadedRobotPack,
) -> None:
    placements = (
        ModulePlacement(instance_id=CUBE_0, module_type_id=MODULE_TYPE),
        ModulePlacement(
            instance_id=CUBE_1,
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((START_GAP_M, 0.0, 0.0)),
        ),
    )
    session = RuntimeSession.create(
        latching_pack,
        SceneSpec(placements=placements),
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
        weld_pool_size=0,
    )
    adapter = session.adapter
    assert isinstance(adapter, MuJoCoBackendAdapter)
    adapter.set_module_twist(CUBE_1, linear_m_s=(-APPROACH_SPEED_M_S, 0.0, 0.0))

    def dock_failed(active: RuntimeSession) -> bool:
        return bool(active.world.event_log.of_kind(DockFailed))

    events = run_until(session, dock_failed)

    failures = [event for event in events if isinstance(event, DockFailed)]
    assert failures, "an exhausted pool should report a failure, not latch"
    assert "weld slots are in use" in failures[0].detail
    assert not session.world.connections


def test_docking_holds_through_an_asymmetric_rotation(latching_pack: LoadedRobotPack) -> None:
    """Guard the relpose rotation convention.

    A front-to-rear mate on this pack is a 180-degree relative rotation, which
    is symmetric enough to hide a sign error. Rolling the approaching module a
    quarter turn about the docking axis exercises an asymmetric relative
    orientation, which the connector type's discrete orientation set allows.
    """
    rolled = Transform(
        translation=(START_GAP_M, 0.0, 0.0),
        rotation=quat_from_axis_angle((1.0, 0.0, 0.0), 1.5707963267948966),
    )
    session = approach_session(latching_pack, second_rotation=rolled)
    run_until(session, docked)
    assert session.world.connections, "a quarter-turn mate never latched"

    connection = next(iter(session.world.connections.values()))
    requested = connection.relative_transform
    for _ in range(500):
        session.step(STEP_S)

    first = session.world.connector(connection.connector_a)
    second = session.world.connector(connection.connector_b)
    measured = second.world_pose.relative_to(first.world_pose)

    assert measured.is_close(
        requested, position_tolerance_m=0.006, orientation_tolerance_rad=0.14
    ), "the weld did not preserve the committed relative orientation"
