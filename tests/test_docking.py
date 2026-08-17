"""End-to-end docking execution against the mock backend."""

from __future__ import annotations

import math

import pytest

from conftest import with_connector_field, with_connector_policy
from modsim.backends.base import BackendAdapter, BackendError
from modsim.backends.mock import MockBackendAdapter
from modsim.core.entities import ConnectorLifecycleState
from modsim.core.events import (
    AssemblyMerged,
    AssemblySplit,
    ConnectorOverloaded,
    DockCandidateDetected,
    DockCommitted,
    DockFailed,
    DockFailureReason,
    Event,
    UndockCommitted,
    UndockFailed,
)
from modsim.core.ids import ConnectionId, ConnectorInstanceId, ConstraintHandle, ModuleInstanceId
from modsim.core.scene import SceneError, SceneSpec
from modsim.core.state import WorldState, WorldStateError
from modsim.core.transforms import Transform, vec_norm, vec_sub
from modsim.robot_packs.schema import AlignmentMode, RobotPack
from modsim.runtime.session import RuntimeSession

CONNECTOR_TYPE = "fixed_face"
MODULE_TYPE = "generic_cube"
SPACING_M = 0.1

CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")
FRONT_1 = ConnectorInstanceId("generic_cube_1/front")
REAR_2 = ConnectorInstanceId("generic_cube_2/rear")
REAR_0 = ConnectorInstanceId("generic_cube_0/rear")


def session_for(
    pack: RobotPack,
    count: int = 2,
    *,
    adapter: MockBackendAdapter | None = None,
) -> RuntimeSession:
    """Return a session with ``count`` cubes spaced so their connectors coincide."""
    scene = SceneSpec.grid(MODULE_TYPE, count, spacing_m=SPACING_M)
    return RuntimeSession.create(pack, scene, adapter or MockBackendAdapter())


def kinds(events: tuple[Event, ...]) -> list[str]:
    return [event.kind for event in events]


def first_of(events: tuple[Event, ...], kind: type[Event]) -> Event:
    for event in events:
        if isinstance(event, kind):
            return event
    raise AssertionError(f"no {kind.__name__} in {kinds(events)}")


# ----------------------------------------------------------------------
# scene and world construction
# ----------------------------------------------------------------------


def test_scene_instantiates_free_modules(example_pack: RobotPack) -> None:
    session = session_for(example_pack, count=3)
    world = session.world

    assert len(world.modules) == 3
    assert len(world.connectors) == 6
    assert world.assemblies.count == 3
    assert world.assemblies.largest_size == 1
    assert all(
        connector.lifecycle_state is ConnectorLifecycleState.FREE
        for connector in world.connectors.values()
    )


def test_scene_rejects_unknown_module_types(example_pack: RobotPack) -> None:
    scene = SceneSpec.grid("not_a_module", 2, spacing_m=SPACING_M)
    with pytest.raises(SceneError, match="unknown module type"):
        scene.validate_against(example_pack)


def test_connector_world_frames_follow_their_module(example_pack: RobotPack) -> None:
    session = session_for(example_pack)
    front = session.world.connector(FRONT_0)
    rear = session.world.connector(REAR_1)

    assert vec_norm(vec_sub(front.world_pose.translation, rear.world_pose.translation)) == (
        pytest.approx(0.0, abs=1e-12)
    )
    assert front.world_docking_axis == pytest.approx((1.0, 0.0, 0.0))
    assert rear.world_docking_axis == pytest.approx((-1.0, 0.0, 0.0))


# ----------------------------------------------------------------------
# commanded docking
# ----------------------------------------------------------------------


def test_docking_requires_an_explicit_command_by_default(example_pack: RobotPack) -> None:
    session = session_for(example_pack)
    events = session.step(0.01)

    assert "DockCommitted" not in kinds(events)
    proposal = session.proposals()[0]
    assert proposal.acceptance.satisfied
    assert proposal.guard.code == "command_required"


def test_commanded_dock_commits_and_merges_assemblies(example_pack: RobotPack) -> None:
    session = session_for(example_pack)
    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    committed = first_of(events, DockCommitted)
    assert isinstance(committed, DockCommitted)
    assert committed.connection_id == ConnectionId(f"{FRONT_0}<->{REAR_1}")
    assert committed.orientation_index == 2  # pi, the rear connector's authored roll

    merged = first_of(events, AssemblyMerged)
    assert isinstance(merged, AssemblyMerged)
    assert merged.merged_from == ("assembly:generic_cube_0", "assembly:generic_cube_1")

    world = session.world
    assert world.assemblies.count == 1
    assert len(world.connections) == 1
    assert world.connector(FRONT_0).lifecycle_state is ConnectorLifecycleState.DOCKED
    assert world.connector(REAR_1).connection_id == committed.connection_id


def test_auto_latching_connectors_dock_without_a_command(example_pack: RobotPack) -> None:
    pack = with_connector_policy(example_pack, CONNECTOR_TYPE, auto_latch=True)
    session = session_for(pack)
    events = session.step(0.01)

    assert "DockCommitted" in kinds(events)
    assert session.world.assemblies.count == 1


def test_a_connector_commits_at_most_once_per_pass(example_pack: RobotPack) -> None:
    pack = with_connector_policy(example_pack, CONNECTOR_TYPE, auto_latch=True)
    session = session_for(pack, count=3)
    events = session.step(0.01)

    assert kinds(events).count("DockCommitted") == 2
    assert session.world.assemblies.count == 1
    assert session.world.assemblies.largest_size == 3


def test_docking_two_connectors_on_one_module_is_refused(example_pack: RobotPack) -> None:
    session = session_for(example_pack)
    session.request_dock(FRONT_0, REAR_0)
    events = session.step(0.01)

    failed = first_of(events, DockFailed)
    assert isinstance(failed, DockFailed)
    assert failed.reason is DockFailureReason.GUARD_REJECTED
    assert "same module" in failed.detail


def test_a_commanded_pair_outside_acceptance_reports_why(example_pack: RobotPack) -> None:
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=1.0)
    session = RuntimeSession.create(example_pack, scene, MockBackendAdapter())
    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    failed = first_of(events, DockFailed)
    assert isinstance(failed, DockFailed)
    assert failed.reason is DockFailureReason.OUTSIDE_ACCEPTANCE_REGION
    assert "position out of tolerance" in failed.detail


def test_a_failed_dock_command_is_consumed_after_one_attempt(example_pack: RobotPack) -> None:
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=1.0)
    session = RuntimeSession.create(example_pack, scene, MockBackendAdapter())
    session.request_dock(FRONT_0, REAR_1)

    first = session.step(0.01)
    second = session.step(0.01)

    assert len([event for event in first if isinstance(event, DockFailed)]) == 1
    assert not [event for event in second if isinstance(event, DockFailed)]
    assert len(session.world.event_log.of_kind(DockFailed)) == 1


# ----------------------------------------------------------------------
# two-phase commit
# ----------------------------------------------------------------------


def test_backend_refusal_leaves_no_logical_connection(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = session_for(example_pack, adapter=adapter)
    adapter.fail_next_connection("no constraint slots available")
    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    failed = first_of(events, DockFailed)
    assert isinstance(failed, DockFailed)
    assert failed.reason is DockFailureReason.BACKEND_REFUSED
    assert failed.detail == "no constraint slots available"

    world = session.world
    assert not world.connections
    assert world.assemblies.count == 2
    assert world.connector(FRONT_0).lifecycle_state is ConnectorLifecycleState.FAILED


def test_a_failed_dock_does_not_strand_the_connector(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = session_for(example_pack, adapter=adapter)
    adapter.fail_next_connection()
    session.request_dock(FRONT_0, REAR_1)
    session.step(0.01)

    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)
    assert "DockCommitted" in kinds(events)


def test_a_backend_without_runtime_constraints_is_refused(example_pack: RobotPack) -> None:
    class ReadOnlyBackend(MockBackendAdapter):
        def capabilities(self):  # type: ignore[no-untyped-def]
            return super().capabilities().__class__(name="read_only")

    adapter = ReadOnlyBackend()
    session = session_for(example_pack, adapter=adapter)
    assert isinstance(adapter, BackendAdapter)
    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    failed = first_of(events, DockFailed)
    assert isinstance(failed, DockFailed)
    assert "cannot create constraints at runtime" in failed.detail
    assert not session.world.connections


# ----------------------------------------------------------------------
# undocking
# ----------------------------------------------------------------------


def test_undocking_splits_the_assembly(example_pack: RobotPack) -> None:
    session = session_for(example_pack)
    session.request_dock(FRONT_0, REAR_1)
    session.step(0.01)
    connection = next(iter(session.world.connections))

    session.request_undock(connection)
    events = session.step(0.01)

    assert isinstance(first_of(events, UndockCommitted), UndockCommitted)
    split = first_of(events, AssemblySplit)
    assert isinstance(split, AssemblySplit)
    assert split.resulting == ("assembly:generic_cube_0", "assembly:generic_cube_1")
    assert session.world.assemblies.count == 2
    assert not session.world.connections
    # The two connectors are still nose to nose, so detection immediately sees
    # them again; what matters is that neither holds a connection any more.
    released = session.world.connector(FRONT_0)
    assert not released.is_engaged
    assert released.connection_id is None


def test_dock_then_undock_restores_the_original_partition(example_pack: RobotPack) -> None:
    session = session_for(example_pack, count=3)
    before = {
        session.world.assemblies.members(name) for name in session.world.assemblies.assemblies
    }

    session.request_dock(FRONT_0, REAR_1)
    session.request_dock(FRONT_1, REAR_2)
    session.step(0.01)
    assert session.world.assemblies.count == 1

    for connection in tuple(session.world.connections):
        session.request_undock(connection)
    session.step(0.01)

    after = {session.world.assemblies.members(name) for name in session.world.assemblies.assemblies}
    assert after == before


def test_undocking_an_unknown_connection_is_reported(example_pack: RobotPack) -> None:
    session = session_for(example_pack)
    session.request_undock(ConnectionId("does<->not_exist"))
    events = session.step(0.01)

    failed = first_of(events, UndockFailed)
    assert isinstance(failed, UndockFailed)
    assert failed.reason is DockFailureReason.UNKNOWN_CONNECTION


def test_a_permanent_connector_refuses_to_undock(example_pack: RobotPack) -> None:
    pack = with_connector_field(example_pack, CONNECTOR_TYPE, supports_undocking=False)
    session = session_for(pack)
    session.request_dock(FRONT_0, REAR_1)
    session.step(0.01)
    connection = next(iter(session.world.connections))

    session.request_undock(connection)
    events = session.step(0.01)

    failed = first_of(events, UndockFailed)
    assert isinstance(failed, UndockFailed)
    assert failed.reason is DockFailureReason.UNDOCKING_UNSUPPORTED
    assert len(session.world.connections) == 1


def test_redock_cooldown_blocks_an_immediate_reattachment(example_pack: RobotPack) -> None:
    pack = with_connector_policy(
        example_pack, CONNECTOR_TYPE, auto_latch=True, redock_cooldown_s=1.0
    )
    session = session_for(pack)
    session.step(0.01)
    connection = next(iter(session.world.connections))

    session.request_undock(connection)
    session.step(0.01)
    assert not session.world.connections

    session.step(0.01)
    assert not session.world.connections
    assert session.proposals()[0].guard.code == "cooldown"

    for _ in range(200):
        session.step(0.01)
    assert session.world.connections


# ----------------------------------------------------------------------
# policy
# ----------------------------------------------------------------------


def test_nominal_alignment_snaps_the_docked_module(example_pack: RobotPack) -> None:
    pack = with_connector_policy(
        example_pack,
        CONNECTOR_TYPE,
        auto_latch=True,
        alignment=AlignmentMode.NOMINAL,
    )
    adapter = MockBackendAdapter()
    session = session_for(pack, adapter=adapter)
    adapter.set_module_pose(CUBE_1, Transform.from_translation((SPACING_M, 0.003, 0.0)))
    session.step(0.01)
    assert session.world.connections

    # The snap happens inside the backend at commit time, so the world sees it
    # on the next ingest rather than in the step that committed the dock.
    session.step(0.01)
    front = session.world.connector(FRONT_0)
    rear = session.world.connector(REAR_1)
    assert vec_norm(vec_sub(front.world_pose.translation, rear.world_pose.translation)) == (
        pytest.approx(0.0, abs=1e-9)
    )


def test_measured_alignment_freezes_the_offset_that_existed_at_latch(
    example_pack: RobotPack,
) -> None:
    pack = with_connector_policy(example_pack, CONNECTOR_TYPE, auto_latch=True)
    adapter = MockBackendAdapter()
    session = session_for(pack, adapter=adapter)
    adapter.set_module_pose(CUBE_1, Transform.from_translation((SPACING_M, 0.003, 0.0)))
    session.step(0.01)

    assert session.world.connections
    front = session.world.connector(FRONT_0)
    rear = session.world.connector(REAR_1)
    assert vec_norm(vec_sub(front.world_pose.translation, rear.world_pose.translation)) == (
        pytest.approx(0.003, abs=1e-9)
    )


def test_break_force_releases_an_overloaded_connection(example_pack: RobotPack) -> None:
    pack = with_connector_policy(example_pack, CONNECTOR_TYPE, auto_latch=True, break_force_n=50.0)
    adapter = MockBackendAdapter()
    session = session_for(pack, adapter=adapter)
    session.step(0.01)
    handle = adapter.welds[0]

    adapter.set_constraint_force(handle, 80.0)
    events = session.step(0.01)

    overload = first_of(events, ConnectorOverloaded)
    assert isinstance(overload, ConnectorOverloaded)
    assert overload.measured_force_n == pytest.approx(80.0)
    assert overload.limit_n == pytest.approx(50.0)
    assert "UndockCommitted" in kinds(events)


def test_conflicting_physical_connection_intent_blocks_docking(
    example_pack: RobotPack,
) -> None:
    data = example_pack.model_dump(mode="python")
    connector_types = data["hardware_catalog"]["connector_types"]
    connector_types["compliant_face"] = {
        **connector_types[CONNECTOR_TYPE],
        "id": "compliant_face",
        "compatible_with": [CONNECTOR_TYPE],
        "physical_connection": {
            "constraint": "compliant",
            "compliance": {
                "translational_stiffness_n_per_m": 1000.0,
                "rotational_stiffness_nm_per_rad": 100.0,
            },
        },
    }
    connector_types[CONNECTOR_TYPE]["compatible_with"] = [CONNECTOR_TYPE, "compliant_face"]
    module = data["hardware_catalog"]["module_types"][MODULE_TYPE]
    module["connectors"][1]["connector_type"] = "compliant_face"
    pack = RobotPack.model_validate(data)

    session = session_for(pack)
    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    failed = first_of(events, DockFailed)
    assert isinstance(failed, DockFailed)
    assert "conflicting physical connections" in failed.detail


def test_loop_closure_is_permitted_but_flagged(example_pack: RobotPack) -> None:
    pack = with_connector_policy(example_pack, CONNECTOR_TYPE, auto_latch=True)
    adapter = MockBackendAdapter()
    session = session_for(pack, count=2, adapter=adapter)
    session.step(0.01)
    assert session.world.assemblies.count == 1

    # Wrap module 1 back around so its free front meets module 0's free rear.
    adapter.set_module_pose(CUBE_1, Transform.from_translation((-SPACING_M, 0.0, 0.0)))
    session.request_dock(REAR_0, FRONT_1)
    proposals = [proposal for proposal in session.proposals() if proposal.connector_a == REAR_0]
    assert proposals
    assert proposals[0].guard.allowed
    assert any("kinematic loop" in warning for warning in proposals[0].guard.warnings)


# ----------------------------------------------------------------------
# invariants
# ----------------------------------------------------------------------


def test_a_docked_chain_moves_as_one_rigid_body(example_pack: RobotPack) -> None:
    pack = with_connector_policy(example_pack, CONNECTOR_TYPE, auto_latch=True)
    adapter = MockBackendAdapter()
    session = session_for(pack, count=3, adapter=adapter)
    session.step(0.01)
    assert session.world.assemblies.largest_size == 3

    before = {
        module_id: module.pose.translation for module_id, module in session.world.modules.items()
    }
    adapter.set_module_twist(CUBE_0, linear_m_s=(0.0, 0.0, 1.0))
    for _ in range(10):
        session.step(0.01)

    for module_id, module in session.world.modules.items():
        moved = vec_sub(module.pose.translation, before[module_id])
        assert moved == pytest.approx((0.0, 0.0, 0.1), abs=1e-9)


def test_a_pair_in_range_is_logged_once_not_every_step(example_pack: RobotPack) -> None:
    """The event log records transitions, not steady state.

    Two connectors parked in range would otherwise emit a detection every step,
    which makes the log useless over a run of any length.
    """
    session = session_for(example_pack)
    for _ in range(20):
        session.step(0.01)

    detections = session.world.event_log.of_kind(DockCandidateDetected)
    assert len(detections) == 1


def test_leaving_and_re_entering_range_logs_a_second_detection(
    example_pack: RobotPack,
) -> None:
    adapter = MockBackendAdapter()
    session = session_for(example_pack, adapter=adapter)
    session.step(0.01)
    assert len(session.world.event_log.of_kind(DockCandidateDetected)) == 1

    adapter.set_module_pose(CUBE_1, Transform.from_translation((5.0, 0.0, 0.0)))
    session.step(0.01)
    adapter.set_module_pose(CUBE_1, Transform.from_translation((SPACING_M, 0.0, 0.0)))
    session.step(0.01)

    assert len(session.world.event_log.of_kind(DockCandidateDetected)) == 2


def test_event_sequence_numbers_are_dense_and_monotonic(example_pack: RobotPack) -> None:
    session = session_for(example_pack, count=3)
    session.request_dock(FRONT_0, REAR_1)
    session.step(0.01)
    session.request_undock(next(iter(session.world.connections)))
    session.step(0.01)

    sequences = [event.sequence for event in session.world.event_log]
    assert sequences == list(range(len(sequences)))


def test_connection_ids_do_not_depend_on_command_order(example_pack: RobotPack) -> None:
    forward = session_for(example_pack)
    forward.request_dock(FRONT_0, REAR_1)
    forward.step(0.01)

    reverse = session_for(example_pack)
    reverse.request_dock(REAR_1, FRONT_0)
    reverse.step(0.01)

    assert tuple(forward.world.connections) == tuple(reverse.world.connections)


def test_metrics_track_the_docking_lifecycle(example_pack: RobotPack) -> None:
    session = session_for(example_pack, count=3)
    session.request_dock(FRONT_0, REAR_1)
    session.step(0.01)
    metrics = session.metrics()

    assert metrics.module_count_total == 3
    assert metrics.module_count_connected == 2
    assert metrics.module_count_free == 1
    assert metrics.assembly_count == 2
    assert metrics.largest_assembly_size == 2
    assert metrics.connection_count_active == 1
    assert metrics.connector_count_docked == 2
    assert metrics.docking_success_count == 1
    assert metrics.docking_success_rate == pytest.approx(1.0)
    assert metrics.as_dict()["assembly_merge_count"] == 1


def test_applying_a_dock_to_an_engaged_connector_is_rejected(example_pack: RobotPack) -> None:
    session = session_for(example_pack)
    session.request_dock(FRONT_0, REAR_1)
    session.step(0.01)
    committed = session.world.event_log.of_kind(DockCommitted)[0]

    with pytest.raises(WorldStateError, match="already engaged"):
        session.world.apply(committed)


def test_lifecycle_states_implying_a_connection_cannot_be_set_directly(
    example_pack: RobotPack,
) -> None:
    world = WorldState.from_scene(example_pack, SceneSpec.grid(MODULE_TYPE, 1, spacing_m=SPACING_M))
    with pytest.raises(WorldStateError, match="must be reached through an event"):
        world.mark_lifecycle(FRONT_0, ConnectorLifecycleState.DOCKED)


def test_mock_backend_rejects_unknown_modules_and_constraints() -> None:
    adapter = MockBackendAdapter()
    with pytest.raises(BackendError, match="unknown module"):
        adapter.set_module_pose(ModuleInstanceId("nobody"), Transform.identity())
    with pytest.raises(BackendError, match="unknown constraint"):
        adapter.set_constraint_force(ConstraintHandle("weld:nothing"), 1.0)
    with pytest.raises(BackendError, match="step backwards"):
        adapter.step(-1.0)
    assert not adapter.remove_physical_connection(ConstraintHandle("weld:nothing"))


def test_rotating_a_module_moves_its_connector_frames(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = session_for(example_pack, adapter=adapter)
    adapter.set_module_twist(CUBE_0, angular_rad_s=(0.0, 0.0, math.pi / 2))
    session.step(1.0)

    front = session.world.connector(FRONT_0)
    assert front.world_pose.translation == pytest.approx((0.0, 0.05, 0.0), abs=1e-9)
    assert front.world_docking_axis == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    # A connector away from the spin axis carries the omega-cross-r term.
    assert vec_norm(front.world_velocity_m_s) > 0.0
