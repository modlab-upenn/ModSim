"""Focused tests for independent ``WorldState`` revision domains."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from modsim.core import (
    AssemblyId,
    AssemblyMerged,
    AssemblySplit,
    BackendStateSnapshot,
    ConnectionId,
    ConnectorInstanceId,
    ConnectorLifecycleState,
    ConnectorOverloaded,
    ConstraintHandle,
    DockCommitted,
    DockFailed,
    DockFailureReason,
    SceneSpec,
    Transform,
    UndockCommitted,
    WorldState,
    WorldStateRevision,
)
from modsim.robot_packs.schema import RobotPack

MODULE_TYPE = "generic_cube"
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")
CONNECTION = ConnectionId(f"{FRONT_0}<->{REAR_1}")


def make_world(pack: RobotPack) -> WorldState:
    """Return a two-module world without starting a backend."""
    return WorldState.from_scene(pack, SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1))


def committed_event() -> DockCommitted:
    """Return the fixed connection event shared by the revision tests."""
    return DockCommitted(
        time_s=0.0,
        connection_id=CONNECTION,
        connector_a=FRONT_0,
        connector_b=REAR_1,
        constraint_handle=ConstraintHandle("constraint:test"),
        relative_transform=Transform.identity(),
    )


def test_revision_starts_at_zero_and_is_frozen(example_pack: RobotPack) -> None:
    world = make_world(example_pack)

    assert world.revision == WorldStateRevision()
    with pytest.raises(FrozenInstanceError):
        world.revision.sample_sequence = 1  # type: ignore[misc]


def test_every_ingested_snapshot_advances_the_sample_sequence(
    example_pack: RobotPack,
) -> None:
    world = make_world(example_pack)
    zero_time_snapshot = BackendStateSnapshot(time_s=0.0)

    world.ingest(zero_time_snapshot)
    world.ingest(zero_time_snapshot)

    assert world.revision == WorldStateRevision(sample_sequence=2)


def test_transient_reset_advances_docking_only_when_state_changes(
    example_pack: RobotPack,
) -> None:
    world = make_world(example_pack)
    world.mark_lifecycle(FRONT_0, ConnectorLifecycleState.CANDIDATE_DETECTED)
    assert world.revision.docking_revision == 1

    world.ingest(BackendStateSnapshot(time_s=0.0))
    assert world.connector(FRONT_0).lifecycle_state is ConnectorLifecycleState.FREE
    assert world.revision == WorldStateRevision(sample_sequence=1, docking_revision=2)

    world.ingest(BackendStateSnapshot(time_s=0.0))
    assert world.revision == WorldStateRevision(sample_sequence=2, docking_revision=2)


def test_mark_lifecycle_does_not_advance_for_unchanged_or_engaged_connectors(
    example_pack: RobotPack,
) -> None:
    world = make_world(example_pack)

    world.mark_lifecycle(FRONT_0, ConnectorLifecycleState.FREE)
    assert world.revision.docking_revision == 0

    world.mark_lifecycle(FRONT_0, ConnectorLifecycleState.ALIGNING)
    world.mark_lifecycle(FRONT_0, ConnectorLifecycleState.ALIGNING)
    assert world.revision.docking_revision == 1

    world.apply(committed_event())
    before = world.revision
    world.mark_lifecycle(FRONT_0, ConnectorLifecycleState.FREE)
    assert world.revision == before


def test_committed_topology_events_advance_topology_and_docking_once(
    example_pack: RobotPack,
) -> None:
    world = make_world(example_pack)

    world.apply(committed_event())
    assert world.revision == WorldStateRevision(
        topology_revision=1,
        docking_revision=1,
        event_revision=1,
    )

    world.apply(
        AssemblyMerged(
            time_s=0.0,
            assembly_id=AssemblyId("assembly:generic_cube_0"),
            merged_from=(
                AssemblyId("assembly:generic_cube_0"),
                AssemblyId("assembly:generic_cube_1"),
            ),
        )
    )
    assert world.revision == WorldStateRevision(
        topology_revision=1,
        docking_revision=1,
        event_revision=2,
    )

    world.apply(
        UndockCommitted(
            time_s=1.0,
            connection_id=CONNECTION,
            connector_a=FRONT_0,
            connector_b=REAR_1,
        )
    )
    assert world.revision == WorldStateRevision(
        topology_revision=2,
        docking_revision=2,
        event_revision=3,
    )

    world.apply(
        AssemblySplit(
            time_s=1.0,
            source_assembly_id=AssemblyId("assembly:generic_cube_0"),
            resulting=(
                AssemblyId("assembly:generic_cube_0"),
                AssemblyId("assembly:generic_cube_1"),
            ),
        )
    )
    assert world.revision == WorldStateRevision(
        topology_revision=2,
        docking_revision=2,
        event_revision=4,
    )


def test_events_always_advance_event_revision_but_docking_requires_a_change(
    example_pack: RobotPack,
) -> None:
    world = make_world(example_pack)
    failure = DockFailed(
        time_s=0.0,
        connector_a=FRONT_0,
        connector_b=REAR_1,
        reason=DockFailureReason.BACKEND_REFUSED,
    )

    world.apply(failure)
    world.apply(failure)

    assert world.revision == WorldStateRevision(docking_revision=1, event_revision=2)


def test_load_revision_advances_only_when_a_connection_measurement_changes(
    example_pack: RobotPack,
) -> None:
    world = make_world(example_pack)
    world.apply(committed_event())
    overload = ConnectorOverloaded(
        time_s=0.1,
        connection_id=CONNECTION,
        measured_force_n=12.0,
        limit_n=10.0,
    )

    world.apply(overload)
    world.apply(overload)
    world.apply(
        ConnectorOverloaded(
            time_s=0.2,
            connection_id=CONNECTION,
            measured_force_n=13.0,
            limit_n=10.0,
        )
    )

    assert world.revision == WorldStateRevision(
        topology_revision=1,
        docking_revision=3,
        event_revision=4,
    )
