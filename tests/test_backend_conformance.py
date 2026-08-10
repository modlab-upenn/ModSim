"""Cross-backend conformance.

Two backends given the same Robot Pack and the same scene must present ModSim
with the same *semantic* picture: the same modules, the same assemblies, the
same connector frames, and the same docking verdicts. Poses are allowed to
differ once real dynamics act, so every comparison here runs with gravity
disabled and no applied motion, which isolates adapter correctness from physics.

This is the test that catches a MuJoCo adapter reporting a connector in the
wrong frame, mismapping a link, or losing a module, because each of those shows
up as a disagreement with the mock's reference semantics.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import pytest

from modsim.backends.registry import installed_backends
from modsim.connectors.docking import DockProposal
from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.transforms import Transform, angle_between, vec_norm, vec_sub
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "generic_cube"
SPACING_M = 0.1
MODULE_COUNT = 3
POSITION_TOLERANCE_M = 1e-6
ANGLE_TOLERANCE_RAD = 1e-6
STEP_S = 0.002

COMPARABLE_BACKENDS = ("mock", "mujoco")


def backends_under_test() -> tuple[str, ...]:
    """Return every comparable backend whose dependencies are installed."""
    installed = installed_backends()
    return tuple(name for name in COMPARABLE_BACKENDS if name in installed)


@pytest.fixture
def loaded_pack(example_pack_dir: Path) -> LoadedRobotPack:
    return RobotPackLoader().load(example_pack_dir)


def make_session(loaded: LoadedRobotPack, backend: str) -> RuntimeSession:
    """Build a session on ``backend`` with gravity neutralised where it exists."""
    scene = SceneSpec.grid(MODULE_TYPE, MODULE_COUNT, spacing_m=SPACING_M)
    if backend == "mock":
        return RuntimeSession.create(loaded, scene, backend)
    return RuntimeSession.create(loaded, scene, backend, gravity=(0.0, 0.0, 0.0))


def sessions(loaded: LoadedRobotPack) -> Iterator[tuple[str, RuntimeSession]]:
    for backend in backends_under_test():
        yield backend, make_session(loaded, backend)


def proposal_verdicts(
    proposals: tuple[DockProposal, ...],
) -> dict[tuple[str, str], tuple[bool, bool, str | None]]:
    """Reduce proposals to a comparable summary keyed by connector pair."""
    return {
        (proposal.connector_a, proposal.connector_b): (
            proposal.compatibility.compatible,
            proposal.acceptance.satisfied,
            proposal.guard.code,
        )
        for proposal in proposals
    }


def requires_two_backends() -> None:
    if len(backends_under_test()) < 2:
        pytest.skip("needs at least two installed backends to compare")


# ----------------------------------------------------------------------
# per-backend invariants
# ----------------------------------------------------------------------


@pytest.mark.parametrize("backend", backends_under_test())
def test_every_backend_instantiates_the_same_scene(
    loaded_pack: LoadedRobotPack, backend: str
) -> None:
    session = make_session(loaded_pack, backend)

    assert set(session.world.modules) == {
        ModuleInstanceId(f"{MODULE_TYPE}_{index}") for index in range(MODULE_COUNT)
    }
    assert session.world.assemblies.count == MODULE_COUNT
    assert len(session.world.connectors) == MODULE_COUNT * 2


@pytest.mark.parametrize("backend", backends_under_test())
def test_every_backend_resolves_all_connector_frames(
    loaded_pack: LoadedRobotPack, backend: str
) -> None:
    session = make_session(loaded_pack, backend)

    assert all(connector.resolved for connector in session.world.connectors.values())


@pytest.mark.parametrize("backend", backends_under_test())
def test_every_backend_reports_its_own_name(loaded_pack: LoadedRobotPack, backend: str) -> None:
    session = make_session(loaded_pack, backend)

    assert session.adapter.capabilities().name == backend


# ----------------------------------------------------------------------
# cross-backend agreement
# ----------------------------------------------------------------------


def test_backends_agree_on_connector_world_frames(loaded_pack: LoadedRobotPack) -> None:
    requires_two_backends()
    reference: dict[str, Transform] | None = None
    reference_axes: dict[str, tuple[float, float, float]] = {}

    for backend, session in sessions(loaded_pack):
        poses = {
            str(connector.id): connector.world_pose
            for connector in session.world.connectors.values()
        }
        axes = {
            str(connector.id): connector.world_docking_axis
            for connector in session.world.connectors.values()
        }
        if reference is None:
            reference, reference_axes = poses, axes
            continue

        assert set(poses) == set(reference), f"{backend} reports a different connector set"
        for name, pose in poses.items():
            offset = vec_norm(vec_sub(pose.translation, reference[name].translation))
            assert offset == pytest.approx(0.0, abs=POSITION_TOLERANCE_M), (
                f"{backend} places connector '{name}' {offset:.3g} m from the reference"
            )
            assert angle_between(axes[name], reference_axes[name]) == pytest.approx(
                0.0, abs=ANGLE_TOLERANCE_RAD
            ), f"{backend} points connector '{name}' in a different direction"


def test_backends_agree_on_docking_proposals(loaded_pack: LoadedRobotPack) -> None:
    requires_two_backends()
    reference: dict[tuple[str, str], tuple[bool, bool, str | None]] | None = None

    for backend, session in sessions(loaded_pack):
        verdicts = proposal_verdicts(session.proposals())
        if reference is None:
            reference = verdicts
            continue
        assert verdicts == reference, (
            f"{backend} reached different docking verdicts than the reference backend"
        )


def test_backends_agree_on_initial_assemblies(loaded_pack: LoadedRobotPack) -> None:
    requires_two_backends()
    reference: set[frozenset[ModuleInstanceId]] | None = None

    for _, session in sessions(loaded_pack):
        index = session.world.assemblies
        partition = {index.members(name) for name in index.assemblies}
        if reference is None:
            reference = partition
            continue
        assert partition == reference


def test_a_quiescent_scene_does_not_drift_on_any_backend(loaded_pack: LoadedRobotPack) -> None:
    requires_two_backends()
    for backend, session in sessions(loaded_pack):
        before = {
            module_id: module.pose.translation
            for module_id, module in session.world.modules.items()
        }
        for _ in range(20):
            session.step(0.005)

        for module_id, module in session.world.modules.items():
            drift = vec_norm(vec_sub(module.pose.translation, before[module_id]))
            assert drift == pytest.approx(0.0, abs=1e-6), (
                f"{backend} drifted module '{module_id}' by {drift:.3g} m with no forces applied"
            )


def test_a_backend_without_runtime_constraints_commits_nothing(
    loaded_pack: LoadedRobotPack,
) -> None:
    requires_two_backends()
    for backend, session in sessions(loaded_pack):
        supports = session.adapter.capabilities().supports_runtime_constraints
        for proposal in session.proposals():
            session.request_dock(proposal.connector_a, proposal.connector_b)
        session.step(0.01)

        committed = bool(session.world.connections)
        assert committed == supports, (
            f"{backend} reports supports_runtime_constraints={supports} "
            f"but {'committed' if committed else 'committed no'} connections"
        )


# ----------------------------------------------------------------------
# event-sequence conformance
# ----------------------------------------------------------------------


def scripted_run(session: RuntimeSession) -> tuple[str, ...]:
    """Dock every viable pair, then release every connection.

    Returns the sequence of event kinds. Poses are allowed to differ between
    backends once dynamics act; the *decisions* ModSim makes are not.
    """
    kinds: list[str] = []
    for proposal in session.proposals():
        session.request_dock(proposal.connector_a, proposal.connector_b)
    kinds.extend(event.kind for event in session.step(STEP_S))

    for connection in sorted(session.world.connections):
        session.request_undock(connection)
    kinds.extend(event.kind for event in session.step(STEP_S))
    return tuple(kinds)


def test_backends_agree_on_the_committed_event_sequence(loaded_pack: LoadedRobotPack) -> None:
    """The strongest correctness signal available for a backend adapter.

    Two engines running the same scenario must reach the same semantic
    conclusions in the same order. A disagreement means an adapter is wrong,
    because every decision in the sequence is made by backend-agnostic core
    code from the state the backend reported.
    """
    requires_two_backends()
    reference: tuple[str, ...] | None = None
    reference_backend = ""

    for backend, session in sessions(loaded_pack):
        sequence = scripted_run(session)
        if reference is None:
            reference, reference_backend = sequence, backend
            continue
        assert sequence == reference, (
            f"{backend} produced {sequence} where {reference_backend} produced {reference}"
        )


def test_backends_agree_on_connection_identity_and_orientation(
    loaded_pack: LoadedRobotPack,
) -> None:
    requires_two_backends()
    reference: dict[str, int | None] | None = None

    for backend, session in sessions(loaded_pack):
        for proposal in session.proposals():
            session.request_dock(proposal.connector_a, proposal.connector_b)
        session.step(STEP_S)
        committed = {
            str(connection.id): connection.orientation_index
            for connection in session.world.connections.values()
        }
        assert committed, f"{backend} committed no connections"
        if reference is None:
            reference = committed
            continue
        assert committed == reference, (
            f"{backend} disagreed on connection identity or mating orientation"
        )


def test_backends_agree_on_the_assembly_partition_after_docking(
    loaded_pack: LoadedRobotPack,
) -> None:
    requires_two_backends()
    reference: set[frozenset[ModuleInstanceId]] | None = None

    for _, session in sessions(loaded_pack):
        for proposal in session.proposals():
            session.request_dock(proposal.connector_a, proposal.connector_b)
        session.step(STEP_S)
        index = session.world.assemblies
        partition = {index.members(name) for name in index.assemblies}
        if reference is None:
            reference = partition
            continue
        assert partition == reference
