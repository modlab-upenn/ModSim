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

import math
from collections.abc import Iterator
from dataclasses import replace
from pathlib import Path

import pytest

from modsim.backends.base import (
    BackendAdapter,
    BackendCapabilities,
    ConnectionOutcome,
    ConnectionRequest,
    ConnectionRetarget,
    ExplainsRetargetRefusals,
    SupportsConnectionRetarget,
)
from modsim.backends.mock import MockBackendAdapter
from modsim.backends.registry import installed_backends
from modsim.connectors.docking import DockProposal
from modsim.core.events import DockCommitted, Event
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
    connection_id,
)
from modsim.core.scene import SceneSpec
from modsim.core.surface import SurfaceAttachment
from modsim.core.transforms import (
    Transform,
    Vec3,
    angle_between,
    quat_angle,
    quat_from_axis_angle,
    vec_add,
    vec_cross,
    vec_norm,
    vec_scale,
    vec_sub,
)
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.robot_packs.schema import (
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    SurfaceRealization,
)
from modsim.runtime.session import RuntimeSession
from modsim.runtime.surface_slide import SurfaceSlideCommand, SurfaceSlideError
from surface_fixtures import strut_pose, surface_pack, surface_scene

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


# ----------------------------------------------------------------------
# point-to-surface conformance (docs/continuous_docking.md 12, Phase 2)
# ----------------------------------------------------------------------
#
# The second fixture is the synthetic sphere/strut pack from
# ``surface_fixtures``, built without wheel or lift joints so that MuJoCo can
# compile it from the cube asset, and with a shell radius large enough that a
# seated strut's cube never touches the node's cube. Gravity is off, so the
# only thing that can move a strut is the adapter itself.

SURFACE_RADIUS_M = 0.12
SURFACE_REALIZATIONS = (
    SurfaceRealization.DRIVEN_FIXED,
    SurfaceRealization.BALL,
    SurfaceRealization.STATIC_FIXED,
)
SHELL = ConnectorInstanceId("node_0/shell")
SLIDE_STEP_S = 0.05
SLIDE_SPEED_M_S = 0.2
SLIDE_SPIN_RAD_S = 0.5


def _upper(index: int) -> ConnectorInstanceId:
    return ConnectorInstanceId(f"strut_{index}/upper")


def _surface_struts() -> tuple[Transform, ...]:
    """Three struts at least 90 degrees apart, with spin, tilt and both gap signs."""
    oblique = (math.sin(0.7) * math.cos(2.0), math.sin(0.7) * math.sin(2.0), -math.cos(0.7))
    return (
        strut_pose((0.0, 0.0, 1.0), radius_m=SURFACE_RADIUS_M, spin_rad=0.3, gap_m=0.001),
        strut_pose(
            (1.0, 0.0, 0.0), radius_m=SURFACE_RADIUS_M, tilt_axis=(0.0, 1.0, 0.0), tilt_rad=0.05
        ),
        strut_pose(oblique, radius_m=SURFACE_RADIUS_M, spin_rad=-1.1, gap_m=-0.0015),
    )


@pytest.fixture
def surface_loaded(loaded_pack: LoadedRobotPack) -> LoadedRobotPack:
    pack = surface_pack(
        loaded_pack.pack,
        realizations=tuple(r.value for r in SURFACE_REALIZATIONS),
        articulated=False,
        radius_m=SURFACE_RADIUS_M,
    )
    return replace(loaded_pack, pack=pack)


def make_surface_session(
    loaded: LoadedRobotPack,
    backend: str | BackendAdapter,
    struts: tuple[Transform, ...] | None = None,
) -> RuntimeSession:
    scene = surface_scene(*(struts if struts is not None else _surface_struts()))
    if backend == "mock" or isinstance(backend, MockBackendAdapter):
        return RuntimeSession.create(loaded, scene, backend)
    return RuntimeSession.create(loaded, scene, backend, gravity=(0.0, 0.0, 0.0))


def surface_sessions(loaded: LoadedRobotPack) -> Iterator[tuple[str, RuntimeSession]]:
    for backend in backends_under_test():
        yield backend, make_surface_session(loaded, backend)


def dock_every_surface_pair(session: RuntimeSession) -> tuple[Event, ...]:
    for proposal in session.proposals():
        session.request_dock(proposal.connector_a, proposal.connector_b)
    return session.step(STEP_S)


def expected_realization(capabilities: BackendCapabilities) -> SurfaceRealization | None:
    """Return the first entry of the authored list this backend's capabilities admit (7.1)."""
    supported = capabilities.supported_constraint_types
    for realization in SURFACE_REALIZATIONS:
        if realization is SurfaceRealization.STATIC_FIXED and PhysicalConstraintType.FIXED in (
            supported
        ):
            return realization
        if (
            realization is SurfaceRealization.DRIVEN_FIXED
            and PhysicalConstraintType.FIXED in supported
            and capabilities.supports_connection_retarget
        ):
            return realization
        if realization is SurfaceRealization.BALL and PhysicalConstraintType.BALL in supported:
            return realization
    return None


def _realization_capabilities(capabilities: BackendCapabilities) -> tuple[object, ...]:
    """The capabilities that decide which surface realisation a backend gets."""
    return (
        capabilities.supported_constraint_types,
        capabilities.supports_connection_retarget,
    )


def _transform_offset(first: Transform, second: Transform) -> tuple[float, float]:
    delta = second.relative_to(first)
    return vec_norm(delta.translation), quat_angle(delta.rotation)


class _MockWithoutRetarget(MockBackendAdapter):
    """The mock with retarget switched off, so the fallback runs without MuJoCo."""

    def capabilities(self) -> BackendCapabilities:
        return replace(super().capabilities(), supports_connection_retarget=False)


class _RecordingWithoutRetarget(_MockWithoutRetarget):
    def __init__(self, requests: list[ConnectionRequest]) -> None:
        super().__init__()
        self.requests = requests

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        self.requests.append(request)
        return super().create_physical_connection(request)


class _MockRefusingContactExclusions(MockBackendAdapter):
    """The mock as a backend that advertises retarget but cannot exclude contacts."""

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        if request.extra_contact_exclusions:
            return ConnectionOutcome.refused("this backend cannot exclude contacts")
        return super().create_physical_connection(request)


def test_backends_agree_on_surface_proposals(surface_loaded: LoadedRobotPack) -> None:
    requires_two_backends()
    reference: dict[tuple[str, str], tuple[object, ...]] | None = None
    reference_backend = ""

    for backend, session in surface_sessions(surface_loaded):
        proposals = session.proposals()
        surface = [p for p in proposals if p.surface_contact is not None]
        assert {p.connector_a for p in surface} == {_upper(i) for i in range(3)}, (
            f"{backend} did not propose every strut against the shell"
        )
        assert all(p.connector_b == SHELL for p in surface), "side A must be the point side"
        summary: dict[tuple[str, str], tuple[object, ...]] = {}
        for proposal in surface:
            contact = proposal.surface_contact
            assert contact is not None
            summary[(str(proposal.connector_a), str(proposal.connector_b))] = (
                proposal.compatibility.compatible,
                proposal.acceptance.satisfied,
                proposal.guard.code,
                contact.normal_world,
                contact.seat_gap_m,
            )
        if reference is None:
            reference, reference_backend = summary, backend
            continue
        assert set(summary) == set(reference)
        for pair, (*verdict, normal, gap) in summary.items():
            *expected_verdict, expected_normal, expected_gap = reference[pair]
            assert verdict == expected_verdict, f"{backend} disagreed on {pair}"
            assert angle_between(normal, expected_normal) == pytest.approx(  # type: ignore[arg-type]
                0.0, abs=ANGLE_TOLERANCE_RAD
            ), f"{backend} found a different normal for {pair} than {reference_backend}"
            assert gap == pytest.approx(expected_gap, abs=POSITION_TOLERANCE_M)


def _surface_commit_summary(events: tuple[Event, ...]) -> tuple[tuple[object, ...], ...]:
    summary: list[tuple[object, ...]] = []
    for event in events:
        if isinstance(event, DockCommitted):
            summary.append(
                (
                    event.kind,
                    str(event.connection_id),
                    str(event.connector_a),
                    str(event.connector_b),
                    event.constraint,
                    event.alignment,
                    event.nominal_alignment_withheld,
                )
            )
        else:
            summary.append((event.kind,))
    return tuple(summary)


def test_backends_agree_on_the_surface_commit_sequence(surface_loaded: LoadedRobotPack) -> None:
    requires_two_backends()
    reference: tuple[tuple[object, ...], ...] | None = None
    reference_backend = ""

    for backend, session in surface_sessions(surface_loaded):
        sequence = _surface_commit_summary(dock_every_surface_pair(session))
        assert sum(1 for item in sequence if item[0] == "DockCommitted") == 3, (
            f"{backend} did not commit all three struts"
        )
        for connection in sorted(session.world.connections):
            session.request_undock(connection)
        sequence += _surface_commit_summary(session.step(STEP_S))
        assert not session.world.connections
        if reference is None:
            reference, reference_backend = sequence, backend
            continue
        assert sequence == reference, (
            f"{backend} produced {sequence} where {reference_backend} produced {reference}"
        )


@pytest.mark.parametrize("backend", backends_under_test())
def test_each_backend_records_the_first_realisation_it_advertises(
    surface_loaded: LoadedRobotPack, backend: str
) -> None:
    """With a list it cannot fully honour, a backend records its fallback (7.1)."""
    session = make_surface_session(surface_loaded, backend)
    expected = expected_realization(session.adapter.capabilities())
    assert expected is not None

    committed = [e for e in dock_every_surface_pair(session) if isinstance(e, DockCommitted)]

    assert [event.realization for event in committed] == [expected] * 3
    assert all(event.constraint is PhysicalConstraintType.FIXED for event in committed)
    for connection in session.world.connections.values():
        assert connection.realization is expected


def test_backends_with_matching_capabilities_choose_the_same_realisation(
    surface_loaded: LoadedRobotPack,
) -> None:
    requires_two_backends()
    chosen: dict[tuple[object, ...], set[SurfaceRealization | None]] = {}

    for _, session in surface_sessions(surface_loaded):
        key = _realization_capabilities(session.adapter.capabilities())
        committed = [e for e in dock_every_surface_pair(session) if isinstance(e, DockCommitted)]
        chosen.setdefault(key, set()).update(event.realization for event in committed)

    assert all(len(realizations) == 1 for realizations in chosen.values()), chosen


def test_backends_agree_on_the_attachment_at_commit(surface_loaded: LoadedRobotPack) -> None:
    requires_two_backends()
    reference: dict[str, SurfaceAttachment] | None = None
    reference_backend = ""

    for backend, session in surface_sessions(surface_loaded):
        attachments: dict[str, SurfaceAttachment] = {}
        for event in dock_every_surface_pair(session):
            if isinstance(event, DockCommitted):
                assert event.attachment is not None
                attachments[str(event.connection_id)] = event.attachment
                runtime = session.world.connections[event.connection_id]
                assert runtime.committed_attachment == event.attachment
        assert len(attachments) == 3
        if reference is None:
            reference, reference_backend = attachments, backend
            continue
        assert set(attachments) == set(reference)
        for name, attachment in attachments.items():
            expected = reference[name]
            where = f"{backend} vs {reference_backend} on '{name}'"
            assert angle_between(attachment.normal_local, expected.normal_local) == pytest.approx(
                0.0, abs=ANGLE_TOLERANCE_RAD
            ), where
            assert attachment.seat_gap_m == pytest.approx(
                expected.seat_gap_m, abs=POSITION_TOLERANCE_M
            ), where
            assert attachment.spin_rad == pytest.approx(expected.spin_rad, abs=ANGLE_TOLERANCE_RAD)
            offset, angle = _transform_offset(expected.frame_local, attachment.frame_local)
            assert offset == pytest.approx(0.0, abs=POSITION_TOLERANCE_M), where
            assert angle == pytest.approx(0.0, abs=ANGLE_TOLERANCE_RAD), where
            assert attachment.magnet_height_m == expected.magnet_height_m


def _commanded_target(start: SurfaceAttachment, dt_s: float) -> tuple[Vec3, Vec3]:
    """Return the normal and frame x axis one slide step should reach.

    Derived independently of the slide driver: a tangential speed v along the
    attachment frame's x axis turns the normal by v dt / R toward that axis,
    and the spin rate turns the frame about the new normal.
    """
    normal = start.normal_local
    x_axis = start.frame_local.apply_direction((1.0, 0.0, 0.0))
    theta = SLIDE_SPEED_M_S * dt_s / SURFACE_RADIUS_M
    moved_normal = vec_add(vec_scale(normal, math.cos(theta)), vec_scale(x_axis, math.sin(theta)))
    moved_x = vec_sub(vec_scale(x_axis, math.cos(theta)), vec_scale(normal, math.sin(theta)))
    spin = SLIDE_SPIN_RAD_S * dt_s
    spun_x = vec_add(
        vec_scale(moved_x, math.cos(spin)),
        vec_scale(vec_cross(moved_normal, moved_x), math.sin(spin)),
    )
    return moved_normal, spun_x


def _slide_one_step(session: RuntimeSession) -> tuple[SurfaceAttachment, SurfaceAttachment]:
    """Dock one strut, command one slide step, and return the start and measured attachment."""
    dock_every_surface_pair(session)
    (connection,) = session.world.connections
    start = session.world.surface_attachment(connection)
    assert start is not None
    session.set_surface_slide(
        connection,
        SurfaceSlideCommand(
            tangential_velocity_m_s=(SLIDE_SPEED_M_S, 0.0), spin_rate_rad_s=SLIDE_SPIN_RAD_S
        ),
    )
    session.step(SLIDE_STEP_S)  # the slide update sends one retarget
    assert session.surface_slide_report.refusals == ()
    session.clear_surface_slide()
    session.step(SLIDE_STEP_S)  # the next ingest measures it
    measured = session.world.surface_attachment(connection)
    assert measured is not None
    return start, measured


def test_retarget_moves_the_strut_to_the_commanded_attachment(
    surface_loaded: LoadedRobotPack,
) -> None:
    """Every backend that can retarget reaches the same, independently computed target."""
    reached: dict[str, tuple[Vec3, Vec3]] = {}
    for backend in backends_under_test():
        session = make_surface_session(surface_loaded, backend, _surface_struts()[:1])
        if not session.adapter.capabilities().supports_connection_retarget:
            continue
        start, measured = _slide_one_step(session)
        normal, x_axis = _commanded_target(start, SLIDE_STEP_S)
        measured_x = measured.frame_local.apply_direction((1.0, 0.0, 0.0))
        assert angle_between(measured.normal_local, normal) == pytest.approx(
            0.0, abs=ANGLE_TOLERANCE_RAD
        ), f"{backend} did not reach the commanded attachment normal"
        assert angle_between(measured_x, x_axis) == pytest.approx(0.0, abs=ANGLE_TOLERANCE_RAD), (
            f"{backend} did not reach the commanded spin"
        )
        assert measured.seat_gap_m == pytest.approx(start.seat_gap_m, abs=POSITION_TOLERANCE_M)
        node = session.world.module(ModuleInstanceId("node_0")).pose
        assert vec_norm(node.translation) == pytest.approx(0.0, abs=POSITION_TOLERANCE_M), (
            f"{backend} moved the node instead of the strut"
        )
        reached[backend] = (measured.normal_local, measured_x)

    assert reached, "no installed backend supports connection retarget"
    (first_normal, first_x), *others = reached.values()
    for normal, x_axis in others:
        assert angle_between(normal, first_normal) == pytest.approx(0.0, abs=ANGLE_TOLERANCE_RAD)
        assert angle_between(x_axis, first_x) == pytest.approx(0.0, abs=ANGLE_TOLERANCE_RAD)


@pytest.mark.parametrize("backend", backends_under_test())
def test_a_backend_that_cannot_retarget_records_the_fallback_and_refuses_slides(
    surface_loaded: LoadedRobotPack, backend: str
) -> None:
    adapter: str | BackendAdapter = backend
    if backend == "mock":
        adapter = _MockWithoutRetarget()
    session = make_surface_session(surface_loaded, adapter, _surface_struts()[:1])
    capabilities = session.adapter.capabilities()
    if capabilities.supports_connection_retarget:
        pytest.skip(f"{backend} supports connection retarget")

    dock_every_surface_pair(session)
    (connection,) = session.world.connections.values()

    assert connection.realization is SurfaceRealization.STATIC_FIXED
    assert connection.realization is expected_realization(capabilities)
    with pytest.raises(SurfaceSlideError, match="static_fixed"):
        session.set_surface_slide(
            connection.id, SurfaceSlideCommand(tangential_velocity_m_s=(0.1, 0.0))
        )


def test_a_refused_contact_exclusion_falls_through_to_the_next_realisation(
    surface_loaded: LoadedRobotPack,
) -> None:
    session = make_surface_session(
        surface_loaded, _MockRefusingContactExclusions(), _surface_struts()[:1]
    )

    events = dock_every_surface_pair(session)

    (committed,) = (event for event in events if isinstance(event, DockCommitted))
    assert committed.realization is SurfaceRealization.STATIC_FIXED


# ----------------------------------------------------------------------
# contact exclusions on the request (A4)
# ----------------------------------------------------------------------


def _strut_request(
    session: RuntimeSession, exclusions: tuple[tuple[str, str], ...]
) -> ConnectionRequest:
    point = session.world.connector(_upper(0))
    shell = session.world.connector(SHELL)
    return ConnectionRequest(
        connection_id=connection_id(point.id, shell.id),
        module_a=point.module_id,
        link_a=point.parent_link,
        connector_a=point.id,
        connector_a_local=point.local_pose,
        module_b=shell.module_id,
        link_b=shell.parent_link,
        connector_b=shell.id,
        connector_b_local=shell.local_pose,
        relative_transform=shell.world_pose.relative_to(point.world_pose),
        physical_connection=PhysicalConnectionSpec(constraint=PhysicalConstraintType.FIXED),
        extra_contact_exclusions=exclusions,
    )


@pytest.mark.parametrize("backend", backends_under_test())
def test_every_backend_honours_exclusions_between_the_welded_links(
    surface_loaded: LoadedRobotPack, backend: str
) -> None:
    session = make_surface_session(surface_loaded, backend, _surface_struts()[:1])

    outcome = session.adapter.create_physical_connection(
        _strut_request(session, (("base_link", "base_link"),))
    )

    assert outcome.success, outcome.detail
    assert outcome.handle is not None
    assert session.adapter.remove_physical_connection(outcome.handle)


@pytest.mark.parametrize("backend", backends_under_test())
def test_every_backend_refuses_exclusions_it_cannot_honour_and_keeps_nothing(
    surface_loaded: LoadedRobotPack, backend: str
) -> None:
    session = make_surface_session(surface_loaded, backend, _surface_struts()[:1])

    refused = session.adapter.create_physical_connection(
        _strut_request(session, (("no_such_link", "base_link"),))
    )
    retried = session.adapter.create_physical_connection(_strut_request(session, ()))

    assert not refused.success
    assert refused.detail
    assert retried.success, "a refused request must not leave its constraint behind"


def test_only_driven_fixed_requests_carry_the_contact_exclusions(
    surface_loaded: LoadedRobotPack,
) -> None:
    requests: list[ConnectionRequest] = []

    class Recording(MockBackendAdapter):
        def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
            requests.append(request)
            return super().create_physical_connection(request)

    for adapter in (Recording(), _RecordingWithoutRetarget(requests)):
        dock_every_surface_pair(
            make_surface_session(surface_loaded, adapter, _surface_struts()[:1])
        )

    driven, static = requests
    assert driven.surface_realization is SurfaceRealization.DRIVEN_FIXED
    assert driven.extra_contact_exclusions == (("base_link", "base_link"),)
    assert static.surface_realization is SurfaceRealization.STATIC_FIXED
    assert static.extra_contact_exclusions == ()


# ----------------------------------------------------------------------
# optional retarget protocols (7.2)
# ----------------------------------------------------------------------


@pytest.mark.parametrize("backend", backends_under_test())
def test_the_retarget_capability_matches_the_protocol(
    surface_loaded: LoadedRobotPack, backend: str
) -> None:
    adapter = make_surface_session(surface_loaded, backend).adapter

    if adapter.capabilities().supports_connection_retarget:
        assert isinstance(adapter, SupportsConnectionRetarget)
    if isinstance(adapter, ExplainsRetargetRefusals):
        assert isinstance(adapter, SupportsConnectionRetarget), (
            "explaining retarget refusals only makes sense for a backend that retargets"
        )


def skip_without_retarget(backend: str, capabilities: BackendCapabilities) -> None:
    """Skip on the capability, not the backend name, so a backend enables itself.

    MuJoCo is the backend this skips today. Its retarget is a Phase 3
    deliverable (docs/continuous_docking.md 14.5); once its capabilities
    advertise ``supports_connection_retarget``, these tests run on it unchanged.
    """
    if not capabilities.supports_connection_retarget:
        pytest.skip(
            f"{backend} does not advertise supports_connection_retarget; MuJoCo retarget "
            "is a Phase 3 deliverable (docs/continuous_docking.md 14.5)"
        )


def _retargeting_session(
    loaded: LoadedRobotPack, backend: str
) -> tuple[RuntimeSession, SupportsConnectionRetarget]:
    session = make_surface_session(loaded, backend, _surface_struts()[:2])
    adapter = session.adapter
    skip_without_retarget(backend, adapter.capabilities())
    assert isinstance(adapter, SupportsConnectionRetarget)
    dock_every_surface_pair(session)
    assert len(session.world.connections) == 2
    return session, adapter


def _nudge(session: RuntimeSession, connection: ConnectionId, angle: float) -> ConnectionRetarget:
    runtime = session.world.connections[connection]
    attachment = session.world.surface_attachment(connection)
    assert attachment is not None
    turn = Transform(rotation=quat_from_axis_angle(attachment.normal_local, angle))
    point = session.world.connector(runtime.connector_a)
    return ConnectionRetarget(
        handle=runtime.constraint_handle,
        connector_a_local=point.local_pose,
        connector_b_local=turn.compose(attachment.frame_local),
        relative_transform=runtime.relative_transform,
    )


def _module_poses(adapter: BackendAdapter) -> dict[ModuleInstanceId, Transform]:
    snapshot = adapter.snapshot()
    return {
        module: next(iter(links.values())).pose for module, links in snapshot.link_states.items()
    }


@pytest.mark.parametrize("backend", backends_under_test())
def test_a_retarget_batch_is_atomic_with_one_flag_per_update(
    surface_loaded: LoadedRobotPack, backend: str
) -> None:
    session, adapter = _retargeting_session(surface_loaded, backend)
    first, second = sorted(session.world.connections)
    valid = _nudge(session, first, 0.2)
    unknown = replace(_nudge(session, second, 0.2), handle=ConstraintHandle("weld:no_such"))
    before = _module_poses(session.adapter)

    assert adapter.retarget_physical_connections(()) == ()
    flags = adapter.retarget_physical_connections((valid, unknown))

    assert flags == (True, False)
    after = _module_poses(session.adapter)
    for module, pose in before.items():
        offset, angle = _transform_offset(pose, after[module])
        assert offset == pytest.approx(0.0, abs=1e-12), f"{backend} applied part of a batch"
        assert angle == pytest.approx(0.0, abs=1e-12)

    assert adapter.retarget_physical_connections((valid,)) == (True,)
    moved = _module_poses(session.adapter)[session.world.connections[first].module_a]
    assert _transform_offset(before[session.world.connections[first].module_a], moved)[1] > 0.1


@pytest.mark.parametrize("backend", backends_under_test())
def test_refusal_reasons_agree_with_the_retarget_flags(
    surface_loaded: LoadedRobotPack, backend: str
) -> None:
    session, adapter = _retargeting_session(surface_loaded, backend)
    if not isinstance(adapter, ExplainsRetargetRefusals):
        pytest.skip(f"{backend} does not explain retarget refusals")
    first, second = sorted(session.world.connections)
    updates = (
        _nudge(session, first, 0.1),
        replace(_nudge(session, second, 0.1), handle=ConstraintHandle("weld:no_such")),
    )

    reasons = tuple(adapter.refusal_reason(update) for update in updates)
    flags = adapter.retarget_physical_connections(updates)

    assert reasons[0] is None
    assert isinstance(reasons[1], str) and reasons[1]
    assert flags == tuple(reason is None for reason in reasons)
