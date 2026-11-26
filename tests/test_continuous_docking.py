"""Continuous point-to-surface docking on the mock backend.

Covers docs/continuous_docking.md sections 4 to 7 and amendments A1, A3 and
A6, using the synthetic sphere-and-strut pack from ``surface_fixtures``.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

import pytest

import modsim.connectors.surface as surface_module
from modsim.backends.base import (
    BackendCapabilities,
    ConnectionRequest,
    ConnectionRetarget,
    SupportsConnectionRetarget,
)
from modsim.backends.mock import MockBackendAdapter
from modsim.connectors.acceptance import AXIS_ALIGNMENT, POSITION, RELATIVE_VELOCITY
from modsim.connectors.docking import DockProposal, free_connectors
from modsim.connectors.surface import candidate_surface_pairs
from modsim.core.entities import ConnectorLifecycleState
from modsim.core.events import (
    DockCommitted,
    DockFailed,
    DockFailureReason,
    Event,
    UndockCommitted,
)
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
    connection_id,
)
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import (
    Transform,
    Vec3,
    angle_between,
    quat_from_axis_angle,
    vec_norm,
    vec_normalize,
    vec_scale,
    vec_sub,
)
from modsim.robot_packs.schema import (
    AlignmentMode,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    RobotPack,
    SurfaceRealization,
)
from modsim.runtime.metrics import collect_metrics
from modsim.runtime.requirements import BackendRequirements
from modsim.runtime.session import RuntimeSession
from surface_fixtures import (
    MIN_SEPARATION_RAD,
    NODE,
    RADIUS_M,
    SEAT_TOLERANCE_M,
    TILT_TOLERANCE_RAD,
    strut_id,
    strut_pose,
    surface_pack,
    surface_scene,
)

SHELL = ConnectorInstanceId("node_0/shell")
DOWN: Vec3 = (0.0, 0.0, -1.0)


def upper(index: int) -> ConnectorInstanceId:
    return ConnectorInstanceId(f"strut_{index}/upper")


def link(index: int) -> ConnectionId:
    return connection_id(upper(index), SHELL)


def make_session(
    pack: RobotPack,
    *poses: Transform,
    adapter: MockBackendAdapter | None = None,
) -> RuntimeSession:
    return RuntimeSession.create(pack, surface_scene(*poses), adapter or MockBackendAdapter())


def pack_with(example_pack: RobotPack, *realizations: str, **magnet: object) -> RobotPack:
    return surface_pack(
        example_pack,
        realizations=realizations or ("driven_fixed", "static_fixed"),
        magnet=dict(magnet) if magnet else None,
    )


def proposal_for(session: RuntimeSession, index: int) -> DockProposal:
    session.request_dock(upper(index), SHELL)
    proposals = session.proposals()
    return next(p for p in proposals if p.connector_a == upper(index))


def first(events: Iterable[Event], kind: type[Event]) -> Event:
    for event in events:
        if isinstance(event, kind):
            return event
    raise AssertionError(f"no {kind.__name__}")


def dock(session: RuntimeSession, *indices: int) -> tuple[Event, ...]:
    for index in indices:
        session.request_dock(upper(index), SHELL)
    return session.step(0.0)


def direction(polar_rad: float, azimuth_rad: float = 0.0) -> Vec3:
    """A unit normal ``polar_rad`` away from straight down, toward +x rotated by azimuth."""
    return (
        math.sin(polar_rad) * math.cos(azimuth_rad),
        math.sin(polar_rad) * math.sin(azimuth_rad),
        -math.cos(polar_rad),
    )


# ----------------------------------------------------------------------
# geometry and acceptance (section 4)
# ----------------------------------------------------------------------


def test_a_seated_strut_is_accepted_with_the_point_side_as_a(example_pack: RobotPack) -> None:
    session = make_session(pack_with(example_pack), strut_pose(DOWN))

    proposal = proposal_for(session, 0)

    assert proposal.connector_a == upper(0)
    assert proposal.connector_b == SHELL
    assert proposal.viable, proposal.acceptance.reason()
    assert proposal.surface_contact is not None
    assert proposal.surface_contact.seat_gap_m == pytest.approx(0.0, abs=1e-12)
    assert proposal.acceptance.criterion(POSITION).measured == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("gap", [-0.0029, 0.0029, -0.0031, 0.0031])
def test_the_position_criterion_is_the_seat_gap(example_pack: RobotPack, gap: float) -> None:
    session = make_session(pack_with(example_pack), strut_pose(DOWN, gap_m=gap))

    criterion = proposal_for(session, 0).acceptance.criterion(POSITION)

    assert criterion.measured == pytest.approx(abs(gap), abs=1e-12)
    assert criterion.satisfied is (abs(gap) <= SEAT_TOLERANCE_M)


@pytest.mark.parametrize("tilt", [0.09, 0.11])
def test_the_axis_criterion_is_tilt_from_the_inward_normal(
    example_pack: RobotPack, tilt: float
) -> None:
    pose = strut_pose(DOWN, tilt_axis=(1.0, 0.0, 0.0), tilt_rad=tilt)
    session = make_session(pack_with(example_pack), pose)

    proposal = proposal_for(session, 0)
    criterion = proposal.acceptance.criterion(AXIS_ALIGNMENT)

    assert criterion.measured == pytest.approx(tilt, abs=1e-9)
    assert criterion.satisfied is (tilt <= TILT_TOLERANCE_RAD)
    assert proposal.acceptance.criterion(POSITION).measured == pytest.approx(0.0, abs=1e-12)


def test_relative_velocity_uses_the_surface_velocity_at_the_attachment(
    example_pack: RobotPack,
) -> None:
    adapter = MockBackendAdapter()
    session = make_session(pack_with(example_pack), strut_pose(DOWN), adapter=adapter)
    # Spinning about x moves the bottom of the sphere at R * omega = 0.06 m/s,
    # above the 0.05 m/s tolerance, while the sphere centre stays still.
    adapter.set_module_twist(NODE, angular_rad_s=(1.0, 0.0, 0.0))
    session.world.ingest(adapter.snapshot())

    assert vec_norm(session.world.connector(SHELL).world_velocity_m_s) == pytest.approx(0.0)
    criterion = proposal_for(session, 0).acceptance.criterion(RELATIVE_VELOCITY)

    assert criterion.measured == pytest.approx(RADIUS_M * 1.0, abs=1e-12)
    assert not criterion.satisfied


def test_a_point_connector_at_the_sphere_centre_is_reported_not_raised(
    example_pack: RobotPack,
) -> None:
    centred = Transform(translation=(0.0, 0.0, -0.03))  # upper connector at the centre
    session = make_session(pack_with(example_pack), centred)

    proposal = proposal_for(session, 0)

    assert not proposal.acceptance.satisfied
    assert "centre" in proposal.acceptance.reason()


@pytest.mark.parametrize("tilt", [0.0, 0.05])
def test_nominal_snap_keeps_tangent_position_and_spin(example_pack: RobotPack, tilt: float) -> None:
    normal = vec_normalize((1.0, 1.0, -1.0))
    pose = strut_pose(normal, gap_m=0.002, spin_rad=0.7, tilt_axis=(0.0, 0.0, 1.0), tilt_rad=tilt)
    pack = pack_with(example_pack, "static_fixed", docking_policy={"alignment": "nominal"})
    session = make_session(pack, pose)
    before = proposal_for(session, 0).surface_contact
    assert before is not None

    events = session.step(0.0)
    committed = first(events, DockCommitted)
    assert isinstance(committed, DockCommitted) and committed.attachment is not None
    # The backend performs the snap; the world sees it on the next ingest.
    session.step(0.0)
    attachment = session.world.surface_attachment(link(0))

    assert attachment is not None
    assert attachment.seat_gap_m == pytest.approx(0.0, abs=1e-9)
    assert angle_between(attachment.normal_local, before.normal_world) == pytest.approx(
        0.0, abs=1e-9
    )
    assert attachment.spin_rad == pytest.approx(committed.attachment.spin_rad, abs=1e-9)
    point = session.world.connector(upper(0))
    assert angle_between(point.world_docking_axis, vec_scale(normal, -1.0)) == pytest.approx(
        0.0, abs=1e-9
    )
    # The node did not move: the strut was snapped onto it, not the reverse.
    assert session.world.modules[NODE].pose.is_close(Transform.identity())


@pytest.mark.parametrize("alignment", ["nominal", "measured"])
def test_orientation_rad_describes_the_committed_pose(
    example_pack: RobotPack, alignment: str
) -> None:
    # A tilt about x gives a small measured roll. A nominal surface snap
    # commits roll zero against A_q, and the records must say so; a measured
    # commit keeps the measured roll.
    pose = strut_pose(DOWN, gap_m=0.001, spin_rad=0.7, tilt_axis=(1.0, 0.0, 0.0), tilt_rad=0.08)
    pack = pack_with(example_pack, "static_fixed", docking_policy={"alignment": alignment})
    session = make_session(pack, pose)
    measured = proposal_for(session, 0).acceptance.orientation_rad
    assert abs(measured) > 1e-4

    committed = first(session.step(0.0), DockCommitted)
    assert isinstance(committed, DockCommitted)
    runtime = session.world.connections[link(0)]

    expected = 0.0 if alignment == "nominal" else measured
    assert committed.alignment is AlignmentMode(alignment)
    assert committed.orientation_rad == pytest.approx(expected, abs=1e-12)
    assert runtime.orientation_rad == pytest.approx(expected, abs=1e-12)


def test_candidate_surface_pairs_finds_only_points_inside_the_shell() -> None:
    near = ConnectorInstanceId("strut_0/upper")
    far = ConnectorInstanceId("strut_1/upper")
    other = ConnectorInstanceId("node_1/shell")
    pairs = candidate_surface_pairs(
        [(far, (0.0, 0.0, -0.2)), (near, (0.0, 0.0, -0.0625))],
        [(SHELL, (0.0, 0.0, 0.0), RADIUS_M), (other, (1.0, 0.0, 0.0), RADIUS_M)],
        0.003,
    )
    assert pairs == ((near, SHELL),)
    assert candidate_surface_pairs([], [], 0.003) == ()


def test_candidate_surface_pairs_come_out_in_sorted_order() -> None:
    # Points and surfaces are given in reverse id order; the output must not
    # depend on that.
    shells = [ConnectorInstanceId(f"node_{i}/shell") for i in (1, 0)]
    centres = {shells[0]: (0.0, 0.0, 0.0), shells[1]: (0.03, 0.0, 0.0)}
    points = [
        (ConnectorInstanceId(f"strut_{i}/upper"), (0.015, 0.0, -0.06 + 0.001 * i))
        for i in (2, 1, 0)
    ]
    surfaces = [(shell, centres[shell], RADIUS_M) for shell in shells]

    pairs = candidate_surface_pairs(points, surfaces, 0.003)

    assert len(pairs) == 6
    assert pairs == tuple(sorted(pairs))
    assert candidate_surface_pairs(points[::-1], surfaces[::-1], 0.003) == pairs


def test_surface_broad_phase_work_grows_linearly(monkeypatch: pytest.MonkeyPatch) -> None:
    checks = 0
    real_norm = surface_module.vec_norm

    def counting_norm(vector: Vec3) -> float:
        nonlocal checks
        checks += 1
        return real_norm(vector)

    monkeypatch.setattr(surface_module, "vec_norm", counting_norm)

    def run(nodes: int) -> int:
        nonlocal checks
        checks = 0
        surfaces = [
            (ConnectorInstanceId(f"node_{i}/shell"), (i * 1.0, 0.0, 0.0), RADIUS_M)
            for i in range(nodes)
        ]
        points = [
            (ConnectorInstanceId(f"strut_{i}_{side}/upper"), (i * 1.0, 0.0, sign * 0.0605))
            for i in range(nodes)
            for side, sign in (("a", 1.0), ("b", -1.0))
        ]
        pairs = candidate_surface_pairs(points, surfaces, 0.003)
        assert len(pairs) == 2 * nodes
        return checks

    assert run(200) <= 2 * run(100) + 4


def test_a_requested_far_pair_is_evaluated_and_reported(example_pack: RobotPack) -> None:
    far = strut_pose(DOWN, gap_m=0.5)
    session = make_session(pack_with(example_pack), far)

    session.request_dock(upper(0), SHELL)
    failed = first(session.step(0.0), DockFailed)

    assert isinstance(failed, DockFailed)
    assert failed.reason is DockFailureReason.OUTSIDE_ACCEPTANCE_REGION


# ----------------------------------------------------------------------
# multi-occupancy (section 5)
# ----------------------------------------------------------------------

_SPREAD: tuple[Vec3, ...] = (
    (1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


def test_a_surface_fills_to_capacity_then_reports_full(example_pack: RobotPack) -> None:
    session = make_session(pack_with(example_pack), *(strut_pose(n) for n in _SPREAD))

    events = dock(session, 0, 1, 2, 3)
    assert sum(isinstance(event, DockCommitted) for event in events) == 4
    shell = session.world.connector(SHELL)
    assert shell.lifecycle_state is ConnectorLifecycleState.DOCKED
    assert shell.connection_ids == tuple(sorted(link(i) for i in range(4)))
    assert shell.connection_id is None
    assert not shell.has_capacity
    assert SHELL not in {c.id for c in free_connectors(session.world)}
    assert collect_metrics(session.world).surface_slots_occupied == 4

    proposal = proposal_for(session, 4)
    assert proposal.guard.code == "surface_full"


@pytest.mark.parametrize("delta", [-1e-3, 1e-3])
def test_separation_is_enforced_at_the_boundary(example_pack: RobotPack, delta: float) -> None:
    second = direction(MIN_SEPARATION_RAD + delta)
    session = make_session(pack_with(example_pack), strut_pose(DOWN), strut_pose(second))
    dock(session, 0)
    assert link(0) in session.world.connections

    proposal = proposal_for(session, 1)

    if delta < 0:
        assert proposal.guard.code == "surface_separation"
    else:
        assert proposal.guard.allowed, proposal.guard.reason


def test_two_struts_on_one_spot_in_one_pass_resolve_by_sorted_order(
    example_pack: RobotPack,
) -> None:
    def run() -> tuple[str, ...]:
        close = direction(0.3)
        session = make_session(pack_with(example_pack), strut_pose(DOWN), strut_pose(close))
        events = dock(session, 0, 1)
        return tuple(sorted(session.world.connections)) + tuple(
            event.detail for event in events if isinstance(event, DockFailed)
        )

    outcome = run()
    assert outcome[0] == link(0)
    assert len(outcome) == 2 and "rad from connection" in outcome[1]
    assert run() == outcome


def test_three_struts_competing_for_one_surface_resolve_by_sorted_id_not_scene_order(
    example_pack: RobotPack,
) -> None:
    # strut_0 sits between the other two: 0.5 rad from strut_1 and 0.7 rad
    # from strut_2, both inside the 60 degree separation, while strut_1 and
    # strut_2 are 1.2 rad apart. The scene lists the struts in reverse id
    # order. Sorted order commits strut_0 alone; scene order would commit
    # strut_2 and strut_1 instead.
    polar = {0: 0.5, 1: 0.0, 2: 1.2}

    def run() -> tuple[tuple[ConnectionId, ...], tuple[ConnectorInstanceId, ...]]:
        placements = [ModulePlacement(NODE, "node", Transform.identity())]
        placements.extend(
            ModulePlacement(strut_id(i), "strut", strut_pose(direction(polar[i])))
            for i in (2, 1, 0)
        )
        session = RuntimeSession.create(
            pack_with(example_pack), SceneSpec.of(placements), MockBackendAdapter()
        )
        assert [m for m in session.world.modules if m != NODE] == [
            strut_id(2),
            strut_id(1),
            strut_id(0),
        ]
        events = dock(session, 2, 1, 0)
        refused = tuple(event.connector_a for event in events if isinstance(event, DockFailed))
        return tuple(sorted(session.world.connections)), refused

    outcome = run()
    assert outcome == ((link(0),), (upper(1), upper(2)))
    assert run() == outcome


def test_undocking_one_strut_leaves_the_others_and_cools_only_the_strut(
    example_pack: RobotPack,
) -> None:
    session = make_session(pack_with(example_pack), *(strut_pose(n) for n in _SPREAD[:3]))
    dock(session, 0, 1, 2)
    session.step(0.05)

    session.request_undock(link(1))
    events = session.step(0.01)

    assert any(isinstance(event, UndockCommitted) for event in events)
    shell = session.world.connector(SHELL)
    assert shell.lifecycle_state is ConnectorLifecycleState.DOCKED
    assert shell.connection_ids == (link(0), link(2))
    assert shell.available_at_s == 0.0
    assert session.world.connector(upper(1)).available_at_s == pytest.approx(
        session.world.time_s + 0.2
    )
    # Still within reach, so the released strut is back in candidacy, not engaged.
    assert not session.world.connector(upper(1)).is_engaged


def test_a_failed_dock_cools_only_the_strut(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = make_session(
        pack_with(example_pack, "static_fixed"), strut_pose(DOWN), adapter=adapter
    )
    adapter.fail_next_connection("injected")

    failed = first(dock(session, 0), DockFailed)

    assert isinstance(failed, DockFailed) and failed.reason is DockFailureReason.BACKEND_REFUSED
    shell = session.world.connector(SHELL)
    assert shell.lifecycle_state is not ConnectorLifecycleState.FAILED
    assert shell.available_at_s == 0.0
    point = session.world.connector(upper(0))
    assert point.lifecycle_state is ConnectorLifecycleState.FAILED
    assert point.available_at_s == pytest.approx(0.2)


def test_the_last_undock_frees_the_surface(example_pack: RobotPack) -> None:
    session = make_session(pack_with(example_pack), strut_pose(DOWN))
    dock(session, 0)
    session.request_undock(link(0))
    session.step(0.01)

    shell = session.world.connector(SHELL)
    assert not shell.is_engaged
    assert shell.lifecycle_state is not ConnectorLifecycleState.DOCKED
    assert shell.connection_ids == ()
    assert session.world.assemblies.count == 2


def test_side_a_is_the_point_connector_on_every_record(example_pack: RobotPack) -> None:
    # "node_0/shell" sorts before "strut_0/upper", so sorted order would put
    # the surface first; the side rule must override it.
    assert upper(0) > SHELL
    session = make_session(pack_with(example_pack), strut_pose(DOWN))

    committed = first(dock(session, 0), DockCommitted)

    assert isinstance(committed, DockCommitted)
    assert committed.connector_a == upper(0) and committed.connector_b == SHELL
    runtime = session.world.connections[link(0)]
    assert (runtime.connector_a, runtime.module_a) == (upper(0), strut_id(0))
    assert runtime.id == f"{SHELL}<->{upper(0)}"


# ----------------------------------------------------------------------
# attachment state (section 6)
# ----------------------------------------------------------------------


def test_the_attachment_is_measured_state_in_the_surface_frame(example_pack: RobotPack) -> None:
    node_pose = Transform(
        translation=(0.2, 0.0, 0.0), rotation=quat_from_axis_angle((0.0, 0.0, 1.0), 0.4)
    )
    normal = direction(0.5, 1.0)
    pose = node_pose.compose(strut_pose(normal, gap_m=0.001, spin_rad=0.3))
    session = RuntimeSession.create(
        pack_with(example_pack),
        surface_scene(pose, node_pose=node_pose),
        MockBackendAdapter(),
    )
    committed = first(dock(session, 0), DockCommitted)
    attachment = session.world.surface_attachment(link(0))

    assert isinstance(committed, DockCommitted)
    assert committed.realization is SurfaceRealization.DRIVEN_FIXED
    assert attachment is not None
    assert angle_between(attachment.normal_local, normal) == pytest.approx(0.0, abs=1e-9)
    assert attachment.polar_rad == pytest.approx(math.pi - 0.5, abs=1e-9)
    assert attachment.azimuth_rad == pytest.approx(1.0, abs=1e-9)
    assert attachment.seat_gap_m == pytest.approx(0.001, abs=1e-12)
    assert attachment.magnet_height_m is None  # the mock reports no joint states
    runtime = session.world.connections[link(0)]
    assert runtime.committed_attachment == committed.attachment
    assert runtime.realization is SurfaceRealization.DRIVEN_FIXED
    # Cached for the sample; a point-to-point connection has none.
    assert session.world.surface_attachment(link(0)) is attachment


def test_spin_is_derived_and_flags_its_degenerate_reference(example_pack: RobotPack) -> None:
    session = make_session(
        pack_with(example_pack),
        strut_pose(DOWN, spin_rad=0.25),
        strut_pose((1.0, 0.0, 0.0)),
    )
    dock(session, 0, 1)

    bottom = session.world.surface_attachment(link(0))
    side = session.world.surface_attachment(link(1))
    assert bottom is not None and side is not None
    assert not bottom.spin_reference_degenerate
    assert side.spin_reference_degenerate


def test_a_magnet_height_comes_from_the_declared_joint(example_pack: RobotPack) -> None:
    from modsim.core.snapshot import BackendStateSnapshot, JointState

    session = make_session(pack_with(example_pack), strut_pose(DOWN))
    snapshot = session.adapter.snapshot()
    session.world.ingest(
        BackendStateSnapshot(
            time_s=snapshot.time_s,
            link_states=snapshot.link_states,
            joint_states={strut_id(0): {"lift_upper": JointState(position=0.004)}},
        )
    )

    assert session.world.magnet_height_m(upper(0)) == pytest.approx(-0.004)
    assert session.world.magnet_height_m(ConnectorInstanceId("strut_0/lower")) is None


# ----------------------------------------------------------------------
# realisation selection (section 7.1, amendment A3)
# ----------------------------------------------------------------------


@pytest.mark.parametrize(
    ("realizations", "expected"),
    [
        (("driven_fixed", "static_fixed"), SurfaceRealization.DRIVEN_FIXED),
        (("ball", "static_fixed"), SurfaceRealization.STATIC_FIXED),
        ((), SurfaceRealization.STATIC_FIXED),
    ],
)
def test_the_first_supported_realization_is_chosen_and_recorded(
    example_pack: RobotPack,
    realizations: tuple[str, ...],
    expected: SurfaceRealization,
) -> None:
    pack = surface_pack(example_pack, realizations=realizations)
    session = make_session(pack, strut_pose(DOWN))

    committed = first(dock(session, 0), DockCommitted)

    assert isinstance(committed, DockCommitted)
    assert committed.realization is expected
    assert committed.constraint is PhysicalConstraintType.FIXED
    assert session.world.connections[link(0)].realization is expected


def test_no_supported_realization_fails_explicitly(example_pack: RobotPack) -> None:
    session = make_session(surface_pack(example_pack, realizations=("ball",)), strut_pose(DOWN))

    failed = first(dock(session, 0), DockFailed)

    assert isinstance(failed, DockFailed)
    assert failed.reason is DockFailureReason.BACKEND_REFUSED
    assert "supports none of the surface realizations [ball]" in failed.detail
    assert not session.world.connections


def test_a_refused_realization_falls_through_to_the_next(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = make_session(pack_with(example_pack), strut_pose(DOWN), adapter=adapter)
    adapter.fail_next_connection("first attempt refused")

    committed = first(dock(session, 0), DockCommitted)

    assert isinstance(committed, DockCommitted)
    assert committed.realization is SurfaceRealization.STATIC_FIXED


def test_surface_slide_needs_retarget_not_ball() -> None:
    requirements = BackendRequirements(surface_slide=True)
    ball_only = BackendCapabilities(
        name="ball_only",
        supports_runtime_constraints=True,
        supported_constraint_types=frozenset(
            {PhysicalConstraintType.FIXED, PhysicalConstraintType.BALL}
        ),
    )

    assert requirements.problems(ball_only) == ("driven surface sliding (connection retarget)",)
    assert requirements.problems(MockBackendAdapter().capabilities()) == ()


def test_a_ball_dock_on_the_mock_is_refused_through_the_pipeline(example_pack: RobotPack) -> None:
    from conftest import with_connector_field

    pack = with_connector_field(
        example_pack,
        "fixed_face",
        physical_connection=PhysicalConnectionSpec.model_validate(
            {"constraint": "ball", "ball": {"anchor_m": [0.0, 0.0, 0.0]}}
        ).model_dump(mode="python"),
    )
    session = RuntimeSession.create(
        pack, SceneSpec.grid("generic_cube", 2, spacing_m=0.1), MockBackendAdapter()
    )
    session.request_dock(
        ConnectorInstanceId("generic_cube_0/front"), ConnectorInstanceId("generic_cube_1/rear")
    )

    failed = first(session.step(0.01), DockFailed)

    assert isinstance(failed, DockFailed)
    assert failed.reason is DockFailureReason.BACKEND_REFUSED
    assert "does not implement ball constraints" in failed.detail


# ----------------------------------------------------------------------
# mock retarget (section 7.2, amendment A1)
# ----------------------------------------------------------------------


def _weld(
    adapter: MockBackendAdapter,
    name: str,
    module_a: str,
    module_b: str,
) -> ConstraintHandle:
    outcome = adapter.create_physical_connection(
        ConnectionRequest(
            connection_id=ConnectionId(name),
            module_a=ModuleInstanceId(module_a),
            link_a="base_link",
            connector_a=ConnectorInstanceId(f"{module_a}/a"),
            connector_a_local=Transform.identity(),
            module_b=ModuleInstanceId(module_b),
            link_b="base_link",
            connector_b=ConnectorInstanceId(f"{module_b}/b"),
            connector_b_local=Transform.identity(),
            relative_transform=Transform.identity(),
            physical_connection=PhysicalConnectionSpec(constraint=PhysicalConstraintType.FIXED),
        )
    )
    assert outcome.success and outcome.handle is not None
    return outcome.handle


def _loaded(example_pack: RobotPack, *placements: tuple[str, str]) -> MockBackendAdapter:
    adapter = MockBackendAdapter()
    pack = pack_with(example_pack)
    adapter.load(
        pack,
        SceneSpec.of(
            ModulePlacement(ModuleInstanceId(name), module_type, Transform.identity())
            for name, module_type in placements
        ),
    )
    return adapter


def test_the_mock_retarget_moves_side_a_and_leaves_the_node_and_other_struts(
    example_pack: RobotPack,
) -> None:
    adapter = _loaded(example_pack, ("node_0", "node"), ("strut_0", "strut"), ("strut_1", "strut"))
    assert isinstance(adapter, SupportsConnectionRetarget)
    assert adapter.capabilities().supports_connection_retarget
    handle = _weld(adapter, "w0", "strut_0", "node_0")
    _weld(adapter, "w1", "strut_1", "node_0")
    new_b = Transform(translation=(0.0, 0.05, 0.0))

    flags = adapter.retarget_physical_connections(
        (ConnectionRetarget(handle, Transform.identity(), new_b, Transform.identity()),)
    )

    assert flags == (True,)
    poses = {
        m: s.pose for m, states in adapter.snapshot().link_states.items() for s in states.values()
    }
    assert poses[ModuleInstanceId("strut_0")].translation == pytest.approx((0.0, 0.05, 0.0))
    assert poses[ModuleInstanceId("node_0")].is_close(Transform.identity())
    assert poses[ModuleInstanceId("strut_1")].is_close(Transform.identity())


def test_the_mock_refuses_a_retarget_that_closes_a_loop(example_pack: RobotPack) -> None:
    adapter = _loaded(
        example_pack,
        ("node_0", "node"),
        ("node_1", "node"),
        ("strut_0", "strut"),
        ("strut_1", "strut"),
    )
    handle = _weld(adapter, "w0", "strut_0", "node_0")
    _weld(adapter, "w1", "strut_0", "node_1")
    _weld(adapter, "w2", "strut_1", "node_1")
    _weld(adapter, "w3", "strut_1", "node_0")
    update = ConnectionRetarget(
        handle, Transform.identity(), Transform(translation=(0.0, 0.05, 0.0)), Transform.identity()
    )
    before = adapter.snapshot()

    assert adapter.retarget_physical_connections((update,)) == (False,)
    reason = adapter.refusal_reason(update)
    assert reason is not None and "closed loop" in reason
    assert adapter.snapshot().link_states == before.link_states
    missing = ConnectionRetarget(
        ConstraintHandle("weld:none"),
        Transform.identity(),
        Transform.identity(),
        Transform.identity(),
    )
    assert adapter.retarget_physical_connections((missing,)) == (False,)


def test_a_rejected_update_applies_nothing_in_its_batch(example_pack: RobotPack) -> None:
    adapter = _loaded(example_pack, ("node_0", "node"), ("strut_0", "strut"))
    handle = _weld(adapter, "w0", "strut_0", "node_0")
    good = ConnectionRetarget(
        handle, Transform.identity(), Transform(translation=(0.1, 0.0, 0.0)), Transform.identity()
    )
    bad = ConnectionRetarget(
        ConstraintHandle("weld:none"),
        Transform.identity(),
        Transform.identity(),
        Transform.identity(),
    )
    before = adapter.snapshot()

    assert adapter.retarget_physical_connections((good, bad)) == (True, False)
    assert adapter.snapshot().link_states == before.link_states


def test_unused_helper_distance() -> None:
    # Guard the fixture's geometry: a seated upper connector sits on the shell.
    pose = strut_pose(DOWN)
    point = pose.apply_point((0.0, 0.0, 0.03))
    assert vec_norm(vec_sub(point, (0.0, 0.0, 0.0))) == pytest.approx(RADIUS_M)


# ----------------------------------------------------------------------
# views and protocol (section 9)
# ----------------------------------------------------------------------


def test_view_edges_carry_the_realization_and_live_attachment(example_pack: RobotPack) -> None:
    from modsim.model_views import ModelViewContext, ModelViewFactory
    from modsim.model_views.models import CubicLatticeView, ModuleTopologyGraphView
    from modsim.robot_packs.schema import ModelViewSpec
    from modsim.runtime.inspection import build_runtime_inspector_frame
    from modsim.runtime.inspection_protocol import (
        RUNTIME_PROTOCOL_VERSION,
        RuntimeFrame,
        decode_runtime_message,
        encode_runtime_message,
    )
    from modsim.runtime.surface_slide import SurfaceSlideCommand

    pack = pack_with(example_pack)
    session = make_session(pack, strut_pose(DOWN, gap_m=0.001))
    dock(session, 0)
    factory = ModelViewFactory()
    context = ModelViewContext(pack, session.world)
    recipe = pack.manifest.model_views[0]

    topology = factory.build(recipe, context)
    assert isinstance(topology, ModuleTopologyGraphView)
    edge = topology.edges[0]
    assert edge.realization is SurfaceRealization.DRIVEN_FIXED
    assert edge.attachment is not None
    assert edge.attachment.seat_gap_m == pytest.approx(0.001, abs=1e-12)
    # acos is ill-conditioned at the pole, hence the looser tolerance.
    assert edge.attachment.polar_rad == pytest.approx(math.pi, abs=1e-6)

    # Sliding changes the attachment on the next sample without a topology change.
    revision = session.world.revision.topology_revision
    session.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0)))
    session.step(0.1)
    session.step(0.0)
    assert session.world.revision.topology_revision == revision
    moved = factory.build(recipe, context)
    assert isinstance(moved, ModuleTopologyGraphView)
    assert moved.edges[0].attachment is not None
    assert moved.edges[0].attachment.polar_rad < edge.attachment.polar_rad

    lattice = factory.build(
        ModelViewSpec.model_validate(
            {
                "id": "surface_lattice",
                "builder": "cubic_lattice",
                "configuration": {
                    "pitch_m": 0.1,
                    "position_tolerance_m": 0.005,
                    "orientation_tolerance_rad": 0.08726646259971647,
                },
            }
        ),
        context,
    )
    assert isinstance(lattice, CubicLatticeView)
    lattice_edge = lattice.edges[0]
    assert lattice_edge.attachment is not None
    faces = {lattice_edge.connector_a: lattice_edge.source_face}
    faces[lattice_edge.connector_b] = lattice_edge.target_face
    assert faces[str(SHELL)] is None
    assert faces[str(upper(0))] is not None

    frame = build_runtime_inspector_frame(session, recipe, factory)
    decoded = decode_runtime_message(encode_runtime_message(RuntimeFrame(frame=frame)))
    assert RUNTIME_PROTOCOL_VERSION == 4
    assert isinstance(decoded, RuntimeFrame)
    assert decoded.frame.view.edges[0].attachment == frame.view.edges[0].attachment
    assert decoded.frame.metrics.surface_slots_occupied == 1


def test_point_to_point_edges_have_no_surface_fields(example_pack: RobotPack) -> None:
    from modsim.model_views import ModelViewContext, ModelViewFactory
    from modsim.model_views.models import ModuleTopologyGraphView

    session = RuntimeSession.create(
        example_pack, SceneSpec.grid("generic_cube", 2, spacing_m=0.1), MockBackendAdapter()
    )
    session.request_dock(
        ConnectorInstanceId("generic_cube_0/front"), ConnectorInstanceId("generic_cube_1/rear")
    )
    session.step(0.0)
    view = ModelViewFactory().build(
        example_pack.manifest.model_views[0], ModelViewContext(example_pack, session.world)
    )
    assert isinstance(view, ModuleTopologyGraphView)
    assert view.edges[0].realization is None and view.edges[0].attachment is None


def test_replaying_events_and_the_sample_reproduces_the_attachments(
    example_pack: RobotPack,
) -> None:
    from modsim.core.state import WorldState
    from modsim.runtime.surface_slide import SurfaceSlideCommand

    pack = pack_with(example_pack)
    scene = surface_scene(strut_pose(DOWN), strut_pose((1.0, 0.0, 0.0)))
    session = RuntimeSession.create(pack, scene, MockBackendAdapter())
    dock(session, 0, 1)
    session.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0), 0.5))
    for _ in range(20):
        session.step(0.01)
    session.step(0.0)

    replay = WorldState.from_scene(pack, scene)
    for event in session.world.event_log:
        replay.apply(event)
    replay.ingest(session.adapter.snapshot())

    assert len(replay.event_log) == len(session.world.event_log)
    for index in (0, 1):
        assert replay.surface_attachment(link(index)) == session.world.surface_attachment(
            link(index)
        )


def test_the_stricter_seat_tolerance_governs_mixed_types(example_pack: RobotPack) -> None:
    strict_shell = surface_pack(
        example_pack,
        shell={
            "acceptance_region": {
                "shape": "sphere",
                "position_tolerance_m": 0.001,
                "orientation_tolerance_rad": 0.1,
                "max_relative_velocity_m_s": 0.05,
            }
        },
    )
    session = make_session(strict_shell, strut_pose(DOWN, gap_m=0.002))

    criterion = proposal_for(session, 0).acceptance.criterion(POSITION)

    assert criterion.tolerance == pytest.approx(0.001)
    assert not criterion.satisfied


# ----------------------------------------------------------------------
# nominal snapping is withheld when the strut is not alone
# ----------------------------------------------------------------------


def _strut_between(start: Vec3, end: Vec3) -> Transform:
    """A strut whose lower end sits on the node at ``start`` and upper on ``end``."""
    from modsim.core.transforms import minimal_rotation, vec_add

    axis = vec_normalize(vec_sub(end, start))
    middle = vec_scale(vec_add(start, end), 0.5)
    return Transform(translation=middle, rotation=minimal_rotation((0.0, 0.0, 1.0), axis))


def test_a_strut_already_on_a_node_closes_a_triangle_without_snapping(
    example_pack: RobotPack,
) -> None:
    pack = surface_pack(
        example_pack,
        realizations=("static_fixed",),
        # An equilateral triangle puts two struts 60 degrees apart on each node.
        shell={"surface": {"radius_m": RADIUS_M, "capacity": 4, "min_separation_rad": 0.5}},
        magnet={"docking_policy": {"alignment": "nominal"}},
    )
    side = 2.0 * (RADIUS_M + 0.03)
    n0: Vec3 = (0.0, 0.0, 0.0)
    n1: Vec3 = (side, 0.0, 0.0)
    # Node 2 sits 1 mm beyond the ideal vertex, so its two struts start with a
    # 0.5 mm seat gap that a snap would visibly remove.
    n2: Vec3 = (side / 2.0, side * math.sqrt(3.0) / 2.0 + 0.001, 0.0)
    placements = [
        ModulePlacement(ModuleInstanceId(f"node_{i}"), "node", Transform(translation=p))
        for i, p in enumerate((n0, n1, n2))
    ] + [
        ModulePlacement(strut_id(0), "strut", _strut_between(n0, n1)),
        ModulePlacement(strut_id(1), "strut", _strut_between(n0, n2)),
        ModulePlacement(strut_id(2), "strut", _strut_between(n1, n2)),
    ]
    adapter = MockBackendAdapter()
    session = RuntimeSession.create(pack, SceneSpec.of(placements), adapter)

    def shell(i: int) -> ConnectorInstanceId:
        return ConnectorInstanceId(f"node_{i}/shell")

    def lower(i: int) -> ConnectorInstanceId:
        return ConnectorInstanceId(f"strut_{i}/lower")

    # A strut on its own snaps; the commits that follow join existing structure.
    session.request_dock(lower(0), shell(0))
    alone = first(session.step(0.0), DockCommitted)
    assert isinstance(alone, DockCommitted)
    assert alone.alignment is AlignmentMode.NOMINAL and not alone.nominal_alignment_withheld

    for point, surface in ((upper(0), 1), (lower(1), 0), (upper(1), 2), (lower(2), 1)):
        session.request_dock(point, shell(surface))
        session.step(0.0)
    assert session.world.assemblies.count == 1
    session.step(0.0)
    before = {m: module.pose for m, module in session.world.modules.items()}

    session.request_dock(upper(2), shell(2))
    events = session.step(0.0)
    session.step(0.0)

    assert not any(isinstance(event, DockFailed) for event in events)
    closing = first(events, DockCommitted)
    assert isinstance(closing, DockCommitted)
    assert closing.connector_a == upper(2)
    assert closing.alignment is AlignmentMode.MEASURED
    assert closing.nominal_alignment_withheld
    for module_id, pose in before.items():
        assert session.world.modules[module_id].pose.is_close(pose), module_id
    attachment = session.world.surface_attachment(connection_id(upper(2), shell(2)))
    assert attachment is not None and attachment.seat_gap_m > 1e-4
