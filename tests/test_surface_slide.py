"""Sliding point connectors over a surface (docs/continuous_docking.md 11, A2, A3)."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from modsim.backends.base import BackendCapabilities, ConnectionOutcome, ConnectionRequest
from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import DockCommitted, UndockCommitted
from modsim.core.ids import ConnectionId, ConnectorInstanceId, connection_id
from modsim.core.surface import central_angle
from modsim.core.transforms import (
    Transform,
    Vec3,
    vec_dot,
    vec_norm,
    vec_normalize,
    vec_scale,
    vec_sub,
)
from modsim.robot_packs.schema import (
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    RobotPack,
    SurfaceDriveSpec,
    SurfaceRealization,
)
from modsim.runtime.session import RuntimeSession
from modsim.runtime.surface_slide import (
    SurfaceSlideCommand,
    SurfaceSlideError,
    differential_slide_command,
)
from surface_fixtures import (
    MIN_SEPARATION_RAD,
    NODE,
    RADIUS_M,
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


def direction(polar_rad: float, azimuth_rad: float = 0.0) -> Vec3:
    return (
        math.sin(polar_rad) * math.cos(azimuth_rad),
        math.sin(polar_rad) * math.sin(azimuth_rad),
        -math.cos(polar_rad),
    )


def docked(
    pack: RobotPack, *normals: Vec3, adapter: MockBackendAdapter | None = None
) -> RuntimeSession:
    session = RuntimeSession.create(
        pack, surface_scene(*(strut_pose(n) for n in normals)), adapter or MockBackendAdapter()
    )
    for index in range(len(normals)):
        session.request_dock(upper(index), SHELL)
    events = session.step(0.0)
    assert sum(isinstance(event, DockCommitted) for event in events) == len(normals)
    return session


def normal_of(session: RuntimeSession, index: int) -> Vec3:
    attachment = session.world.surface_attachment(link(index))
    assert attachment is not None
    return attachment.normal_local


def toward(session: RuntimeSession, index: int, target: Vec3, speed: float) -> SurfaceSlideCommand:
    """Return a command moving strut ``index`` along the great circle toward ``target``."""
    attachment = session.world.surface_attachment(link(index))
    assert attachment is not None
    normal = attachment.normal_local
    tangent = vec_normalize(vec_sub(target, vec_scale(normal, vec_dot(target, normal))))
    x_axis = attachment.frame_local.apply_direction((1.0, 0.0, 0.0))
    y_axis = attachment.frame_local.apply_direction((0.0, 1.0, 0.0))
    return SurfaceSlideCommand(
        tangential_velocity_m_s=(speed * vec_dot(tangent, x_axis), speed * vec_dot(tangent, y_axis))
    )


class _BallCapableMock(MockBackendAdapter):
    """A test double that accepts ball requests, only to reach the A3 slide check."""

    __slots__ = ()

    def capabilities(self) -> BackendCapabilities:
        base = super().capabilities()
        return replace(
            base,
            supported_constraint_types=frozenset(
                {PhysicalConstraintType.FIXED, PhysicalConstraintType.BALL}
            ),
        )

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        if request.physical_connection.constraint is PhysicalConstraintType.BALL:
            request = replace(
                request,
                physical_connection=PhysicalConnectionSpec(constraint=PhysicalConstraintType.FIXED),
            )
        return super().create_physical_connection(request)


# ----------------------------------------------------------------------
# validation (A3)
# ----------------------------------------------------------------------


def test_only_driven_fixed_connections_accept_slide_commands(example_pack: RobotPack) -> None:
    static = docked(surface_pack(example_pack, realizations=("static_fixed",)), DOWN)
    with pytest.raises(SurfaceSlideError, match="static_fixed"):
        static.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0)))

    ball = docked(
        surface_pack(example_pack, realizations=("ball", "static_fixed")),
        DOWN,
        adapter=_BallCapableMock(),
    )
    runtime = ball.world.connections[link(0)]
    assert runtime.realization is SurfaceRealization.BALL
    assert runtime.constraint is PhysicalConstraintType.BALL
    with pytest.raises(SurfaceSlideError, match="'ball', which cannot be driven"):
        ball.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0)))


def test_invalid_slide_commands_are_rejected(example_pack: RobotPack) -> None:
    session = docked(surface_pack(example_pack), DOWN)
    with pytest.raises(SurfaceSlideError, match="no active connection"):
        session.set_surface_slide(ConnectionId("nope"), SurfaceSlideCommand())
    with pytest.raises(SurfaceSlideError, match="finite"):
        session.set_surface_slide(link(0), SurfaceSlideCommand((math.nan, 0.0)))
    with pytest.raises(SurfaceSlideError, match="finite"):
        session.set_surface_slide(link(0), SurfaceSlideCommand((0.0, 0.0), math.inf))


# ----------------------------------------------------------------------
# motion
# ----------------------------------------------------------------------


def test_a_slide_moves_the_strut_along_the_surface_without_events(
    example_pack: RobotPack,
) -> None:
    session = docked(surface_pack(example_pack), DOWN)
    before_events = len(session.world.event_log)
    start = normal_of(session, 0)
    speed, dt, steps = 0.01, 0.001, 1000

    session.set_surface_slide(link(0), toward(session, 0, (1.0, 0.0, 0.0), speed))
    for _ in range(steps):
        session.step(dt)
    # Each step's retarget is measured at the next ingest; a zero step only observes.
    session.step(0.0)

    assert len(session.world.event_log) == before_events
    travelled = central_angle(start, normal_of(session, 0))
    assert travelled == pytest.approx(speed * dt * steps / RADIUS_M, rel=1e-9)
    attachment = session.world.surface_attachment(link(0))
    assert attachment is not None
    assert attachment.seat_gap_m == pytest.approx(0.0, abs=1e-9)
    assert session.world.modules[NODE].pose.is_close(Transform.identity())


def test_a_spin_command_turns_the_strut_about_the_normal(example_pack: RobotPack) -> None:
    session = docked(surface_pack(example_pack), DOWN)
    before = session.world.surface_attachment(link(0))
    assert before is not None

    session.set_surface_slide(link(0), SurfaceSlideCommand((0.0, 0.0), 1.0))
    for _ in range(50):
        session.step(0.01)
    session.step(0.0)

    after = session.world.surface_attachment(link(0))
    assert after is not None
    assert after.spin_rad - before.spin_rad == pytest.approx(0.5, abs=1e-9)
    assert central_angle(before.normal_local, after.normal_local) == pytest.approx(0.0, abs=1e-12)


def test_clearing_and_undocking_stop_the_slide(example_pack: RobotPack) -> None:
    session = docked(surface_pack(example_pack), DOWN)
    session.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0)))
    session.clear_surface_slide()
    start = normal_of(session, 0)
    session.step(0.1)
    session.step(0.0)
    assert central_angle(start, normal_of(session, 0)) == pytest.approx(0.0, abs=1e-12)

    session.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0)))
    session.request_undock(link(0))
    session.step(0.1)
    assert link(0) not in session.surface_slides.commands


def _redocking_pack(example_pack: RobotPack) -> RobotPack:
    """No cooldown, so the same pair can release and re-dock in one docking pass."""
    return surface_pack(
        example_pack,
        magnet={
            "docking_policy": {
                "auto_latch": False,
                "alignment": "measured",
                "redock_cooldown_s": None,
                "break_force_n": None,
            }
        },
    )


def test_a_release_and_redock_in_one_step_drops_the_slide(example_pack: RobotPack) -> None:
    # Regression for review finding K1: the connection id and the backend
    # handle are both pair-derived, so the re-formed connection looks the same.
    session = docked(_redocking_pack(example_pack), DOWN)
    session.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0)))
    for _ in range(5):
        session.step(0.1)

    session.request_undock(link(0))
    session.request_dock(upper(0), SHELL)
    events = session.step(0.1)
    kinds = [type(event) for event in events]
    assert UndockCommitted in kinds
    assert DockCommitted in kinds
    assert link(0) in session.world.connections
    assert link(0) not in session.surface_slides.commands

    session.step(0.0)
    start = normal_of(session, 0)
    for _ in range(5):
        session.step(0.1)
    session.step(0.0)
    # Compare vectors: acos in central_angle turns ulp differences into ~1e-8.
    assert normal_of(session, 0) == pytest.approx(start, abs=1e-12)


def test_a_release_outside_the_step_is_caught_by_the_commit_time(
    example_pack: RobotPack,
) -> None:
    session = docked(_redocking_pack(example_pack), DOWN)
    session.set_surface_slide(link(0), SurfaceSlideCommand((0.01, 0.0)))
    session.step(0.1)

    # A docking pass run directly is not part of any step's events.
    session.request_undock(link(0))
    released = session.process_docking()
    assert any(isinstance(event, UndockCommitted) for event in released)
    session.request_dock(upper(0), SHELL)
    session.step(0.1)
    assert link(0) in session.world.connections
    assert link(0) not in session.surface_slides.commands


# ----------------------------------------------------------------------
# separation while sliding (A2)
# ----------------------------------------------------------------------


def test_a_slide_toward_a_neighbour_stops_at_the_minimum_separation(
    example_pack: RobotPack,
) -> None:
    session = docked(surface_pack(example_pack), DOWN, direction(math.radians(80.0)))

    # 0.05 m/s for 0.5 s is about 24 degrees, enough to cross the 60 degree limit.
    session.set_surface_slide(link(1), toward(session, 1, DOWN, 0.05))
    session.step(0.5)
    clamps = session.surface_slide_report.clamps
    session.step(0.0)

    angle = central_angle(normal_of(session, 0), normal_of(session, 1))
    assert angle == pytest.approx(MIN_SEPARATION_RAD, abs=1e-9)
    assert angle >= MIN_SEPARATION_RAD - 1e-12
    assert len(clamps) == 1
    assert (clamps[0].connection_id, clamps[0].neighbour) == (link(1), link(0))
    assert 0.0 < clamps[0].allowed_fraction < 1.0

    session.step(0.5)
    session.step(0.0)
    assert central_angle(normal_of(session, 0), normal_of(session, 1)) == pytest.approx(
        MIN_SEPARATION_RAD, abs=1e-9
    )


def test_a_strut_inside_the_boundary_may_only_move_away(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = docked(
        surface_pack(example_pack), DOWN, direction(math.radians(80.0)), adapter=adapter
    )
    # Push strut 1 inside the boundary, as physics might.
    adapter.set_module_pose(strut_id(1), strut_pose(direction(math.radians(50.0))))
    session.step(0.0)
    inside = central_angle(normal_of(session, 0), normal_of(session, 1))
    assert inside == pytest.approx(math.radians(50.0), abs=1e-9)

    session.set_surface_slide(link(1), toward(session, 1, DOWN, 0.01))
    session.step(0.1)
    session.step(0.0)
    assert central_angle(normal_of(session, 0), normal_of(session, 1)) == pytest.approx(
        inside, abs=1e-12
    )

    session.set_surface_slide(link(1), toward(session, 1, (1.0, 0.0, 0.0), 0.01))
    session.step(0.1)
    session.step(0.0)
    assert central_angle(normal_of(session, 0), normal_of(session, 1)) > inside


def test_two_struts_closing_in_one_step_resolve_deterministically(
    example_pack: RobotPack,
) -> None:
    def run() -> tuple[Vec3, Vec3]:
        a, b = direction(math.radians(40.0), 0.0), direction(math.radians(40.0), math.pi)
        session = docked(surface_pack(example_pack), a, b)
        session.set_surface_slide(link(0), toward(session, 0, b, 0.05))
        session.set_surface_slide(link(1), toward(session, 1, a, 0.05))
        session.step(0.5)
        session.step(0.0)
        first, second = normal_of(session, 0), normal_of(session, 1)
        assert central_angle(first, second) >= MIN_SEPARATION_RAD - 1e-9
        return first, second

    first = run()
    for _ in range(3):
        again = run()
        assert vec_norm(vec_sub(again[0], first[0])) == 0.0
        assert vec_norm(vec_sub(again[1], first[1])) == 0.0


# ----------------------------------------------------------------------
# differential drive model
# ----------------------------------------------------------------------


def test_the_differential_drive_model_maps_wheel_rates_to_a_slide() -> None:
    drive = SurfaceDriveSpec(
        left_joint="wheel_left", right_joint="wheel_right", wheel_radius_m=0.01, track_m=0.05
    )

    straight = differential_slide_command(drive, 2.0, 2.0)
    assert straight.tangential_velocity_m_s == pytest.approx((0.02, 0.0))
    assert straight.spin_rate_rad_s == pytest.approx(0.0)

    turning = differential_slide_command(drive, -1.0, 1.0)
    assert turning.tangential_velocity_m_s == pytest.approx((0.0, 0.0))
    assert turning.spin_rate_rad_s == pytest.approx(0.02 / 0.05)
