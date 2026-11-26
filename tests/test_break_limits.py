"""Opt-in per-axis break limits (docs/continuous_docking.md 8.1 and amendment A5)."""

from __future__ import annotations

from dataclasses import replace

import pytest

from modsim.backends.base import BackendCapabilities, BackendError
from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import ConnectorOverloaded, DockCommitted, UndockCommitted
from modsim.core.ids import ConnectionId, ConnectorInstanceId, connection_id
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, ConstraintWrench
from modsim.core.transforms import Vec3
from modsim.robot_packs.schema import RobotPack
from modsim.runtime.inspection import format_event_detail
from modsim.runtime.session import RuntimeSession
from surface_fixtures import strut_pose, surface_pack, surface_scene

SHELL = ConnectorInstanceId("node_0/shell")
UPPER = ConnectorInstanceId("strut_0/upper")
LINK: ConnectionId = connection_id(UPPER, SHELL)
DOWN: Vec3 = (0.0, 0.0, -1.0)
# A strut seated below the node has its upper docking axis along world +z, so
# world z is the normal direction and x, y are shear in the tests below. The
# wrench is the force the connection applies to the strut (side A): +z pulls
# the strut toward the node, which is tension, and -z pushes it away from the
# node, which is compression.


class _TorqueMock(MockBackendAdapter):
    """A test double that reports an injected moment, to exercise moment limits."""

    __slots__ = ("torque",)

    def __init__(self) -> None:
        super().__init__()
        self.torque: Vec3 = (0.0, 0.0, 0.0)

    def capabilities(self) -> BackendCapabilities:
        return replace(super().capabilities(), supports_constraint_torques=True)

    def snapshot(self) -> BackendStateSnapshot:
        base = super().snapshot()
        return replace(
            base,
            constraint_wrenches={
                handle: ConstraintWrench(force_n=wrench.force_n, torque_nm=self.torque)
                for handle, wrench in base.constraint_wrenches.items()
            },
        )


def _pack(example_pack: RobotPack, **limits: float) -> RobotPack:
    return surface_pack(
        example_pack,
        realizations=("static_fixed",),
        magnet={
            "docking_policy": {"redock_cooldown_s": 0.2, "break_limits": dict(limits)},
        },
    )


def _docked(pack: RobotPack, adapter: MockBackendAdapter) -> RuntimeSession:
    session = RuntimeSession.create(pack, surface_scene(strut_pose(DOWN)), adapter)
    session.request_dock(UPPER, SHELL)
    assert any(isinstance(event, DockCommitted) for event in session.step(0.0))
    return session


@pytest.mark.parametrize(
    ("force", "quantity", "measured"),
    [
        ((0.0, 0.0, 80.0), "normal_force", 80.0),
        ((30.0, 40.0, 10.0), "shear_force", 50.0),
        ((30.0, 40.0, -80.0), "shear_force", 50.0),
    ],
)
def test_an_exceeded_force_axis_releases_the_connection(
    example_pack: RobotPack, force: Vec3, quantity: str, measured: float
) -> None:
    adapter = MockBackendAdapter()
    session = _docked(_pack(example_pack, max_normal_force_n=79.0, max_shear_force_n=49.0), adapter)
    adapter.set_constraint_force(session.world.connections[LINK].constraint_handle, force)

    events = session.step(0.0)

    overload = next(event for event in events if isinstance(event, ConnectorOverloaded))
    assert overload.quantity == quantity
    assert overload.measured_force_n == pytest.approx(measured)
    assert overload.released
    assert any(isinstance(event, UndockCommitted) for event in events)
    assert LINK not in session.world.connections
    # A per-axis value is not the whole constraint force, so it is not recorded as one.
    assert "N exceeded" in format_event_detail(overload) and quantity in format_event_detail(
        overload
    )


@pytest.mark.parametrize("normal_n", [-80.0, -500.0])
def test_compression_never_counts_as_normal_overload(
    example_pack: RobotPack, normal_n: float
) -> None:
    # The strut pressed into the node, as when its wheels carry load: only
    # tension separates a magnetic connector.
    adapter = MockBackendAdapter()
    session = _docked(_pack(example_pack, max_normal_force_n=79.0), adapter)
    adapter.set_constraint_force(
        session.world.connections[LINK].constraint_handle, (0.0, 0.0, normal_n)
    )

    events = session.step(0.0)

    assert not any(isinstance(event, ConnectorOverloaded) for event in events)
    assert LINK in session.world.connections


def test_loads_within_every_limit_keep_the_connection(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = _docked(_pack(example_pack, max_normal_force_n=79.0, max_shear_force_n=49.0), adapter)
    adapter.set_constraint_force(
        session.world.connections[LINK].constraint_handle, (30.0, 0.0, 70.0)
    )

    assert not any(isinstance(event, ConnectorOverloaded) for event in session.step(0.0))
    assert LINK in session.world.connections


def test_descriptive_limits_never_break_a_connection(example_pack: RobotPack) -> None:
    # The generic cube declares 90 N normal and 40 N shear as datasheet values.
    adapter = MockBackendAdapter()
    scene = SceneSpec.grid("generic_cube", 2, spacing_m=0.1)
    session = RuntimeSession.create(example_pack, scene, adapter)
    front = ConnectorInstanceId("generic_cube_0/front")
    rear = ConnectorInstanceId("generic_cube_1/rear")
    session.request_dock(front, rear)
    session.step(0.0)
    handle = session.world.connections[connection_id(front, rear)].constraint_handle
    adapter.set_constraint_force(handle, (500.0, 500.0, 0.0))

    assert not any(isinstance(event, ConnectorOverloaded) for event in session.step(0.0))


@pytest.mark.parametrize(
    ("torque", "quantity", "measured"),
    [
        ((0.0, 0.0, 1.4), "torsion_moment", 1.4),
        ((1.2, 1.6, 0.0), "bending_moment", 2.0),
    ],
)
def test_an_exceeded_moment_axis_releases_the_connection(
    example_pack: RobotPack, torque: Vec3, quantity: str, measured: float
) -> None:
    adapter = _TorqueMock()
    session = _docked(
        _pack(example_pack, max_bending_moment_nm=1.6, max_torsion_moment_nm=1.3), adapter
    )
    adapter.torque = torque

    overload = next(e for e in session.step(0.0) if isinstance(e, ConnectorOverloaded))

    assert overload.quantity == quantity
    assert overload.measured_force_n == pytest.approx(measured)
    assert "N*m" in format_event_detail(overload)


def test_moment_limits_need_a_backend_that_reports_moments(example_pack: RobotPack) -> None:
    pack = _pack(example_pack, max_torsion_moment_nm=1.3)
    with pytest.raises(BackendError, match="does not report constraint moments"):
        RuntimeSession.create(pack, surface_scene(strut_pose(DOWN)), MockBackendAdapter())
    # Force-only limits are fine on the mock.
    RuntimeSession.create(
        _pack(example_pack, max_normal_force_n=79.0),
        surface_scene(strut_pose(DOWN)),
        MockBackendAdapter(),
    )


def test_the_mock_reports_moments_as_unmeasured(example_pack: RobotPack) -> None:
    adapter = MockBackendAdapter()
    session = _docked(_pack(example_pack), adapter)

    wrench = adapter.snapshot().constraint_wrenches[
        session.world.connections[LINK].constraint_handle
    ]
    assert wrench.torque_nm is None
    assert not adapter.capabilities().supports_constraint_torques


def test_the_whole_force_check_still_reports_force_magnitude(example_pack: RobotPack) -> None:
    pack = surface_pack(
        example_pack,
        realizations=("static_fixed",),
        magnet={
            "docking_policy": {"break_force_n": 50.0, "break_limits": {"max_shear_force_n": 10.0}}
        },
    )
    adapter = MockBackendAdapter()
    session = _docked(pack, adapter)
    adapter.set_constraint_force(
        session.world.connections[LINK].constraint_handle, (0.0, 0.0, 60.0)
    )

    overload = next(e for e in session.step(0.0) if isinstance(e, ConnectorOverloaded))

    assert overload.quantity == "force_magnitude"
    assert overload.limit_n == 50.0
