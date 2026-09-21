"""Backend-neutral schema, lifecycle, validation, and view tests for hinges."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from modsim.backends.base import ConnectionOutcome, ConnectionRequest
from modsim.backends.mock import MockBackendAdapter
from modsim.core.entities import ConnectionRuntime
from modsim.core.events import DockCommitted, DockFailed, DockFailureReason
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
)
from modsim.core.scene import SceneSpec
from modsim.core.state import WorldState
from modsim.core.transforms import Transform
from modsim.model_views import (
    CubicLatticeView,
    ModelViewContext,
    ModelViewFactory,
    ModuleTopologyGraphView,
)
from modsim.robot_packs import (
    HingeConstraintSpec,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    RobotPack,
    RobotPackLoader,
    RobotPackValidator,
    ValidationProfile,
)
from modsim.robot_packs.schema import ModelViewSpec
from modsim.runtime.inspection import build_runtime_inspector_frame
from modsim.runtime.inspection_protocol import (
    RuntimeFrame,
    decode_runtime_message,
    encode_runtime_message,
)
from modsim.runtime.session import RuntimeSession

MODULE_TYPE = "generic_cube"
FRONT = ConnectorInstanceId("generic_cube_0/front")
REAR = ConnectorInstanceId("generic_cube_1/rear")


def _hinge_parameters(
    *,
    axis: tuple[float, float, float] = (0.0, 1.0, 0.0),
    anchor_separation_m: float = 0.04,
) -> dict[str, object]:
    return {
        "axis": axis,
        "anchor_separation_m": anchor_separation_m,
    }


def _with_hinge_connection(pack: RobotPack) -> RobotPack:
    data: dict[str, Any] = pack.model_dump(mode="python")
    connection = data["hardware_catalog"]["connector_types"]["fixed_face"]
    connection["physical_connection"] = {
        "constraint": "hinge",
        "compliance": None,
        "hinge": _hinge_parameters(),
    }
    return RobotPack.model_validate(data)


def _session(pack: RobotPack, adapter: MockBackendAdapter) -> RuntimeSession:
    return RuntimeSession.create(
        pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1),
        adapter,
    )


class _HingeAcceptingBackend(MockBackendAdapter):
    """Test double that records a hinge request without modelling its dynamics."""

    __slots__ = ("last_request",)

    def __init__(self) -> None:
        super().__init__()
        self.last_request: ConnectionRequest | None = None

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        self.last_request = request
        return ConnectionOutcome.accepted(ConstraintHandle(f"hinge:{request.connection_id}"))


def test_hinge_schema_is_strict_and_round_trips() -> None:
    connection = PhysicalConnectionSpec(
        constraint=PhysicalConstraintType.HINGE,
        hinge=HingeConstraintSpec(
            axis=(0.0, 1.0, 0.0),
            anchor_separation_m=0.04,
        ),
    )

    assert connection.hinge is not None
    assert connection.hinge.axis == (0.0, 1.0, 0.0)
    assert connection.hinge.anchor_separation_m == pytest.approx(0.04)
    assert PhysicalConnectionSpec.model_validate_json(connection.model_dump_json()) == connection


def test_hinge_schema_requires_only_valid_hinge_parameters() -> None:
    with pytest.raises(ValidationError, match="hinge constraints require hinge parameters"):
        PhysicalConnectionSpec(constraint=PhysicalConstraintType.HINGE)

    with pytest.raises(ValidationError, match="hinge parameters require a hinge constraint"):
        PhysicalConnectionSpec(
            constraint=PhysicalConstraintType.FIXED,
            hinge=HingeConstraintSpec(axis=(0.0, 1.0, 0.0), anchor_separation_m=0.04),
        )

    with pytest.raises(ValidationError, match="unit length"):
        HingeConstraintSpec(axis=(0.0, 2.0, 0.0), anchor_separation_m=0.04)

    with pytest.raises(ValidationError):
        HingeConstraintSpec(axis=(0.0, 1.0, 0.0), anchor_separation_m=0.0)


def test_simulation_validator_accepts_hinge_but_still_rejects_ball(
    example_pack_dir: Path,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    hinge_loaded = loaded.with_pack(_with_hinge_connection(loaded.pack))

    hinge_report = RobotPackValidator().validate(
        hinge_loaded,
        profile=ValidationProfile.SIMULATION,
    )

    assert hinge_report.valid
    assert not any(
        issue.code == "connector.physical_connection_unsupported" for issue in hinge_report.issues
    )

    data: dict[str, Any] = loaded.pack.model_dump(mode="python")
    data["hardware_catalog"]["connector_types"]["fixed_face"]["physical_connection"] = {
        "constraint": "ball",
        "compliance": None,
        "hinge": None,
    }
    ball_loaded = loaded.with_pack(RobotPack.model_validate(data))
    ball_report = RobotPackValidator().validate(
        ball_loaded,
        profile=ValidationProfile.SIMULATION,
    )
    assert any(
        issue.code == "connector.physical_connection_unsupported" for issue in ball_report.errors
    )


@pytest.mark.parametrize(
    "other_hinge",
    (
        _hinge_parameters(axis=(1.0, 0.0, 0.0)),
        _hinge_parameters(anchor_separation_m=0.02),
    ),
)
def test_docking_guard_rejects_conflicting_hinge_parameters(
    example_pack: RobotPack,
    other_hinge: dict[str, object],
) -> None:
    data: dict[str, Any] = example_pack.model_dump(mode="python")
    connector_types = data["hardware_catalog"]["connector_types"]
    first_type = connector_types["fixed_face"]
    first_type["compatible_with"] = ["other_hinge"]
    first_type["physical_connection"] = {
        "constraint": "hinge",
        "compliance": None,
        "hinge": _hinge_parameters(),
    }
    connector_types["other_hinge"] = {
        **first_type,
        "id": "other_hinge",
        "compatible_with": ["fixed_face"],
        "physical_connection": {
            "constraint": "hinge",
            "compliance": None,
            "hinge": other_hinge,
        },
    }
    module = data["hardware_catalog"]["module_types"][MODULE_TYPE]
    module["connectors"][1]["connector_type"] = "other_hinge"
    session = _session(RobotPack.model_validate(data), MockBackendAdapter())

    session.request_dock(FRONT, REAR)
    events = session.step(0.01)

    failed = next(event for event in events if isinstance(event, DockFailed))
    assert failed.reason is DockFailureReason.GUARD_REJECTED
    assert "conflicting hinge parameters" in failed.detail


def test_mock_backend_explicitly_refuses_an_accepted_hinge_request(
    example_pack: RobotPack,
) -> None:
    session = _session(_with_hinge_connection(example_pack), MockBackendAdapter())
    session.request_dock(FRONT, REAR)

    proposal = next(item for item in session.proposals() if item.requested)
    assert proposal.guard.allowed
    events = session.step(0.01)

    failed = next(event for event in events if isinstance(event, DockFailed))
    assert failed.reason is DockFailureReason.BACKEND_REFUSED
    assert "does not implement hinge constraints" in failed.detail
    assert not session.world.connections


def test_hinge_constraint_survives_commit_replay_views_and_inspector_protocol(
    example_pack: RobotPack,
) -> None:
    pack = _with_hinge_connection(example_pack)
    adapter = _HingeAcceptingBackend()
    session = _session(pack, adapter)
    session.request_dock(FRONT, REAR)
    events = session.step(0.01)

    committed = next(event for event in events if isinstance(event, DockCommitted))
    assert committed.constraint is PhysicalConstraintType.HINGE
    assert adapter.last_request is not None
    assert adapter.last_request.physical_connection.constraint is PhysicalConstraintType.HINGE
    connection = session.world.connections[committed.connection_id]
    assert connection.constraint is PhysicalConstraintType.HINGE

    replay = WorldState.from_scene(pack, SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1))
    replay.apply(committed)
    assert replay.connections[committed.connection_id].constraint is PhysicalConstraintType.HINGE

    factory = ModelViewFactory()
    context = ModelViewContext(pack, session.world)
    topology = factory.build(pack.manifest.model_views[0], context)
    assert isinstance(topology, ModuleTopologyGraphView)
    assert topology.edges[0].constraint is PhysicalConstraintType.HINGE
    assert topology.model_dump(mode="json")["edges"][0]["constraint"] == "hinge"

    lattice_recipe = ModelViewSpec.model_validate(
        {
            "id": "hinge_lattice",
            "builder": "cubic_lattice",
            "configuration": {
                "pitch_m": 0.1,
                "position_tolerance_m": 0.005,
                "orientation_tolerance_rad": 0.08726646259971647,
            },
        }
    )
    lattice = factory.build(lattice_recipe, context)
    assert isinstance(lattice, CubicLatticeView)
    assert lattice.edges[0].constraint is PhysicalConstraintType.HINGE

    frame = build_runtime_inspector_frame(
        session,
        pack.manifest.model_views[0],
        factory,
    )
    decoded = decode_runtime_message(encode_runtime_message(RuntimeFrame(frame=frame)))
    assert isinstance(decoded, RuntimeFrame)
    assert decoded.frame.view.edges[0].constraint is PhysicalConstraintType.HINGE
    dock_row = next(row for row in decoded.frame.events if row.kind == "DockCommitted")
    assert "hinge" in dock_row.detail


def test_legacy_commit_defaults_to_a_fixed_constraint(example_pack: RobotPack) -> None:
    world = WorldState.from_scene(
        example_pack,
        SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1),
    )
    event = DockCommitted(
        time_s=0.0,
        connection_id=ConnectionId("legacy"),
        connector_a=FRONT,
        connector_b=REAR,
        constraint_handle=ConstraintHandle("legacy"),
        relative_transform=Transform.identity(),
    )

    world.apply(event)

    assert event.constraint is PhysicalConstraintType.FIXED
    assert world.connections[event.connection_id].constraint is PhysicalConstraintType.FIXED


def test_connection_runtime_preserves_legacy_measured_force_positional_slot() -> None:
    connection = ConnectionRuntime(
        ConnectionId("legacy"),
        FRONT,
        REAR,
        ModuleInstanceId("generic_cube_0"),
        ModuleInstanceId("generic_cube_1"),
        Transform.identity(),
        0.0,
        None,
        ConstraintHandle("legacy"),
        0.0,
        4.25,
    )

    assert connection.measured_force_n == pytest.approx(4.25)
    assert connection.constraint is PhysicalConstraintType.FIXED
