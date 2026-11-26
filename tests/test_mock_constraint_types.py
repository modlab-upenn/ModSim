"""The mock backend realises fixed connections only and refuses every other kind.

Before this contract the mock advertised ``compliant``, ``ball`` and ``custom``
and welded them rigidly, which is the silent faking the backend contract
forbids. ``compliant`` passes the docking guards, so that faking was live.
"""

from __future__ import annotations

from typing import Any

import pytest

from modsim.backends.base import ConnectionRequest
from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import DockFailed, DockFailureReason
from modsim.core.ids import ConnectionId, ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import SceneSpec
from modsim.core.transforms import Transform
from modsim.robot_packs.schema import (
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    RobotPack,
)
from modsim.runtime.session import RuntimeSession

CONNECTOR_TYPE = "fixed_face"
MODULE_TYPE = "generic_cube"
CUBE_0 = ModuleInstanceId("generic_cube_0")
CUBE_1 = ModuleInstanceId("generic_cube_1")
FRONT_0 = ConnectorInstanceId("generic_cube_0/front")
REAR_1 = ConnectorInstanceId("generic_cube_1/rear")


def _request(spec: PhysicalConnectionSpec) -> ConnectionRequest:
    return ConnectionRequest(
        connection_id=ConnectionId("generic_cube_0/front<->generic_cube_1/rear"),
        module_a=CUBE_0,
        link_a="base_link",
        connector_a=FRONT_0,
        connector_a_local=Transform.identity(),
        module_b=CUBE_1,
        link_b="base_link",
        connector_b=REAR_1,
        connector_b_local=Transform.identity(),
        relative_transform=Transform.identity(),
        physical_connection=spec,
    )


def _loaded_mock(example_pack: RobotPack) -> MockBackendAdapter:
    adapter = MockBackendAdapter()
    adapter.load(example_pack, SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1))
    return adapter


def test_mock_advertises_fixed_only() -> None:
    capabilities = MockBackendAdapter().capabilities()

    assert capabilities.supported_constraint_types == frozenset({PhysicalConstraintType.FIXED})


@pytest.mark.parametrize(
    "spec",
    [
        PhysicalConnectionSpec.model_validate(
            {
                "constraint": "compliant",
                "compliance": {"translational_stiffness_n_per_m": 1000.0},
            }
        ),
        PhysicalConnectionSpec(constraint=PhysicalConstraintType.BALL),
        PhysicalConnectionSpec(constraint=PhysicalConstraintType.CUSTOM),
        PhysicalConnectionSpec.model_validate(
            {"constraint": "hinge", "hinge": {"axis": [0, 1, 0], "anchor_separation_m": 0.04}}
        ),
    ],
    ids=["compliant", "ball", "custom", "hinge"],
)
def test_mock_refuses_every_non_fixed_request(
    example_pack: RobotPack, spec: PhysicalConnectionSpec
) -> None:
    adapter = _loaded_mock(example_pack)

    outcome = adapter.create_physical_connection(_request(spec))

    assert not outcome.success
    assert outcome.detail is not None
    assert f"does not implement {spec.constraint.value} constraints" in outcome.detail
    assert adapter.welds == ()


def test_mock_still_welds_a_fixed_request(example_pack: RobotPack) -> None:
    adapter = _loaded_mock(example_pack)

    outcome = adapter.create_physical_connection(
        _request(PhysicalConnectionSpec(constraint=PhysicalConstraintType.FIXED))
    )

    assert outcome.success


def test_a_compliant_dock_fails_as_backend_refused_instead_of_welding(
    example_pack: RobotPack,
) -> None:
    data: dict[str, Any] = example_pack.model_dump(mode="python")
    data["hardware_catalog"]["connector_types"][CONNECTOR_TYPE]["physical_connection"] = {
        "constraint": "compliant",
        "compliance": {"translational_stiffness_n_per_m": 1000.0},
    }
    pack = RobotPack.model_validate(data)
    scene = SceneSpec.grid(MODULE_TYPE, 2, spacing_m=0.1)
    session = RuntimeSession.create(pack, scene, MockBackendAdapter())

    session.request_dock(FRONT_0, REAR_1)
    events = session.step(0.01)

    failed = next(event for event in events if isinstance(event, DockFailed))
    assert failed.reason is DockFailureReason.BACKEND_REFUSED
    assert "does not implement compliant constraints" in failed.detail
    assert not session.world.connections
