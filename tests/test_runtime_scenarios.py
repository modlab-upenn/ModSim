from __future__ import annotations

import math

import pytest

from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import AssemblyMerged, AssemblySplit, DockCommitted, Event, UndockCommitted
from modsim.core.ids import ConnectorInstanceId, connection_id
from modsim.core.scene import SceneSpec
from modsim.robot_packs import RobotPack
from modsim.runtime.reconfiguration import (
    ReconfigurationPhase,
    ReconfigurationPlanError,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
    connector_pair_plan,
    stage_docking_assembly_pair,
)
from modsim.runtime.session import RuntimeSession

FIXED = ConnectorInstanceId("generic_cube_0/front")
MOVING = ConnectorInstanceId("generic_cube_1/front")


def _scenario(
    pack: RobotPack,
    *,
    include_undock: bool = False,
    release_after_s: float | None = None,
    retract_speed_m_s: float = 0.0,
) -> ScriptedReconfigurationScenario:
    session = RuntimeSession.create(
        pack,
        SceneSpec.grid("generic_cube", 2, spacing_m=1.0),
        MockBackendAdapter(),
    )
    return ScriptedReconfigurationScenario.create(
        session,
        connector_pair_plan(FIXED, MOVING, include_undock=include_undock),
        ScriptedReconfigurationConfig(
            gap_m=0.005,
            approach_speed_m_s=0.0,
            dt_s=0.01,
            initial_hold_s=0.0,
            connected_hold_s=0.0,
            release_after_s=release_after_s,
            retract_speed_m_s=retract_speed_m_s,
        ),
    )


def test_stage_docking_pair_uses_measured_connector_frames(example_pack: RobotPack) -> None:
    session = RuntimeSession.create(
        example_pack,
        SceneSpec.grid("generic_cube", 2, spacing_m=1.0),
        MockBackendAdapter(),
    )
    setup = stage_docking_assembly_pair(session, FIXED, MOVING, gap_m=0.005)

    moving = session.world.connector(setup.moving_connector)
    assert moving.world_pose.translation == pytest.approx((0.055, 0.0, 0.0))
    assert moving.world_docking_axis == pytest.approx((-1.0, 0.0, 0.0), abs=1e-9)
    assert setup.approach_direction == pytest.approx((-1.0, 0.0, 0.0))
    proposal = next(
        item for item in session.proposals() if item.connection_id == connection_id(FIXED, MOVING)
    )
    assert proposal.acceptance.satisfied
    assert proposal.acceptance.criterion("position").measured == pytest.approx(0.005)


def test_stage_docking_pair_supports_requested_roll(example_pack: RobotPack) -> None:
    session = RuntimeSession.create(
        example_pack,
        SceneSpec.grid("generic_cube", 2, spacing_m=1.0),
        MockBackendAdapter(),
    )
    stage_docking_assembly_pair(session, FIXED, MOVING, gap_m=0.0, orientation_rad=math.pi / 2)
    proposal = next(item for item in session.proposals() if item.acceptance.satisfied)
    assert proposal.acceptance.orientation_index == 1


def test_stage_docking_pair_rejects_bad_requests(example_pack: RobotPack) -> None:
    session = RuntimeSession.create(
        example_pack,
        SceneSpec.grid("generic_cube", 2, spacing_m=1.0),
        MockBackendAdapter(),
    )
    with pytest.raises(ValueError, match="must not be negative"):
        stage_docking_assembly_pair(session, FIXED, MOVING, gap_m=-0.001)
    with pytest.raises(ReconfigurationPlanError, match="unknown connector"):
        stage_docking_assembly_pair(
            session, ConnectorInstanceId("generic_cube_0/missing"), MOVING, gap_m=0.0
        )
    with pytest.raises(ReconfigurationPlanError, match="different modules"):
        stage_docking_assembly_pair(
            session, FIXED, ConnectorInstanceId("generic_cube_0/rear"), gap_m=0.0
        )


def test_pair_plan_docks_through_the_general_scenario(example_pack: RobotPack) -> None:
    scenario = _scenario(example_pack)
    events = scenario.step()

    assert any(isinstance(event, DockCommitted) for event in events)
    assert any(isinstance(event, AssemblyMerged) for event in events)
    assert tuple(scenario.session.world.connections) == (connection_id(FIXED, MOVING),)
    assert scenario.status.phase is ReconfigurationPhase.DOCKING
    scenario.step()
    assert scenario.status.phase is ReconfigurationPhase.HOLDING_CONNECTED


def test_pair_plan_docks_undocks_and_retracts(example_pack: RobotPack) -> None:
    scenario = _scenario(example_pack, include_undock=True, release_after_s=0.02)
    all_events: list[Event] = []
    for _ in range(5):
        all_events.extend(scenario.step())

    assert any(isinstance(event, DockCommitted) for event in all_events)
    assert any(isinstance(event, AssemblyMerged) for event in all_events)
    assert any(isinstance(event, UndockCommitted) for event in all_events)
    assert any(isinstance(event, AssemblySplit) for event in all_events)
    assert not scenario.session.world.connections
    assert scenario.status.phase is ReconfigurationPhase.COMPLETE


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("gap_m", -0.1, "gap_m must not be negative"),
        ("orientation_rad", math.inf, "orientation_rad must be finite"),
        ("approach_speed_m_s", math.nan, "approach_speed_m_s must be finite"),
        ("dt_s", 0.0, "dt_s must be greater than zero"),
        ("release_after_s", -1.0, "release_after_s must not be negative"),
        ("retract_speed_m_s", math.inf, "retract_speed_m_s must be finite"),
    ),
)
def test_scenario_config_rejects_invalid_numbers(field: str, value: float, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        ScriptedReconfigurationConfig(**{field: value})  # type: ignore[arg-type]
