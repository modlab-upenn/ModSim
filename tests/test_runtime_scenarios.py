from __future__ import annotations

import math

import pytest

from conftest import with_connector_policy
from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import AssemblyMerged, AssemblySplit, DockCommitted, UndockCommitted
from modsim.core.ids import ConnectorInstanceId, connection_id
from modsim.core.scene import SceneSpec
from modsim.robot_packs import RobotPack
from modsim.runtime.scenarios import (
    DockingPairPhase,
    DockingPairScenario,
    DockingPairScenarioConfig,
    ScenarioSetupError,
    stage_docking_pair,
)
from modsim.runtime.session import RuntimeSession


def test_stage_docking_pair_uses_measured_connector_frames(example_pack: RobotPack) -> None:
    scene = SceneSpec.grid("generic_cube", 2, spacing_m=1.0)
    session = RuntimeSession.create(example_pack, scene, MockBackendAdapter())

    setup = stage_docking_pair(
        session,
        ConnectorInstanceId("generic_cube_0/front"),
        ConnectorInstanceId("generic_cube_1/front"),
        gap_m=0.005,
    )

    fixed = session.world.connector(setup.fixed_connector)
    moving = session.world.connector(setup.moving_connector)
    assert moving.world_pose.translation == pytest.approx((0.055, 0.0, 0.0))
    assert moving.world_docking_axis == pytest.approx((-1.0, 0.0, 0.0), abs=1e-9)
    assert setup.approach_direction == pytest.approx((-1.0, 0.0, 0.0))
    proposal = next(
        item
        for item in session.proposals()
        if {item.connector_a, item.connector_b} == {setup.fixed_connector, setup.moving_connector}
    )
    assert proposal.acceptance.satisfied
    assert proposal.acceptance.criterion("position").measured == pytest.approx(0.005)
    assert fixed.module_id != moving.module_id


def test_stage_docking_pair_supports_requested_roll(example_pack: RobotPack) -> None:
    scene = SceneSpec.grid("generic_cube", 2, spacing_m=1.0)
    session = RuntimeSession.create(example_pack, scene, MockBackendAdapter())

    stage_docking_pair(
        session,
        ConnectorInstanceId("generic_cube_0/front"),
        ConnectorInstanceId("generic_cube_1/front"),
        gap_m=0.0,
        orientation_rad=math.pi / 2.0,
    )

    proposal = next(item for item in session.proposals() if item.acceptance.satisfied)
    assert proposal.acceptance.orientation_index == 1


def test_stage_docking_pair_rejects_bad_requests(example_pack: RobotPack) -> None:
    scene = SceneSpec.grid("generic_cube", 2, spacing_m=1.0)
    session = RuntimeSession.create(example_pack, scene, MockBackendAdapter())

    with pytest.raises(ScenarioSetupError, match="must not be negative"):
        stage_docking_pair(
            session,
            ConnectorInstanceId("generic_cube_0/front"),
            ConnectorInstanceId("generic_cube_1/front"),
            gap_m=-0.001,
        )
    with pytest.raises(ScenarioSetupError, match="unknown connector"):
        stage_docking_pair(
            session,
            ConnectorInstanceId("generic_cube_0/missing"),
            ConnectorInstanceId("generic_cube_1/front"),
            gap_m=0.0,
        )
    with pytest.raises(ScenarioSetupError, match="different modules"):
        stage_docking_pair(
            session,
            ConnectorInstanceId("generic_cube_0/front"),
            ConnectorInstanceId("generic_cube_0/rear"),
            gap_m=0.0,
        )


def test_docking_pair_scenario_targets_one_pair_and_reports_status(
    example_pack: RobotPack,
) -> None:
    scene = SceneSpec.grid("generic_cube", 2, spacing_m=1.0)
    session = RuntimeSession.create(example_pack, scene, MockBackendAdapter())
    fixed = ConnectorInstanceId("generic_cube_0/front")
    moving = ConnectorInstanceId("generic_cube_1/front")
    scenario = DockingPairScenario.create(
        session,
        DockingPairScenarioConfig(
            fixed_connector=fixed,
            moving_connector=moving,
            gap_m=0.005,
            approach_speed_m_s=0.0,
            dt_s=0.01,
        ),
    )

    events = scenario.step()

    target = connection_id(fixed, moving)
    assert tuple(session.world.connections) == (target,)
    assert any(isinstance(event, DockCommitted) for event in events)
    assert any(isinstance(event, AssemblyMerged) for event in events)
    assert scenario.status.phase is DockingPairPhase.DOCKED
    assert scenario.status.target_connection_id == target
    assert scenario.status.connected
    assert not scenario.status.release_requested


def test_docking_pair_scenario_release_is_relative_and_retract_zero_completes(
    example_pack: RobotPack,
) -> None:
    scene = SceneSpec.grid("generic_cube", 2, spacing_m=1.0)
    session = RuntimeSession.create(example_pack, scene, MockBackendAdapter())
    session.step(0.5)
    fixed = ConnectorInstanceId("generic_cube_0/front")
    moving = ConnectorInstanceId("generic_cube_1/front")
    scenario = DockingPairScenario.create(
        session,
        DockingPairScenarioConfig(
            fixed_connector=fixed,
            moving_connector=moving,
            gap_m=0.005,
            approach_speed_m_s=0.0,
            dt_s=0.01,
            release_after_s=0.01,
            retract_speed_m_s=0.0,
        ),
    )

    dock_events = scenario.step()
    assert any(isinstance(event, DockCommitted) for event in dock_events)
    assert scenario.status.time_s == pytest.approx(0.51)
    assert scenario.status.phase is DockingPairPhase.DOCKED

    release_events = scenario.step()
    assert any(isinstance(event, UndockCommitted) for event in release_events)
    assert any(isinstance(event, AssemblySplit) for event in release_events)
    assert not session.world.connections
    assert scenario.status.phase is DockingPairPhase.COMPLETE
    assert scenario.status.release_requested


def test_release_scheduled_before_auto_latch_is_applied_after_connection(
    example_pack: RobotPack,
) -> None:
    latching = with_connector_policy(
        example_pack,
        "fixed_face",
        auto_latch=True,
        redock_cooldown_s=1.0,
    )
    session = RuntimeSession.create(
        latching,
        SceneSpec.grid("generic_cube", 2, spacing_m=1.0),
        MockBackendAdapter(),
    )
    scenario = DockingPairScenario.create(
        session,
        DockingPairScenarioConfig(
            fixed_connector=ConnectorInstanceId("generic_cube_0/front"),
            moving_connector=ConnectorInstanceId("generic_cube_1/front"),
            gap_m=0.005,
            approach_speed_m_s=0.0,
            dt_s=0.01,
            release_after_s=0.0,
            retract_speed_m_s=0.0,
        ),
    )

    assert any(isinstance(event, DockCommitted) for event in scenario.step())
    assert scenario.status.connected
    assert scenario.status.release_requested

    assert any(isinstance(event, UndockCommitted) for event in scenario.step())
    assert not scenario.status.connected
    assert scenario.status.phase is DockingPairPhase.COMPLETE


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
def test_docking_pair_scenario_config_rejects_invalid_numbers(
    field: str,
    value: float,
    message: str,
) -> None:
    values: dict[str, object] = {
        "fixed_connector": ConnectorInstanceId("generic_cube_0/front"),
        "moving_connector": ConnectorInstanceId("generic_cube_1/front"),
        field: value,
    }
    with pytest.raises(ValueError, match=message):
        DockingPairScenarioConfig(**values)  # type: ignore[arg-type]
