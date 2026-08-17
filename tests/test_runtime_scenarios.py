from __future__ import annotations

import math

import pytest

from modsim.backends.mock import MockBackendAdapter
from modsim.core.ids import ConnectorInstanceId
from modsim.core.scene import SceneSpec
from modsim.robot_packs import RobotPack
from modsim.runtime.scenarios import ScenarioSetupError, stage_docking_pair
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
