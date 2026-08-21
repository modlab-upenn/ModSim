from __future__ import annotations

import math
from typing import Any, cast

import pytest

from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import DockCommitted, UndockCommitted
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ModuleInstanceId,
    connection_id,
)
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.model_views import (
    ModelViewContext,
    ModelViewFactory,
    ModuleTopologyGraphView,
)
from modsim.robot_packs import RobotPack
from modsim.runtime.presets import (
    SMORES_DRIVER_TO_SNAKE_SOURCE,
    smores_driver_to_snake_plan,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationAction,
    ReconfigurationPhase,
    ReconfigurationPlan,
    ReconfigurationPlanError,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
    stage_docking_assembly_pair,
)
from modsim.runtime.session import RuntimeSession

_SOURCE_URL = (
    "https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf"
)


def _undirected_pair(pair: ConnectorPairRef) -> frozenset[str]:
    return frozenset(str(connector) for connector in pair.connectors)


def test_builtin_driver_to_snake_plan_matches_the_published_connector_actions() -> None:
    plan = smores_driver_to_snake_plan()

    assert plan.source_url == SMORES_DRIVER_TO_SNAKE_SOURCE == _SOURCE_URL
    assert tuple(str(module_id) for module_id in plan.module_ids) == tuple(
        f"module_{number}" for number in range(1, 8)
    )
    assert tuple(_undirected_pair(pair) for pair in plan.initial_connections) == (
        frozenset(("module_1/bottom", "module_2/pan")),
        frozenset(("module_2/bottom", "module_3/pan")),
        frozenset(("module_2/right", "module_4/left")),
        frozenset(("module_4/right", "module_5/left")),
        frozenset(("module_5/pan", "module_6/bottom")),
        frozenset(("module_5/bottom", "module_7/pan")),
    )
    assert tuple(
        (_undirected_pair(action.undock), _undirected_pair(action.dock)) for action in plan.actions
    ) == (
        (
            frozenset(("module_1/bottom", "module_2/pan")),
            frozenset(("module_1/pan", "module_3/bottom")),
        ),
        (
            frozenset(("module_7/pan", "module_5/bottom")),
            frozenset(("module_7/bottom", "module_6/pan")),
        ),
        (
            frozenset(("module_2/right", "module_4/left")),
            frozenset(("module_2/pan", "module_4/bottom")),
        ),
        (
            frozenset(("module_5/left", "module_4/right")),
            frozenset(("module_5/bottom", "module_4/pan")),
        ),
    )


def _four_face_pack(example_pack: RobotPack) -> RobotPack:
    data: dict[str, Any] = example_pack.model_dump(mode="python")
    module = data["hardware_catalog"]["module_types"]["generic_cube"]
    module["connectors"] = [
        {
            "id": "pan",
            "connector_type": "fixed_face",
            "parent_link": "base_link",
            "frame": None,
            "local_pose": {
                "xyz_m": [0.05, 0.0, 0.0],
                "rpy_rad": [0.0, 0.0, 0.0],
            },
            "docking_axis": [1.0, 0.0, 0.0],
            "approach_axis": [1.0, 0.0, 0.0],
        },
        {
            "id": "bottom",
            "connector_type": "fixed_face",
            "parent_link": "base_link",
            "frame": None,
            "local_pose": {
                "xyz_m": [-0.05, 0.0, 0.0],
                "rpy_rad": [0.0, 0.0, math.pi],
            },
            "docking_axis": [-1.0, 0.0, 0.0],
            "approach_axis": [-1.0, 0.0, 0.0],
        },
        {
            "id": "right",
            "connector_type": "fixed_face",
            "parent_link": "base_link",
            "frame": None,
            "local_pose": {
                "xyz_m": [0.0, 0.05, 0.0],
                "rpy_rad": [0.0, 0.0, math.pi / 2.0],
            },
            "docking_axis": [0.0, 1.0, 0.0],
            "approach_axis": [0.0, 1.0, 0.0],
        },
        {
            "id": "left",
            "connector_type": "fixed_face",
            "parent_link": "base_link",
            "frame": None,
            "local_pose": {
                "xyz_m": [0.0, -0.05, 0.0],
                "rpy_rad": [0.0, 0.0, -math.pi / 2.0],
            },
            "docking_axis": [0.0, -1.0, 0.0],
            "approach_axis": [0.0, -1.0, 0.0],
        },
    ]
    return RobotPack.model_validate(data)


def _module(number: int) -> ModuleInstanceId:
    return ModuleInstanceId(f"smores_{number}")


def _connector(number: int, face: str) -> ConnectorInstanceId:
    return ConnectorInstanceId(f"{_module(number)}/{face}")


def _pair(
    fixed_number: int,
    fixed_face: str,
    moving_number: int,
    moving_face: str,
) -> ConnectorPairRef:
    return ConnectorPairRef(
        fixed_connector=_connector(fixed_number, fixed_face),
        moving_connector=_connector(moving_number, moving_face),
    )


def _driver_to_snake_plan() -> ReconfigurationPlan:
    return ReconfigurationPlan(
        id="smores_driver_to_snake",
        name="SMORES-EP Driver to Snake",
        module_ids=tuple(_module(number) for number in range(1, 8)),
        initial_connections=(
            _pair(2, "pan", 1, "bottom"),
            _pair(2, "bottom", 3, "pan"),
            _pair(2, "right", 4, "left"),
            _pair(4, "right", 5, "left"),
            _pair(5, "pan", 6, "bottom"),
            _pair(5, "bottom", 7, "pan"),
        ),
        actions=(
            ReconfigurationAction(
                label="Move module 1 from module 2 to module 3",
                undock=_pair(2, "pan", 1, "bottom"),
                dock=_pair(3, "bottom", 1, "pan"),
            ),
            ReconfigurationAction(
                label="Move module 7 from module 5 to module 6",
                undock=_pair(5, "bottom", 7, "pan"),
                dock=_pair(6, "pan", 7, "bottom"),
            ),
            ReconfigurationAction(
                label="Move modules 1-3 onto module 4",
                undock=_pair(4, "left", 2, "right"),
                dock=_pair(4, "bottom", 2, "pan"),
            ),
            ReconfigurationAction(
                label="Move modules 5-7 onto module 4",
                undock=_pair(4, "right", 5, "left"),
                dock=_pair(4, "pan", 5, "bottom"),
            ),
        ),
        source_url=_SOURCE_URL,
    )


def _session(pack: RobotPack) -> tuple[RuntimeSession, MockBackendAdapter]:
    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=_module(number),
            module_type_id="generic_cube",
            pose=Transform.from_translation((0.5 * number, 0.0, 0.0)),
        )
        for number in range(1, 8)
    )
    adapter = MockBackendAdapter()
    return RuntimeSession.create(pack, scene, adapter), adapter


def _topology(session: RuntimeSession) -> ModuleTopologyGraphView:
    views = ModelViewFactory().build_default_runtime(
        ModelViewContext(pack=session.world.pack, world=session.world)
    )
    assert len(views) == 1
    return cast(ModuleTopologyGraphView, views[0])


def test_driver_to_snake_reconfiguration_preserves_tree_and_canonical_events(
    example_pack: RobotPack,
) -> None:
    pack = _four_face_pack(example_pack)
    session, adapter = _session(pack)
    plan = _driver_to_snake_plan()
    scenario = ScriptedReconfigurationScenario.create(
        session,
        plan,
        ScriptedReconfigurationConfig(
            dt_s=0.01,
            gap_m=0.01,
            approach_speed_m_s=0.02,
            initial_hold_s=0.01,
            separated_hold_s=0.01,
            connected_hold_s=0.01,
        ),
    )

    initial = _topology(session)
    assert len(initial.nodes) == 7
    assert len(initial.edges) == 6
    assert session.world.assemblies.count == 1
    assert len(adapter.welds) == 6
    initial_status = scenario.status
    assert initial_status.phase is ReconfigurationPhase.HOLDING_INITIAL
    assert initial_status.action_index is None
    assert initial_status.action_count == 4

    undock_edge_counts: list[int] = []
    dock_edge_counts: list[int] = []
    action_docks: list[DockCommitted] = []
    action_undocks: list[UndockCommitted] = []
    observed_three_module_approach = False
    approach_reference: dict[ModuleInstanceId, tuple[float, float, float]] | None = None
    for _ in range(2_000):
        events = scenario.step()
        for event in events:
            if isinstance(event, UndockCommitted):
                action_undocks.append(event)
                undock_edge_counts.append(len(_topology(session).edges))
            elif isinstance(event, DockCommitted):
                action_docks.append(event)
                dock_edge_counts.append(len(_topology(session).edges))
        current_status = scenario.status
        if (
            current_status.phase is ReconfigurationPhase.APPROACHING
            and current_status.action_index == 2
        ):
            translations = {
                _module(number): session.world.modules[_module(number)].pose.translation
                for number in (1, 2, 3)
            }
            if approach_reference is None:
                approach_reference = translations
            else:
                deltas = tuple(
                    tuple(
                        translations[module_id][axis] - approach_reference[module_id][axis]
                        for axis in range(3)
                    )
                    for module_id in (_module(1), _module(2), _module(3))
                )
                if any(abs(component) > 1e-12 for component in deltas[0]):
                    assert deltas[1] == pytest.approx(deltas[0])
                    assert deltas[2] == pytest.approx(deltas[0])
                    observed_three_module_approach = True
        if current_status.phase in (
            ReconfigurationPhase.COMPLETE,
            ReconfigurationPhase.FAILED,
        ):
            break
    else:  # pragma: no cover - protects the test from hanging on a bad phase transition
        pytest.fail(f"scenario did not finish: {scenario.status}")

    assert scenario.status.phase is ReconfigurationPhase.COMPLETE
    assert scenario.status.action_index == 3
    assert scenario.status.detail == "Completed 4 reconfiguration action(s)"
    assert len(action_undocks) == 4
    assert len(action_docks) == 4
    assert observed_three_module_approach
    assert undock_edge_counts == [5, 5, 5, 5]
    assert dock_edge_counts == [6, 6, 6, 6]

    expected_final: set[ConnectionId] = {
        connection_id(_connector(3, "bottom"), _connector(1, "pan")),
        connection_id(_connector(2, "bottom"), _connector(3, "pan")),
        connection_id(_connector(4, "bottom"), _connector(2, "pan")),
        connection_id(_connector(4, "pan"), _connector(5, "bottom")),
        connection_id(_connector(5, "pan"), _connector(6, "bottom")),
        connection_id(_connector(6, "pan"), _connector(7, "bottom")),
    }
    assert set(session.world.connections) == expected_final
    assert len(adapter.welds) == 6
    final = _topology(session)
    assert len(final.nodes) == 7
    assert {ConnectionId(edge.connection_id) for edge in final.edges} == expected_final

    metrics = session.metrics()
    assert metrics.connection_count_active == 6
    assert metrics.assembly_count == 1
    assert metrics.docking_success_count == 10  # six initial + four replacement edges
    assert metrics.undocking_success_count == 4
    assert metrics.docking_failure_count == 0
    assert metrics.undocking_failure_count == 0

    completed_at_s = session.world.time_s
    scenario.step()
    assert session.world.time_s == pytest.approx(completed_at_s + scenario.config.dt_s)
    assert scenario.status.phase is ReconfigurationPhase.COMPLETE


def test_assembly_staging_moves_every_member_by_one_rigid_transform(
    example_pack: RobotPack,
) -> None:
    pack = _four_face_pack(example_pack)
    session, _ = _session(pack)
    first = _pair(2, "bottom", 3, "pan")
    stage_docking_assembly_pair(
        session,
        first.fixed_connector,
        first.moving_connector,
        gap_m=0.0,
    )
    session.request_dock(first.fixed_connector, first.moving_connector)
    session.step(0.01)
    assert first.connection_id in session.world.connections

    before = session.world.modules[_module(3)].pose.relative_to(
        session.world.modules[_module(2)].pose
    )
    setup = stage_docking_assembly_pair(
        session,
        _connector(4, "left"),
        _connector(2, "right"),
        gap_m=0.02,
    )
    after = session.world.modules[_module(3)].pose.relative_to(
        session.world.modules[_module(2)].pose
    )

    assert setup.moving_modules == (_module(2), _module(3))
    assert after.is_close(before)
    assert first.connection_id in session.world.connections


def test_reconfiguration_plan_validation_rejects_unknown_connectors_before_mutation(
    example_pack: RobotPack,
) -> None:
    pack = _four_face_pack(example_pack)
    session, adapter = _session(pack)
    valid = _driver_to_snake_plan()
    invalid = ReconfigurationPlan(
        id=valid.id,
        name=valid.name,
        module_ids=valid.module_ids,
        initial_connections=valid.initial_connections,
        actions=(
            ReconfigurationAction(
                label="Unknown target",
                undock=valid.actions[0].undock,
                dock=ConnectorPairRef(
                    fixed_connector=ConnectorInstanceId("smores_3/missing"),
                    moving_connector=_connector(1, "pan"),
                ),
            ),
            *valid.actions[1:],
        ),
    )

    with pytest.raises(ReconfigurationPlanError, match="unknown connector 'smores_3/missing'"):
        ScriptedReconfigurationScenario.create(session, invalid)

    assert not session.world.connections
    assert not adapter.welds
    assert session.world.time_s == 0.0


def test_reconfiguration_plan_validation_rejects_non_tree_action(
    example_pack: RobotPack,
) -> None:
    pack = _four_face_pack(example_pack)
    session, _ = _session(pack)
    valid = _driver_to_snake_plan()
    invalid = ReconfigurationPlan(
        id="bad_driver_to_snake",
        name=valid.name,
        module_ids=valid.module_ids,
        initial_connections=valid.initial_connections,
        actions=(
            ReconfigurationAction(
                label="Try to undock an absent edge",
                undock=_pair(1, "pan", 7, "bottom"),
                dock=valid.actions[0].dock,
            ),
        ),
    )

    with pytest.raises(ReconfigurationPlanError, match="cannot undock inactive connection"):
        ScriptedReconfigurationScenario.create(session, invalid)

    assert not session.world.connections
    assert session.world.time_s == 0.0


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("dt_s", 0.0, "dt_s must be greater than zero"),
        ("gap_m", -0.01, "gap_m must not be negative"),
        ("approach_speed_m_s", math.inf, "approach_speed_m_s must be finite"),
        ("initial_hold_s", -1.0, "initial_hold_s must not be negative"),
        ("separated_hold_s", math.nan, "separated_hold_s must be finite"),
        ("connected_hold_s", -1.0, "connected_hold_s must not be negative"),
    ),
)
def test_reconfiguration_config_rejects_invalid_numbers(
    field: str,
    value: float,
    message: str,
) -> None:
    values: dict[str, float] = {field: value}
    with pytest.raises(ValueError, match=message):
        ScriptedReconfigurationConfig(**values)  # type: ignore[arg-type]


def test_connector_pair_requires_full_runtime_connector_ids() -> None:
    with pytest.raises(ValueError, match="full runtime connector ID"):
        ConnectorPairRef(
            fixed_connector=ConnectorInstanceId("pan"),
            moving_connector=ConnectorInstanceId("smores_2/bottom"),
        )
