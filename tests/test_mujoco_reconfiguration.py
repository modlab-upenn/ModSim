"""Real-physics regressions for scripted multi-module reconfiguration.

The tests use the committed generic-cube URDF and add four semantic connector
faces in memory. No private SMORES-EP assets are required.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Protocol, cast

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.backends.base import BackendAdapter, SupportsModuleKinematics
from modsim.core.events import DockCommitted, DockFailed, Event, UndockCommitted, UndockFailed
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, vec_norm
from modsim.robot_packs import LoadedRobotPack, RobotPack, RobotPackLoader
from modsim.runtime.presets import smores_driver_to_snake_plan
from modsim.runtime.reconfiguration import (
    ReconfigurationPhase,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
)
from modsim.runtime.scenarios import stage_docking_pair
from modsim.runtime.session import RuntimeSession

pytestmark = pytest.mark.mujoco

MODULE_TYPE = "generic_cube"
MODULE_1 = ModuleInstanceId("module_1")
MODULE_2 = ModuleInstanceId("module_2")


class _WeldPoolView(Protocol):
    @property
    def in_use(self) -> int:
        """Return the number of active physical weld slots."""
        ...


class _MuJoCoAdapterView(BackendAdapter, SupportsModuleKinematics, Protocol):
    """Structural test view over the optional adapter's observable controls."""

    @property
    def time_s(self) -> float:
        """Return backend simulation time."""
        ...

    @property
    def weld_pool(self) -> _WeldPoolView:
        """Return the adapter's runtime weld allocator."""
        ...


@pytest.fixture
def four_face_loaded_pack(example_pack_dir: Path) -> LoadedRobotPack:
    """Return the public cube pack with SMORES-like face names in memory."""
    loaded = RobotPackLoader().load(example_pack_dir)
    data: dict[str, Any] = loaded.pack.model_dump(mode="python")
    module = data["hardware_catalog"]["module_types"][MODULE_TYPE]
    module["connectors"] = [
        _connector_spec(
            "pan",
            position=(0.05, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0),
            axis=(1.0, 0.0, 0.0),
        ),
        _connector_spec(
            "bottom",
            position=(-0.05, 0.0, 0.0),
            rotation=(0.0, 0.0, math.pi),
            axis=(-1.0, 0.0, 0.0),
        ),
        _connector_spec(
            "right",
            position=(0.0, 0.05, 0.0),
            rotation=(0.0, 0.0, math.pi / 2.0),
            axis=(0.0, 1.0, 0.0),
        ),
        _connector_spec(
            "left",
            position=(0.0, -0.05, 0.0),
            rotation=(0.0, 0.0, -math.pi / 2.0),
            axis=(0.0, -1.0, 0.0),
        ),
    ]
    return loaded.with_pack(RobotPack.model_validate(data))


def _connector_spec(
    identifier: str,
    *,
    position: tuple[float, float, float],
    rotation: tuple[float, float, float],
    axis: tuple[float, float, float],
) -> dict[str, object]:
    return {
        "id": identifier,
        "connector_type": "fixed_face",
        "parent_link": "base_link",
        "frame": None,
        "local_pose": {
            "xyz_m": list(position),
            "rpy_rad": list(rotation),
        },
        "docking_axis": list(axis),
        "approach_axis": list(axis),
    }


def _seven_module_scene() -> SceneSpec:
    plan = smores_driver_to_snake_plan()
    return SceneSpec.of(
        ModulePlacement(
            instance_id=module_id,
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((0.3 * index, 0.0, 0.0)),
        )
        for index, module_id in enumerate(plan.module_ids)
    )


def _weightless_session(
    loaded: LoadedRobotPack,
    scene: SceneSpec,
) -> tuple[RuntimeSession, _MuJoCoAdapterView]:
    session = RuntimeSession.create(
        loaded,
        scene,
        "mujoco",
        gravity=(0.0, 0.0, 0.0),
        timestep_s=0.002,
    )
    adapter = session.adapter
    assert adapter.capabilities().name == "mujoco"
    assert isinstance(adapter, SupportsModuleKinematics)
    return session, cast(_MuJoCoAdapterView, adapter)


def _all_module_state_is_finite(session: RuntimeSession) -> bool:
    return all(
        math.isfinite(value)
        for module in session.world.modules.values()
        for value in (
            *module.pose.translation,
            *module.pose.rotation,
            *module.linear_velocity_m_s,
            *module.angular_velocity_rad_s,
        )
    )


def test_driver_to_snake_completes_with_six_real_mujoco_welds(
    four_face_loaded_pack: LoadedRobotPack,
) -> None:
    """Run the complete seven-module paper sequence through real MuJoCo."""
    plan = smores_driver_to_snake_plan()
    session, adapter = _weightless_session(four_face_loaded_pack, _seven_module_scene())
    action_events: list[Event] = []

    try:
        scenario = ScriptedReconfigurationScenario.create(
            session,
            plan,
            ScriptedReconfigurationConfig(
                dt_s=0.002,
                gap_m=0.01,
                approach_speed_m_s=0.03,
                initial_hold_s=0.002,
                separated_hold_s=0.002,
                connected_hold_s=0.002,
            ),
        )

        # Initial staging uses the zero-time docking pass. All six constraints
        # therefore exist before unconstrained contact physics can disturb a
        # later connector pair.
        assert session.world.time_s == 0.0
        assert len(session.world.modules) == 7
        assert len(session.world.connections) == 6
        assert session.world.assemblies.count == 1
        assert adapter.weld_pool.in_use == 6
        assert _all_module_state_is_finite(session)

        undock_connection_counts: list[int] = []
        dock_connection_counts: list[int] = []
        for _ in range(2_000):
            events = scenario.step()
            action_events.extend(events)
            if any(isinstance(event, UndockCommitted) for event in events):
                undock_connection_counts.append(len(session.world.connections))
            if any(isinstance(event, DockCommitted) for event in events):
                dock_connection_counts.append(len(session.world.connections))
            assert _all_module_state_is_finite(session)
            if scenario.status.phase in (
                ReconfigurationPhase.COMPLETE,
                ReconfigurationPhase.FAILED,
            ):
                break
        else:  # pragma: no cover - guards against an accidental non-terminating phase
            pytest.fail(f"MuJoCo reconfiguration did not finish: {scenario.status}")

        assert scenario.status.phase is ReconfigurationPhase.COMPLETE
        assert undock_connection_counts == [5, 5, 5, 5]
        assert dock_connection_counts == [6, 6, 6, 6]
        assert len([event for event in action_events if isinstance(event, UndockCommitted)]) == 4
        assert len([event for event in action_events if isinstance(event, DockCommitted)]) == 4
        assert not [
            event for event in action_events if isinstance(event, DockFailed | UndockFailed)
        ]

        expected = {pair.connection_id for pair in plan.initial_connections}
        for action in plan.actions:
            expected.remove(action.undock.connection_id)
            expected.add(action.dock.connection_id)
        assert set(session.world.connections) == expected
        assert session.world.assemblies.count == 1
        assert adapter.weld_pool.in_use == 6

        metrics = session.metrics()
        assert metrics.docking_success_count == 10
        assert metrics.undocking_success_count == 4
        assert metrics.connection_count_active == 6

        # A completed scenario remains a live simulation for the viewer. Keep
        # stepping briefly and require the final welded state to stay finite.
        for _ in range(10):
            scenario.step()
        assert len(session.world.connections) == 6
        assert adapter.weld_pool.in_use == 6
        assert _all_module_state_is_finite(session)
    finally:
        session.shutdown()


def test_process_docking_commits_without_advancing_mujoco_time(
    four_face_loaded_pack: LoadedRobotPack,
) -> None:
    """An exact staged pair can be constrained before the next physics step."""
    scene = SceneSpec.of(
        (
            ModulePlacement(instance_id=MODULE_1, module_type_id=MODULE_TYPE),
            ModulePlacement(
                instance_id=MODULE_2,
                module_type_id=MODULE_TYPE,
                pose=Transform.from_translation((0.5, 0.0, 0.0)),
            ),
        )
    )
    session, adapter = _weightless_session(four_face_loaded_pack, scene)
    fixed = ConnectorInstanceId("module_1/pan")
    moving = ConnectorInstanceId("module_2/pan")

    try:
        stage_docking_pair(session, fixed, moving, gap_m=0.0)
        before_world_s = session.world.time_s
        before_backend_s = adapter.time_s
        session.request_dock(fixed, moving)

        events = session.process_docking()

        assert any(isinstance(event, DockCommitted) for event in events)
        assert not [event for event in events if isinstance(event, DockFailed)]
        assert session.world.time_s == before_world_s
        assert adapter.time_s == before_backend_s
        assert len(session.world.connections) == 1
        assert adapter.weld_pool.in_use == 1
    finally:
        session.shutdown()


def test_set_module_pose_clears_mujoco_root_velocity(
    four_face_loaded_pack: LoadedRobotPack,
) -> None:
    """Teleporting a staged module cannot carry stale kinetic state with it."""
    scene = SceneSpec.of((ModulePlacement(instance_id=MODULE_1, module_type_id=MODULE_TYPE),))
    session, adapter = _weightless_session(four_face_loaded_pack, scene)

    try:
        adapter.set_module_twist(
            MODULE_1,
            linear_m_s=(1.0, -2.0, 3.0),
            angular_rad_s=(0.2, -0.1, 0.3),
        )
        moving = adapter.snapshot().body(MODULE_1, "base_link")
        assert moving is not None
        assert vec_norm(moving.linear_velocity_m_s) > 1.0
        assert vec_norm(moving.angular_velocity_rad_s) > 0.1
        before_s = adapter.time_s

        target = Transform.from_translation((0.2, -0.3, 0.4))
        adapter.set_module_pose(MODULE_1, target)
        session.world.ingest(adapter.snapshot())

        stopped = adapter.snapshot().body(MODULE_1, "base_link")
        assert stopped is not None
        assert stopped.pose.is_close(target)
        assert stopped.linear_velocity_m_s == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert stopped.angular_velocity_rad_s == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert adapter.time_s == before_s
        assert session.world.time_s == before_s
    finally:
        session.shutdown()
