"""Qt-free Runtime Inspector execution ownership tests."""

from __future__ import annotations

import math
import shutil
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

from modsim.backends.mock import MockBackendAdapter
from modsim.core.events import Event
from modsim.core.scene import SceneSpec
from modsim.robot_packs import LoadedRobotPack
from modsim.runtime import (
    ReconfigurationPhase,
    ReconfigurationStatus,
    RuntimeDemo,
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
    RuntimeInspectorSetupError,
    ScriptedReconfigurationScenario,
)
from modsim.runtime.inspector_runner import validate_runtime_inspector_config
from modsim.runtime.physical_reconfiguration import (
    DifferentialDriveReconfigurationConfig,
)
from modsim.runtime.reconfiguration import ReconfigurationPlan
from modsim.runtime.session import RuntimeSession


def test_runtime_inspector_config_defaults_to_real_time_playback() -> None:
    assert RuntimeInspectorConfig(pack_path=Path("pack")).real_time_factor == 1.0


@pytest.mark.parametrize("real_time_factor", (0.0, -1.0, math.nan, math.inf))
def test_runtime_inspector_config_rejects_invalid_real_time_factor(
    real_time_factor: float,
) -> None:
    config = RuntimeInspectorConfig(
        pack_path=Path("pack"),
        backend="mock",
        real_time_factor=real_time_factor,
    )

    with pytest.raises(RuntimeInspectorSetupError, match="real_time_factor must"):
        validate_runtime_inspector_config(config)


def test_runner_owns_scenario_frame_cursor_and_shutdown(example_pack_dir: Path) -> None:
    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            backend="mock",
            connector_gap_m=0.005,
            approach_m_s=0.03,
            duration_s=0.05,
            dt_s=0.01,
            publish_hz=100.0,
            # Presentation topology is intentionally irrelevant to execution.
            viewer_enabled=True,
        ),
        status_callback=statuses.append,
    )
    try:
        assert runner.step_count == 5
        assert runner.event_cursor == 0
        initial = runner.frame()
        assert initial.view.edges == ()
        assert initial.events == ()
        assert runner.event_cursor == 0

        for _ in range(runner.step_count):
            runner.step()

        docked = runner.frame()
        assert len(docked.view.edges) == 1
        assert [row.kind for row in docked.events] == [
            "DockCandidateDetected",
            "DockCommitted",
            "AssemblyMerged",
        ]
        assert docked.event_start_sequence == 0
        assert docked.next_event_sequence == 3
        assert runner.event_cursor == 3

        unchanged = runner.frame()
        assert unchanged.events == ()
        assert unchanged.event_start_sequence == unchanged.next_event_sequence == 3
        assert statuses == [
            "Loading and validating Robot Pack…",
            "Starting mock backend…",
            "Running generic_cube: front ↔ front",
        ]
    finally:
        runner.shutdown()

    runner.shutdown()
    assert runner.closed
    with pytest.raises(RuntimeError, match="shut down"):
        runner.step()
    with pytest.raises(RuntimeError, match="shut down"):
        runner.frame()


def test_runner_rejects_incomplete_connector_pair_before_backend_start(
    example_pack_dir: Path,
) -> None:
    with pytest.raises(ValueError, match="must be supplied together"):
        RuntimeInspectorRunner.create(
            RuntimeInspectorConfig(
                pack_path=example_pack_dir,
                backend="mock",
                fixed_connector="front",
            )
        )


def test_runner_loads_smores_plan_from_the_example_tree(smores_pack_dir: Path) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=smores_pack_dir,
            demo=RuntimeDemo.SMORES_DRIVER_TO_SNAKE,
            backend="mock",
            duration_s=0.01,
            dt_s=0.01,
        )
    )
    try:
        assert isinstance(runner.scenario, ScriptedReconfigurationScenario)
        assert runner.scenario.plan.id == "smores_driver_to_snake"
        assert len(runner.scenario.plan.module_ids) == 7
        assert runner.session.world.assemblies.count == 1
        assert len(runner.session.world.connections) == 6
    finally:
        runner.shutdown()


def test_runner_loads_smores_plan_for_a_staged_pack_copy(
    tmp_path: Path,
    smores_pack_dir: Path,
) -> None:
    staged_pack = tmp_path / ".modsim" / "robot_packs" / "smores_ep"
    shutil.copytree(smores_pack_dir, staged_pack)

    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=staged_pack,
            demo=RuntimeDemo.SMORES_DRIVER_TO_SNAKE,
            backend="mock",
            duration_s=0.01,
            dt_s=0.01,
        )
    )
    try:
        assert isinstance(runner.scenario, ScriptedReconfigurationScenario)
        assert runner.scenario.plan.id == "smores_driver_to_snake"
        assert len(runner.session.world.modules) == 7
        assert len(runner.session.world.connections) == 6
    finally:
        runner.shutdown()


def test_runner_wires_physical_driver_to_snake_without_starting_mujoco(
    monkeypatch: pytest.MonkeyPatch,
    smores_pack_dir: Path,
) -> None:
    import modsim.runtime.inspector_runner as runner_module

    captured: dict[str, object] = {}

    class StubPhysicalScenario:
        def __init__(self, session: RuntimeSession, plan: ReconfigurationPlan) -> None:
            self.session = session
            self.plan = plan

        @property
        def status(self) -> ReconfigurationStatus:
            return ReconfigurationStatus(
                phase=ReconfigurationPhase.HOLDING_INITIAL,
                time_s=self.session.world.time_s,
                plan_id=self.plan.id,
                plan_name=self.plan.name,
                action_index=None,
                action_count=len(self.plan.actions),
                detail="Physical scenario stub",
            )

        def step(self) -> tuple[Event, ...]:
            return ()

    def fake_create_session(
        loaded: LoadedRobotPack,
        scene: SceneSpec,
        _config: RuntimeInspectorConfig,
    ) -> RuntimeSession:
        captured["scene"] = scene
        return RuntimeSession.create(loaded, scene, MockBackendAdapter())

    def fake_create_scenario(
        session: RuntimeSession,
        plan: ReconfigurationPlan,
        config: DifferentialDriveReconfigurationConfig,
    ) -> StubPhysicalScenario:
        captured["plan"] = plan
        captured["physical_config"] = config
        return StubPhysicalScenario(session, plan)

    monkeypatch.setattr(runner_module, "_create_session", fake_create_session)
    monkeypatch.setattr(
        runner_module.DifferentialDriveReconfigurationScenario,
        "create",
        staticmethod(fake_create_scenario),
    )
    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=smores_pack_dir,
            demo=RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
            backend="mujoco",
            duration_s=180.0,
            dt_s=0.002,
            gravity=True,
            ground=True,
            height_m=0.05,
        ),
        status_callback=statuses.append,
    )
    try:
        scene = captured["scene"]
        plan = captured["plan"]
        physical_config = captured["physical_config"]
        assert isinstance(scene, SceneSpec)
        assert isinstance(plan, ReconfigurationPlan)
        assert isinstance(physical_config, DifferentialDriveReconfigurationConfig)
        assert len(scene.placements) == 7
        assert tuple(scene.instance_ids) == plan.module_ids
        assert all(placement.pose.translation[2] == 0.05 for placement in scene.placements)
        assert len(plan.actions) == len(physical_config.routes) == 4
        assert physical_config.dt_s == pytest.approx(0.002)
        assert physical_config.navigation_speed_m_s == pytest.approx(0.03)
        assert physical_config.approach_speed_m_s == pytest.approx(0.03)
        assert runner.fixed_connector is None
        assert runner.moving_connector is None
        assert statuses[-1] == (
            "Running smores_physical_driver_to_snake: SMORES-EP Driver to Snake with 7 modules"
        )
    finally:
        runner.shutdown()


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"backend": "mock"}, "smores_physical_driver_to_snake requires the MuJoCo backend"),
        ({"gravity": False}, "requires gravity and the ground plane"),
        ({"ground": False}, "requires gravity and the ground plane"),
        ({"height_m": 0.03}, "requires height_m >= 0.04"),
        ({"dt_s": 0.01}, "requires dt_s <= 0.005"),
        ({"orientation_rad": 0.1}, "orientation_rad must be zero"),
        (
            {"fixed_connector": "pan", "moving_connector": "pan"},
            "defines its connector actions",
        ),
        ({"undock_at_s": 1.0}, "defines its own releases"),
        ({"connector_gap_m": 0.02}, "leave connector_gap_m unspecified"),
        ({"retract_m_s": 0.03}, "leave retract_m_s unspecified"),
    ),
)
def test_runner_validates_physical_driver_to_snake_options(
    smores_pack_dir: Path,
    overrides: dict[str, Any],
    message: str,
) -> None:
    base = RuntimeInspectorConfig(
        pack_path=smores_pack_dir,
        demo=RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
        backend="mujoco",
        gravity=True,
        ground=True,
        height_m=0.05,
        dt_s=0.002,
    )

    with pytest.raises(RuntimeInspectorSetupError, match=message):
        validate_runtime_inspector_config(replace(base, **overrides))


def test_runner_reports_missing_smores_example_content(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    smores_pack_dir: Path,
) -> None:
    missing_scenario = tmp_path / "missing" / "smores_driver_to_snake.py"
    monkeypatch.setattr(
        "modsim.runtime.inspector_runner._SMORES_EXAMPLE_SCENARIO",
        missing_scenario,
    )

    with pytest.raises(
        RuntimeInspectorSetupError,
        match="requires a ModSim source checkout",
    ):
        RuntimeInspectorRunner.create(
            RuntimeInspectorConfig(
                pack_path=smores_pack_dir,
                demo=RuntimeDemo.SMORES_DRIVER_TO_SNAKE,
                backend="mock",
            )
        )


def test_runner_dock_undock_demo_supplies_a_release_schedule(
    example_pack_dir: Path,
) -> None:
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=example_pack_dir,
            demo=RuntimeDemo.DOCK_UNDOCK,
            backend="mock",
            connector_gap_m=0.005,
            approach_m_s=0.03,
            retract_m_s=0.0,
            duration_s=0.1,
            dt_s=0.01,
            publish_hz=100.0,
        )
    )
    try:
        assert isinstance(runner.scenario, ScriptedReconfigurationScenario)
        assert runner.scenario.plan.id == "dock_undock"
        assert runner.scenario.config.release_after_s == pytest.approx(0.055)
        frames = [runner.frame()]
        for _ in range(runner.step_count):
            runner.step()
            frames.append(runner.frame())
    finally:
        runner.shutdown()

    phases: list[ReconfigurationPhase] = []
    for frame in frames:
        assert isinstance(frame.scenario, ReconfigurationStatus)
        phases.append(frame.scenario.phase)
    assert phases[0] is ReconfigurationPhase.APPROACHING
    assert ReconfigurationPhase.HOLDING_CONNECTED in phases
    assert phases[-1] is ReconfigurationPhase.COMPLETE
    assert [len(frame.view.edges) for frame in frames] == [0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0]
    assert [(frame.event_start_sequence, frame.next_event_sequence) for frame in frames] == [
        (0, 0),
        (0, 3),
        (3, 3),
        (3, 3),
        (3, 3),
        (3, 3),
        (3, 3),
        (3, 6),
        (6, 6),
        (6, 6),
        (6, 6),
    ]
    assert [row.kind for frame in frames for row in frame.events] == [
        "DockCandidateDetected",
        "DockCommitted",
        "AssemblyMerged",
        "UndockCommitted",
        "AssemblySplit",
        # The zero-retract faces remain coincident, so candidacy is observed
        # again during the release step without being explicitly re-latched.
        "DockCandidateDetected",
    ]
    final = frames[-1]
    assert final.metrics.docking_success_count == 1
    assert final.metrics.undocking_success_count == 1
    assert final.metrics.connection_count_active == 0
