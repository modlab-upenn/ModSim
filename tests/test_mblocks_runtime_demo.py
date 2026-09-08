"""Runtime Inspector and CLI integration for the M-Blocks demonstrations."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
from typer.testing import CliRunner

from modsim.backends.mock import MockBackendAdapter
from modsim.cli import app
from modsim.core.events import Event
from modsim.core.scene import SceneSpec
from modsim.model_views import CubicLatticeView
from modsim.robot_packs import LoadedRobotPack, RobotPackLoader
from modsim.runtime import (
    KinematicPivotScenario,
    ReconfigurationPhase,
    ReconfigurationStatus,
    RuntimeDemo,
    RuntimeInspectorConfig,
    RuntimeInspectorRunner,
    RuntimeInspectorSetupError,
)
from modsim.runtime.inspector_runner import validate_runtime_inspector_config
from modsim.runtime.momentum_pivot import (
    MAX_MOMENTUM_TIMESTEP_S,
    MomentumPivotConfig,
)
from modsim.runtime.session import RuntimeSession

_ROOT = Path(__file__).resolve().parents[1]
_PACK_PATH = _ROOT / "examples" / "robot_packs" / "mblocks_3d"


def test_runner_loads_five_module_pivot_with_default_lattice_recipe() -> None:
    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=_PACK_PATH,
            demo=RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
            backend="mock",
            duration_s=0.01,
            dt_s=0.01,
        ),
        status_callback=statuses.append,
    )
    try:
        assert isinstance(runner.scenario, KinematicPivotScenario)
        assert runner.scenario.plan.id == "mblocks_five_module_pivot"
        assert len(runner.scenario.plan.module_ids) == 5
        assert len(runner.scenario.routes) == 3
        assert runner.recipe.id == "mblocks_lattice"
        assert runner.recipe.builder == "cubic_lattice"
        assert runner.fixed_connector is None
        assert runner.moving_connector is None
        assert len(runner.session.world.modules) == 5
        assert len(runner.session.world.connections) == 4
        assert runner.session.world.assemblies.count == 1
        frame = runner.frame()
        assert isinstance(frame.view, CubicLatticeView)
        assert [(node.id, node.cell) for node in frame.view.nodes] == [
            ("block_1", (0, 0, 0)),
            ("block_2", (1, 0, 0)),
            ("block_3", (2, 0, 0)),
            ("block_4", (3, 0, 0)),
            ("block_5", (0, 0, 1)),
        ]
        assert len(frame.view.nodes) == 5
        assert len(frame.view.edges) == 4
        assert frame.scenario is not None
        assert frame.scenario.plan_id == "mblocks_five_module_pivot"
        assert statuses == [
            "Loading and validating Robot Pack…",
            "Starting mock backend…",
            "Running M-Blocks Five-Module Kinematic Pivot with 5 modules",
        ]
    finally:
        runner.shutdown()


def test_runner_loads_twelve_module_line_with_default_lattice_recipe() -> None:
    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=_PACK_PATH,
            demo=RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE,
            backend="mock",
            duration_s=0.01,
            dt_s=0.01,
        ),
        status_callback=statuses.append,
    )
    try:
        assert isinstance(runner.scenario, KinematicPivotScenario)
        assert runner.scenario.plan.id == "mblocks_twelve_module_line"
        assert len(runner.scenario.plan.module_ids) == 12
        assert len(runner.scenario.routes) == 11
        assert runner.recipe.id == "mblocks_lattice"
        assert runner.fixed_connector is None
        assert runner.moving_connector is None
        assert len(runner.session.world.modules) == 12
        assert len(runner.session.world.connections) == 11
        assert runner.session.world.assemblies.count == 1
        frame = runner.frame()
        assert isinstance(frame.view, CubicLatticeView)
        assert len(frame.view.nodes) == 12
        assert len(frame.view.edges) == 11
        assert frame.scenario is not None
        assert frame.scenario.plan_id == "mblocks_twelve_module_line"
        assert statuses[-1] == ("Running M-Blocks Twelve-Module Structure-to-Line with 12 modules")
    finally:
        runner.shutdown()


def test_runner_wires_two_module_momentum_pivot_without_starting_mujoco(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import modsim.runtime.inspector_runner as runner_module

    captured: dict[str, object] = {}

    class StubMomentumScenario:
        def __init__(self, session: RuntimeSession, config: MomentumPivotConfig) -> None:
            self.session = session
            self.config = config

        @property
        def status(self) -> ReconfigurationStatus:
            return ReconfigurationStatus(
                phase=ReconfigurationPhase.HOLDING_INITIAL,
                time_s=self.session.world.time_s,
                plan_id=self.config.plan_id,
                plan_name=self.config.plan_name,
                action_index=None,
                action_count=1,
                detail="Momentum scenario stub",
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
        config: MomentumPivotConfig,
    ) -> StubMomentumScenario:
        captured["momentum_config"] = config
        return StubMomentumScenario(session, config)

    monkeypatch.setattr(runner_module, "_create_session", fake_create_session)
    monkeypatch.setattr(
        runner_module.MomentumPivotScenario,
        "create",
        staticmethod(fake_create_scenario),
    )
    statuses: list[str] = []
    runner = RuntimeInspectorRunner.create(
        RuntimeInspectorConfig(
            pack_path=_PACK_PATH,
            demo=RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT,
            backend="mujoco",
            gravity=True,
            ground=True,
            height_m=0.025,
            duration_s=5.0,
            dt_s=0.00025,
        ),
        status_callback=statuses.append,
    )
    try:
        scene = captured["scene"]
        momentum_config = captured["momentum_config"]
        assert isinstance(scene, SceneSpec)
        assert isinstance(momentum_config, MomentumPivotConfig)
        assert tuple(scene.instance_ids) == ("support_block", "moving_block")
        assert [placement.pose.translation[2] for placement in scene.placements] == pytest.approx(
            [0.025, 0.075]
        )
        assert momentum_config.dt_s == pytest.approx(0.00025)
        assert runner.recipe.id == "mblocks_physics_lattice"
        assert runner.fixed_connector is None
        assert runner.moving_connector is None
        frame = runner.frame()
        assert isinstance(frame.view, CubicLatticeView)
        assert [(node.id, node.cell) for node in frame.view.nodes] == [
            ("moving_block", (0, 0, 1)),
            ("support_block", (0, 0, 0)),
        ]
        assert all(not node.off_lattice for node in frame.view.nodes)
        assert statuses[-1] == ("Running M-Blocks two-module physical edge roll with 2 modules")
    finally:
        runner.shutdown()


def test_runner_wires_physical_twelve_module_line_without_starting_mujoco(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import modsim.runtime.inspector_runner as runner_module

    captured: dict[str, object] = {}
    scene = SceneSpec.grid(
        "mblocks_3d",
        12,
        spacing_m=0.05,
        prefix="block",
        origin=(0.0, 0.0, 0.025),
    )

    class StubPhysicalScenario:
        def __init__(self, session: RuntimeSession) -> None:
            self.session = session

        @property
        def status(self) -> ReconfigurationStatus:
            return ReconfigurationStatus(
                phase=ReconfigurationPhase.HOLDING_INITIAL,
                time_s=self.session.world.time_s,
                plan_id="mblocks_physical_twelve_module_line",
                plan_name="M-Blocks Twelve-Module Physical Structure-to-Line",
                action_index=None,
                action_count=11,
                detail="Physical scenario stub",
            )

        def step(self) -> tuple[Event, ...]:
            return ()

    def fake_builder(session: RuntimeSession) -> StubPhysicalScenario:
        captured["builder_session"] = session
        return StubPhysicalScenario(session)

    def fake_load(
        config: RuntimeInspectorConfig,
    ) -> tuple[SceneSpec, Any]:
        captured["loader_config"] = config
        return scene, fake_builder

    def fake_create_session(
        loaded: LoadedRobotPack,
        requested_scene: SceneSpec,
        _config: RuntimeInspectorConfig,
    ) -> RuntimeSession:
        captured["scene"] = requested_scene
        return RuntimeSession.create(loaded, requested_scene, MockBackendAdapter())

    monkeypatch.setattr(
        runner_module,
        "_load_mblocks_physical_twelve_module_example",
        fake_load,
    )
    monkeypatch.setattr(runner_module, "_create_session", fake_create_session)
    statuses: list[str] = []
    config = RuntimeInspectorConfig(
        pack_path=_PACK_PATH,
        demo=RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE,
        backend="mujoco",
        gravity=True,
        ground=True,
        height_m=0.025,
        duration_s=20.0,
        dt_s=0.00025,
    )
    runner = RuntimeInspectorRunner.create(config, status_callback=statuses.append)
    try:
        assert captured["loader_config"] is config
        assert captured["scene"] is scene
        assert captured["builder_session"] is runner.session
        assert len(runner.session.world.modules) == 12
        assert runner.recipe.id == "mblocks_physics_lattice"
        assert runner.fixed_connector is None
        assert runner.moving_connector is None
        assert runner.scenario.status.plan_id == "mblocks_physical_twelve_module_line"
        assert statuses[-1] == (
            "Running M-Blocks Twelve-Module Physical Structure-to-Line with 12 modules"
        )
    finally:
        runner.shutdown()


def test_physical_twelve_module_session_reserves_route_sized_constraint_pools(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import modsim.runtime.inspector_runner as runner_module

    captured: dict[str, object] = {}

    def fake_create_backend(name: str, **options: object) -> MockBackendAdapter:
        captured["name"] = name
        captured["options"] = options
        return MockBackendAdapter()

    monkeypatch.setattr(runner_module, "create_backend", fake_create_backend)
    loaded = RobotPackLoader().load(_PACK_PATH)
    scene = SceneSpec.grid("mblocks_3d", 12, spacing_m=0.05)
    config = RuntimeInspectorConfig(
        pack_path=_PACK_PATH,
        demo=RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE,
        backend="mujoco",
        gravity=True,
        ground=True,
        height_m=0.025,
        duration_s=20.0,
        dt_s=0.00025,
    )

    session = runner_module._create_session(  # pyright: ignore[reportPrivateUsage]
        loaded,
        scene,
        config,
    )
    try:
        assert captured == {
            "name": "mujoco",
            "options": {
                "gravity": (0.0, 0.0, -9.81),
                "ground": True,
                "timestep_s": 0.00025,
                "weld_pool_size": 12,
                "hinge_pool_size": 2,
            },
        }
    finally:
        session.shutdown()


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"fixed_connector": "pos_x", "moving_connector": "neg_x"}, "connector actions"),
        ({"undock_at_s": 1.0}, "own releases"),
        ({"connector_gap_m": 0.01}, "connector_gap_m unspecified"),
        ({"retract_m_s": 0.01}, "retract_m_s unspecified"),
        ({"orientation_rad": 0.1}, "orientation_rad must be zero"),
        ({"gravity": True}, "gravity and ground disabled"),
        ({"ground": True}, "gravity and ground disabled"),
    ),
)
def test_runner_rejects_non_kinematic_mblocks_demo_options(
    overrides: dict[str, Any],
    message: str,
) -> None:
    base = RuntimeInspectorConfig(
        pack_path=_PACK_PATH,
        demo=RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
        backend="mock",
    )

    with pytest.raises(RuntimeInspectorSetupError, match=message):
        validate_runtime_inspector_config(replace(base, **overrides))


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"backend": "mock"}, "requires the MuJoCo backend"),
        ({"gravity": False}, "requires gravity and the ground plane"),
        ({"ground": False}, "requires gravity and the ground plane"),
        ({"height_m": 0.024}, "requires height_m >= 0.025"),
        (
            {"dt_s": MAX_MOMENTUM_TIMESTEP_S * 2.0},
            "requires dt_s <= 0.0005",
        ),
        (
            {"fixed_connector": "pos_z", "moving_connector": "neg_z"},
            "connector transitions",
        ),
        ({"undock_at_s": 1.0}, "controls its own releases"),
        ({"connector_gap_m": 0.01}, "connector_gap_m unspecified"),
        ({"retract_m_s": 0.01}, "retract_m_s unspecified"),
        ({"orientation_rad": 0.1}, "orientation_rad must be zero"),
    ),
)
def test_runner_rejects_unsupported_momentum_pivot_options(
    overrides: dict[str, Any],
    message: str,
) -> None:
    base = RuntimeInspectorConfig(
        pack_path=_PACK_PATH,
        demo=RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT,
        backend="mujoco",
        gravity=True,
        ground=True,
        height_m=0.025,
        dt_s=0.00025,
    )

    with pytest.raises(RuntimeInspectorSetupError, match=message):
        validate_runtime_inspector_config(replace(base, **overrides))


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"backend": "mock"}, "requires the MuJoCo backend"),
        ({"gravity": False}, "requires gravity and the ground plane"),
        ({"ground": False}, "requires gravity and the ground plane"),
        ({"height_m": 0.024}, "requires height_m >= 0.025"),
        (
            {"dt_s": MAX_MOMENTUM_TIMESTEP_S * 2.0},
            "requires dt_s <= 0.0005",
        ),
        (
            {"fixed_connector": "pos_z", "moving_connector": "neg_z"},
            "connector transitions",
        ),
        ({"undock_at_s": 1.0}, "controls its own releases"),
        ({"connector_gap_m": 0.01}, "connector_gap_m unspecified"),
        ({"retract_m_s": 0.01}, "retract_m_s unspecified"),
        ({"orientation_rad": 0.1}, "orientation_rad must be zero"),
    ),
)
def test_runner_rejects_unsupported_physical_twelve_module_options(
    overrides: dict[str, Any],
    message: str,
) -> None:
    base = RuntimeInspectorConfig(
        pack_path=_PACK_PATH,
        demo=RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE,
        backend="mujoco",
        gravity=True,
        ground=True,
        height_m=0.025,
        dt_s=0.00025,
    )

    with pytest.raises(RuntimeInspectorSetupError, match=message):
        validate_runtime_inspector_config(replace(base, **overrides))


@pytest.mark.parametrize(
    "demo",
    (RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT, RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE),
)
def test_runner_rejects_physics_for_kinematic_mblocks_demos(demo: RuntimeDemo) -> None:
    with pytest.raises(RuntimeInspectorSetupError, match="explicitly kinematic"):
        validate_runtime_inspector_config(
            RuntimeInspectorConfig(
                pack_path=_PACK_PATH,
                demo=demo,
                backend="mock",
                gravity=True,
            )
        )


def test_cli_supplies_safe_mblocks_kinematic_defaults(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}
    runtime_app = ModuleType("modsim_studio.runtime_app")

    def fake_main(config: object) -> int:
        captured["config"] = config
        return 0

    runtime_app.main = fake_main  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "modsim_studio.runtime_app", runtime_app)

    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT.value,
            "--backend",
            "mock",
            "--no-viewer",
        ],
    )

    assert result.exit_code == 0, result.output
    config = captured["config"]
    assert isinstance(config, RuntimeInspectorConfig)
    assert config.demo is RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT
    assert config.duration_s == pytest.approx(8.0)
    assert config.connector_gap_m is None
    assert config.retract_m_s is None
    assert not config.gravity
    assert not config.ground
    assert config.height_m == 0.0


def test_cli_supplies_safe_twelve_module_line_defaults(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}
    runtime_app = ModuleType("modsim_studio.runtime_app")

    def fake_main(config: object) -> int:
        captured["config"] = config
        return 0

    runtime_app.main = fake_main  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "modsim_studio.runtime_app", runtime_app)

    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE.value,
            "--backend",
            "mock",
            "--no-viewer",
        ],
    )

    assert result.exit_code == 0, result.output
    config = captured["config"]
    assert isinstance(config, RuntimeInspectorConfig)
    assert config.demo is RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE
    assert config.duration_s == pytest.approx(24.0)
    assert config.dt_s == pytest.approx(0.002)
    assert config.connector_gap_m is None
    assert config.retract_m_s is None
    assert not config.gravity
    assert not config.ground
    assert config.height_m == 0.0


def test_cli_supplies_safe_mblocks_momentum_pivot_defaults(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}
    runtime_app = ModuleType("modsim_studio.runtime_app")

    def fake_main(config: object) -> int:
        captured["config"] = config
        return 0

    runtime_app.main = fake_main  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "modsim_studio.runtime_app", runtime_app)

    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT.value,
            "--no-viewer",
        ],
    )

    assert result.exit_code == 0, result.output
    config = captured["config"]
    assert isinstance(config, RuntimeInspectorConfig)
    assert config.demo is RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT
    assert config.backend == "mujoco"
    assert config.duration_s == pytest.approx(5.0)
    assert config.dt_s == pytest.approx(0.00025)
    assert config.connector_gap_m is None
    assert config.retract_m_s is None
    assert config.gravity
    assert config.ground
    assert config.height_m == pytest.approx(0.025)


def test_cli_supplies_safe_physical_twelve_module_line_defaults(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}
    runtime_app = ModuleType("modsim_studio.runtime_app")

    def fake_main(config: object) -> int:
        captured["config"] = config
        return 0

    runtime_app.main = fake_main  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "modsim_studio.runtime_app", runtime_app)

    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE.value,
            "--no-viewer",
        ],
    )

    assert result.exit_code == 0, result.output
    config = captured["config"]
    assert isinstance(config, RuntimeInspectorConfig)
    assert config.demo is RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE
    assert config.backend == "mujoco"
    assert config.duration_s == pytest.approx(12.0)
    assert config.dt_s == pytest.approx(0.0005)
    assert config.connector_gap_m is None
    assert config.retract_m_s is None
    assert config.gravity
    assert config.ground
    assert config.height_m == pytest.approx(0.025)


@pytest.mark.parametrize(
    ("extra_args", "message"),
    (
        (("--connector-gap", "0.01"), "exact edge-pivot routes"),
        (("--ground",), "explicitly kinematic"),
    ),
)
def test_cli_rejects_non_kinematic_mblocks_demo_options(
    extra_args: tuple[str, ...],
    message: str,
) -> None:
    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT.value,
            "--backend",
            "mock",
            *extra_args,
        ],
    )

    assert result.exit_code == 2
    assert message in result.output


@pytest.mark.parametrize(
    ("extra_args", "message"),
    (
        (("--backend", "mock"), "requires --backend mujoco"),
        (("--no-ground",), "requires gravity and ground"),
        (("--height", "0.024"), "requires --height >= 0.025"),
        (("--dt", "0.001"), "requires --dt <= 0.0005"),
        (("--undock-at", "1"), "controls its own releases"),
        (("--connector-gap", "0.01"), "do not supply --connector-gap"),
        (("--retract", "0.01"), "do not supply --retract"),
        (("--orientation", "0.1"), "defines its cube orientation"),
        (
            ("--fixed-connector", "pos_z", "--moving-connector", "neg_z"),
            "connector transitions",
        ),
    ),
)
def test_cli_rejects_unsupported_mblocks_momentum_pivot_options(
    extra_args: tuple[str, ...],
    message: str,
) -> None:
    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT.value,
            "--no-viewer",
            *extra_args,
        ],
    )

    assert result.exit_code == 2
    assert message in result.output


@pytest.mark.parametrize(
    ("extra_args", "message"),
    (
        (("--backend", "mock"), "requires --backend mujoco"),
        (("--no-ground",), "requires gravity and ground"),
        (("--height", "0.024"), "requires --height >= 0.025"),
        (("--dt", "0.001"), "requires --dt <= 0.0005"),
        (("--undock-at", "1"), "controls its own releases"),
        (("--connector-gap", "0.01"), "do not supply --connector-gap"),
        (("--retract", "0.01"), "do not supply --retract"),
        (("--orientation", "0.1"), "defines its cube orientations"),
        (
            ("--fixed-connector", "pos_z", "--moving-connector", "neg_z"),
            "connector transitions",
        ),
    ),
)
def test_cli_rejects_unsupported_physical_twelve_module_options(
    extra_args: tuple[str, ...],
    message: str,
) -> None:
    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE.value,
            "--no-viewer",
            *extra_args,
        ],
    )

    assert result.exit_code == 2
    assert message in result.output


@pytest.mark.parametrize(
    ("extra_args", "message"),
    (
        (("--undock-at", "1"), "defines its own releases"),
        (("--orientation", "0.1"), "paper's nominal orientations"),
        (("--gravity",), "requires --no-gravity"),
    ),
)
def test_cli_retains_scripted_smores_validation_path(
    extra_args: tuple[str, ...],
    message: str,
) -> None:
    result = CliRunner().invoke(
        app,
        [
            "runtime",
            str(_PACK_PATH),
            "--demo",
            RuntimeDemo.SMORES_DRIVER_TO_SNAKE.value,
            "--backend",
            "mock",
            *extra_args,
        ],
    )

    assert result.exit_code == 2
    assert message in result.output
