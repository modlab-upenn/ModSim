"""Qt-free execution boundary for one Runtime Inspector demonstration.

The runner is the sole owner of its mutable :class:`RuntimeSession`, scripted
docking scenario, model-view factory, and event cursor.  Presentation clients
may execute it in a Qt worker thread or in a dedicated simulator-viewer
process, but they receive only immutable :class:`RuntimeInspectorFrame`
values.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass, field, replace
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import Protocol, cast

from modsim.backends.registry import create_backend
from modsim.connectors.compatibility import evaluate_compatibility
from modsim.core.events import Event
from modsim.core.ids import ModuleInstanceId, connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, quat_from_rpy
from modsim.core.validation import (
    require_finite,
    require_finite_nonnegative,
    require_finite_positive,
)
from modsim.model_views import ModelViewFactory
from modsim.planning.smores import (
    driver_to_snake_goal,
    mobile_manipulator_goal,
    paper_initial_poses,
)
from modsim.robot_packs import (
    LoadedRobotPack,
    ModelViewMode,
    ModelViewSpec,
    RobotPack,
    RobotPackLoader,
    RobotPackValidator,
    ValidationProfile,
)
from modsim.runtime.demos import RuntimeDemo
from modsim.runtime.inspection import RuntimeInspectorFrame, build_runtime_inspector_frame
from modsim.runtime.kinematic_pivot import (
    KinematicPivotConfig,
    KinematicPivotRoute,
    KinematicPivotScenario,
)
from modsim.runtime.momentum_pivot import (
    MAX_MOMENTUM_TIMESTEP_S,
    MomentumPivotConfig,
    MomentumPivotScenario,
)
from modsim.runtime.online_planning import OnlineAssemblyScenario
from modsim.runtime.physical_reconfiguration import (
    DifferentialDriveReconfigurationConfig,
    DifferentialDriveReconfigurationScenario,
)
from modsim.runtime.physics_docking import (
    MAX_PHYSICAL_TIMESTEP_S,
    DifferentialDriveDockingConfig,
    DifferentialDriveDockingScenario,
)
from modsim.runtime.reconfiguration import (
    ReconfigurationPlan,
    ReconfigurationStatus,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
    connector_pair_plan,
)
from modsim.runtime.session import RuntimeSession

_LOGGER = logging.getLogger("modsim.runtime_inspector")
_INITIAL_SCENE_SPACING_M = 0.2
_RECONFIGURATION_SCENE_SPACING_M = 0.12
_DEFAULT_CONNECTOR_GAP_M = 0.02
_MBLOCKS_PHYSICS_LATTICE_VIEW_ID = "mblocks_physics_lattice"
_SMORES_EXAMPLE_SCENARIO = (
    Path(__file__).resolve().parents[3] / "examples" / "scenarios" / "smores_driver_to_snake.py"
)
_SMORES_PHYSICS_SCENARIO = (
    Path(__file__).resolve().parents[3]
    / "examples"
    / "scenarios"
    / "smores_ep_diff_drive_dock_undock.py"
)
_SMORES_PHYSICAL_RECONFIGURATION_SCENARIO = (
    Path(__file__).resolve().parents[3]
    / "examples"
    / "scenarios"
    / "smores_ep_physical_driver_to_snake.py"
)
_MBLOCKS_KINEMATIC_PIVOT_SCENARIO = (
    Path(__file__).resolve().parents[3] / "examples" / "scenarios" / "mblocks_five_module_pivot.py"
)
_MBLOCKS_MOMENTUM_PIVOT_SCENARIO = (
    Path(__file__).resolve().parents[3]
    / "examples"
    / "scenarios"
    / "mblocks_two_module_momentum_pivot.py"
)
_MBLOCKS_TWELVE_MODULE_LINE_SCENARIO = (
    Path(__file__).resolve().parents[3] / "examples" / "scenarios" / "mblocks_twelve_module_line.py"
)
_MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE_SCENARIO = (
    Path(__file__).resolve().parents[3]
    / "examples"
    / "scenarios"
    / "mblocks_twelve_module_physics.py"
)
_MBLOCKS_TWELVE_MODULE_STAIRCASE_SCENARIO = (
    Path(__file__).resolve().parents[3]
    / "examples"
    / "scenarios"
    / "mblocks_twelve_module_staircase.py"
)

StatusCallback = Callable[[str], None]


class RuntimeInspectorSetupError(ValueError):
    """Raised when an inspector configuration cannot produce a runtime."""


class RuntimeScenario(Protocol):
    """Minimal scenario surface consumed by the execution owner."""

    @property
    def status(self) -> ReconfigurationStatus:
        """Return immutable presentation state."""
        ...

    def step(self) -> tuple[Event, ...]:
        """Advance one configured simulation step."""
        ...


class _TimedScenarioBuilder(Protocol):
    def __call__(self, session: RuntimeSession, *, dt_s: float) -> object:
        """Build one scenario using the runner's fixed timestep."""
        ...


RuntimeScenarioBuilder = Callable[[RuntimeSession], RuntimeScenario]


@dataclass(frozen=True, slots=True)
class RuntimeInspectorConfig:
    """Complete launch request for one Runtime Inspector demonstration.

    ``viewer_enabled`` selects presentation/execution topology for a client;
    the Qt-free runner deliberately ignores it. Connector names are
    module-local Robot Pack IDs. When both are omitted, the first declared
    connector whose type is self-compatible is selected.
    """

    pack_path: Path
    demo: RuntimeDemo = RuntimeDemo.DOCK
    module_type: str | None = None
    backend: str = "mujoco"
    fixed_connector: str | None = None
    moving_connector: str | None = None
    connector_gap_m: float | None = None
    orientation_rad: float = 0.0
    approach_m_s: float = 0.03
    retract_m_s: float | None = None
    duration_s: float = 4.0
    dt_s: float = 0.002
    undock_at_s: float | None = None
    gravity: bool = False
    ground: bool = False
    height_m: float = 0.0
    view_id: str | None = None
    publish_hz: float = 20.0
    viewer_enabled: bool = False
    real_time_factor: float = 1.0


@dataclass(slots=True)
class RuntimeInspectorRunner:
    """Authoritative mutable execution state behind immutable inspector frames."""

    config: RuntimeInspectorConfig
    session: RuntimeSession
    scenario: RuntimeScenario
    recipe: ModelViewSpec
    module_type: str
    fixed_connector: str | None
    moving_connector: str | None
    _factory: ModelViewFactory = field(default_factory=ModelViewFactory)
    _event_cursor: int = 0
    _closed: bool = False

    @classmethod
    def create(
        cls,
        config: RuntimeInspectorConfig,
        *,
        status_callback: StatusCallback | None = None,
    ) -> RuntimeInspectorRunner:
        """Load, validate, stage, and start the configured demonstration."""
        report_status = status_callback if status_callback is not None else _ignore_status
        report_status("Loading and validating Robot Pack…")
        validate_runtime_inspector_config(config)
        loaded = RobotPackLoader().load(config.pack_path)
        validation = RobotPackValidator().validate(
            loaded,
            profile=ValidationProfile.SIMULATION,
        )
        if not validation.valid:
            details = "; ".join(
                f"{issue.location}: {issue.message}" for issue in validation.errors[:4]
            )
            remainder = len(validation.errors) - 4
            if remainder > 0:
                details += f"; and {remainder} more error(s)"
            raise RuntimeInspectorSetupError("Robot Pack is not simulation-ready. " + details)

        pack = loaded.pack
        module_type = resolve_runtime_module_type(pack, config.module_type)
        demo = _resolve_demo(config.demo)
        requested_view = config.view_id
        if (
            demo
            in {
                RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT,
                RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE,
                RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
                RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
            }
            and requested_view is None
        ):
            requested_view = _MBLOCKS_PHYSICS_LATTICE_VIEW_ID
        recipe = resolve_runtime_recipe(pack, requested_view)
        release_after_s: float | None = None
        plan: ReconfigurationPlan | None = None
        pivot_routes: tuple[KinematicPivotRoute, ...] | None = None
        momentum_config: MomentumPivotConfig | None = None
        physical_mblocks_builder: RuntimeScenarioBuilder | None = None
        if demo in {
            RuntimeDemo.SMORES_DRIVER_TO_SNAKE,
            RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
            RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE,
        }:
            plan = _load_smores_example_plan()
            fixed_local = moving_local = None
            scene = SceneSpec.of(
                ModulePlacement(
                    instance_id=module_id,
                    module_type_id=module_type,
                    pose=Transform.from_translation(
                        (
                            index * _RECONFIGURATION_SCENE_SPACING_M,
                            0.0,
                            config.height_m,
                        )
                    ),
                )
                for index, module_id in enumerate(plan.module_ids)
            )
        elif demo is RuntimeDemo.SMORES_ONLINE_ASSEMBLY:
            fixed_local = moving_local = None
            scene = SceneSpec.of(
                ModulePlacement(
                    instance_id=ModuleInstanceId(module),
                    module_type_id=module_type,
                    pose=Transform(
                        translation=(pose.x, pose.y, config.height_m),
                        rotation=quat_from_rpy((0.0, 0.0, pose.yaw)),
                    ),
                )
                for module, pose in paper_initial_poses().items()
            )
        elif demo in {
            RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
            RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE,
        }:
            if demo is RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT:
                plan, pivot_routes = _load_mblocks_kinematic_pivot_example()
            else:
                plan, pivot_routes = _load_mblocks_twelve_module_line_example()
            fixed_local = moving_local = None
            scene = SceneSpec.of(
                ModulePlacement(
                    instance_id=module_id,
                    module_type_id=module_type,
                    pose=Transform.from_translation(
                        (
                            index * _RECONFIGURATION_SCENE_SPACING_M,
                            0.0,
                            config.height_m,
                        )
                    ),
                )
                for index, module_id in enumerate(plan.module_ids)
            )
        elif demo is RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT:
            fixed_local = moving_local = None
            scene, momentum_config = _load_mblocks_momentum_pivot_example(config)
            if any(placement.module_type_id != module_type for placement in scene.placements):
                raise RuntimeInspectorSetupError(
                    "M-Blocks momentum-pivot example does not match the selected module type "
                    f"'{module_type}'"
                )
        elif demo is RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE:
            fixed_local = moving_local = None
            scene, physical_mblocks_builder = _load_mblocks_physical_twelve_module_example(config)
            if any(placement.module_type_id != module_type for placement in scene.placements):
                raise RuntimeInspectorSetupError(
                    "M-Blocks physical twelve-module example does not match the selected "
                    f"module type '{module_type}'"
                )
        elif demo in {
            RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
            RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
        }:
            fixed_local = moving_local = None
            scene, physical_mblocks_builder = _load_mblocks_staircase_example(
                config,
                physical=demo is RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
            )
            if any(placement.module_type_id != module_type for placement in scene.placements):
                raise RuntimeInspectorSetupError(
                    "M-Blocks staircase example does not match the selected module type "
                    f"'{module_type}'"
                )
        elif demo is RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK:
            fixed_local, moving_local = resolve_runtime_connector_pair(
                pack,
                module_type,
                "bottom",
                "pan",
            )
            scene = SceneSpec.grid(
                module_type,
                2,
                spacing_m=_INITIAL_SCENE_SPACING_M,
                origin=(0.0, 0.0, config.height_m),
            )
        else:
            fixed_local, moving_local = resolve_runtime_connector_pair(
                pack,
                module_type,
                config.fixed_connector,
                config.moving_connector,
            )
            scene = SceneSpec.grid(
                module_type,
                2,
                spacing_m=_INITIAL_SCENE_SPACING_M,
                origin=(0.0, 0.0, config.height_m),
            )
            fixed_id = connector_instance_id(scene.instance_ids[0], fixed_local)
            moving_id = connector_instance_id(scene.instance_ids[1], moving_local)
            release_after_s = config.undock_at_s
            include_undock = demo is RuntimeDemo.DOCK_UNDOCK or release_after_s is not None
            if demo is RuntimeDemo.DOCK_UNDOCK and release_after_s is None:
                release_after_s = config.duration_s * 0.55
            plan = connector_pair_plan(
                fixed_id,
                moving_id,
                include_undock=include_undock,
            )

        report_status(f"Starting {config.backend} backend…")
        session = _create_session(loaded, scene, config)
        try:
            if demo in {
                RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
                RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE,
            }:
                if plan is None or pivot_routes is None:  # pragma: no cover - branch invariant
                    raise AssertionError("kinematic pivot runtime demo requires a plan and routes")
                scenario: RuntimeScenario = KinematicPivotScenario.create(
                    session,
                    plan,
                    pivot_routes,
                    KinematicPivotConfig(dt_s=config.dt_s),
                )
            elif demo is RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT:
                if momentum_config is None:  # pragma: no cover - branch invariant
                    raise AssertionError("momentum-pivot runtime demo requires a configuration")
                scenario = MomentumPivotScenario.create(session, momentum_config)
            elif demo is RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE:
                if physical_mblocks_builder is None:  # pragma: no cover - branch invariant
                    raise AssertionError(
                        "physical twelve-module runtime demo requires a scenario builder"
                    )
                scenario = physical_mblocks_builder(session)
            elif demo in {
                RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
                RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
            }:
                if physical_mblocks_builder is None:  # pragma: no cover - branch invariant
                    raise AssertionError("staircase runtime demo requires a scenario builder")
                scenario = physical_mblocks_builder(session)
            elif demo is RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK:
                physical_config = _load_smores_physics_config(
                    scene.instance_ids[0],
                    scene.instance_ids[1],
                    config,
                )
                scenario = DifferentialDriveDockingScenario.create(
                    session,
                    physical_config,
                )
            elif demo in {
                RuntimeDemo.SMORES_ONLINE_ASSEMBLY,
                RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE,
            }:
                reconfiguration_config = _load_smores_physical_reconfiguration_config(config)
                scenario = OnlineAssemblyScenario.create(
                    session,
                    mobile_manipulator_goal()
                    if demo is RuntimeDemo.SMORES_ONLINE_ASSEMBLY
                    else driver_to_snake_goal(),
                    reconfiguration_config,
                    initial=plan,
                )
            elif demo is RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE:
                if plan is None:  # pragma: no cover - branch invariant
                    raise AssertionError("physical runtime demo requires a plan")
                reconfiguration_config = _load_smores_physical_reconfiguration_config(config)
                scenario = DifferentialDriveReconfigurationScenario.create(
                    session,
                    plan,
                    reconfiguration_config,
                )
            else:
                if plan is None:  # pragma: no cover - branch invariant
                    raise AssertionError("scripted runtime demo requires a plan")
                scenario = ScriptedReconfigurationScenario.create(
                    session,
                    plan,
                    ScriptedReconfigurationConfig(
                        dt_s=config.dt_s,
                        gap_m=_resolved_connector_gap_m(config),
                        orientation_rad=config.orientation_rad,
                        approach_speed_m_s=config.approach_m_s,
                        initial_hold_s=(1.0 if demo is RuntimeDemo.SMORES_DRIVER_TO_SNAKE else 0.0),
                        retract_speed_m_s=(
                            config.approach_m_s
                            if config.retract_m_s is None
                            else config.retract_m_s
                        ),
                        release_after_s=(
                            None if demo is RuntimeDemo.SMORES_DRIVER_TO_SNAKE else release_after_s
                        ),
                    ),
                )
            if demo in {
                RuntimeDemo.SMORES_DRIVER_TO_SNAKE,
                RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
                RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
                RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE,
            }:
                assert plan is not None
                if demo is RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE:
                    report_status(
                        "Running smores_physical_driver_to_snake: "
                        f"{plan.name} with {len(plan.module_ids)} modules"
                    )
                else:
                    report_status(f"Running {plan.name} with {len(plan.module_ids)} modules")
            elif demo is RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT:
                assert momentum_config is not None
                report_status(f"Running {momentum_config.plan_name} with 2 modules")
            elif demo in {
                RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE,
                RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
                RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
            }:
                report_status(
                    f"Running {scenario.status.plan_name} with {len(scene.instance_ids)} modules"
                )
            elif isinstance(scenario, OnlineAssemblyScenario):
                report_status(f"Running {demo.value} with generated routes and online scheduling")
            else:
                assert fixed_local is not None and moving_local is not None
                report_status(f"Running {module_type}: {fixed_local} ↔ {moving_local}")
            return cls(
                config=config,
                session=session,
                scenario=scenario,
                recipe=recipe,
                module_type=module_type,
                fixed_connector=fixed_local,
                moving_connector=moving_local,
            )
        except Exception:
            try:
                session.shutdown()
            except Exception:
                _LOGGER.exception("Backend cleanup after failed inspector setup also failed")
            raise

    @property
    def step_count(self) -> int:
        """Return the configured number of fixed-duration scenario steps."""
        return math.ceil(self.config.duration_s / self.config.dt_s)

    @property
    def execution_finished(self) -> bool:
        """Whether the scenario has reported a terminal result to the interactive host."""
        return self.scenario.status.phase.value in {"complete", "failed"}

    @property
    def event_cursor(self) -> int:
        """Return the next canonical event sequence that a frame will publish."""
        return self._event_cursor

    @property
    def closed(self) -> bool:
        """Return whether backend shutdown has been requested."""
        return self._closed

    def step(self) -> tuple[Event, ...]:
        """Advance one configured physics/scenario step."""
        self._require_open()
        return self.scenario.step()

    def frame(self) -> RuntimeInspectorFrame:
        """Build one immutable frame and atomically advance its event cursor."""
        self._require_open()
        frame = build_runtime_inspector_frame(
            self.session,
            self.recipe,
            self._factory,
            event_cursor=self._event_cursor,
            scenario_status=self.scenario.status,
            planning=(
                self.scenario.planning_snapshot
                if isinstance(self.scenario, OnlineAssemblyScenario)
                else None
            ),
        )
        self._event_cursor = frame.next_event_sequence
        return frame

    def shutdown(self) -> None:
        """Release the backend exactly once."""
        if self._closed:
            return
        self._closed = True
        self.session.shutdown()

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("Runtime Inspector runner is shut down")


def validate_runtime_inspector_config(config: RuntimeInspectorConfig) -> None:
    """Validate execution fields without interpreting presentation mode."""
    if not config.backend.strip():
        raise RuntimeInspectorSetupError("backend must not be empty")
    if (config.fixed_connector is None) != (config.moving_connector is None):
        raise RuntimeInspectorSetupError(
            "fixed_connector and moving_connector must be supplied together"
        )
    demo = _resolve_demo(config.demo)
    if demo is RuntimeDemo.SMORES_DRIVER_TO_SNAKE:
        if config.fixed_connector is not None or config.moving_connector is not None:
            raise RuntimeInspectorSetupError(
                "smores_driver_to_snake defines its connector actions; do not supply "
                "fixed_connector or moving_connector"
            )
        if config.undock_at_s is not None:
            raise RuntimeInspectorSetupError(
                "smores_driver_to_snake defines its own releases; do not supply undock_at_s"
            )
        if config.orientation_rad != 0.0:
            raise RuntimeInspectorSetupError(
                "smores_driver_to_snake uses the paper's nominal orientations; "
                "orientation_rad must be zero"
            )
        if config.gravity or config.ground:
            raise RuntimeInspectorSetupError(
                "smores_driver_to_snake currently requires gravity and ground disabled; "
                "the scripted topology demo does not yet model supported locomotion"
            )
    elif demo in {
        RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
        RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE,
        RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
    }:
        demo_name = demo.value
        if config.fixed_connector is not None or config.moving_connector is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines its connector actions; do not supply fixed_connector "
                "or moving_connector"
            )
        if config.undock_at_s is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines its own releases; do not supply undock_at_s"
            )
        if config.connector_gap_m is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines exact edge-pivot routes; leave connector_gap_m unspecified"
            )
        if config.retract_m_s is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines exact edge-pivot routes; leave retract_m_s unspecified"
            )
        if config.orientation_rad != 0.0:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines its face orientations; orientation_rad must be zero"
            )
        if config.gravity or config.ground:
            raise RuntimeInspectorSetupError(
                f"{demo_name} is explicitly kinematic and requires gravity and ground disabled"
            )
    elif demo is RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT:
        if config.backend != "mujoco":
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot requires the MuJoCo backend's joint, hinge, "
                "contact, and gravity dynamics"
            )
        if config.fixed_connector is not None or config.moving_connector is not None:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot defines its face and edge connector transitions; "
                "do not supply fixed_connector or moving_connector"
            )
        if config.undock_at_s is not None:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot controls its own releases; do not supply undock_at_s"
            )
        if config.connector_gap_m is not None:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot defines its initial face-connected scene; leave "
                "connector_gap_m unspecified"
            )
        if config.retract_m_s is not None:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot is flywheel-driven; leave retract_m_s unspecified"
            )
        if config.orientation_rad != 0.0:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot defines its cube orientation; orientation_rad must be zero"
            )
        if not config.gravity or not config.ground:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot requires gravity and the ground plane; enable both options"
            )
        if config.height_m < 0.025:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot requires height_m >= 0.025 so the support cube "
                "starts on or above the ground"
            )
        if config.dt_s > MAX_MOMENTUM_TIMESTEP_S:
            raise RuntimeInspectorSetupError(
                "mblocks_momentum_pivot requires dt_s <= "
                f"{MAX_MOMENTUM_TIMESTEP_S:g} for the flywheel brake impulse"
            )
    elif demo in {
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE,
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
    }:
        demo_name = demo.value
        if config.backend != "mujoco":
            raise RuntimeInspectorSetupError(
                f"{demo_name} requires the MuJoCo backend's "
                "joint, hinge, contact, and gravity dynamics"
            )
        if config.fixed_connector is not None or config.moving_connector is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines its connector transitions; "
                "do not supply fixed_connector or moving_connector"
            )
        if config.undock_at_s is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} controls its own releases; do not supply undock_at_s"
            )
        if config.connector_gap_m is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines its initial scene; leave connector_gap_m unspecified"
            )
        if config.retract_m_s is not None:
            raise RuntimeInspectorSetupError(
                f"{demo_name} is flywheel-driven; leave retract_m_s unspecified"
            )
        if config.orientation_rad != 0.0:
            raise RuntimeInspectorSetupError(
                f"{demo_name} defines its cube orientations; orientation_rad must be zero"
            )
        if not config.gravity or not config.ground:
            raise RuntimeInspectorSetupError(
                f"{demo_name} requires gravity and the ground plane; enable both options"
            )
        if config.height_m < 0.025:
            raise RuntimeInspectorSetupError(
                f"{demo_name} requires height_m >= 0.025 so the "
                "lowest cube starts on or above the ground"
            )
        if config.dt_s > MAX_MOMENTUM_TIMESTEP_S:
            raise RuntimeInspectorSetupError(
                f"{demo_name} requires dt_s <= "
                f"{MAX_MOMENTUM_TIMESTEP_S:g} for its flywheel brake impulses"
            )
    elif demo is RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK:
        if config.backend != "mujoco":
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock requires the MuJoCo backend's "
                "joint and contact dynamics"
            )
        if config.fixed_connector is not None or config.moving_connector is not None:
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock uses fixed bottom and moving pan; "
                "do not supply fixed_connector or moving_connector"
            )
        if config.undock_at_s is not None:
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock performs its own post-contact release; "
                "do not supply undock_at_s"
            )
        if config.orientation_rad != 0.0:
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock keeps both modules upright; "
                "orientation_rad must be zero"
            )
        if not config.gravity or not config.ground:
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock requires gravity and the ground plane; "
                "enable both options"
            )
        if config.height_m < 0.04:
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock requires height_m >= 0.04 so the tires "
                "start above the ground"
            )
        if config.dt_s > MAX_PHYSICAL_TIMESTEP_S:
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock requires dt_s <= "
                f"{MAX_PHYSICAL_TIMESTEP_S:g} for the tuned contact/controller model"
            )
        if config.retract_m_s == 0.0:
            raise RuntimeInspectorSetupError(
                "smores_diff_drive_dock_undock requires a positive retract_m_s"
            )
    elif demo in {
        RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
        RuntimeDemo.SMORES_ONLINE_ASSEMBLY,
        RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE,
    }:
        if config.backend != "mujoco":
            raise RuntimeInspectorSetupError(
                f"{demo.value} requires the MuJoCo backend's joint and contact dynamics"
            )
        if config.fixed_connector is not None or config.moving_connector is not None:
            raise RuntimeInspectorSetupError(
                f"{demo.value} defines its connector actions; do not "
                "supply fixed_connector or moving_connector"
            )
        if config.undock_at_s is not None:
            raise RuntimeInspectorSetupError(
                f"{demo.value} defines its own releases; do not supply undock_at_s"
            )
        if config.connector_gap_m is not None:
            raise RuntimeInspectorSetupError(
                f"{demo.value} defines its initial seven-module "
                "staging; leave connector_gap_m unspecified"
            )
        if config.retract_m_s is not None:
            raise RuntimeInspectorSetupError(
                f"{demo.value} defines its wheel-driven routes; leave retract_m_s unspecified"
            )
        if config.orientation_rad != 0.0:
            raise RuntimeInspectorSetupError(
                f"{demo.value} keeps the staged modules upright; orientation_rad must be zero"
            )
        if not config.gravity or not config.ground:
            raise RuntimeInspectorSetupError(
                f"{demo.value} requires gravity and the ground plane; enable both options"
            )
        if config.height_m < 0.04:
            raise RuntimeInspectorSetupError(
                f"{demo.value} requires height_m >= 0.04 so the tires start above the ground"
            )
        if config.dt_s > MAX_PHYSICAL_TIMESTEP_S:
            raise RuntimeInspectorSetupError(
                f"{demo.value} requires dt_s <= "
                f"{MAX_PHYSICAL_TIMESTEP_S:g} for the tuned contact/controller model"
            )
    try:
        if config.connector_gap_m is not None:
            require_finite_nonnegative(config.connector_gap_m, "connector_gap_m")
        require_finite(config.orientation_rad, "orientation_rad")
        require_finite_positive(config.approach_m_s, "approach_m_s")
        require_finite_positive(config.duration_s, "duration_s")
        require_finite_positive(config.dt_s, "dt_s")
        require_finite(config.height_m, "height_m")
        require_finite_positive(config.publish_hz, "publish_hz")
        require_finite_positive(config.real_time_factor, "real_time_factor")
        if config.retract_m_s is not None:
            require_finite_nonnegative(config.retract_m_s, "retract_m_s")
        if config.undock_at_s is not None:
            require_finite_nonnegative(config.undock_at_s, "undock_at_s")
    except ValueError as error:
        raise RuntimeInspectorSetupError(str(error)) from error


def resolve_runtime_module_type(pack: RobotPack, requested: str | None) -> str:
    """Resolve the module type instantiated by the selected demonstration."""
    module_types = pack.hardware_catalog.module_types
    if requested is not None:
        if requested not in module_types:
            available = ", ".join(module_types) or "none"
            raise RuntimeInspectorSetupError(
                f"unknown module type '{requested}'; available: {available}"
            )
        return requested
    if len(module_types) == 1:
        return next(iter(module_types))
    available = ", ".join(module_types) or "none"
    raise RuntimeInspectorSetupError(
        "module_type is required when a Robot Pack defines multiple module types; "
        f"available: {available}"
    )


def resolve_runtime_connector_pair(
    pack: RobotPack,
    module_type_id: str,
    fixed: str | None,
    moving: str | None,
) -> tuple[str, str]:
    """Resolve explicit local connectors or the first self-compatible one."""
    module = pack.hardware_catalog.module_types[module_type_id]
    connector_ids = tuple(connector.id for connector in module.connectors)
    if fixed is not None and moving is not None:
        missing = tuple(
            identifier for identifier in (fixed, moving) if identifier not in connector_ids
        )
        if missing:
            available = ", ".join(connector_ids) or "none"
            raise RuntimeInspectorSetupError(
                f"module type '{module_type_id}' has no connector(s) "
                f"{', '.join(missing)}; available: {available}"
            )
        return fixed, moving

    if not connector_ids:
        raise RuntimeInspectorSetupError(
            f"module type '{module_type_id}' has no connectors; add connectors in Studio"
        )
    connector_types = pack.hardware_catalog.connector_types
    for connector in module.connectors:
        connector_type = connector_types.get(connector.connector_type)
        if (
            connector_type is not None
            and evaluate_compatibility(connector_type, connector_type).compatible
        ):
            return connector.id, connector.id
    raise RuntimeInspectorSetupError(
        f"module type '{module_type_id}' has no connector whose type is self-compatible; "
        "specify both fixed_connector and moving_connector explicitly"
    )


def resolve_runtime_recipe(pack: RobotPack, requested: str | None) -> ModelViewSpec:
    """Resolve an explicit recipe or the pack's first default runtime recipe."""
    recipes = pack.manifest.model_views
    if requested is not None:
        recipe = next((candidate for candidate in recipes if candidate.id == requested), None)
        if recipe is None:
            available = ", ".join(candidate.id for candidate in recipes) or "none"
            raise RuntimeInspectorSetupError(
                f"unknown model-view recipe '{requested}'; available: {available}"
            )
        if ModelViewMode.RUNTIME not in recipe.modes:
            raise RuntimeInspectorSetupError(
                f"model-view recipe '{requested}' is not enabled for runtime use"
            )
        return recipe

    recipe = next(
        (
            candidate
            for candidate in recipes
            if candidate.default and ModelViewMode.RUNTIME in candidate.modes
        ),
        None,
    )
    if recipe is None:
        raise RuntimeInspectorSetupError(
            "Robot Pack has no default runtime model-view recipe; mark one as default "
            "in Studio or supply view_id"
        )
    return recipe


def runtime_frame_signature(frame: RuntimeInspectorFrame) -> object:
    """Return fields that determine whether a final frame adds information."""
    source = frame.view.source
    return (
        source.sample_sequence,
        source.topology_revision,
        source.docking_revision,
        source.event_revision,
        frame.next_event_sequence,
        frame.scenario,
        frame.planning,
    )


def _create_session(
    loaded: LoadedRobotPack,
    scene: SceneSpec,
    config: RuntimeInspectorConfig,
) -> RuntimeSession:
    if config.backend != "mujoco":
        if config.gravity or config.ground:
            raise RuntimeInspectorSetupError(
                "gravity and ground options are currently supported only by the MuJoCo backend"
            )
        adapter = create_backend(config.backend)
    else:
        backend_options: dict[str, object] = {
            "gravity": (0.0, 0.0, -9.81) if config.gravity else (0.0, 0.0, 0.0),
            "ground": config.ground,
            "timestep_s": config.dt_s,
        }
        demo = _resolve_demo(config.demo)
        if demo is RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT:
            backend_options.update(weld_pool_size=2, hinge_pool_size=2)
        elif demo is RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE:
            # The route has eleven simultaneous face bonds and one transient
            # hinge. Avoid deriving 84 inactive slots from all 168 directed
            # connector instances in this known demonstration scene.
            backend_options.update(weld_pool_size=12, hinge_pool_size=2)
        elif demo in {
            RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
            RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
        }:
            # The mat begins with 16 cyclic face bonds and finishes with 18;
            # only one edge hinge is active during a coordinated slab pivot.
            backend_options.update(weld_pool_size=24, hinge_pool_size=2)
        adapter = create_backend(
            config.backend,
            **backend_options,
        )
    try:
        return RuntimeSession.create(loaded, scene, adapter)
    except Exception:
        try:
            adapter.shutdown()
        except Exception:
            _LOGGER.exception("Backend cleanup after failed session creation also failed")
        raise


def _ignore_status(message: str) -> None:
    del message


def _load_smores_example_plan() -> ReconfigurationPlan:
    scenario_path = _SMORES_EXAMPLE_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "SMORES Driver-to-Snake example content was not found. This demonstration "
            "requires a ModSim source checkout containing "
            "'examples/scenarios/smores_driver_to_snake.py'. The Robot Pack may be "
            "copied or staged anywhere inside that checkout."
        )
    spec = spec_from_file_location("modsim_example_smores_driver_to_snake", scenario_path)
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"SMORES Driver-to-Snake example is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load SMORES scenario '{scenario_path}': {error}"
        ) from error
    build_plan = getattr(module, "build_plan", None)
    if not callable(build_plan):
        raise RuntimeInspectorSetupError(
            f"SMORES scenario '{scenario_path}' does not define build_plan()"
        )
    try:
        plan = build_plan()
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build SMORES scenario '{scenario_path}': {error}"
        ) from error
    if not isinstance(plan, ReconfigurationPlan):
        raise RuntimeInspectorSetupError(
            f"SMORES scenario '{scenario_path}' returned an invalid plan"
        )
    return plan


def _load_mblocks_kinematic_pivot_example() -> tuple[
    ReconfigurationPlan, tuple[KinematicPivotRoute, ...]
]:
    """Load the source-checkout five-module M-Blocks kinematic example."""
    scenario_path = _MBLOCKS_KINEMATIC_PIVOT_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "M-Blocks five-module pivot example content was not found. This demonstration "
            "requires a ModSim source checkout containing "
            "'examples/scenarios/mblocks_five_module_pivot.py'."
        )
    spec = spec_from_file_location("modsim_example_mblocks_five_module_pivot", scenario_path)
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"M-Blocks five-module pivot example is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load M-Blocks scenario '{scenario_path}': {error}"
        ) from error
    build_plan = getattr(module, "build_plan", None)
    build_routes = getattr(module, "build_routes", None)
    if not callable(build_plan) or not callable(build_routes):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' must define build_plan() and build_routes()"
        )
    try:
        plan = cast(Callable[[], object], build_plan)()
        route_result = cast(Callable[[], object], build_routes)()
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build M-Blocks scenario '{scenario_path}': {error}"
        ) from error
    if not isinstance(route_result, tuple):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' returned invalid routes"
        )
    route_values = cast(tuple[object, ...], route_result)
    if not isinstance(plan, ReconfigurationPlan) or not all(
        isinstance(route, KinematicPivotRoute) for route in route_values
    ):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' returned an invalid plan or routes"
        )
    return plan, cast(tuple[KinematicPivotRoute, ...], route_values)


def _load_mblocks_twelve_module_line_example() -> tuple[
    ReconfigurationPlan, tuple[KinematicPivotRoute, ...]
]:
    """Load the source-checkout twelve-module M-Blocks line benchmark."""
    scenario_path = _MBLOCKS_TWELVE_MODULE_LINE_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "M-Blocks twelve-module line example content was not found. This demonstration "
            "requires a ModSim source checkout containing "
            "'examples/scenarios/mblocks_twelve_module_line.py'."
        )
    spec = spec_from_file_location("modsim_example_mblocks_twelve_module_line", scenario_path)
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"M-Blocks twelve-module line example is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load M-Blocks twelve-module scenario '{scenario_path}': {error}"
        ) from error
    build_plan = getattr(module, "build_plan", None)
    build_routes = getattr(module, "build_routes", None)
    if not callable(build_plan) or not callable(build_routes):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' must define build_plan() and build_routes()"
        )
    try:
        plan = cast(Callable[[], object], build_plan)()
        route_result = cast(Callable[[], object], build_routes)()
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build M-Blocks twelve-module scenario '{scenario_path}': {error}"
        ) from error
    if not isinstance(route_result, tuple):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' returned invalid routes"
        )
    route_values = cast(tuple[object, ...], route_result)
    if not isinstance(plan, ReconfigurationPlan) or not all(
        isinstance(route, KinematicPivotRoute) for route in route_values
    ):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' returned an invalid plan or routes"
        )
    return plan, cast(tuple[KinematicPivotRoute, ...], route_values)


def _load_mblocks_momentum_pivot_example(
    config: RuntimeInspectorConfig,
) -> tuple[SceneSpec, MomentumPivotConfig]:
    """Load and parameterize the source-checkout physical M-Blocks pivot."""
    scenario_path = _MBLOCKS_MOMENTUM_PIVOT_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "M-Blocks momentum-pivot example content was not found. This demonstration "
            "requires a ModSim source checkout containing "
            "'examples/scenarios/mblocks_two_module_momentum_pivot.py'."
        )
    spec = spec_from_file_location("modsim_example_mblocks_momentum_pivot", scenario_path)
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"M-Blocks momentum-pivot example is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load M-Blocks momentum-pivot scenario '{scenario_path}': {error}"
        ) from error
    build_scene = getattr(module, "build_scene", None)
    build_config = getattr(module, "build_config", None)
    if not callable(build_scene) or not callable(build_config):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' must define build_scene() and build_config()"
        )
    try:
        scene = cast(Callable[[], object], build_scene)()
        momentum_config = cast(Callable[[], object], build_config)()
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build M-Blocks momentum-pivot scenario '{scenario_path}': {error}"
        ) from error
    if not isinstance(scene, SceneSpec) or not isinstance(momentum_config, MomentumPivotConfig):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' returned an invalid scene or configuration"
        )

    minimum_center_height = min(placement.pose.translation[2] for placement in scene.placements)
    height_offset = config.height_m - minimum_center_height
    if abs(height_offset) > 1e-12:
        scene = SceneSpec.of(
            replace(
                placement,
                pose=Transform.from_translation((0.0, 0.0, height_offset)).compose(placement.pose),
            )
            for placement in scene.placements
        )
    return scene, replace(momentum_config, dt_s=config.dt_s)


def _load_mblocks_physical_twelve_module_example(
    config: RuntimeInspectorConfig,
) -> tuple[SceneSpec, RuntimeScenarioBuilder]:
    """Load the source-checkout physical twelve-module M-Blocks example."""
    scenario_path = _MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "M-Blocks physical twelve-module example content was not found. This "
            "demonstration requires a ModSim source checkout containing "
            "'examples/scenarios/mblocks_twelve_module_physics.py'."
        )
    spec = spec_from_file_location(
        "modsim_example_mblocks_physical_twelve_module_line",
        scenario_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"M-Blocks physical twelve-module example is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load M-Blocks physical twelve-module scenario '{scenario_path}': {error}"
        ) from error
    build_scene = getattr(module, "build_scene", None)
    build_scenario = getattr(module, "build_scenario", None)
    if not callable(build_scene) or not callable(build_scenario):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' must define build_scene() and "
            "build_scenario(session, *, dt_s)"
        )
    try:
        scene = cast(Callable[[], object], build_scene)()
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build M-Blocks physical twelve-module scene '{scenario_path}': {error}"
        ) from error
    if not isinstance(scene, SceneSpec):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' returned an invalid scene"
        )

    minimum_center_height = min(placement.pose.translation[2] for placement in scene.placements)
    height_offset = config.height_m - minimum_center_height
    if abs(height_offset) > 1e-12:
        scene = SceneSpec.of(
            replace(
                placement,
                pose=Transform.from_translation((0.0, 0.0, height_offset)).compose(placement.pose),
            )
            for placement in scene.placements
        )

    def scenario_builder(session: RuntimeSession) -> RuntimeScenario:
        try:
            scenario = build_scenario(session, dt_s=config.dt_s)
        except Exception as error:
            raise RuntimeInspectorSetupError(
                f"Could not build M-Blocks physical twelve-module scenario "
                f"'{scenario_path}': {error}"
            ) from error
        if not isinstance(getattr(scenario, "status", None), ReconfigurationStatus) or not callable(
            getattr(scenario, "step", None)
        ):
            raise RuntimeInspectorSetupError(
                f"M-Blocks scenario '{scenario_path}' returned an invalid runtime scenario"
            )
        return cast(RuntimeScenario, scenario)

    return scene, scenario_builder


def _load_mblocks_staircase_example(
    config: RuntimeInspectorConfig,
    *,
    physical: bool,
) -> tuple[SceneSpec, RuntimeScenarioBuilder]:
    """Load either executor for the shared twelve-module staircase route."""
    scenario_path = _MBLOCKS_TWELVE_MODULE_STAIRCASE_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "M-Blocks staircase example content was not found. This demonstration "
            "requires a ModSim source checkout containing "
            "'examples/scenarios/mblocks_twelve_module_staircase.py'."
        )
    spec = spec_from_file_location(
        "modsim_example_mblocks_twelve_module_staircase",
        scenario_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"M-Blocks staircase example is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load M-Blocks staircase scenario '{scenario_path}': {error}"
        ) from error

    build_scene = getattr(module, "build_scene", None)
    builder_name = "build_physical_scenario" if physical else "build_kinematic_scenario"
    build_scenario = getattr(module, builder_name, None)
    if not callable(build_scene) or not callable(build_scenario):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' must define build_scene() and "
            f"{builder_name}(session, *, dt_s)"
        )
    try:
        scene = cast(Callable[[], object], build_scene)()
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build M-Blocks staircase scene '{scenario_path}': {error}"
        ) from error
    if not isinstance(scene, SceneSpec):
        raise RuntimeInspectorSetupError(
            f"M-Blocks scenario '{scenario_path}' returned an invalid scene"
        )

    minimum_center_height = min(placement.pose.translation[2] for placement in scene.placements)
    height_offset = config.height_m - minimum_center_height
    if abs(height_offset) > 1e-12:
        scene = SceneSpec.of(
            replace(
                placement,
                pose=Transform.from_translation((0.0, 0.0, height_offset)).compose(placement.pose),
            )
            for placement in scene.placements
        )
    timed_builder = cast(_TimedScenarioBuilder, build_scenario)

    def scenario_builder(session: RuntimeSession) -> RuntimeScenario:
        try:
            scenario = timed_builder(session, dt_s=config.dt_s)
        except Exception as error:
            mode = "physical" if physical else "kinematic"
            raise RuntimeInspectorSetupError(
                f"Could not build {mode} M-Blocks staircase scenario '{scenario_path}': {error}"
            ) from error
        if not isinstance(getattr(scenario, "status", None), ReconfigurationStatus) or not callable(
            getattr(scenario, "step", None)
        ):
            raise RuntimeInspectorSetupError(
                f"M-Blocks scenario '{scenario_path}' returned an invalid runtime scenario"
            )
        return cast(RuntimeScenario, scenario)

    return scene, scenario_builder


def _load_smores_physics_config(
    fixed_module: ModuleInstanceId,
    moving_module: ModuleInstanceId,
    config: RuntimeInspectorConfig,
) -> DifferentialDriveDockingConfig:
    """Load the source-checkout SMORES-EP dynamics configuration."""
    scenario_path = _SMORES_PHYSICS_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "SMORES differential-drive example content was not found. This demonstration "
            "requires a ModSim source checkout containing "
            "'examples/scenarios/smores_ep_diff_drive_dock_undock.py'."
        )
    spec = spec_from_file_location("modsim_example_smores_physical_docking", scenario_path)
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"SMORES differential-drive example is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load SMORES differential-drive scenario '{scenario_path}': {error}"
        ) from error
    build_config = getattr(module, "build_config", None)
    if not callable(build_config):
        raise RuntimeInspectorSetupError(
            f"SMORES differential-drive scenario '{scenario_path}' does not define build_config()"
        )
    try:
        physical_config = build_config(
            fixed_module,
            moving_module,
            dt_s=config.dt_s,
            initial_gap_m=_resolved_connector_gap_m(config),
            approach_speed_m_s=config.approach_m_s,
            retract_speed_m_s=(
                config.approach_m_s if config.retract_m_s is None else config.retract_m_s
            ),
        )
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build SMORES differential-drive scenario '{scenario_path}': {error}"
        ) from error
    if not isinstance(physical_config, DifferentialDriveDockingConfig):
        raise RuntimeInspectorSetupError(
            f"SMORES differential-drive scenario '{scenario_path}' returned invalid configuration"
        )
    return physical_config


def _resolved_connector_gap_m(config: RuntimeInspectorConfig) -> float:
    """Return the pair-demo staging default when the option was omitted."""
    return _DEFAULT_CONNECTOR_GAP_M if config.connector_gap_m is None else config.connector_gap_m


def _load_smores_physical_reconfiguration_config(
    config: RuntimeInspectorConfig,
) -> DifferentialDriveReconfigurationConfig:
    """Load the source-checkout physical Driver-to-Snake configuration."""
    scenario_path = _SMORES_PHYSICAL_RECONFIGURATION_SCENARIO
    if not scenario_path.is_file():
        raise RuntimeInspectorSetupError(
            "SMORES physical Driver-to-Snake example content was not found. The "
            "smores_physical_driver_to_snake demo requires a ModSim source checkout "
            "containing 'examples/scenarios/smores_ep_physical_driver_to_snake.py'."
        )
    spec = spec_from_file_location(
        "modsim_example_smores_physical_driver_to_snake",
        scenario_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeInspectorSetupError(
            f"smores_physical_driver_to_snake is unavailable at '{scenario_path}'"
        )
    module = module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not load smores_physical_driver_to_snake scenario '{scenario_path}': {error}"
        ) from error
    build_config = getattr(module, "build_config", None)
    if not callable(build_config):
        raise RuntimeInspectorSetupError(
            "smores_physical_driver_to_snake scenario "
            f"'{scenario_path}' does not define build_config()"
        )
    try:
        physical_config = build_config(
            dt_s=config.dt_s,
            navigation_speed_m_s=config.approach_m_s,
            approach_speed_m_s=config.approach_m_s,
        )
    except Exception as error:
        raise RuntimeInspectorSetupError(
            f"Could not build smores_physical_driver_to_snake scenario '{scenario_path}': {error}"
        ) from error
    if not isinstance(physical_config, DifferentialDriveReconfigurationConfig):
        raise RuntimeInspectorSetupError(
            "smores_physical_driver_to_snake scenario "
            f"'{scenario_path}' returned invalid configuration"
        )
    return physical_config


def _resolve_demo(value: RuntimeDemo | str) -> RuntimeDemo:
    try:
        return RuntimeDemo(value)
    except ValueError as error:
        choices = ", ".join(item.value for item in RuntimeDemo)
        raise RuntimeInspectorSetupError(
            f"unknown runtime demo '{value}'; available: {choices}"
        ) from error


__all__ = [
    "RuntimeInspectorConfig",
    "RuntimeInspectorRunner",
    "RuntimeInspectorSetupError",
    "resolve_runtime_connector_pair",
    "resolve_runtime_module_type",
    "resolve_runtime_recipe",
    "runtime_frame_signature",
    "validate_runtime_inspector_config",
]
