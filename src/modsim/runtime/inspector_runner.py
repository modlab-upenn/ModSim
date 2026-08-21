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
from dataclasses import dataclass, field
from pathlib import Path

from modsim.backends.registry import create_backend
from modsim.connectors.compatibility import evaluate_compatibility
from modsim.core.events import Event
from modsim.core.ids import connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.model_views import ModelViewFactory
from modsim.robot_packs import (
    LoadedRobotPack,
    ModelViewMode,
    ModelViewSpec,
    RobotPack,
    RobotPackLoader,
    RobotPackValidator,
    ValidationProfile,
)
from modsim.runtime.inspection import RuntimeInspectorFrame, build_runtime_inspector_frame
from modsim.runtime.presets import RuntimeDemo, smores_driver_to_snake_plan
from modsim.runtime.reconfiguration import (
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
)
from modsim.runtime.scenarios import (
    DockingPairScenario,
    DockingPairScenarioConfig,
)
from modsim.runtime.session import RuntimeSession

_LOGGER = logging.getLogger("modsim.runtime_inspector")
_INITIAL_SCENE_SPACING_M = 0.2
_RECONFIGURATION_SCENE_SPACING_M = 0.12

StatusCallback = Callable[[str], None]


class RuntimeInspectorSetupError(ValueError):
    """Raised when an inspector configuration cannot produce a runtime."""


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
    connector_gap_m: float = 0.02
    orientation_rad: float = 0.0
    approach_m_s: float = 0.03
    retract_m_s: float | None = 0.03
    duration_s: float = 4.0
    dt_s: float = 0.002
    undock_at_s: float | None = None
    gravity: bool = False
    ground: bool = False
    height_m: float = 0.0
    view_id: str | None = None
    publish_hz: float = 20.0
    viewer_enabled: bool = False


@dataclass(slots=True)
class RuntimeInspectorRunner:
    """Authoritative mutable execution state behind immutable inspector frames."""

    config: RuntimeInspectorConfig
    session: RuntimeSession
    scenario: DockingPairScenario | ScriptedReconfigurationScenario
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
        recipe = resolve_runtime_recipe(pack, config.view_id)
        demo = _resolve_demo(config.demo)
        if demo is RuntimeDemo.SMORES_DRIVER_TO_SNAKE:
            plan = smores_driver_to_snake_plan()
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
        else:
            plan = None
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

        report_status(f"Starting {config.backend} backend…")
        session = _create_session(loaded, scene, config)
        try:
            if plan is not None:
                scenario = ScriptedReconfigurationScenario.create(
                    session,
                    plan,
                    ScriptedReconfigurationConfig(
                        dt_s=config.dt_s,
                        gap_m=config.connector_gap_m,
                        approach_speed_m_s=config.approach_m_s,
                    ),
                )
                report_status(f"Running {plan.name} with {len(plan.module_ids)} modules")
            else:
                assert fixed_local is not None and moving_local is not None
                release_after_s = config.undock_at_s
                if demo is RuntimeDemo.DOCK_UNDOCK and release_after_s is None:
                    release_after_s = config.duration_s * 0.55
                scenario = DockingPairScenario.create(
                    session,
                    DockingPairScenarioConfig(
                        fixed_connector=connector_instance_id(
                            scene.instance_ids[0],
                            fixed_local,
                        ),
                        moving_connector=connector_instance_id(
                            scene.instance_ids[1],
                            moving_local,
                        ),
                        gap_m=config.connector_gap_m,
                        orientation_rad=config.orientation_rad,
                        approach_speed_m_s=config.approach_m_s,
                        dt_s=config.dt_s,
                        release_after_s=release_after_s,
                        retract_speed_m_s=config.retract_m_s,
                    ),
                )
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
    _require_finite_nonnegative(config.connector_gap_m, "connector_gap_m")
    _require_finite(config.orientation_rad, "orientation_rad")
    _require_finite_positive(config.approach_m_s, "approach_m_s")
    _require_finite_positive(config.duration_s, "duration_s")
    _require_finite_positive(config.dt_s, "dt_s")
    _require_finite(config.height_m, "height_m")
    _require_finite_positive(config.publish_hz, "publish_hz")
    if config.retract_m_s is not None:
        _require_finite_nonnegative(config.retract_m_s, "retract_m_s")
    if config.undock_at_s is not None:
        _require_finite_nonnegative(config.undock_at_s, "undock_at_s")


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
        adapter = create_backend(
            config.backend,
            gravity=(0.0, 0.0, -9.81) if config.gravity else (0.0, 0.0, 0.0),
            ground=config.ground,
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


def _resolve_demo(value: RuntimeDemo | str) -> RuntimeDemo:
    try:
        return RuntimeDemo(value)
    except ValueError as error:
        choices = ", ".join(item.value for item in RuntimeDemo)
        raise RuntimeInspectorSetupError(
            f"unknown runtime demo '{value}'; available: {choices}"
        ) from error


def _require_finite(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise RuntimeInspectorSetupError(f"{name} must be finite")


def _require_finite_positive(value: float, name: str) -> None:
    _require_finite(value, name)
    if value <= 0.0:
        raise RuntimeInspectorSetupError(f"{name} must be greater than zero")


def _require_finite_nonnegative(value: float, name: str) -> None:
    _require_finite(value, name)
    if value < 0.0:
        raise RuntimeInspectorSetupError(f"{name} must not be negative")


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
