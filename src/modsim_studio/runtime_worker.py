# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
"""Background execution for the standalone Studio Runtime Inspector.

Only :class:`RuntimeInspectorWorker` touches a live runtime session.  The Qt
thread boundary carries frozen :class:`~modsim.runtime.RuntimeInspectorFrame`
objects, never a backend adapter or mutable ``WorldState``.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event as ThreadEvent

from PySide6.QtCore import QObject, QThread, Signal, Slot

from modsim.backends.registry import create_backend
from modsim.connectors.compatibility import evaluate_compatibility
from modsim.core.ids import connector_instance_id
from modsim.core.scene import SceneSpec
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
from modsim.runtime import (
    DockingPairScenario,
    DockingPairScenarioConfig,
    RuntimeInspectorFrame,
    RuntimeSession,
    build_runtime_inspector_frame,
)

_LOGGER = logging.getLogger("modsim.runtime_inspector")
_INITIAL_SCENE_SPACING_M = 0.2
_INTERRUPTION_POLL_S = 0.01


class RuntimeInspectorSetupError(ValueError):
    """Raised when an inspector configuration cannot produce a runtime."""


@dataclass(frozen=True, slots=True)
class RuntimeInspectorConfig:
    """Complete, backend-neutral launch request for the first inspector.

    Connector names are module-local Robot Pack IDs.  When both are omitted,
    the worker chooses the first declared connector whose connector type is
    self-compatible.  Supplying only one side is an error.
    """

    pack_path: Path
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


class RuntimeInspectorWorker(QObject):
    """Load and execute one two-module scenario on its owning Qt thread."""

    frame_ready = Signal(object)
    status_changed = Signal(str)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, config: RuntimeInspectorConfig) -> None:
        super().__init__()
        self._config = config
        self._interruption = ThreadEvent()

    def request_interruption(self) -> None:
        """Request a prompt, cooperative stop from any thread."""
        self._interruption.set()

    @Slot()
    def run(self) -> None:
        """Run the configured scenario, reporting failures instead of escaping."""
        session: RuntimeSession | None = None
        failure: Exception | None = None
        try:
            self.status_changed.emit("Loading and validating Robot Pack…")
            _validate_config(self._config)
            if self._interrupted():
                self.status_changed.emit("Run stopped")
                return
            loaded = RobotPackLoader().load(self._config.pack_path)
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
            if self._interrupted():
                self.status_changed.emit("Run stopped")
                return

            pack = loaded.pack
            module_type = _resolve_module_type(pack, self._config.module_type)
            fixed_local, moving_local = _resolve_connector_pair(
                pack,
                module_type,
                self._config.fixed_connector,
                self._config.moving_connector,
            )
            recipe = _resolve_runtime_recipe(pack, self._config.view_id)
            scene = SceneSpec.grid(
                module_type,
                2,
                spacing_m=_INITIAL_SCENE_SPACING_M,
                origin=(0.0, 0.0, self._config.height_m),
            )

            self.status_changed.emit(f"Starting {self._config.backend} backend…")
            session = _create_session(loaded, scene, self._config)
            if self._interrupted():
                self.status_changed.emit("Run stopped")
                return
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
                    gap_m=self._config.connector_gap_m,
                    orientation_rad=self._config.orientation_rad,
                    approach_speed_m_s=self._config.approach_m_s,
                    dt_s=self._config.dt_s,
                    release_after_s=self._config.undock_at_s,
                    retract_speed_m_s=self._config.retract_m_s,
                ),
            )
            self.status_changed.emit(f"Running {module_type}: {fixed_local} ↔ {moving_local}")
            interrupted = self._run_scenario(session, scenario, recipe)
            self.status_changed.emit("Run stopped" if interrupted else "Run complete")
        except Exception as error:
            failure = error
            _LOGGER.exception("Runtime Inspector worker failed")
            self.failed.emit(_failure_message(error))
        finally:
            if session is not None:
                try:
                    session.shutdown()
                except Exception as error:
                    _LOGGER.exception("Runtime Inspector backend shutdown failed")
                    if failure is None:
                        self.failed.emit(
                            "The scenario ended, but the backend could not shut down cleanly: "
                            f"{error}"
                        )
            self.finished.emit()

    def _run_scenario(
        self,
        session: RuntimeSession,
        scenario: DockingPairScenario,
        recipe: ModelViewSpec,
    ) -> bool:
        factory = ModelViewFactory()
        event_cursor = 0
        last_signature: object | None = None

        frame = build_runtime_inspector_frame(
            session,
            recipe,
            factory,
            event_cursor=event_cursor,
            scenario_status=scenario.status,
        )
        self.frame_ready.emit(frame)
        event_cursor = frame.next_event_sequence
        last_signature = _frame_signature(frame)

        wall_started = time.monotonic()
        simulated_started = session.world.time_s
        publish_interval_s = 1.0 / self._config.publish_hz
        next_publish_at = wall_started + publish_interval_s
        step_count = math.ceil(self._config.duration_s / self._config.dt_s)

        for _ in range(step_count):
            if self._interrupted():
                break
            scenario.step()

            simulated_elapsed = session.world.time_s - simulated_started
            target_wall_time = wall_started + min(simulated_elapsed, self._config.duration_s)
            if not self._wait_until(target_wall_time):
                break

            now = time.monotonic()
            if now >= next_publish_at:
                frame = build_runtime_inspector_frame(
                    session,
                    recipe,
                    factory,
                    event_cursor=event_cursor,
                    scenario_status=scenario.status,
                )
                self.frame_ready.emit(frame)
                event_cursor = frame.next_event_sequence
                last_signature = _frame_signature(frame)
                while next_publish_at <= now:
                    next_publish_at += publish_interval_s

        final_frame = build_runtime_inspector_frame(
            session,
            recipe,
            factory,
            event_cursor=event_cursor,
            scenario_status=scenario.status,
        )
        if _frame_signature(final_frame) != last_signature:
            self.frame_ready.emit(final_frame)
        return self._interrupted()

    def _wait_until(self, target_s: float) -> bool:
        while not self._interrupted():
            remaining_s = target_s - time.monotonic()
            if remaining_s <= 0.0:
                return True
            self._interruption.wait(min(remaining_s, _INTERRUPTION_POLL_S))
        return False

    def _interrupted(self) -> bool:
        thread = QThread.currentThread()
        return self._interruption.is_set() or thread.isInterruptionRequested()


def _validate_config(config: RuntimeInspectorConfig) -> None:
    if not config.backend.strip():
        raise RuntimeInspectorSetupError("backend must not be empty")
    if (config.fixed_connector is None) != (config.moving_connector is None):
        raise RuntimeInspectorSetupError(
            "fixed_connector and moving_connector must be supplied together"
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


def _resolve_module_type(pack: RobotPack, requested: str | None) -> str:
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


def _resolve_connector_pair(
    pack: RobotPack,
    module_type_id: str,
    fixed: str | None,
    moving: str | None,
) -> tuple[str, str]:
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
            and evaluate_compatibility(
                connector_type,
                connector_type,
            ).compatible
        ):
            return connector.id, connector.id
    raise RuntimeInspectorSetupError(
        f"module type '{module_type_id}' has no connector whose type is self-compatible; "
        "specify both fixed_connector and moving_connector explicitly"
    )


def _resolve_runtime_recipe(pack: RobotPack, requested: str | None) -> ModelViewSpec:
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


def _frame_signature(frame: RuntimeInspectorFrame) -> object:
    source = frame.view.source
    return (
        source.sample_sequence,
        source.topology_revision,
        source.docking_revision,
        source.event_revision,
        frame.next_event_sequence,
        frame.scenario,
    )


def _failure_message(error: Exception) -> str:
    message = str(error).strip() or type(error).__name__
    return f"Runtime Inspector could not start or continue: {message}"


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
    "RuntimeInspectorSetupError",
    "RuntimeInspectorWorker",
]
