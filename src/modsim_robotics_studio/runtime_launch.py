"""Prepare and check Studio runtime requests without starting a simulator.

These application helpers keep the desktop form independent of Qt and keep
Inspector-specific requirements out of generic Robot Pack validation. Readiness
checks inspect data and construct a small derived view; they never instantiate
a backend or advance physics. A successful report is still not proof that the
selected engine can compile its mechanical assets or finish a demonstration.
"""

from __future__ import annotations

from pathlib import Path

from modsim_robotics.backends.base import BackendError
from modsim_robotics.backends.registry import backend_entry
from modsim_robotics.connectors.compatibility import evaluate_compatibility
from modsim_robotics.core.scene import SceneSpec
from modsim_robotics.core.state import WorldState
from modsim_robotics.model_views import ModelViewContext, ModelViewError, ModelViewFactory
from modsim_robotics.robot_packs import (
    LoadedRobotPack,
    ModelViewMode,
    ModelViewSpec,
    RobotPack,
    RobotPackValidator,
    Severity,
    ValidationIssue,
    ValidationProfile,
    ValidationReport,
)
from modsim_robotics.runtime import RuntimeDemo, RuntimeInspectorConfig
from modsim_robotics.runtime.inspector_runner import (
    RuntimeInspectorSetupError,
    resolve_runtime_connector_pair,
    resolve_runtime_module_type,
    resolve_runtime_recipe,
    validate_runtime_inspector_config,
)
from modsim_robotics_studio.project import StudioProject

_PHYSICAL_MBLOCKS_SEQUENCE = frozenset(
    {
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE,
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE,
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE,
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE_LARGE,
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE_ELBOW,
    }
)
_PHYSICAL_MBLOCKS = _PHYSICAL_MBLOCKS_SEQUENCE | {RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT}
_PHYSICAL_SMORES = frozenset(
    {
        RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK,
        RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
        RuntimeDemo.SMORES_ONLINE_ASSEMBLY,
        RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE,
    }
)
_KINEMATIC_MBLOCKS = frozenset(
    {
        RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT,
        RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE,
        RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE,
    }
)
_DURATIONS_S = {
    RuntimeDemo.DOCK: 4.0,
    RuntimeDemo.DOCK_UNDOCK: 6.0,
    RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK: 12.0,
    RuntimeDemo.SMORES_DRIVER_TO_SNAKE: 14.0,
    RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE: 210.0,
    RuntimeDemo.SMORES_ONLINE_ASSEMBLY: 360.0,
    RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE: 360.0,
    RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT: 8.0,
    RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT: 5.0,
    RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE: 12.0,
    RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE: 12.0,
    RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE: 24.0,
    RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE: 28.0,
    RuntimeDemo.MBLOCKS_ONLINE_LATTICE: 120.0,
    RuntimeDemo.MBLOCKS_ONLINE_LATTICE_LARGE: 180.0,
    RuntimeDemo.MBLOCKS_ONLINE_LATTICE_ELBOW: 240.0,
}


def runtime_config_defaults(
    pack_path: Path,
    demo: RuntimeDemo = RuntimeDemo.DOCK_UNDOCK,
    backend: str = "mujoco",
) -> RuntimeInspectorConfig:
    """Return editable Inspector defaults matching the CLI's demonstration presets.

    Args:
        pack_path: Directory containing the Robot Pack root manifest.
        demo: Demonstration whose duration and physics defaults should be used.
            Studio initially uses the six-second docking and undocking example.
        backend: Explicit registered backend name. MuJoCo enables its native
            companion viewer; other backends start with that viewer disabled.

    Returns:
        An immutable launch request. Use ``dataclasses.replace`` to apply form
        values and ``validate_runtime_launch`` before starting the Inspector.

    Raises:
        ValueError: If ``demo`` is not a known demonstration identifier.
    """
    demo = RuntimeDemo(demo)
    physical_mblocks = demo in _PHYSICAL_MBLOCKS
    physical_smores = demo in _PHYSICAL_SMORES
    physical = physical_mblocks or physical_smores
    dt_s = (
        0.0005 if demo in _PHYSICAL_MBLOCKS_SEQUENCE else (0.00025 if physical_mblocks else 0.002)
    )
    height_m = (
        0.025
        if physical_mblocks or demo is RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE
        else (0.05 if physical_smores else 0.0)
    )
    predefined_scene = (
        demo in _KINEMATIC_MBLOCKS
        or physical_mblocks
        or demo
        in {
            RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
            RuntimeDemo.SMORES_ONLINE_ASSEMBLY,
            RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE,
        }
    )
    return RuntimeInspectorConfig(
        pack_path=Path(pack_path),
        demo=demo,
        backend=backend,
        duration_s=_DURATIONS_S[demo],
        dt_s=dt_s,
        gravity=physical,
        ground=physical,
        height_m=height_m,
        connector_gap_m=None if predefined_scene else 0.02,
        view_id=(
            "mblocks_physics_lattice"
            if physical_mblocks or demo is RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE
            else None
        ),
        viewer_enabled=backend == "mujoco",
    )


def runtime_recipes(pack: RobotPack) -> tuple[ModelViewSpec, ...]:
    """Return the pack's runtime-enabled model recipes in manifest order.

    Args:
        pack: Current Robot Pack document, including unsaved Studio changes.

    Returns:
        Runtime-enabled recipes. Unknown builders remain visible so readiness
        can explain installation or configuration problems instead of hiding data.
    """
    return tuple(
        recipe for recipe in pack.manifest.model_views if ModelViewMode.RUNTIME in recipe.modes
    )


def add_default_topology_view(project: StudioProject) -> StudioProject:
    """Add a generic runtime topology recipe through Studio's normal edit boundary.

    Args:
        project: Authoring document to copy. Its files and existing recipes are
            left untouched. The caller must adopt the returned project.

    Returns:
        A dirty project with a uniquely named ``module_topology`` recipe. The
        recipe is marked default only when no runtime default already exists.
        Nothing is saved until the normal Studio save operation is requested.
    """
    identifiers = {recipe.id for recipe in project.pack.manifest.model_views}
    identifier = "module_topology"
    suffix = 2
    while identifier in identifiers:
        identifier = f"module_topology_{suffix}"
        suffix += 1
    return project.add_model_view(
        ModelViewSpec(
            id=identifier,
            name="Module Topology",
            builder="module_topology_graph",
            modes=(ModelViewMode.RUNTIME,),
            default=not any(recipe.default for recipe in runtime_recipes(project.pack)),
            configuration={},
        )
    )


def validate_runtime_launch(
    loaded: LoadedRobotPack,
    config: RuntimeInspectorConfig,
) -> ValidationReport:
    """Check pack, backend, demonstration inputs, and Inspector view without physics.

    Args:
        loaded: Current in-memory Robot Pack and its original asset directory.
            Unsaved edits are checked exactly as shown in Studio.
        config: Complete launch request assembled from the Runtime workspace.

    Returns:
        A deterministic report combining simulation-profile issues with
        Inspector-specific errors and suggested fixes. The selected backend's
        factory is imported but never instantiated. A one-module canonical
        world is constructed to validate the recipe and renderer. No engine,
        native viewer, output files, or simulation resources are created.
        A valid report does not guarantee asset compilation, scenario success,
        or all third-party backend capabilities.
    """
    issues = list(
        RobotPackValidator().validate(loaded, profile=ValidationProfile.SIMULATION).issues
    )
    try:
        validate_runtime_inspector_config(config)
    except RuntimeInspectorSetupError as error:
        issues.append(
            _issue("runtime.config", str(error), "Review the Runtime settings and preset.")
        )

    try:
        backend_entry(config.backend).factory()
    except (BackendError, OSError) as error:
        issues.append(
            _issue(
                "runtime.backend",
                str(error),
                "Choose an installed backend or install the dependency named in this error.",
            )
        )
    if config.viewer_enabled and config.backend != "mujoco":
        issues.append(
            _issue(
                "runtime.viewer",
                "The native 3D companion viewer requires the MuJoCo backend.",
                "Disable the native viewer or select MuJoCo.",
            )
        )

    module_type: str | None = None
    try:
        module_type = resolve_runtime_module_type(loaded.pack, config.module_type)
    except RuntimeInspectorSetupError as error:
        issues.append(_issue("runtime.module_type", str(error), "Select a module type in Runtime."))
    if module_type is not None and config.demo in set(RuntimeDemo):
        _check_demo_pack(loaded.pack, module_type, config, issues)

    requested_view = config.view_id
    if requested_view is None and (
        config.demo in _PHYSICAL_MBLOCKS
        or config.demo is RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE
    ):
        requested_view = "mblocks_physics_lattice"
    try:
        recipe = resolve_runtime_recipe(loaded.pack, requested_view)
        if module_type is not None:
            world = WorldState.from_scene(
                loaded.pack,
                SceneSpec.grid(module_type, 1, spacing_m=0.2),
            )
            result = ModelViewFactory().build(
                recipe, ModelViewContext(pack=loaded.pack, world=world)
            )
            if result.view_type not in {"module_topology_graph", "cubic_lattice"}:
                raise RuntimeInspectorSetupError(
                    f"Model view '{recipe.id}' produces unsupported Inspector result "
                    f"'{result.view_type}'."
                )
    except (RuntimeInspectorSetupError, ModelViewError, ValueError) as error:
        issues.append(
            _issue(
                "runtime.model_view",
                str(error),
                "Select a runtime model view, add a topology view, "
                "or edit its recipe in Model Views.",
            )
        )
    return ValidationReport.from_issues(issues)


def _issue(code: str, message: str, suggested_fix: str) -> ValidationIssue:
    return ValidationIssue(
        severity=Severity.ERROR,
        code=code,
        message=message,
        document="Runtime setup",
        suggested_fix=suggested_fix,
    )


def _check_demo_pack(
    pack: RobotPack,
    module_type: str,
    config: RuntimeInspectorConfig,
    issues: list[ValidationIssue],
) -> None:
    demo = RuntimeDemo(config.demo)
    module = pack.hardware_catalog.module_types[module_type]
    if demo in {RuntimeDemo.DOCK, RuntimeDemo.DOCK_UNDOCK}:
        try:
            fixed, moving = resolve_runtime_connector_pair(
                pack, module_type, config.fixed_connector, config.moving_connector
            )
            connectors = {connector.id: connector for connector in module.connectors}
            compatibility = evaluate_compatibility(
                pack.hardware_catalog.connector_types[connectors[fixed].connector_type],
                pack.hardware_catalog.connector_types[connectors[moving].connector_type],
            )
            if not compatibility.compatible:
                raise RuntimeInspectorSetupError(
                    f"Connectors '{fixed}' and '{moving}' have incompatible connector types."
                )
        except (RuntimeInspectorSetupError, KeyError) as error:
            issues.append(
                _issue(
                    "runtime.connectors",
                    str(error),
                    "Choose a compatible connector pair or edit connector definitions in Builder.",
                )
            )
        return
    required_connectors: set[str] = set()
    required_joints: set[str] = set()
    if demo.value.startswith("smores_"):
        required_connectors = {"bottom", "pan"}
        if demo in _PHYSICAL_SMORES:
            required_joints = {"joint_left_wheel", "joint_right_wheel", "joint_tilt", "joint_pan"}
    elif demo.value.startswith("mblocks_"):
        required_connectors = {"pos_x", "neg_x", "pos_z", "neg_z"}
        if demo in _PHYSICAL_MBLOCKS:
            required_joints = {"flywheel_spin"}
    missing_connectors = required_connectors - {connector.id for connector in module.connectors}
    missing_joints = required_joints - {joint.id for joint in module.joints}
    if missing_connectors or missing_joints:
        missing: list[str] = []
        if missing_connectors:
            missing.append("connectors: " + ", ".join(sorted(missing_connectors)))
        if missing_joints:
            missing.append("joints: " + ", ".join(sorted(missing_joints)))
        issues.append(
            _issue(
                "runtime.demo_pack",
                f"Preset '{demo.value}' needs platform-specific module data; "
                f"selected module '{module_type}' is missing " + "; ".join(missing) + ".",
                "Open the matching platform pack or select the generic dock/dock_undock preset.",
            )
        )
