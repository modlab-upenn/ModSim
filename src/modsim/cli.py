"""ModSim command-line interface."""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from enum import StrEnum
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console
from rich.table import Table

from modsim import __version__
from modsim.backends.base import BackendError, SupportsModuleKinematics
from modsim.backends.registry import (
    BACKEND_ENV_VAR,
    DEFAULT_BACKEND,
    describe_backends,
    resolve_backend_name,
)
from modsim.core.events import Event
from modsim.core.ids import ModuleInstanceId, connector_instance_id
from modsim.core.scene import SceneError, SceneSpec
from modsim.core.state import WorldState
from modsim.core.transforms import Vec3, vec_scale
from modsim.importers import DraftPackBuilder
from modsim.model_views import (
    ModelView,
    ModelViewContext,
    ModelViewError,
    ModelViewFactory,
    ModuleTopologyGraphView,
)
from modsim.robot_packs import (
    LoadedRobotPack,
    ModelViewMode,
    RobotPack,
    RobotPackLoader,
    RobotPackLoadError,
    RobotPackValidator,
    Severity,
    ValidationIssue,
    ValidationProfile,
    ValidationReport,
)
from modsim.runtime import RuntimeInspectorConfig
from modsim.runtime.inspection import format_event_detail
from modsim.runtime.presets import RuntimeDemo
from modsim.runtime.scenarios import (
    DockingPairScenario,
    DockingPairScenarioConfig,
    ScenarioSetupError,
)
from modsim.runtime.session import RuntimeSession


class OutputFormat(StrEnum):
    """Supported CLI output encodings."""

    TEXT = "text"
    JSON = "json"


def _require_positive_finite(value: float, option: str) -> None:
    """Reject unsafe time arguments before a command starts a session."""
    if not math.isfinite(value) or value <= 0.0:
        typer.echo(f"{option} must be a finite number greater than 0.", err=True)
        raise typer.Exit(code=2)


def _require_finite(value: float, option: str) -> None:
    """Reject non-finite numeric arguments before they enter typed state."""
    if not math.isfinite(value):
        typer.echo(f"{option} must be a finite number.", err=True)
        raise typer.Exit(code=2)


app = typer.Typer(
    name="modsim",
    help="Define and validate modular-robot Robot Packs.",
    no_args_is_help=True,
    pretty_exceptions_enable=False,
)
pack_app = typer.Typer(
    name="pack",
    help="Inspect and validate Robot Packs.",
    no_args_is_help=True,
)
app.add_typer(pack_app)


def _version_callback(value: bool) -> None:
    if value:
        typer.echo(f"modsim {__version__}")
        raise typer.Exit()


@app.callback()
def main(
    version: Annotated[
        bool,
        typer.Option(
            "--version",
            callback=_version_callback,
            is_eager=True,
            help="Show the ModSim version and exit.",
        ),
    ] = False,
) -> None:
    """Run ModSim commands."""


@app.command("studio")
def studio_command(
    pack_path: Annotated[
        Path | None,
        typer.Argument(
            exists=False,
            file_okay=True,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Optional Robot Pack to open.",
        ),
    ] = None,
) -> None:
    """Launch the optional native ModSim Studio application."""
    try:
        from modsim_studio.app import main as studio_main
    except ImportError as error:
        typer.echo(
            "ModSim Studio dependencies are not installed. "
            "Install with: pip install 'modsim-robotics[studio]'",
            err=True,
        )
        raise typer.Exit(code=2) from error
    raise typer.Exit(code=studio_main(pack_path))


@app.command("runtime")
def runtime_command(
    pack_path: Annotated[
        Path,
        typer.Argument(
            exists=False,
            file_okay=True,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Robot Pack directory or robot_pack.yaml path.",
        ),
    ],
    demo: Annotated[
        RuntimeDemo,
        typer.Option(
            "--demo",
            case_sensitive=False,
            help=(
                "Named scenario: dock, dock_undock, or the seven-module "
                "smores_driver_to_snake reconfiguration."
            ),
        ),
    ] = RuntimeDemo.DOCK,
    module_type: Annotated[
        str | None,
        typer.Option("--module-type", "-m", help="Module type used by the selected demo."),
    ] = None,
    fixed_connector: Annotated[
        str | None,
        typer.Option(
            "--fixed-connector",
            help="Pair demos: module-local connector ID on the fixed module.",
        ),
    ] = None,
    moving_connector: Annotated[
        str | None,
        typer.Option(
            "--moving-connector",
            help="Pair demos: module-local connector ID on the moving module.",
        ),
    ] = None,
    connector_gap_m: Annotated[
        float,
        typer.Option(
            "--connector-gap",
            min=0.0,
            help="Staged separation between connector origins before approach.",
        ),
    ] = 0.02,
    orientation_rad: Annotated[
        float,
        typer.Option(
            "--orientation",
            help="Requested relative roll about the docking axis in radians.",
        ),
    ] = 0.0,
    approach_m_s: Annotated[
        float,
        typer.Option("--approach", help="Staged approach speed toward each docking target."),
    ] = 0.03,
    retract_m_s: Annotated[
        float | None,
        typer.Option(
            "--retract",
            help="Speed away after release. Defaults to the approach speed.",
        ),
    ] = None,
    duration_s: Annotated[
        float | None,
        typer.Option(
            "--duration",
            help="Simulated seconds to display. The selected demo supplies a default.",
        ),
    ] = None,
    dt_s: Annotated[float, typer.Option("--dt", help="Physics step size in seconds.")] = 0.002,
    undock_at_s: Annotated[
        float | None,
        typer.Option(
            "--undock-at",
            help="Optionally release the connection at this simulated time.",
        ),
    ] = None,
    backend: Annotated[
        str,
        typer.Option("--backend", "-b", help="Physics backend used by the runtime owner."),
    ] = "mujoco",
    gravity: Annotated[
        bool,
        typer.Option(
            "--gravity/--no-gravity",
            help="Gravity is off by default to isolate the docking demonstration.",
        ),
    ] = False,
    ground: Annotated[
        bool,
        typer.Option("--ground", help="Add a backend ground plane at z = 0."),
    ] = False,
    height_m: Annotated[
        float,
        typer.Option("--height", help="Lift demo modules above the scene origin."),
    ] = 0.0,
    view_id: Annotated[
        str | None,
        typer.Option(
            "--model-view",
            help="Runtime model-view recipe ID. Defaults to the pack's default recipe.",
        ),
    ] = None,
    publish_hz: Annotated[
        float,
        typer.Option("--publish-hz", help="Maximum inspector refresh frequency."),
    ] = 20.0,
    viewer: Annotated[
        bool | None,
        typer.Option(
            "--viewer/--no-viewer",
            help=(
                "Open the backend's native 3D viewer beside the Runtime Inspector. "
                "Defaults to on for MuJoCo and off for other backends."
            ),
        ),
    ] = None,
) -> None:
    """Open live semantic and optional native 3D views of a runtime demo."""
    resolved_duration_s = (
        duration_s
        if duration_s is not None
        else {
            RuntimeDemo.DOCK: 4.0,
            RuntimeDemo.DOCK_UNDOCK: 6.0,
            RuntimeDemo.SMORES_DRIVER_TO_SNAKE: 14.0,
        }[demo]
    )
    _require_positive_finite(resolved_duration_s, "--duration")
    _require_positive_finite(dt_s, "--dt")
    _require_positive_finite(approach_m_s, "--approach")
    _require_positive_finite(publish_hz, "--publish-hz")
    _require_finite(connector_gap_m, "--connector-gap")
    _require_finite(orientation_rad, "--orientation")
    _require_finite(height_m, "--height")
    if connector_gap_m < 0.0:
        typer.echo("--connector-gap must not be negative.", err=True)
        raise typer.Exit(code=2)
    if retract_m_s is not None:
        _require_finite(retract_m_s, "--retract")
        if retract_m_s < 0.0:
            typer.echo("--retract must not be negative.", err=True)
            raise typer.Exit(code=2)
    if undock_at_s is not None:
        _require_finite(undock_at_s, "--undock-at")
        if undock_at_s < 0.0:
            typer.echo("--undock-at must not be negative.", err=True)
            raise typer.Exit(code=2)
    if (fixed_connector is None) != (moving_connector is None):
        typer.echo(
            "--fixed-connector and --moving-connector must be supplied together",
            err=True,
        )
        raise typer.Exit(code=2)
    if demo is RuntimeDemo.SMORES_DRIVER_TO_SNAKE:
        if fixed_connector is not None or moving_connector is not None:
            typer.echo(
                "--demo smores_driver_to_snake defines its connector actions; "
                "do not supply --fixed-connector or --moving-connector.",
                err=True,
            )
            raise typer.Exit(code=2)
        if undock_at_s is not None:
            typer.echo(
                "--demo smores_driver_to_snake defines its own releases; "
                "do not supply --undock-at.",
                err=True,
            )
            raise typer.Exit(code=2)
        if orientation_rad != 0.0:
            typer.echo(
                "--demo smores_driver_to_snake uses the paper's nominal orientations; "
                "do not supply --orientation.",
                err=True,
            )
            raise typer.Exit(code=2)
        if gravity or ground:
            typer.echo(
                "--demo smores_driver_to_snake currently requires --no-gravity and no "
                "ground plane; supported locomotion is not implemented yet.",
                err=True,
            )
            raise typer.Exit(code=2)

    viewer_enabled = backend == "mujoco" if viewer is None else viewer
    if viewer_enabled and backend != "mujoco":
        typer.echo(
            "--viewer requires --backend mujoco; use --no-viewer for the semantic "
            "Runtime Inspector only.",
            err=True,
        )
        raise typer.Exit(code=2)

    try:
        from modsim_studio.runtime_app import main as runtime_main
    except ImportError as error:
        typer.echo(
            "Runtime Inspector dependencies are not installed. Install with: "
            "pip install 'modsim-robotics[studio,mujoco]'",
            err=True,
        )
        raise typer.Exit(code=2) from error

    config = RuntimeInspectorConfig(
        pack_path=pack_path,
        demo=demo,
        module_type=module_type,
        backend=backend,
        fixed_connector=fixed_connector,
        moving_connector=moving_connector,
        connector_gap_m=connector_gap_m,
        orientation_rad=orientation_rad,
        approach_m_s=approach_m_s,
        retract_m_s=retract_m_s,
        duration_s=resolved_duration_s,
        dt_s=dt_s,
        undock_at_s=undock_at_s,
        gravity=gravity,
        ground=ground,
        height_m=height_m,
        view_id=view_id,
        publish_hz=publish_hz,
        viewer_enabled=viewer_enabled,
    )
    raise typer.Exit(code=runtime_main(config))


@pack_app.command("validate")
def validate_pack_command(
    pack_path: Annotated[
        Path,
        typer.Argument(
            exists=False,
            file_okay=True,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Robot Pack directory or robot_pack.yaml path.",
        ),
    ],
    profile: Annotated[
        ValidationProfile,
        typer.Option(
            "--profile",
            "-p",
            case_sensitive=False,
            help="Authoring permits incomplete docking data; simulation requires it.",
        ),
    ] = ValidationProfile.AUTHORING,
    output: Annotated[
        OutputFormat,
        typer.Option(
            "--output",
            "-o",
            case_sensitive=False,
            help="Output format.",
        ),
    ] = OutputFormat.TEXT,
) -> None:
    """Validate the structure, local assets, and semantic references of a Robot Pack."""
    try:
        loaded = RobotPackLoader().load(pack_path)
    except RobotPackLoadError as error:
        report = ValidationReport.from_issues(list(error.issues))
        _render_report(report, profile=profile, output=output, pack_id=None)
        raise typer.Exit(code=1) from error

    report = RobotPackValidator().validate(loaded, profile=profile)
    _render_report(
        report,
        profile=profile,
        output=output,
        pack_id=f"{loaded.pack.id}@{loaded.pack.version}",
    )
    if not report.valid:
        raise typer.Exit(code=1)


@pack_app.command("inspect")
def inspect_pack_command(
    pack_path: Annotated[
        Path,
        typer.Argument(
            exists=False,
            file_okay=True,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Robot Pack directory or robot_pack.yaml path.",
        ),
    ],
    output: Annotated[
        OutputFormat,
        typer.Option(
            "--output",
            "-o",
            case_sensitive=False,
            help="Output format.",
        ),
    ] = OutputFormat.TEXT,
) -> None:
    """Show a loaded Robot Pack's aggregate contents and counts."""
    try:
        loaded = RobotPackLoader().load(pack_path)
    except RobotPackLoadError as error:
        report = ValidationReport.from_issues(list(error.issues))
        _render_report(
            report,
            profile=ValidationProfile.AUTHORING,
            output=output,
            pack_id=None,
        )
        raise typer.Exit(code=1) from error

    pack = loaded.pack
    if output is OutputFormat.JSON:
        typer.echo(json.dumps(pack.model_dump(mode="json"), indent=2))
        return

    table = Table(title=f"{pack.manifest.name} ({pack.id}@{pack.version})")
    table.add_column("Catalog")
    table.add_column("Count", justify="right")
    table.add_row("Module types", str(len(pack.hardware_catalog.module_types)))
    table.add_row("Connector types", str(len(pack.hardware_catalog.connector_types)))
    table.add_row("Capabilities", str(len(pack.capability_catalog.capabilities)))
    table.add_row("Model views", str(len(pack.manifest.model_views)))
    table.add_row("Backend mappings", str(len(pack.backend_mappings)))
    Console(width=120).print(table)


@pack_app.command("init")
def initialize_pack_command(
    urdf_path: Annotated[
        Path,
        typer.Option(
            "--from-urdf",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
            resolve_path=False,
            help="Source URDF to import.",
        ),
    ],
    destination: Annotated[
        Path,
        typer.Option(
            "--out",
            exists=False,
            file_okay=False,
            dir_okay=True,
            resolve_path=False,
            help="New Robot Pack directory.",
        ),
    ],
    asset_root: Annotated[
        list[Path] | None,
        typer.Option(
            "--asset-root",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Additional local root used to resolve package:// or relative meshes.",
        ),
    ] = None,
) -> None:
    """Create a self-contained draft Robot Pack from a URDF."""
    try:
        result = DraftPackBuilder().build(
            urdf_path,
            destination,
            asset_roots=tuple(asset_root or ()),
        )
    except (OSError, ValueError) as error:
        typer.echo(f"Could not create Robot Pack: {error}", err=True)
        raise typer.Exit(code=1) from error
    typer.echo(f"Created Robot Pack: {result.loaded.root}")
    for warning in result.warnings:
        typer.echo(f"Warning: {warning}")


@app.command("views")
def model_views_command(
    pack_path: Annotated[
        Path,
        typer.Argument(
            exists=False,
            file_okay=True,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Robot Pack directory or robot_pack.yaml path.",
        ),
    ],
    view_id: Annotated[
        str | None,
        typer.Option(
            "--view",
            help="Robot Pack model-view recipe ID to generate.",
        ),
    ] = None,
    module_type: Annotated[
        str | None,
        typer.Option(
            "--module-type",
            "-m",
            help="Module type to instantiate. Required when the pack defines more than one.",
        ),
    ] = None,
    count: Annotated[
        int,
        typer.Option("--count", "-n", min=1, help="Number of module nodes to instantiate."),
    ] = 3,
    spacing_m: Annotated[
        float,
        typer.Option(
            "--spacing",
            "-s",
            help="Distance in metres between initial module origins.",
        ),
    ] = 0.1,
    output: Annotated[
        OutputFormat,
        typer.Option("--output", "-o", case_sensitive=False, help="Output format."),
    ] = OutputFormat.TEXT,
) -> None:
    """List model-view recipes or generate one physics-free runtime snapshot."""
    try:
        loaded = RobotPackLoader().load(pack_path)
    except RobotPackLoadError as error:
        report = ValidationReport.from_issues(list(error.issues))
        _render_report(
            report,
            profile=ValidationProfile.AUTHORING,
            output=output,
            pack_id=None,
        )
        raise typer.Exit(code=1) from error

    pack = loaded.pack
    factory = ModelViewFactory()
    if view_id is None:
        _render_model_view_catalog(pack, factory, output=output)
        return

    recipe = next(
        (candidate for candidate in pack.manifest.model_views if candidate.id == view_id),
        None,
    )
    if recipe is None:
        available = ", ".join(item.id for item in pack.manifest.model_views) or "none"
        typer.echo(
            f"Unknown model-view recipe '{view_id}' for pack '{pack.id}'. "
            f"Available recipes: {available}.",
            err=True,
        )
        raise typer.Exit(code=2)
    if ModelViewMode.RUNTIME not in recipe.modes:
        modes = ", ".join(mode.value for mode in recipe.modes)
        typer.echo(
            f"Model-view recipe '{recipe.id}' is not enabled for runtime generation; "
            f"configured modes: {modes}.",
            err=True,
        )
        raise typer.Exit(code=2)

    _require_finite(spacing_m, "--spacing")
    resolved_type = _resolve_module_type(pack, module_type)
    try:
        scene = SceneSpec.grid(resolved_type, count, spacing_m=spacing_m)
        world = WorldState.from_scene(pack, scene)
    except SceneError as error:
        typer.echo(f"Could not create the initial model-view scene: {error}", err=True)
        raise typer.Exit(code=2) from error

    try:
        generated = factory.build(recipe, ModelViewContext(pack=pack, world=world))
    except ModelViewError as error:
        typer.echo(f"Could not build model-view recipe '{recipe.id}': {error}", err=True)
        raise typer.Exit(code=2) from error
    _render_generated_model_view(generated, output=output)


@app.command("dock")
def dock_command(
    pack_path: Annotated[
        Path,
        typer.Argument(
            exists=False,
            file_okay=True,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Robot Pack directory or robot_pack.yaml path.",
        ),
    ],
    module_type: Annotated[
        str | None,
        typer.Option(
            "--module-type",
            "-m",
            help="Module type to instantiate. Required when the pack defines more than one.",
        ),
    ] = None,
    count: Annotated[
        int,
        typer.Option("--count", "-n", min=1, help="Number of module instances to place."),
    ] = 3,
    spacing_m: Annotated[
        float,
        typer.Option(
            "--spacing",
            "-s",
            help="Distance in metres between placed modules along the x axis.",
        ),
    ] = 0.1,
    steps: Annotated[
        int,
        typer.Option("--steps", min=1, help="Number of simulation steps to run."),
    ] = 1,
    dt_s: Annotated[
        float,
        typer.Option("--dt", help="Simulation step size in seconds."),
    ] = 0.01,
    latch: Annotated[
        bool,
        typer.Option(
            "--latch/--no-latch",
            help="Issue a dock command for every pair whose geometry already satisfies acceptance.",
        ),
    ] = True,
    undock: Annotated[
        bool,
        typer.Option(
            "--undock", help="Release every connection after the run and report the split."
        ),
    ] = False,
    backend: Annotated[
        str | None,
        typer.Option(
            "--backend",
            "-b",
            help=(
                "Physics backend to run against. Defaults to "
                f"${BACKEND_ENV_VAR}, then '{DEFAULT_BACKEND}'."
            ),
        ),
    ] = None,
    gravity: Annotated[
        bool,
        typer.Option(
            "--gravity/--no-gravity",
            help="Disable gravity to compare a physics backend against the kinematic mock.",
        ),
    ] = True,
    output: Annotated[
        OutputFormat,
        typer.Option("--output", "-o", case_sensitive=False, help="Output format."),
    ] = OutputFormat.TEXT,
) -> None:
    """Run a docking session to check whether a Robot Pack's connectors mate.

    This exercises the real semantic pipeline (compatibility, acceptance regions,
    guards, two-phase commit) against the selected backend. On the kinematic mock
    it runs no physics, so it answers "is this pack authored such that these
    connectors would latch?" rather than "will this robot work?".
    """
    _require_positive_finite(dt_s, "--dt")

    try:
        loaded = RobotPackLoader().load(pack_path)
    except RobotPackLoadError as error:
        report = ValidationReport.from_issues(list(error.issues))
        _render_report(report, profile=ValidationProfile.AUTHORING, output=output, pack_id=None)
        raise typer.Exit(code=1) from error

    pack = loaded.pack
    report = RobotPackValidator().validate(loaded, profile=ValidationProfile.AUTHORING)
    if not report.valid:
        _render_report(
            report,
            profile=ValidationProfile.AUTHORING,
            output=output,
            pack_id=f"{pack.id}@{pack.version}",
        )
        raise typer.Exit(code=1)

    available = sorted(pack.hardware_catalog.module_types)
    if module_type is None:
        if len(available) != 1:
            typer.echo(
                "--module-type is required when a pack defines several module types: "
                + ", ".join(available),
                err=True,
            )
            raise typer.Exit(code=2)
        module_type = available[0]

    backend_name = resolve_backend_name(backend)
    # Gravity is engine-specific, so it travels as a backend option rather than
    # through the shared interface. The mock has no gravity at all.
    disable_gravity = not gravity and backend_name != DEFAULT_BACKEND
    if not gravity and backend_name == DEFAULT_BACKEND:
        typer.echo(
            "Note: the mock backend is kinematic and has no gravity, "
            "so --no-gravity changes nothing.",
            err=True,
        )

    try:
        scene = SceneSpec.grid(module_type, count, spacing_m=spacing_m)
        session = (
            RuntimeSession.create(loaded, scene, backend_name, gravity=(0.0, 0.0, 0.0))
            if disable_gravity
            else RuntimeSession.create(loaded, scene, backend_name)
        )
    except SceneError as error:
        typer.echo(f"Could not build a scene: {error}", err=True)
        raise typer.Exit(code=2) from error
    except BackendError as error:
        typer.echo(f"Could not start the '{backend_name}' backend: {error}", err=True)
        raise typer.Exit(code=2) from error

    events: list[Event] = []
    for _ in range(steps):
        if latch:
            for proposal in session.proposals():
                if proposal.compatibility.compatible and proposal.acceptance.satisfied:
                    session.request_dock(proposal.connector_a, proposal.connector_b)
        events.extend(session.step(dt_s))
    if undock:
        for connection in tuple(session.world.connections):
            session.request_undock(connection)
        events.extend(session.step(dt_s))

    _render_docking_session(
        session, tuple(events), output=output, latch=latch, backend=backend_name
    )


@app.command("run")
def run_command(
    pack_path: Annotated[
        Path,
        typer.Argument(
            exists=False,
            file_okay=True,
            dir_okay=True,
            readable=True,
            resolve_path=False,
            help="Robot Pack directory or robot_pack.yaml path.",
        ),
    ],
    module_type: Annotated[
        str | None,
        typer.Option("--module-type", "-m", help="Module type to instantiate."),
    ] = None,
    count: Annotated[
        int, typer.Option("--count", "-n", min=2, help="Number of modules to place.")
    ] = 2,
    spacing_m: Annotated[
        float, typer.Option("--spacing", "-s", help="Initial gap between module origins.")
    ] = 0.15,
    fixed_connector: Annotated[
        str | None,
        typer.Option(
            "--fixed-connector",
            help=(
                "Module-local connector ID on the first module. Use with "
                "--moving-connector to arrange a connector-to-connector demo."
            ),
        ),
    ] = None,
    moving_connector: Annotated[
        str | None,
        typer.Option(
            "--moving-connector",
            help="Module-local connector ID on the second module.",
        ),
    ] = None,
    connector_gap_m: Annotated[
        float,
        typer.Option(
            "--connector-gap",
            min=0.0,
            help="Initial separation between the selected connector origins.",
        ),
    ] = 0.03,
    orientation_rad: Annotated[
        float,
        typer.Option(
            "--orientation",
            help="Requested relative roll about the selected docking axis in radians.",
        ),
    ] = 0.0,
    approach_m_s: Annotated[
        float,
        typer.Option(
            "--approach",
            help="Speed at which the last module is driven toward the first.",
        ),
    ] = 0.03,
    retract_m_s: Annotated[
        float | None,
        typer.Option(
            "--retract",
            help=(
                "Speed at which the driven module backs away after release. "
                "Defaults to the approach speed; pass 0 to let the pair coast together."
            ),
        ),
    ] = None,
    duration_s: Annotated[
        float, typer.Option("--duration", help="Simulated seconds to run.")
    ] = 8.0,
    dt_s: Annotated[float, typer.Option("--dt", help="Step size in seconds.")] = 0.002,
    undock_at_s: Annotated[
        float | None,
        typer.Option(
            "--undock-at",
            help=(
                "Release every connection at this simulated time. "
                "Latching stops afterwards, so the modules stay apart."
            ),
        ),
    ] = None,
    backend: Annotated[
        str | None,
        typer.Option("--backend", "-b", help=f"Backend. Defaults to ${BACKEND_ENV_VAR}."),
    ] = None,
    gravity: Annotated[
        bool,
        typer.Option(
            "--gravity/--no-gravity",
            help="Gravity is off by default so a docking demo is not also a falling demo.",
        ),
    ] = False,
    ground: Annotated[bool, typer.Option("--ground", help="Add a ground plane at z = 0.")] = False,
    height_m: Annotated[
        float, typer.Option("--height", help="Lift the whole row, to clear a ground plane.")
    ] = 0.0,
    view: Annotated[
        bool,
        typer.Option("--view", help="Open the MuJoCo passive viewer and run in real time."),
    ] = False,
    output: Annotated[
        OutputFormat,
        typer.Option("--output", "-o", case_sensitive=False, help="Output format."),
    ] = OutputFormat.TEXT,
) -> None:
    """Run a scripted docking scenario against a physics backend.

    By default modules are placed in a row and the last is driven along world
    -X. Supplying a fixed and moving connector instead measures their loaded
    backend frames, rotates the second module into a valid mating orientation,
    and drives it along the selected docking axis. This supports connectors on
    articulated links without putting URDF forward kinematics in the CLI.
    """
    _require_positive_finite(duration_s, "--duration")
    _require_positive_finite(dt_s, "--dt")
    _require_finite(spacing_m, "--spacing")
    _require_finite(connector_gap_m, "--connector-gap")
    _require_finite(orientation_rad, "--orientation")
    _require_finite(approach_m_s, "--approach")
    _require_finite(height_m, "--height")
    if connector_gap_m < 0.0:
        typer.echo("--connector-gap must not be negative.", err=True)
        raise typer.Exit(code=2)
    if approach_m_s < 0.0:
        typer.echo("--approach must not be negative.", err=True)
        raise typer.Exit(code=2)
    if retract_m_s is not None:
        _require_finite(retract_m_s, "--retract")
        if retract_m_s < 0.0:
            typer.echo("--retract must not be negative.", err=True)
            raise typer.Exit(code=2)
    if undock_at_s is not None:
        _require_finite(undock_at_s, "--undock-at")
        if undock_at_s < 0.0:
            typer.echo("--undock-at must not be negative.", err=True)
            raise typer.Exit(code=2)

    try:
        loaded = RobotPackLoader().load(pack_path)
    except RobotPackLoadError as error:
        report = ValidationReport.from_issues(list(error.issues))
        _render_report(report, profile=ValidationProfile.AUTHORING, output=output, pack_id=None)
        raise typer.Exit(code=1) from error

    pack = loaded.pack
    resolved_type = _resolve_module_type(pack, module_type)
    backend_name = resolve_backend_name(backend)
    pair_requested = fixed_connector is not None or moving_connector is not None
    if pair_requested and (fixed_connector is None or moving_connector is None):
        typer.echo(
            "--fixed-connector and --moving-connector must be supplied together",
            err=True,
        )
        raise typer.Exit(code=2)
    if pair_requested and count != 2:
        typer.echo("connector-pair scenarios currently require --count 2", err=True)
        raise typer.Exit(code=2)
    if view and backend_name != "mujoco":
        typer.echo("--view requires the MuJoCo backend", err=True)
        raise typer.Exit(code=2)

    scene = SceneSpec.grid(resolved_type, count, spacing_m=spacing_m, origin=(0.0, 0.0, height_m))
    try:
        session = _create_session(loaded, scene, backend_name, gravity=gravity, ground=ground)
    except (SceneError, BackendError) as error:
        typer.echo(f"Could not start the scenario: {error}", err=True)
        raise typer.Exit(code=2) from error

    retract = approach_m_s if retract_m_s is None else retract_m_s
    stepper: Callable[[], tuple[Event, ...]]
    if fixed_connector is not None and moving_connector is not None:
        try:
            scenario = DockingPairScenario.create(
                session,
                DockingPairScenarioConfig(
                    fixed_connector=connector_instance_id(scene.instance_ids[0], fixed_connector),
                    moving_connector=connector_instance_id(
                        scene.instance_ids[-1], moving_connector
                    ),
                    gap_m=connector_gap_m,
                    orientation_rad=orientation_rad,
                    approach_speed_m_s=approach_m_s,
                    dt_s=dt_s,
                    release_after_s=undock_at_s,
                    retract_speed_m_s=retract,
                ),
            )
        except ScenarioSetupError as error:
            session.shutdown()
            typer.echo(f"Could not arrange the connector pair: {error}", err=True)
            raise typer.Exit(code=2) from error
        stepper = scenario.step
    else:
        driver = scene.instance_ids[-1]
        approach_direction: Vec3 = (-1.0, 0.0, 0.0)
        adapter = session.adapter
        if isinstance(adapter, SupportsModuleKinematics):
            adapter.set_module_twist(
                driver,
                linear_m_s=vec_scale(approach_direction, approach_m_s),
            )
        else:
            typer.echo(
                f"The '{backend_name}' backend cannot drive modules, so nothing will approach.",
                err=True,
            )
        stepper = _make_stepper(
            session,
            dt_s,
            undock_at_s,
            driver,
            retract,
            retract_direction=vec_scale(approach_direction, -1.0),
        )
    if view:
        try:
            from modsim_backend_mujoco.viewer import run_with_viewer
        except ImportError as error:  # pragma: no cover - requires a broken install
            typer.echo(f"Could not open the viewer: {error}", err=True)
            raise typer.Exit(code=2) from error
        typer.echo(
            "Running in the MuJoCo viewer. The window stays open when the scenario "
            "ends; close it to see the report.",
            err=True,
        )
        try:
            events = run_with_viewer(session, duration_s=duration_s, step_once=stepper)
        except BackendError as error:
            typer.echo(str(error), err=True)
            raise typer.Exit(code=2) from error
    else:
        events = _run_headless(session, duration_s, stepper)
    _render_docking_session(session, events, output=output, latch=True, backend=backend_name)


def _resolve_module_type(pack: RobotPack, requested: str | None) -> str:
    if requested is not None:
        return requested
    available = sorted(pack.hardware_catalog.module_types)
    if len(available) != 1:
        typer.echo(
            "--module-type is required when a pack defines several module types: "
            + ", ".join(available),
            err=True,
        )
        raise typer.Exit(code=2)
    return available[0]


def _create_session(
    loaded: LoadedRobotPack,
    scene: SceneSpec,
    backend_name: str,
    *,
    gravity: bool,
    ground: bool,
) -> RuntimeSession:
    """Build a session, passing engine options only to backends that accept them."""
    if backend_name == DEFAULT_BACKEND:
        return RuntimeSession.create(loaded, scene, backend_name)
    return RuntimeSession.create(
        loaded,
        scene,
        backend_name,
        gravity=(0.0, 0.0, -9.81) if gravity else (0.0, 0.0, 0.0),
        ground=ground,
    )


def _make_stepper(
    session: RuntimeSession,
    dt_s: float,
    undock_at_s: float | None,
    driver: ModuleInstanceId,
    retract_m_s: float,
    *,
    retract_direction: Vec3 = (1.0, 0.0, 0.0),
) -> Callable[[], tuple[Event, ...]]:
    """Return a closure performing one scripted step.

    Latching stops once the scheduled release fires, so a released pair stays
    released instead of re-forming the instant it is let go.

    Releasing a constraint does not by itself separate anything. Two welded
    modules share a velocity, so on release they simply coast along together.
    Backing the driven module away is an actuation step, which is what a real
    reconfiguration would also have to do, and it is what makes the undock
    visible.
    """
    released = False
    retracted = False

    def step_once() -> tuple[Event, ...]:
        nonlocal released, retracted
        if undock_at_s is not None and not released and session.world.time_s >= undock_at_s:
            for connection in sorted(session.world.connections):
                session.request_undock(connection)
            released = True
        if not released:
            for proposal in session.proposals():
                if proposal.compatibility.compatible and proposal.acceptance.satisfied:
                    session.request_dock(proposal.connector_a, proposal.connector_b)

        events = session.step(dt_s)

        # Retract only once the constraint is actually gone, so the pull does
        # not fight a weld that is still active for the rest of this step.
        adapter = session.adapter
        if (
            released
            and not retracted
            and not session.world.connections
            and retract_m_s > 0.0
            and isinstance(adapter, SupportsModuleKinematics)
        ):
            adapter.set_module_twist(
                driver,
                linear_m_s=vec_scale(retract_direction, retract_m_s),
            )
            retracted = True
        return events

    return step_once


def _run_headless(
    session: RuntimeSession,
    duration_s: float,
    step_once: Callable[[], tuple[Event, ...]],
) -> tuple[Event, ...]:
    collected: list[Event] = []
    while session.world.time_s < duration_s:
        collected.extend(step_once())
    return tuple(collected)


def _render_model_view_catalog(
    pack: RobotPack,
    factory: ModelViewFactory,
    *,
    output: OutputFormat,
) -> None:
    """Show the recipes in one pack and builders installed in this process."""
    recipes = pack.manifest.model_views
    builders = factory.descriptors()
    if output is OutputFormat.JSON:
        typer.echo(
            json.dumps(
                {
                    "pack": f"{pack.id}@{pack.version}",
                    "recipes": [recipe.model_dump(mode="json") for recipe in recipes],
                    "builders": [builder.model_dump(mode="json") for builder in builders],
                },
                indent=2,
            )
        )
        return

    console = Console(width=160)
    recipes_table = Table(title=f"Model-view recipes for {pack.id}@{pack.version}")
    recipes_table.add_column("Recipe", no_wrap=True)
    recipes_table.add_column("Name")
    recipes_table.add_column("Builder", no_wrap=True)
    recipes_table.add_column("Modes", no_wrap=True)
    recipes_table.add_column("Default", no_wrap=True)
    if recipes:
        for recipe in recipes:
            recipes_table.add_row(
                recipe.id,
                recipe.name or "",
                recipe.builder,
                ", ".join(mode.value for mode in recipe.modes),
                "yes" if recipe.default else "no",
            )
    else:
        recipes_table.add_row("(none)", "", "", "", "")
    console.print(recipes_table)

    builders_table = Table(title="Registered model-view builders")
    builders_table.add_column("Builder", no_wrap=True)
    builders_table.add_column("Name")
    builders_table.add_column("View type", no_wrap=True)
    builders_table.add_column("Modes", no_wrap=True)
    builders_table.add_column("Requires world", no_wrap=True)
    if builders:
        for builder in builders:
            builders_table.add_row(
                builder.builder,
                builder.name,
                builder.view_type,
                ", ".join(mode.value for mode in builder.modes),
                "yes" if builder.requires_world else "no",
            )
    else:  # pragma: no cover - the default factory always has a built-in
        builders_table.add_row("(none)", "", "", "", "")
    console.print(builders_table)


def _render_generated_model_view(generated: ModelView, *, output: OutputFormat) -> None:
    """Render a generated immutable model-view result."""
    if output is OutputFormat.JSON:
        typer.echo(json.dumps(generated.model_dump(mode="json"), indent=2))
        return

    console = Console(width=160)
    console.print(
        f"Generated [bold]{generated.id}[/bold] with "
        f"[bold]{generated.builder}[/bold] from "
        f"{generated.source.pack_id}@{generated.source.pack_version}."
    )
    if not isinstance(generated, ModuleTopologyGraphView):
        console.print(json.dumps(generated.model_dump(mode="json"), indent=2))
        return

    nodes = Table(title=f"Module nodes ({len(generated.nodes)})")
    nodes.add_column("Module", no_wrap=True)
    nodes.add_column("Module type", no_wrap=True)
    nodes.add_column("Assembly", no_wrap=True)
    nodes.add_column("World position (m)", no_wrap=True)
    for node in generated.nodes:
        nodes.add_row(
            node.id,
            node.module_type_id,
            node.assembly_id,
            "(" + ", ".join(f"{value:.6g}" for value in node.world_position_m) + ")",
        )
    console.print(nodes)

    if not generated.edges:
        console.print("Committed connection edges (0): none")
        return
    edges = Table(title=f"Committed connection edges ({len(generated.edges)})")
    edges.add_column("Connection", no_wrap=True)
    edges.add_column("Source module", no_wrap=True)
    edges.add_column("Target module", no_wrap=True)
    edges.add_column("Connectors")
    edges.add_column("Created (s)", justify="right", no_wrap=True)
    for edge in generated.edges:
        edges.add_row(
            edge.connection_id,
            edge.source,
            edge.target,
            f"{edge.connector_a} <-> {edge.connector_b}",
            f"{edge.created_at_s:.6g}",
        )
    console.print(edges)


@app.command("backends")
def backends_command(
    output: Annotated[
        OutputFormat,
        typer.Option("--output", "-o", case_sensitive=False, help="Output format."),
    ] = OutputFormat.TEXT,
) -> None:
    """List the physics backends ModSim can run against."""
    entries = describe_backends()
    active = resolve_backend_name(None)
    if output is OutputFormat.JSON:
        typer.echo(
            json.dumps(
                {
                    "active": active,
                    "backends": [
                        {"name": name, "installed": installed, "summary": summary}
                        for name, installed, summary in entries
                    ],
                },
                indent=2,
            )
        )
        return

    table = Table(title="Backends", show_header=True)
    table.add_column("Name", no_wrap=True)
    table.add_column("Installed", no_wrap=True)
    table.add_column("Summary")
    for name, installed, summary in entries:
        marker = " (active)" if name == active else ""
        table.add_row(
            f"{name}{marker}",
            "[green]yes[/green]" if installed else "[yellow]no[/yellow]",
            summary,
        )
    Console(width=120).print(table)


def _render_docking_session(
    session: RuntimeSession,
    recorded: tuple[Event, ...],
    *,
    output: OutputFormat,
    latch: bool,
    backend: str,
) -> None:
    world = session.world
    metrics = session.metrics()
    proposals = session.proposals()

    if output is OutputFormat.JSON:
        payload = {
            "pack": f"{world.pack.id}@{world.pack.version}",
            "backend": backend,
            "modules": sorted(world.modules),
            "module_positions": {
                module_id: list(module.pose.translation)
                for module_id, module in sorted(world.modules.items())
            },
            "assemblies": {
                name: sorted(world.assemblies.members(name)) for name in world.assemblies.assemblies
            },
            "connections": sorted(world.connections),
            "events": [
                {"sequence": event.sequence, "time_s": event.time_s, "kind": event.kind}
                for event in recorded
            ],
            "metrics": metrics.as_dict(),
        }
        typer.echo(json.dumps(payload, indent=2))
        return

    console = Console(width=160)
    console.print(
        f"Docking session for [bold]{world.pack.id}@{world.pack.version}[/bold] "
        f"on the [bold]{backend}[/bold] backend: "
        f"{metrics.module_count_total} module(s), "
        f"{metrics.docking_success_count} dock(s), "
        f"{metrics.docking_failure_count} failure(s), "
        f"{metrics.undocking_success_count} undock(s)"
    )

    if recorded:
        table = Table(title="Events", show_header=True)
        table.add_column("Seq", justify="right", no_wrap=True)
        table.add_column("Time (s)", justify="right", no_wrap=True)
        table.add_column("Event", no_wrap=True)
        table.add_column("Detail")
        for event in recorded:
            table.add_row(
                str(event.sequence),
                f"{event.time_s:.4g}",
                event.kind,
                format_event_detail(event),
            )
        console.print(table)

    assemblies = Table(title="Assemblies", show_header=True)
    assemblies.add_column("Assembly", no_wrap=True)
    assemblies.add_column("Modules")
    for name in world.assemblies.assemblies:
        assemblies.add_row(name, ", ".join(sorted(world.assemblies.members(name))))
    console.print(assemblies)

    # Blocked pairs are only worth reporting as a diagnostic. After a successful
    # run the same pairs reappear as "command required", which would be noise.
    interesting = metrics.docking_success_count == 0 or metrics.docking_failure_count > 0
    blocked = [proposal for proposal in proposals if not proposal.viable] if interesting else []
    if blocked:
        diagnostics = Table(title="Pairs that did not dock", show_header=True)
        diagnostics.add_column("Connector A", no_wrap=True)
        diagnostics.add_column("Connector B", no_wrap=True)
        diagnostics.add_column("Reason")
        for proposal in blocked:
            _, detail = proposal.failure()
            diagnostics.add_row(proposal.connector_a, proposal.connector_b, detail)
        console.print(diagnostics)

    if metrics.docking_success_count:
        return
    capabilities = session.adapter.capabilities()
    if not capabilities.supports_runtime_constraints:
        console.print(
            f"[yellow]The '{capabilities.name}' backend cannot create constraints at runtime, "
            "so no connection was committed. Detection, compatibility, and acceptance still "
            "ran, and the pairs above show what would have latched.[/yellow]"
        )
    elif not proposals:
        radius = session.docking.detection_radius(world)
        console.print(
            f"[yellow]No connector pair came within the {radius:.4g} m detection radius, "
            "so nothing was evaluated. Reduce --spacing so that two connectors meet, or widen "
            "the connector type's acceptance region.[/yellow]"
        )
    elif not latch:
        console.print(
            "[yellow]No docks were attempted. Pass --latch, or give the connector type a "
            "docking_policy with auto_latch: true.[/yellow]"
        )


def _render_report(
    report: ValidationReport,
    *,
    profile: ValidationProfile,
    output: OutputFormat,
    pack_id: str | None,
) -> None:
    if output is OutputFormat.JSON:
        payload = {
            "valid": report.valid,
            "profile": profile.value,
            "pack": pack_id,
            "summary": {
                "errors": len(report.errors),
                "warnings": len(report.warnings),
                "infos": len(report.infos),
            },
            "issues": [
                {
                    **issue.model_dump(mode="json"),
                    "location": issue.location,
                }
                for issue in report.issues
            ],
        }
        typer.echo(json.dumps(payload, indent=2))
        return

    console = Console(width=160)
    status = "[green]VALID[/green]" if report.valid else "[red]INVALID[/red]"
    subject = pack_id or "Robot Pack"
    console.print(
        f"{status} {subject} ({profile.value} structural validation): "
        f"{len(report.errors)} error(s), {len(report.warnings)} warning(s), "
        f"{len(report.infos)} info message(s)"
    )
    if report.issues:
        table = Table(show_header=True)
        table.add_column("Severity")
        table.add_column("Code", no_wrap=True)
        table.add_column("Location", no_wrap=True)
        table.add_column("Message")
        for issue in report.issues:
            table.add_row(
                _severity_label(issue),
                issue.code,
                issue.location,
                issue.message,
            )
        console.print(table)


def _severity_label(issue: ValidationIssue) -> str:
    colors = {
        Severity.ERROR: "red",
        Severity.WARNING: "yellow",
        Severity.INFO: "blue",
    }
    color = colors[issue.severity]
    return f"[{color}]{issue.severity.value}[/{color}]"
