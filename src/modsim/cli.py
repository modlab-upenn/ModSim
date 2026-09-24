"""ModSim command-line interface."""

from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console
from rich.table import Table

from modsim import __version__
from modsim.backends.base import BackendError, SupportsModuleKinematics
from modsim.backends.registry import (
    DEFAULT_BACKEND,
    describe_backends,
    resolve_backend_name,
)
from modsim.cli_options import (
    APP_EPILOG,
    DOCK_EPILOG,
    RUN_EPILOG,
    VIEWS_EPILOG,
    ApproachOpt,
    BackendOpt,
    ConnectorGapOpt,
    DemoOpt,
    DockCountOpt,
    DockSpacingOpt,
    DtOpt,
    DurationOpt,
    FixedConnectorOpt,
    GravityOpt,
    GroundOpt,
    GuiFlag,
    HeightOpt,
    LatchOpt,
    ModelViewOpt,
    ModuleTypeOpt,
    MovingConnectorOpt,
    OrientationOpt,
    OutputFormat,
    OutputOpt,
    PackPathArg,
    PublishHzOpt,
    RetractOpt,
    RunCountOpt,
    RunSpacingOpt,
    ScenarioOptions,
    StepsOpt,
    UndockAtOpt,
    UndockFlag,
    ViewerOpt,
    ViewFlag,
    option_error,
)
from modsim.cli_options import require_finite_option as _require_finite
from modsim.cli_options import require_positive_finite as _require_positive_finite
from modsim.cli_render import render_docking_session as _render_docking_session
from modsim.cli_render import render_generated_model_view as _render_generated_model_view
from modsim.cli_render import render_model_view_catalog as _render_model_view_catalog
from modsim.cli_render import render_report as _render_report
from modsim.core.events import Event
from modsim.core.ids import ModuleInstanceId, connector_instance_id
from modsim.core.scene import SceneError, SceneSpec
from modsim.core.state import WorldState
from modsim.core.transforms import Vec3, vec_scale
from modsim.importers import DraftPackBuilder
from modsim.model_views import (
    ModelViewContext,
    ModelViewError,
    ModelViewFactory,
)
from modsim.robot_packs import (
    LoadedRobotPack,
    ModelViewMode,
    RobotPack,
    RobotPackLoader,
    RobotPackLoadError,
    RobotPackValidator,
    ValidationProfile,
    ValidationReport,
)
from modsim.runtime import RuntimeDemo, RuntimeInspectorConfig, RuntimeInspectorRunner
from modsim.runtime.reconfiguration import (
    ReconfigurationPlanError,
    ReconfigurationScenarioError,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
    connector_pair_plan,
)
from modsim.runtime.session import RuntimeSession
from modsim.runtime.spatial import SpatialReconfigurationScenario

app = typer.Typer(
    name="modsim",
    help="Define and validate modular-robot Robot Packs.",
    epilog=APP_EPILOG,
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


@app.command("views", epilog=VIEWS_EPILOG)
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


@app.command("dock", epilog=DOCK_EPILOG)
def dock_command(
    pack_path: PackPathArg,
    module_type: ModuleTypeOpt = None,
    count: DockCountOpt = 3,
    spacing_m: DockSpacingOpt = 0.1,
    steps: StepsOpt = 1,
    dt_s: DtOpt = 0.01,
    latch: LatchOpt = True,
    undock: UndockFlag = False,
    backend: BackendOpt = None,
    gravity: GravityOpt = True,
    output: OutputOpt = OutputFormat.TEXT,
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


@app.command("run", epilog=RUN_EPILOG)
def run_command(
    pack_path: PackPathArg,
    gui: GuiFlag = False,
    view: ViewFlag = False,
    demo: DemoOpt = RuntimeDemo.DOCK,
    module_type: ModuleTypeOpt = None,
    count: RunCountOpt = 2,
    spacing_m: RunSpacingOpt = 0.15,
    fixed_connector: FixedConnectorOpt = None,
    moving_connector: MovingConnectorOpt = None,
    connector_gap_m: ConnectorGapOpt = 0.03,
    orientation_rad: OrientationOpt = 0.0,
    approach_m_s: ApproachOpt = 0.03,
    retract_m_s: RetractOpt = None,
    duration_s: DurationOpt = None,
    dt_s: DtOpt = 0.002,
    undock_at_s: UndockAtOpt = None,
    backend: BackendOpt = None,
    gravity: GravityOpt = False,
    ground: GroundOpt = False,
    height_m: HeightOpt = 0.0,
    publish_hz: PublishHzOpt = 20.0,
    view_id: ModelViewOpt = None,
    viewer: ViewerOpt = None,
    output: OutputOpt = OutputFormat.TEXT,
) -> None:
    """Run a scripted docking scenario, headless or in the Runtime Inspector.

    By default modules are placed in a row and the last is driven along world
    -X, and a report is printed when the run ends. Supplying --fixed-connector
    and --moving-connector instead measures their loaded backend frames, rotates
    the moving module into a valid mating orientation, and drives it along the
    selected docking axis. This supports connectors on articulated links without
    putting URDF forward kinematics in the CLI.

    Pass --view to watch a headless run in the MuJoCo passive viewer, or --gui to
    open the live Runtime Inspector, whose named --demo scenarios include the
    seven-module smores_driver_to_snake reconfiguration. The smores_spatial_handoff
    demo uses a ModSim feedback planner under gravity with two fixed supports
    and runs in either mode.
    """
    if gui and view:
        option_error(
            "Use --view for the headless MuJoCo viewer or --gui for the Runtime "
            "Inspector, not both."
        )
    options = ScenarioOptions(
        pack_path=pack_path,
        demo=demo,
        module_type=module_type,
        count=count,
        spacing_m=spacing_m,
        fixed_connector=fixed_connector,
        moving_connector=moving_connector,
        connector_gap_m=connector_gap_m,
        orientation_rad=orientation_rad,
        approach_m_s=approach_m_s,
        retract_m_s=retract_m_s,
        duration_s=duration_s,
        dt_s=dt_s,
        undock_at_s=undock_at_s,
        backend=backend,
        gravity=gravity,
        ground=ground,
        height_m=height_m,
        publish_hz=publish_hz,
        view_id=view_id,
        viewer=viewer,
        view=view,
        gui=gui,
        output=output,
    )
    resolved_duration_s = options.validate()
    if gui:
        _launch_runtime_inspector(options, resolved_duration_s)
    else:
        _run_headless_scenario(options, resolved_duration_s)


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


def _launch_runtime_inspector(options: ScenarioOptions, resolved_duration_s: float) -> None:
    """Open the live Runtime Inspector for a scenario (modsim run --gui)."""
    if options.demo is RuntimeDemo.SMORES_DRIVER_TO_SNAKE:
        if options.fixed_connector is not None or options.moving_connector is not None:
            option_error(
                "--demo smores_driver_to_snake defines its connector actions; "
                "do not supply --fixed-connector or --moving-connector."
            )
        if options.undock_at_s is not None:
            option_error(
                "--demo smores_driver_to_snake defines its own releases; do not supply --undock-at."
            )
        if options.orientation_rad != 0.0:
            option_error(
                "--demo smores_driver_to_snake uses the paper's nominal orientations; "
                "do not supply --orientation."
            )
        if options.gravity or options.ground:
            option_error(
                "--demo smores_driver_to_snake currently requires --no-gravity and no "
                "ground plane; supported locomotion is not implemented yet."
            )

    backend = options.backend or "mujoco"
    viewer_enabled = backend == "mujoco" if options.viewer is None else options.viewer
    if viewer_enabled and backend != "mujoco":
        option_error(
            "--viewer requires --backend mujoco; use --no-viewer for the semantic "
            "Runtime Inspector only."
        )

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
        pack_path=options.pack_path,
        demo=options.demo,
        module_type=options.module_type,
        backend=backend,
        fixed_connector=options.fixed_connector,
        moving_connector=options.moving_connector,
        connector_gap_m=options.connector_gap_m,
        orientation_rad=options.orientation_rad,
        approach_m_s=options.approach_m_s,
        retract_m_s=options.retract_m_s,
        duration_s=resolved_duration_s,
        dt_s=options.dt_s,
        undock_at_s=options.undock_at_s,
        gravity=options.gravity,
        ground=options.ground,
        height_m=options.height_m,
        view_id=options.view_id,
        publish_hz=options.publish_hz,
        viewer_enabled=viewer_enabled,
    )
    raise typer.Exit(code=runtime_main(config))


def _run_spatial_scenario(options: ScenarioOptions, duration_s: float) -> None:
    """Run the same ModSim controller used by the Runtime Inspector."""
    config = RuntimeInspectorConfig(
        pack_path=options.pack_path,
        demo=options.demo,
        module_type=options.module_type,
        backend=options.backend or "mujoco",
        duration_s=duration_s,
        dt_s=options.dt_s,
        fixed_connector=options.fixed_connector,
        moving_connector=options.moving_connector,
        undock_at_s=options.undock_at_s,
        orientation_rad=options.orientation_rad,
        height_m=options.height_m,
        view_id=options.view_id,
    )
    try:
        runner = RuntimeInspectorRunner.create(config)
    except (ValueError, BackendError, ImportError) as error:
        typer.echo(f"Could not start the spatial demo: {error}", err=True)
        raise typer.Exit(code=2) from error
    try:
        scenario = runner.scenario
        assert isinstance(scenario, SpatialReconfigurationScenario)
        if options.view:
            from modsim_backend_mujoco.spatial_viewer import view_spatial_scenario

            view_spatial_scenario(scenario)
        else:
            while not runner.finished:
                runner.step()
        if options.output is OutputFormat.JSON:
            typer.echo(json.dumps(scenario.report(), indent=2))
        else:
            typer.echo(f"{scenario.phase.upper()}: {scenario.detail}")
            typer.echo(
                f"Time: {runner.session.world.time_s:.3f} s; "
                f"target error: {scenario.target_error_m() * 1000:.2f} mm; "
                f"peak effort: {scenario.peak_effort_nm:.3f} Nm"
            )
        if scenario.phase != "complete":
            raise typer.Exit(code=1)
    finally:
        runner.shutdown()


def _run_headless_scenario(options: ScenarioOptions, resolved_duration_s: float) -> None:
    """Run a scripted scenario against a physics backend and render a report."""
    if options.demo is RuntimeDemo.SMORES_SPATIAL_HANDOFF:
        _run_spatial_scenario(options, resolved_duration_s)
        return
    try:
        loaded = RobotPackLoader().load(options.pack_path)
    except RobotPackLoadError as error:
        report = ValidationReport.from_issues(list(error.issues))
        _render_report(
            report, profile=ValidationProfile.AUTHORING, output=options.output, pack_id=None
        )
        raise typer.Exit(code=1) from error

    pack = loaded.pack
    resolved_type = _resolve_module_type(pack, options.module_type)
    backend_name = resolve_backend_name(options.backend)
    if options.pair_requested and options.count != 2:
        option_error("connector-pair scenarios currently require --count 2")
    if options.view and backend_name != "mujoco":
        option_error("--view requires the MuJoCo backend")

    scene = SceneSpec.grid(
        resolved_type,
        options.count,
        spacing_m=options.spacing_m,
        origin=(0.0, 0.0, options.height_m),
    )
    try:
        session = _create_session(
            loaded, scene, backend_name, gravity=options.gravity, ground=options.ground
        )
    except (SceneError, BackendError) as error:
        typer.echo(f"Could not start the scenario: {error}", err=True)
        raise typer.Exit(code=2) from error

    retract = options.approach_m_s if options.retract_m_s is None else options.retract_m_s
    stepper: Callable[[], tuple[Event, ...]]
    if options.fixed_connector is not None and options.moving_connector is not None:
        try:
            fixed_id = connector_instance_id(scene.instance_ids[0], options.fixed_connector)
            moving_id = connector_instance_id(scene.instance_ids[-1], options.moving_connector)
            scenario = ScriptedReconfigurationScenario.create(
                session,
                connector_pair_plan(
                    fixed_id,
                    moving_id,
                    include_undock=options.undock_at_s is not None,
                ),
                ScriptedReconfigurationConfig(
                    gap_m=options.connector_gap_m,
                    orientation_rad=options.orientation_rad,
                    approach_speed_m_s=options.approach_m_s,
                    dt_s=options.dt_s,
                    initial_hold_s=0.0,
                    release_after_s=options.undock_at_s,
                    retract_speed_m_s=retract,
                ),
            )
        except (ReconfigurationPlanError, ReconfigurationScenarioError) as error:
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
                linear_m_s=vec_scale(approach_direction, options.approach_m_s),
            )
        else:
            typer.echo(
                f"The '{backend_name}' backend cannot drive modules, so nothing will approach.",
                err=True,
            )
        stepper = _make_stepper(
            session,
            options.dt_s,
            options.undock_at_s,
            driver,
            retract,
            retract_direction=vec_scale(approach_direction, -1.0),
        )

    if options.view:
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
            events = run_with_viewer(session, duration_s=resolved_duration_s, step_once=stepper)
        except BackendError as error:
            typer.echo(str(error), err=True)
            raise typer.Exit(code=2) from error
    else:
        events = _run_headless(session, resolved_duration_s, stepper)
    _render_docking_session(
        session, events, output=options.output, latch=True, backend=backend_name
    )
