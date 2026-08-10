"""ModSim command-line interface."""

from __future__ import annotations

import json
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
from modsim.core.events import (
    AssemblyMerged,
    AssemblySplit,
    ConnectorOverloaded,
    DockCandidateDetected,
    DockCommitted,
    DockFailed,
    Event,
    UndockCommitted,
    UndockFailed,
)
from modsim.core.ids import ModuleInstanceId
from modsim.core.scene import SceneError, SceneSpec
from modsim.importers import DraftPackBuilder
from modsim.robot_packs import (
    LoadedRobotPack,
    RobotPack,
    RobotPackLoader,
    RobotPackLoadError,
    RobotPackValidator,
    Severity,
    ValidationIssue,
    ValidationProfile,
    ValidationReport,
)
from modsim.runtime.session import RuntimeSession


class OutputFormat(StrEnum):
    """Supported CLI output encodings."""

    TEXT = "text"
    JSON = "json"


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

    Modules are placed in a row, the last is driven toward the first, and every
    pair that satisfies acceptance is commanded to dock. This is the smallest
    scenario that exercises approach, latch, and release under real dynamics.
    """
    try:
        loaded = RobotPackLoader().load(pack_path)
    except RobotPackLoadError as error:
        report = ValidationReport.from_issues(list(error.issues))
        _render_report(report, profile=ValidationProfile.AUTHORING, output=output, pack_id=None)
        raise typer.Exit(code=1) from error

    pack = loaded.pack
    resolved_type = _resolve_module_type(pack, module_type)
    backend_name = resolve_backend_name(backend)
    if view and backend_name != "mujoco":
        typer.echo("--view requires the MuJoCo backend", err=True)
        raise typer.Exit(code=2)

    scene = SceneSpec.grid(resolved_type, count, spacing_m=spacing_m, origin=(0.0, 0.0, height_m))
    try:
        session = _create_session(loaded, scene, backend_name, gravity=gravity, ground=ground)
    except (SceneError, BackendError) as error:
        typer.echo(f"Could not start the scenario: {error}", err=True)
        raise typer.Exit(code=2) from error

    driver = scene.instance_ids[-1]
    adapter = session.adapter
    if isinstance(adapter, SupportsModuleKinematics):
        adapter.set_module_twist(driver, linear_m_s=(-approach_m_s, 0.0, 0.0))
    else:
        typer.echo(
            f"The '{backend_name}' backend cannot drive modules, so nothing will approach.",
            err=True,
        )

    retract = approach_m_s if retract_m_s is None else retract_m_s
    stepper = _make_stepper(session, dt_s, undock_at_s, driver, retract)
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
            adapter.set_module_twist(driver, linear_m_s=(retract_m_s, 0.0, 0.0))
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
                _event_detail(event),
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


def _event_detail(event: Event) -> str:
    if isinstance(event, DockCommitted):
        orientation = (
            "continuous" if event.orientation_index is None else f"index {event.orientation_index}"
        )
        return f"{event.connector_a} <-> {event.connector_b} ({orientation})"
    if isinstance(event, DockCandidateDetected | UndockCommitted):
        return f"{event.connector_a} <-> {event.connector_b}"
    if isinstance(event, DockFailed):
        return f"{event.connector_a} <-> {event.connector_b}: {event.detail}"
    if isinstance(event, UndockFailed):
        return f"{event.connection_id}: {event.detail}"
    if isinstance(event, AssemblyMerged):
        return f"{' + '.join(event.merged_from)} -> {event.assembly_id}"
    if isinstance(event, AssemblySplit):
        return f"{event.source_assembly_id} -> {' + '.join(event.resulting)}"
    if isinstance(event, ConnectorOverloaded):
        return (
            f"{event.connection_id}: {event.measured_force_n:.4g} N exceeded {event.limit_n:.4g} N"
        )
    return ""


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
