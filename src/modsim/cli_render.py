"""Rich/JSON rendering helpers for the ModSim CLI.

These functions turn validation reports, docking sessions, and model-view
results into either a machine-readable JSON payload or a Rich console view.
They are kept out of ``cli.py`` so that module is only argument parsing and
dispatch.
"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

import typer
from rich.console import Console
from rich.table import Table

from modsim.cli_options import OutputFormat
from modsim.model_views import ModuleTopologyGraphView
from modsim.robot_packs import Severity
from modsim.runtime.inspection import format_event_detail

if TYPE_CHECKING:
    from modsim.core.events import Event
    from modsim.model_views import ModelView, ModelViewFactory
    from modsim.robot_packs import (
        RobotPack,
        ValidationIssue,
        ValidationProfile,
        ValidationReport,
    )
    from modsim.runtime.session import RuntimeSession


def render_model_view_catalog(
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


def render_generated_model_view(generated: ModelView, *, output: OutputFormat) -> None:
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


def render_docking_session(
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


def render_report(
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
