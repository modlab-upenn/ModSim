"""Shared CLI option definitions and scenario argument validation.

This module centralises the argument vocabulary the scenario commands share so
that ``cli.py`` is left with parsing and dispatch. Three things live here:

* :class:`OutputFormat`, the encoding shared by every command that renders.
* A set of reusable ``Annotated`` option aliases. Each alias fixes an option's
  flag names, help text, constraints, and ``--help`` panel once; commands still
  supply their own default at the call site, so ``dock`` and ``run`` can share
  ``GravityOpt`` while disagreeing on whether gravity starts on.
* :class:`ScenarioOptions`, which bundles the ``run`` command's arguments and
  performs the finite / non-negative / "supplied together" checks that used to
  be copied inline across ``run`` and ``runtime``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import Annotated, NoReturn

import typer

from modsim.backends.registry import BACKEND_ENV_VAR, DEFAULT_BACKEND
from modsim.core.validation import require_finite, require_finite_positive
from modsim.runtime import RuntimeDemo


class OutputFormat(StrEnum):
    """Supported CLI output encodings."""

    TEXT = "text"
    JSON = "json"


# ``run --gui`` supplies its own duration per demo; headless runs default to a
# fixed window. Both are overridable with ``--duration``.
DEFAULT_HEADLESS_DURATION_S = 8.0
DEMO_DEFAULTS: dict[RuntimeDemo, float] = {
    RuntimeDemo.SMORES_SPATIAL_HANDOFF: 45.0,
    RuntimeDemo.DOCK: 4.0,
    RuntimeDemo.DOCK_UNDOCK: 6.0,
    RuntimeDemo.SMORES_DRIVER_TO_SNAKE: 14.0,
}


def option_error(message: str) -> NoReturn:
    """Report a bad-argument condition and exit with Typer's usage code."""
    typer.echo(message, err=True)
    raise typer.Exit(code=2)


def require_positive_finite(value: float, option: str) -> None:
    """Reject unsafe time arguments before a command starts a session."""
    try:
        require_finite_positive(value, option)
    except ValueError:
        option_error(f"{option} must be a finite number greater than 0.")


def require_finite_option(value: float, option: str) -> None:
    """Reject non-finite numeric arguments before they enter typed state."""
    try:
        require_finite(value, option)
    except ValueError:
        option_error(f"{option} must be a finite number.")


# --- Help epilogs ----------------------------------------------------------
# The scenario commands carry a lot of runtime flags, so each one ends its
# ``--help`` with a few copy-pasteable invocations. Rich markup (the CLI's
# default help mode) is used for light styling; single newlines are preserved,
# so command lines stay intact. ``PACK`` stands for a Robot Pack directory or
# its robot_pack.yaml.
APP_EPILOG = (
    "[bold]Common workflows[/bold]  (PACK = a Robot Pack directory or robot_pack.yaml)\n\n"
    "[dim]Check a pack loads and is well-formed[/dim]\n"
    "  modsim pack validate PACK\n\n"
    "[dim]Check its connectors latch — fast, no physics[/dim]\n"
    "  modsim dock PACK --count 3\n\n"
    "[dim]Watch a docking scenario play out in MuJoCo[/dim]\n"
    "  modsim run PACK --backend mujoco --count 2 --duration 8 --view\n\n"
    "[dim]Open the live Runtime Inspector[/dim]\n"
    "  modsim run --gui PACK --backend mujoco --demo dock_undock\n\n"
    "Every command takes --help. The flag-heavy ones (run, dock, views) group "
    "their options by purpose and list worked examples at the bottom."
)

RUN_EPILOG = (
    "[bold]Examples[/bold]  (PACK = a Robot Pack directory or robot_pack.yaml)\n\n"
    "[dim]Place 2 modules in a row, drive them together, print a report[/dim]\n"
    "  modsim run PACK --count 2 --duration 8\n\n"
    "[dim]Choose the connectors to mate; release at t=4s; JSON output[/dim]\n"
    "  modsim run PACK --fixed-connector pan --moving-connector pan "
    "--undock-at 4 --output json\n\n"
    "[dim]Watch it in the MuJoCo passive viewer, in real time[/dim]\n"
    "  modsim run PACK --backend mujoco --count 2 --duration 8 --view\n\n"
    "[dim]Open the live Runtime Inspector on a named demo[/dim]\n"
    "  modsim run --gui PACK --backend mujoco --demo dock_undock\n\n"
    "[dim]--demo, --publish-hz, --model-view, --speed and --viewer apply only with --gui; "
    "--count, --spacing, --view and --output apply only to headless runs.[/dim]"
)

DOCK_EPILOG = (
    "[bold]Examples[/bold]  (PACK = a Robot Pack directory or robot_pack.yaml)\n\n"
    "[dim]Does a 3-module row latch? (kinematic mock, no physics)[/dim]\n"
    "  modsim dock PACK --count 3\n\n"
    "[dim]Report why nothing latched instead of auto-docking[/dim]\n"
    "  modsim dock PACK --no-latch\n\n"
    "[dim]Dock, then release everything, as machine-readable JSON[/dim]\n"
    "  modsim dock PACK --count 3 --undock --output json\n\n"
    "[dim]Run against a physics backend with gravity enabled[/dim]\n"
    "  modsim dock PACK --backend mujoco --gravity"
)

VIEWS_EPILOG = (
    "[bold]Examples[/bold]  (PACK = a Robot Pack directory or robot_pack.yaml)\n\n"
    "[dim]List the model-view recipes a pack defines[/dim]\n"
    "  modsim views PACK\n\n"
    "[dim]Generate one recipe over a 3-module scene as JSON[/dim]\n"
    "  modsim views PACK --view smores_topology --count 3 --output json"
)


# --- Help panels -----------------------------------------------------------
# Grouping the fifteen-plus options into named panels turns ``--help`` from a
# flat wall into a scannable page. Panel names are shared so the same concept
# lands in the same place across commands.
PANEL_MODE = "Mode"
PANEL_SCENARIO = "Scenario"
PANEL_MOTION = "Motion"
PANEL_PHYSICS = "Physics"
PANEL_INSPECTOR = "Runtime Inspector (--gui)"
PANEL_OUTPUT = "Output"


# --- Shared option aliases -------------------------------------------------
PackPathArg = Annotated[
    Path,
    typer.Argument(
        exists=False,
        file_okay=True,
        dir_okay=True,
        readable=True,
        resolve_path=False,
        help="Robot Pack directory or robot_pack.yaml path.",
    ),
]
ModuleTypeOpt = Annotated[
    str | None,
    typer.Option(
        "--module-type",
        "-m",
        help="Module type to instantiate. Required when the pack defines more than one.",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
FixedConnectorOpt = Annotated[
    str | None,
    typer.Option(
        "--fixed-connector",
        help="Module-local connector ID on the fixed module (connector-pair scenario).",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
MovingConnectorOpt = Annotated[
    str | None,
    typer.Option(
        "--moving-connector",
        help="Module-local connector ID on the moving module (connector-pair scenario).",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
ConnectorGapOpt = Annotated[
    float | None,
    typer.Option(
        "--connector-gap",
        min=0.0,
        help="Initial connector separation. Defaults to 0.03 m headless, or the demo's value.",
        rich_help_panel=PANEL_MOTION,
    ),
]
OrientationOpt = Annotated[
    float,
    typer.Option(
        "--orientation",
        help="Requested relative roll about the docking axis in radians.",
        rich_help_panel=PANEL_MOTION,
    ),
]
ApproachOpt = Annotated[
    float,
    typer.Option(
        "--approach",
        help="Approach speed toward the docking target.",
        rich_help_panel=PANEL_MOTION,
    ),
]
RetractOpt = Annotated[
    float | None,
    typer.Option(
        "--retract",
        help="Speed away after release. Defaults to the approach speed; pass 0 to coast together.",
        rich_help_panel=PANEL_MOTION,
    ),
]
DurationOpt = Annotated[
    float | None,
    typer.Option(
        "--duration",
        help="Simulated seconds to run. Defaults to 8s headless, or a per-demo default with --gui.",
        rich_help_panel=PANEL_MOTION,
    ),
]
DtOpt = Annotated[
    float,
    typer.Option("--dt", help="Physics step size in seconds.", rich_help_panel=PANEL_MOTION),
]
UndockAtOpt = Annotated[
    float | None,
    typer.Option(
        "--undock-at",
        help="Release every connection at this simulated time; latching stops afterwards.",
        rich_help_panel=PANEL_MOTION,
    ),
]
BackendOpt = Annotated[
    str | None,
    typer.Option(
        "--backend",
        "-b",
        help=(
            f"Physics backend. Defaults to ${BACKEND_ENV_VAR} then '{DEFAULT_BACKEND}' "
            "(headless), or 'mujoco' with --gui."
        ),
        rich_help_panel=PANEL_PHYSICS,
    ),
]
GravityOpt = Annotated[
    bool,
    typer.Option(
        "--gravity/--no-gravity",
        help="Enable gravity in the physics backend. The kinematic mock has none.",
        rich_help_panel=PANEL_PHYSICS,
    ),
]
GroundOpt = Annotated[
    bool | None,
    typer.Option(
        "--ground/--no-ground",
        help="Add a ground plane at z = 0. Defaults on for physical GUI demos, off otherwise.",
        rich_help_panel=PANEL_PHYSICS,
    ),
]
OutputOpt = Annotated[
    OutputFormat,
    typer.Option(
        "--output",
        "-o",
        case_sensitive=False,
        help="Output format.",
        rich_help_panel=PANEL_OUTPUT,
    ),
]

# --- run-specific aliases --------------------------------------------------
RunDtOpt = Annotated[
    float | None,
    typer.Option(
        "--dt",
        help="Physics step in seconds. Defaults to 0.002 headless, or the demo's safe value.",
        rich_help_panel=PANEL_MOTION,
    ),
]
RunGravityOpt = Annotated[
    bool | None,
    typer.Option(
        "--gravity/--no-gravity",
        help="Enable gravity. Defaults on for physical GUI demos, off otherwise.",
        rich_help_panel=PANEL_PHYSICS,
    ),
]
RealTimeFactorOpt = Annotated[
    float,
    typer.Option(
        "--speed",
        "--real-time-factor",
        help="Wall-clock playback factor (1 = real time, 2 = twice as fast). Physics is unchanged.",
        rich_help_panel=PANEL_INSPECTOR,
    ),
]
GuiFlag = Annotated[
    bool,
    typer.Option(
        "--gui",
        help="Open the live Runtime Inspector (Qt) instead of running headless.",
        rich_help_panel=PANEL_MODE,
    ),
]
ViewFlag = Annotated[
    bool,
    typer.Option(
        "--view",
        help="Open the MuJoCo passive viewer and run in real time (headless mode; MuJoCo only).",
        rich_help_panel=PANEL_MODE,
    ),
]
DemoOpt = Annotated[
    RuntimeDemo,
    typer.Option(
        "--demo",
        case_sensitive=False,
        help="Named demo; smores_spatial_handoff also supports headless execution.",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
RunCountOpt = Annotated[
    int,
    typer.Option(
        "--count",
        "-n",
        min=2,
        help="Number of modules to place (headless grid).",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
RunSpacingOpt = Annotated[
    float,
    typer.Option(
        "--spacing",
        "-s",
        help="Initial gap between module origins (headless grid).",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
HeightOpt = Annotated[
    float | None,
    typer.Option(
        "--height",
        help="Scene height in metres. Defaults to 0 headless, or the demo's safe value.",
        rich_help_panel=PANEL_PHYSICS,
    ),
]
PublishHzOpt = Annotated[
    float,
    typer.Option(
        "--publish-hz",
        help="Maximum inspector refresh frequency (--gui).",
        rich_help_panel=PANEL_INSPECTOR,
    ),
]
ModelViewOpt = Annotated[
    str | None,
    typer.Option(
        "--model-view",
        help="Runtime model-view recipe ID (--gui). Defaults to the pack's default recipe.",
        rich_help_panel=PANEL_INSPECTOR,
    ),
]
ViewerOpt = Annotated[
    bool | None,
    typer.Option(
        "--viewer/--no-viewer",
        help="Open the native 3D viewer beside the Runtime Inspector (--gui). MuJoCo only.",
        rich_help_panel=PANEL_INSPECTOR,
    ),
]

# --- dock-specific aliases -------------------------------------------------
DockCountOpt = Annotated[
    int,
    typer.Option(
        "--count",
        "-n",
        min=1,
        help="Number of module instances to place.",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
DockSpacingOpt = Annotated[
    float,
    typer.Option(
        "--spacing",
        "-s",
        help="Distance in metres between placed modules along the x axis.",
        rich_help_panel=PANEL_SCENARIO,
    ),
]
StepsOpt = Annotated[
    int,
    typer.Option(
        "--steps",
        min=1,
        help="Number of simulation steps to run.",
        rich_help_panel=PANEL_MOTION,
    ),
]
LatchOpt = Annotated[
    bool,
    typer.Option(
        "--latch/--no-latch",
        help="Issue a dock command for every pair whose geometry already satisfies acceptance.",
        rich_help_panel=PANEL_MOTION,
    ),
]
UndockFlag = Annotated[
    bool,
    typer.Option(
        "--undock",
        help="Release every connection after the run and report the split.",
        rich_help_panel=PANEL_MOTION,
    ),
]


@dataclass(frozen=True)
class ScenarioOptions:
    """The ``run`` command's arguments, plus the checks shared by both modes.

    ``run`` drives two very different back ends — a headless physics session
    that renders a report, and the live Runtime Inspector (``--gui``) — but they
    share almost all of their arguments and every numeric guard. Bundling them
    here keeps the command body to "gather, validate, dispatch".
    """

    pack_path: Path
    demo: RuntimeDemo
    module_type: str | None
    count: int
    spacing_m: float
    fixed_connector: str | None
    moving_connector: str | None
    connector_gap_m: float
    orientation_rad: float
    approach_m_s: float
    retract_m_s: float | None
    duration_s: float | None
    dt_s: float
    undock_at_s: float | None
    backend: str | None
    gravity: bool
    ground: bool
    height_m: float
    publish_hz: float
    view_id: str | None
    viewer: bool | None
    view: bool
    gui: bool
    output: OutputFormat

    @property
    def pair_requested(self) -> bool:
        return self.fixed_connector is not None or self.moving_connector is not None

    def resolved_duration_s(self) -> float:
        """Fill an omitted ``--duration`` from the mode-appropriate default."""
        if self.duration_s is not None:
            return self.duration_s
        if self.demo is RuntimeDemo.SMORES_SPATIAL_HANDOFF:
            return DEMO_DEFAULTS[self.demo]
        return DEMO_DEFAULTS[self.demo] if self.gui else DEFAULT_HEADLESS_DURATION_S

    def validate(self) -> float:
        """Run the guards both modes share and return the resolved duration.

        Mode-specific structural checks (smores demo constraints, native-viewer
        backend rules, the connector-pair count rule) stay in their dispatch
        helper, where the resolved backend and scene are in hand.
        """
        resolved_duration_s = self.resolved_duration_s()
        require_positive_finite(resolved_duration_s, "--duration")
        require_positive_finite(self.dt_s, "--dt")

        # The Runtime Inspector drives continuously, so it treats the approach
        # speed as strictly positive; the headless scenario allows a standing
        # start (0) and only rejects negatives.
        if self.gui:
            require_positive_finite(self.approach_m_s, "--approach")
            require_positive_finite(self.publish_hz, "--publish-hz")
        else:
            require_finite_option(self.approach_m_s, "--approach")
            if self.approach_m_s < 0.0:
                option_error("--approach must not be negative.")
            require_finite_option(self.spacing_m, "--spacing")

        require_finite_option(self.connector_gap_m, "--connector-gap")
        require_finite_option(self.orientation_rad, "--orientation")
        require_finite_option(self.height_m, "--height")
        if self.connector_gap_m < 0.0:
            option_error("--connector-gap must not be negative.")
        if self.retract_m_s is not None:
            require_finite_option(self.retract_m_s, "--retract")
            if self.retract_m_s < 0.0:
                option_error("--retract must not be negative.")
        if self.undock_at_s is not None:
            require_finite_option(self.undock_at_s, "--undock-at")
            if self.undock_at_s < 0.0:
                option_error("--undock-at must not be negative.")
        if (self.fixed_connector is None) != (self.moving_connector is None):
            option_error("--fixed-connector and --moving-connector must be supplied together")
        return resolved_duration_s
