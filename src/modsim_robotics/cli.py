"""ModSim command-line interface for ``modsim-robotics``."""

from __future__ import annotations

import json
import shlex
import sys
from collections.abc import Callable
from dataclasses import asdict, replace
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console
from rich.table import Table

from modsim_robotics import __version__
from modsim_robotics.backends.base import BackendError, SupportsModuleKinematics
from modsim_robotics.backends.registry import (
    DEFAULT_BACKEND,
    describe_backends,
    resolve_backend_name,
)
from modsim_robotics.cli_options import (
    APP_EPILOG,
    DOCK_EPILOG,
    RUN_EPILOG,
    VIEWS_EPILOG,
    AngleUnitOpt,
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
    HomeOpt,
    LatchOpt,
    LengthUnitOpt,
    ModelViewOpt,
    ModuleTypeOpt,
    MovingConnectorOpt,
    OrientationOpt,
    OutputFormat,
    OutputOpt,
    PackPathArg,
    PackPathOpt,
    PublishHzOpt,
    RealTimeFactorOpt,
    RetractOpt,
    RunCountOpt,
    RunDtOpt,
    RunGravityOpt,
    RunSpacingOpt,
    ScenarioOptions,
    StepsOpt,
    TimeUnitOpt,
    UndockAtOpt,
    UndockFlag,
    UnitAwareCommand,
    ViewerOpt,
    ViewFlag,
    option_error,
)
from modsim_robotics.cli_options import require_finite_option as _require_finite
from modsim_robotics.cli_options import require_positive_finite as _require_positive_finite
from modsim_robotics.cli_render import render_docking_session as _render_docking_session
from modsim_robotics.cli_render import render_generated_model_view as _render_generated_model_view
from modsim_robotics.cli_render import render_model_view_catalog as _render_model_view_catalog
from modsim_robotics.cli_render import render_report as _render_report
from modsim_robotics.core.events import Event
from modsim_robotics.core.ids import ModuleInstanceId, connector_instance_id
from modsim_robotics.core.scene import SceneError, SceneSpec
from modsim_robotics.core.state import WorldState
from modsim_robotics.core.transforms import Vec3, vec_scale
from modsim_robotics.importers import DraftPackBuilder
from modsim_robotics.model_views import (
    ModelViewContext,
    ModelViewError,
    ModelViewFactory,
)
from modsim_robotics.robot_packs import (
    LoadedRobotPack,
    ModelViewMode,
    RobotPack,
    RobotPackLoader,
    RobotPackLoadError,
    RobotPackValidator,
    ValidationProfile,
    ValidationReport,
)
from modsim_robotics.runtime import RuntimeDemo, RuntimeInspectorConfig
from modsim_robotics.runtime.momentum_pivot import MAX_MOMENTUM_TIMESTEP_S
from modsim_robotics.runtime.physics_docking import MAX_PHYSICAL_TIMESTEP_S
from modsim_robotics.runtime.reconfiguration import (
    ReconfigurationPlanError,
    ReconfigurationScenarioError,
    ScriptedReconfigurationConfig,
    ScriptedReconfigurationScenario,
    connector_pair_plan,
)
from modsim_robotics.runtime.session import RuntimeSession
from modsim_robotics.unit_settings import (
    UnitPreferencesError,
    load_unit_preferences,
    save_unit_preferences,
)
from modsim_robotics.units import QuantityKind, UnitError, UnitPreferences, get_units
from modsim_robotics.user_data import (
    UserDataError,
    initialize_user_home,
    list_robot_packs,
    resolve_robot_pack,
)

app = typer.Typer(
    name="modsim-robotics",
    help="Define and validate modular-robot Robot Packs.",
    epilog=APP_EPILOG,
    no_args_is_help=True,
    pretty_exceptions_enable=False,
)
pack_app = typer.Typer(
    name="pack",
    help="List, inspect, and validate Robot Packs.",
    no_args_is_help=True,
)
app.add_typer(pack_app)
units_app = typer.Typer(
    name="units", help="Inspect or save local input/display unit preferences.", no_args_is_help=True
)
app.add_typer(units_app)


def _display_preferences(
    home: Path | None,
    length: str | None,
    angle: str | None,
    time: str | None,
    *,
    read_saved: bool = True,
) -> UnitPreferences:
    """Resolve presentation choices without changing canonical numeric inputs."""
    try:
        preferences = load_unit_preferences(home) if read_saved else UnitPreferences()
        changes = {
            name: value
            for name, value in (("length_unit", length), ("angle_unit", angle), ("time_unit", time))
            if value is not None
        }
        return replace(preferences, **changes)
    except (UnitError, UnitPreferencesError) as error:
        option_error(str(error))


def _render_unit_preferences(
    preferences: UnitPreferences, output: OutputFormat, *, supported: bool = False
) -> None:
    if output is OutputFormat.JSON:
        typer.echo(
            json.dumps(
                {
                    "preferences": asdict(preferences),
                    "effective": {kind.value: preferences.unit_for(kind) for kind in QuantityKind},
                    **(
                        {
                            "supported": {
                                kind.value: [unit.symbol for unit in get_units(kind)]
                                for kind in QuantityKind
                            }
                        }
                        if supported
                        else {}
                    ),
                },
                indent=2,
            )
        )
        return
    table = Table(title="Local display units (simulation and pack data remain SI)")
    table.add_column("Quantity")
    table.add_column("Display unit")
    if supported:
        table.add_column("Supported choices")
    for kind in QuantityKind:
        row = [kind.value.replace("_", " "), preferences.unit_for(kind)]
        if supported:
            row.append(", ".join(unit.symbol for unit in get_units(kind)))
        table.add_row(*row)
    Console().print(table)


@units_app.command("show")
def show_units_command(
    home: HomeOpt = None,
    output: OutputOpt = OutputFormat.TEXT,
    supported: Annotated[
        bool, typer.Option("--supported", help="Also list supported units for every quantity.")
    ] = False,
) -> None:
    """Show local display choices without creating or changing any files.

    Args:
        home: ModSim data home, or the environment/default home when omitted.
        output: Human-readable table or a JSON preference/effective-unit mapping.
        supported: Include every supported unit choice for each quantity.

    Raises:
        typer.Exit: Preferences cannot be read or validated.
    """
    preferences = _display_preferences(home, None, None, None)
    _render_unit_preferences(preferences, output, supported=supported)


@units_app.command("set")
def set_units_command(
    home: HomeOpt = None,
    length_unit: LengthUnitOpt = None,
    angle_unit: AngleUnitOpt = None,
    time_unit: TimeUnitOpt = None,
    mass_unit: Annotated[str | None, typer.Option("--mass-unit")] = None,
    force_unit: Annotated[str | None, typer.Option("--force-unit")] = None,
    torque_unit: Annotated[str | None, typer.Option("--torque-unit")] = None,
    inertia_unit: Annotated[str | None, typer.Option("--inertia-unit")] = None,
    linear_velocity_unit: Annotated[
        str | None,
        typer.Option("--linear-velocity-unit", help="Unit override, or auto to follow length."),
    ] = None,
    angular_velocity_unit: Annotated[
        str | None,
        typer.Option("--angular-velocity-unit", help="Unit override, or auto to follow angles."),
    ] = None,
    linear_acceleration_unit: Annotated[
        str | None,
        typer.Option("--linear-acceleration-unit", help="Unit override, or auto to follow length."),
    ] = None,
    linear_stiffness_unit: Annotated[str | None, typer.Option("--linear-stiffness-unit")] = None,
    angular_stiffness_unit: Annotated[str | None, typer.Option("--angular-stiffness-unit")] = None,
    linear_damping_unit: Annotated[str | None, typer.Option("--linear-damping-unit")] = None,
    angular_damping_unit: Annotated[str | None, typer.Option("--angular-damping-unit")] = None,
    frequency_unit: Annotated[str | None, typer.Option("--frequency-unit")] = None,
    reset: Annotated[bool, typer.Option("--reset", help="Start from default preferences.")] = False,
    output: OutputOpt = OutputFormat.TEXT,
) -> None:
    """Save explicit local display choices while keeping Robot Packs untouched.

    Args:
        home: ModSim data home, or the environment/default home when omitted.
        length_unit: Optional supported length display unit.
        angle_unit: Optional supported angle display unit.
        time_unit: Optional supported time display unit.
        mass_unit: Optional supported mass display unit.
        force_unit: Optional supported force display unit.
        torque_unit: Optional supported torque display unit.
        inertia_unit: Optional supported moment-of-inertia display unit.
        linear_velocity_unit: Optional linear-speed override; auto restores derivation.
        angular_velocity_unit: Optional angular-speed override; auto restores derivation.
        linear_acceleration_unit: Optional acceleration override; auto restores derivation.
        linear_stiffness_unit: Optional linear stiffness display unit.
        angular_stiffness_unit: Optional rotational stiffness display unit.
        linear_damping_unit: Optional linear damping display unit.
        angular_damping_unit: Optional rotational damping display unit.
        frequency_unit: Optional frequency display unit.
        reset: Start from defaults rather than merging current preferences.
        output: Human-readable table or JSON describing the resulting choices.

    Raises:
        typer.Exit: No change is requested, a unit is incompatible, or persistence fails.
    """
    choices = {
        "length_unit": length_unit,
        "angle_unit": angle_unit,
        "time_unit": time_unit,
        "mass_unit": mass_unit,
        "force_unit": force_unit,
        "torque_unit": torque_unit,
        "inertia_unit": inertia_unit,
        "linear_velocity_unit": linear_velocity_unit,
        "angular_velocity_unit": angular_velocity_unit,
        "linear_acceleration_unit": linear_acceleration_unit,
        "linear_stiffness_unit": linear_stiffness_unit,
        "angular_stiffness_unit": angular_stiffness_unit,
        "linear_damping_unit": linear_damping_unit,
        "angular_damping_unit": angular_damping_unit,
        "frequency_unit": frequency_unit,
    }
    changes = {name: value for name, value in choices.items() if value is not None}
    if not changes and not reset:
        option_error("Choose a unit option or use --reset; no preferences were changed.")
    try:
        current = UnitPreferences() if reset else load_unit_preferences(home)
        normalized: dict[str, str | None] = dict(changes)
        for name in ("linear_velocity_unit", "angular_velocity_unit", "linear_acceleration_unit"):
            if normalized.get(name) == "auto":
                normalized[name] = None
        preferences = replace(current, **normalized)
        save_unit_preferences(preferences, home)
    except (UnitError, UnitPreferencesError) as error:
        option_error(str(error))
    _render_unit_preferences(preferences, output)


def _version_callback(value: bool) -> None:
    if value:
        typer.echo(f"modsim-robotics {__version__}")
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


@app.command("setup")
def setup_command(
    home: HomeOpt = None,
    output: OutputOpt = OutputFormat.TEXT,
) -> None:
    """Prepare user data and the bundled Generic Cube pack for an offline first run."""
    try:
        result = initialize_user_home(home)
    except UserDataError as error:
        typer.echo(str(error), err=True)
        raise typer.Exit(code=1) from error
    if output is OutputFormat.JSON:
        typer.echo(json.dumps(result.as_dict(), indent=2))
    else:
        typer.echo(f"ModSim home: {result.home}")
        status = "Installed" if result.created else "Kept existing"
        typer.echo(f"{status} starter Robot Pack: {result.starter_pack}")
        typer.echo(f"Configuration: {result.home / 'config.toml'}")
        typer.echo(
            "Choose a pack by name or path for each command; 'pack list' shows installed names."
        )
        _print_home_environment_instructions(result.home)


def _print_home_environment_instructions(home: Path) -> None:
    """Show opt-in persistence commands without changing profiles or environments."""
    if sys.platform == "win32":
        literal = "'" + str(home).replace("'", "''") + "'"
        typer.echo("Set MODSIM_PATH for this PowerShell session:")
        typer.echo(f"  $env:MODSIM_PATH = {literal}")
        typer.echo("Persist MODSIM_PATH in the Windows User environment for future terminals:")
        typer.echo(f"  [Environment]::SetEnvironmentVariable('MODSIM_PATH', {literal}, 'User')")
    else:
        export = f"export MODSIM_PATH={shlex.quote(str(home))}"
        typer.echo("Set MODSIM_PATH for this terminal:")
        typer.echo(f"  {export}")
        typer.echo("Persist it in the profile for your shell (choose one):")
        typer.echo(f"  printf '%s\\n' {shlex.quote(export)} >> ~/.bashrc")
        typer.echo(f"  printf '%s\\n' {shlex.quote(export)} >> ~/.zshrc")


def _resolve_cli_pack(
    pack_path: str | Path | None, pack_override: str | Path | None, home: Path | None
) -> Path:
    """Select a pack once, leaving explicit pack validation to each command's loader."""
    if pack_path is not None and pack_override is not None:
        option_error("Supply the Robot Pack name or path positionally or with --pack, not both.")
    try:
        return resolve_robot_pack(pack_path if pack_path is not None else pack_override, home=home)
    except UserDataError as error:
        typer.echo(str(error), err=True)
        raise typer.Exit(code=1) from error


@app.command("studio", help="Open a blank Studio workspace, or load a selected RobotPack.")
def studio_command(
    pack_path: PackPathArg = None,
    pack_override: PackPathOpt = None,
    home: HomeOpt = None,
) -> None:
    """Launch the Builder with an empty workspace or a selected RobotPack.

    Args:
        pack_path: Installed pack ID, version-qualified ID, or explicit path.
            None opens the welcome screen unless pack_override is supplied.
        pack_override: Alternative pack selector supplied with --pack. It cannot
            be combined with pack_path.
        home: ModSim home used for display preferences and named pack lookup.
            Display preferences also apply to blank launches and explicit pack
            paths; a blank launch does not read or initialize its RobotPack library.

    Raises:
        typer.Exit: After the application exits, or when selection or missing
            Studio dependencies prevent startup.
    """
    resolved_pack = (
        _resolve_cli_pack(pack_path, pack_override, home)
        if pack_path is not None or pack_override is not None
        else None
    )
    try:
        from modsim_robotics_studio.app import main as studio_main
    except ImportError as error:
        typer.echo(
            "ModSim Studio dependencies are not installed. "
            "Install with: pip install modsim-robotics",
            err=True,
        )
        raise typer.Exit(code=2) from error
    raise typer.Exit(code=studio_main(resolved_pack, home=home))


@app.command("runtime", cls=UnitAwareCommand)
def runtime_command(
    pack_path: PackPathArg = None,
    demo: Annotated[
        RuntimeDemo,
        typer.Option(
            "--demo",
            case_sensitive=False,
            help=(
                "Named scenario: dock, dock_undock, physical SMORES differential-drive "
                "docking, scripted/physical seven-module Driver-to-Snake "
                "reconfiguration, or the five-module, momentum-pivot, and twelve-module "
                "M-Blocks demonstrations."
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
    connector_gap_m: ConnectorGapOpt = None,
    orientation_rad: OrientationOpt = 0.0,
    approach_m_s: ApproachOpt = 0.03,
    retract_m_s: RetractOpt = None,
    duration_s: DurationOpt = None,
    dt_s: RunDtOpt = None,
    undock_at_s: UndockAtOpt = None,
    backend: Annotated[
        str,
        typer.Option("--backend", "-b", help="Physics backend used by the runtime owner."),
    ] = "mujoco",
    gravity: Annotated[
        bool | None,
        typer.Option(
            "--gravity/--no-gravity",
            help=("Enable gravity. Defaults on for physical demos and off for kinematic demos."),
        ),
    ] = None,
    ground: Annotated[
        bool | None,
        typer.Option(
            "--ground/--no-ground",
            help=("Add a ground plane at z = 0. Defaults on for physical demos and off otherwise."),
        ),
    ] = None,
    height_m: HeightOpt = None,
    view_id: Annotated[
        str | None,
        typer.Option(
            "--model-view",
            help="Runtime model-view recipe ID. Defaults to the pack's default recipe.",
        ),
    ] = None,
    publish_hz: PublishHzOpt = 20.0,
    real_time_factor: RealTimeFactorOpt = 1.0,
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
    pack_override: PackPathOpt = None,
    home: HomeOpt = None,
    length_unit: LengthUnitOpt = None,
    angle_unit: AngleUnitOpt = None,
    time_unit: TimeUnitOpt = None,
) -> None:
    """Open live semantic and optional native 3D views of a runtime demo."""
    preferences = (
        _display_preferences(home, length_unit, angle_unit, time_unit)
        if any(value is not None for value in (length_unit, angle_unit, time_unit))
        else None
    )
    resolved_pack = _resolve_cli_pack(pack_path, pack_override, home)
    physical_pair = demo is RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK
    physical_reconfiguration = demo in {
        RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE,
        RuntimeDemo.SMORES_ONLINE_ASSEMBLY,
        RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE,
    }
    mblocks_kinematic_pivot = demo is RuntimeDemo.MBLOCKS_FIVE_MODULE_PIVOT
    mblocks_momentum_pivot = demo is RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT
    mblocks_twelve_module_line = demo is RuntimeDemo.MBLOCKS_TWELVE_MODULE_LINE
    mblocks_physical_twelve_module_line = demo is RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE
    mblocks_twelve_module_staircase = demo is RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE
    mblocks_physical_twelve_module_staircase = (
        demo is RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE
    )
    mblocks_kinematic = (
        mblocks_kinematic_pivot or mblocks_twelve_module_line or mblocks_twelve_module_staircase
    )
    physical_mblocks_sequence = (
        mblocks_physical_twelve_module_line
        or mblocks_physical_twelve_module_staircase
        or demo
        in {
            RuntimeDemo.MBLOCKS_ONLINE_LATTICE,
            RuntimeDemo.MBLOCKS_ONLINE_LATTICE_LARGE,
            RuntimeDemo.MBLOCKS_ONLINE_LATTICE_ELBOW,
        }
    )
    physical_mblocks = mblocks_momentum_pivot or physical_mblocks_sequence
    physical_smores = physical_pair or physical_reconfiguration
    physical_demo = physical_smores or physical_mblocks
    resolved_duration_s = (
        duration_s
        if duration_s is not None
        else {
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
        }[demo]
    )
    if dt_s is not None:
        resolved_dt_s = dt_s
    elif physical_mblocks_sequence:
        resolved_dt_s = MAX_MOMENTUM_TIMESTEP_S
    elif physical_mblocks:
        resolved_dt_s = 0.00025
    else:
        resolved_dt_s = 0.002
    resolved_gravity = physical_demo if gravity is None else gravity
    resolved_ground = physical_demo if ground is None else ground
    if height_m is None:
        resolved_height_m = (
            0.025
            if physical_mblocks or mblocks_twelve_module_staircase
            else (0.05 if physical_smores else 0.0)
        )
    else:
        resolved_height_m = height_m
    _require_positive_finite(resolved_duration_s, "--duration")
    _require_positive_finite(resolved_dt_s, "--dt")
    _require_positive_finite(approach_m_s, "--approach")
    _require_positive_finite(publish_hz, "--publish-hz")
    _require_positive_finite(real_time_factor, "--speed")
    if connector_gap_m is not None:
        _require_finite(connector_gap_m, "--connector-gap")
    _require_finite(orientation_rad, "--orientation")
    _require_finite(resolved_height_m, "--height")
    if connector_gap_m is not None and connector_gap_m < 0.0:
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
        if resolved_gravity or resolved_ground:
            typer.echo(
                "--demo smores_driver_to_snake currently requires --no-gravity and no "
                "ground plane; supported locomotion is not implemented yet.",
                err=True,
            )
            raise typer.Exit(code=2)
    if mblocks_kinematic:
        demo_name = demo.value
        if fixed_connector is not None or moving_connector is not None:
            typer.echo(
                f"--demo {demo_name} defines its connector actions; "
                "do not supply --fixed-connector or --moving-connector.",
                err=True,
            )
            raise typer.Exit(code=2)
        if undock_at_s is not None:
            typer.echo(
                f"--demo {demo_name} defines its own releases; do not supply --undock-at.",
                err=True,
            )
            raise typer.Exit(code=2)
        if connector_gap_m is not None:
            typer.echo(
                f"--demo {demo_name} defines exact edge-pivot routes; "
                "do not supply --connector-gap.",
                err=True,
            )
            raise typer.Exit(code=2)
        if retract_m_s is not None:
            typer.echo(
                f"--demo {demo_name} defines exact edge-pivot routes; do not supply --retract.",
                err=True,
            )
            raise typer.Exit(code=2)
        if orientation_rad != 0.0:
            typer.echo(
                f"--demo {demo_name} defines its face orientations; do not supply --orientation.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_gravity or resolved_ground:
            typer.echo(
                f"--demo {demo_name} is explicitly kinematic; "
                "do not enable gravity or the ground plane.",
                err=True,
            )
            raise typer.Exit(code=2)
    if mblocks_momentum_pivot:
        if fixed_connector is not None or moving_connector is not None:
            typer.echo(
                "--demo mblocks_momentum_pivot defines its face and edge connector "
                "transitions; do not supply --fixed-connector or --moving-connector.",
                err=True,
            )
            raise typer.Exit(code=2)
        if undock_at_s is not None:
            typer.echo(
                "--demo mblocks_momentum_pivot controls its own releases; "
                "do not supply --undock-at.",
                err=True,
            )
            raise typer.Exit(code=2)
        if connector_gap_m is not None:
            typer.echo(
                "--demo mblocks_momentum_pivot defines its initial face-connected scene; "
                "do not supply --connector-gap.",
                err=True,
            )
            raise typer.Exit(code=2)
        if retract_m_s is not None:
            typer.echo(
                "--demo mblocks_momentum_pivot is flywheel-driven; do not supply --retract.",
                err=True,
            )
            raise typer.Exit(code=2)
        if orientation_rad != 0.0:
            typer.echo(
                "--demo mblocks_momentum_pivot defines its cube orientation; "
                "do not supply --orientation.",
                err=True,
            )
            raise typer.Exit(code=2)
        if backend != "mujoco":
            typer.echo(
                "--demo mblocks_momentum_pivot requires --backend mujoco.",
                err=True,
            )
            raise typer.Exit(code=2)
        if not resolved_gravity or not resolved_ground:
            typer.echo(
                "--demo mblocks_momentum_pivot requires gravity and ground; "
                "do not pass --no-gravity or --no-ground.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_height_m < 0.025:
            typer.echo(
                "--demo mblocks_momentum_pivot requires --height >= 0.025.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_dt_s > MAX_MOMENTUM_TIMESTEP_S:
            typer.echo(
                "--demo mblocks_momentum_pivot requires --dt <= "
                f"{MAX_MOMENTUM_TIMESTEP_S:g} for its flywheel brake impulse.",
                err=True,
            )
            raise typer.Exit(code=2)
    if physical_mblocks_sequence:
        demo_name = demo.value
        if fixed_connector is not None or moving_connector is not None:
            typer.echo(
                f"--demo {demo_name} defines its connector "
                "transitions; do not supply --fixed-connector or --moving-connector.",
                err=True,
            )
            raise typer.Exit(code=2)
        if undock_at_s is not None:
            typer.echo(
                f"--demo {demo_name} controls its own releases; do not supply --undock-at.",
                err=True,
            )
            raise typer.Exit(code=2)
        if connector_gap_m is not None:
            typer.echo(
                f"--demo {demo_name} defines its initial scene; do not supply --connector-gap.",
                err=True,
            )
            raise typer.Exit(code=2)
        if retract_m_s is not None:
            typer.echo(
                f"--demo {demo_name} is flywheel-driven; do not supply --retract.",
                err=True,
            )
            raise typer.Exit(code=2)
        if orientation_rad != 0.0:
            typer.echo(
                f"--demo {demo_name} defines its cube orientations; do not supply --orientation.",
                err=True,
            )
            raise typer.Exit(code=2)
        if backend != "mujoco":
            typer.echo(
                f"--demo {demo_name} requires --backend mujoco.",
                err=True,
            )
            raise typer.Exit(code=2)
        if not resolved_gravity or not resolved_ground:
            typer.echo(
                f"--demo {demo_name} requires gravity and ground; "
                "do not pass --no-gravity or --no-ground.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_height_m < 0.025:
            typer.echo(
                f"--demo {demo_name} requires --height >= 0.025.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_dt_s > MAX_MOMENTUM_TIMESTEP_S:
            typer.echo(
                f"--demo {demo_name} requires --dt <= "
                f"{MAX_MOMENTUM_TIMESTEP_S:g} for its flywheel brake impulses.",
                err=True,
            )
            raise typer.Exit(code=2)
    if physical_pair:
        if fixed_connector is not None or moving_connector is not None:
            typer.echo(
                "--demo smores_diff_drive_dock_undock uses fixed bottom and moving pan; "
                "do not supply --fixed-connector or --moving-connector.",
                err=True,
            )
            raise typer.Exit(code=2)
        if undock_at_s is not None:
            typer.echo(
                "--demo smores_diff_drive_dock_undock performs its own release; "
                "do not supply --undock-at.",
                err=True,
            )
            raise typer.Exit(code=2)
        if orientation_rad != 0.0:
            typer.echo(
                "--demo smores_diff_drive_dock_undock keeps both modules upright; "
                "do not supply --orientation.",
                err=True,
            )
            raise typer.Exit(code=2)
        if backend != "mujoco":
            typer.echo(
                "--demo smores_diff_drive_dock_undock requires --backend mujoco.",
                err=True,
            )
            raise typer.Exit(code=2)
        if not resolved_gravity or not resolved_ground:
            typer.echo(
                "--demo smores_diff_drive_dock_undock requires gravity and ground; "
                "do not pass --no-gravity or --no-ground.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_height_m < 0.04:
            typer.echo(
                "--demo smores_diff_drive_dock_undock requires --height >= 0.04.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_dt_s > MAX_PHYSICAL_TIMESTEP_S:
            typer.echo(
                "--demo smores_diff_drive_dock_undock requires --dt <= "
                f"{MAX_PHYSICAL_TIMESTEP_S:g} for its tuned contact/controller model.",
                err=True,
            )
            raise typer.Exit(code=2)
        if retract_m_s == 0.0:
            typer.echo(
                "--demo smores_diff_drive_dock_undock requires --retract > 0.",
                err=True,
            )
            raise typer.Exit(code=2)
    if physical_reconfiguration:
        if fixed_connector is not None or moving_connector is not None:
            typer.echo(
                f"--demo {demo.value} defines its connector actions; "
                "do not supply --fixed-connector or --moving-connector.",
                err=True,
            )
            raise typer.Exit(code=2)
        if undock_at_s is not None:
            typer.echo(
                f"--demo {demo.value} defines its own releases; do not supply --undock-at.",
                err=True,
            )
            raise typer.Exit(code=2)
        if connector_gap_m is not None:
            typer.echo(
                f"--demo {demo.value} defines its initial "
                "seven-module staging; do not supply --connector-gap.",
                err=True,
            )
            raise typer.Exit(code=2)
        if retract_m_s is not None:
            typer.echo(
                f"--demo {demo.value} defines its wheel-driven routes; do not supply --retract.",
                err=True,
            )
            raise typer.Exit(code=2)
        if orientation_rad != 0.0:
            typer.echo(
                f"--demo {demo.value} keeps the staged modules "
                "upright; do not supply --orientation.",
                err=True,
            )
            raise typer.Exit(code=2)
        if backend != "mujoco":
            typer.echo(
                f"--demo {demo.value} requires --backend mujoco.",
                err=True,
            )
            raise typer.Exit(code=2)
        if not resolved_gravity or not resolved_ground:
            typer.echo(
                f"--demo {demo.value} requires gravity and ground; "
                "do not pass --no-gravity or --no-ground.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_height_m < 0.04:
            typer.echo(
                f"--demo {demo.value} requires --height >= 0.04.",
                err=True,
            )
            raise typer.Exit(code=2)
        if resolved_dt_s > MAX_PHYSICAL_TIMESTEP_S:
            typer.echo(
                f"--demo {demo.value} requires --dt <= "
                f"{MAX_PHYSICAL_TIMESTEP_S:g} for its tuned contact/controller model.",
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
        from modsim_robotics_studio.runtime_app import main as runtime_main
    except ImportError as error:
        typer.echo(
            "Runtime Inspector dependencies are not installed. Install with: "
            "pip install modsim-robotics",
            err=True,
        )
        raise typer.Exit(code=2) from error

    config = RuntimeInspectorConfig(
        pack_path=resolved_pack,
        demo=demo,
        module_type=module_type,
        backend=backend,
        fixed_connector=fixed_connector,
        moving_connector=moving_connector,
        connector_gap_m=(
            None
            if physical_reconfiguration or mblocks_kinematic or physical_mblocks
            else (0.02 if connector_gap_m is None else connector_gap_m)
        ),
        orientation_rad=orientation_rad,
        approach_m_s=approach_m_s,
        retract_m_s=retract_m_s,
        duration_s=resolved_duration_s,
        dt_s=resolved_dt_s,
        undock_at_s=undock_at_s,
        gravity=resolved_gravity,
        ground=resolved_ground,
        height_m=resolved_height_m,
        view_id=view_id,
        publish_hz=publish_hz,
        viewer_enabled=viewer_enabled,
        real_time_factor=real_time_factor,
    )
    raise typer.Exit(code=runtime_main(config, home=home, preferences=preferences))


@pack_app.command("list")
def list_packs_command(
    home: HomeOpt = None,
    output: OutputOpt = OutputFormat.TEXT,
) -> None:
    """List installed Robot Packs by manifest ID, version, and filesystem path."""
    try:
        packs = list_robot_packs(home=home)
    except UserDataError as error:
        typer.echo(str(error), err=True)
        raise typer.Exit(code=1) from error
    if output is OutputFormat.JSON:
        typer.echo(json.dumps({"packs": [pack.as_dict() for pack in packs]}, indent=2))
        return
    table = Table(title="Installed Robot Packs")
    table.add_column("ID", no_wrap=True)
    table.add_column("Version", no_wrap=True)
    table.add_column("Path")
    for pack in packs:
        table.add_row(pack.id, pack.version, str(pack.path))
    console = Console(width=160, markup=False)
    console.print(table)
    if not packs:
        console.print("No Robot Packs are installed in this library.")


@pack_app.command("validate")
def validate_pack_command(
    pack_path: PackPathArg = None,
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
    pack_override: PackPathOpt = None,
    home: HomeOpt = None,
) -> None:
    """Validate the structure, local assets, and semantic references of a Robot Pack."""
    resolved_pack = _resolve_cli_pack(pack_path, pack_override, home)
    try:
        loaded = RobotPackLoader().load(resolved_pack)
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
    pack_path: PackPathArg = None,
    output: Annotated[
        OutputFormat,
        typer.Option(
            "--output",
            "-o",
            case_sensitive=False,
            help="Output format.",
        ),
    ] = OutputFormat.TEXT,
    pack_override: PackPathOpt = None,
    home: HomeOpt = None,
) -> None:
    """Show a loaded Robot Pack's aggregate contents and counts."""
    resolved_pack = _resolve_cli_pack(pack_path, pack_override, home)
    try:
        loaded = RobotPackLoader().load(resolved_pack)
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


@app.command("views", epilog=VIEWS_EPILOG, cls=UnitAwareCommand)
def model_views_command(
    pack_path: PackPathArg = None,
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
    spacing_m: DockSpacingOpt = 0.1,
    output: Annotated[
        OutputFormat,
        typer.Option("--output", "-o", case_sensitive=False, help="Output format."),
    ] = OutputFormat.TEXT,
    pack_override: PackPathOpt = None,
    home: HomeOpt = None,
    length_unit: LengthUnitOpt = None,
    angle_unit: AngleUnitOpt = None,
    time_unit: TimeUnitOpt = None,
) -> None:
    """List model-view recipes or generate one physics-free runtime snapshot."""
    preferences = _display_preferences(
        home, length_unit, angle_unit, time_unit, read_saved=output is not OutputFormat.JSON
    )
    resolved_pack = _resolve_cli_pack(pack_path, pack_override, home)
    try:
        loaded = RobotPackLoader().load(resolved_pack)
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
    _render_generated_model_view(generated, output=output, units=preferences)


@app.command("dock", epilog=DOCK_EPILOG, cls=UnitAwareCommand)
def dock_command(
    pack_path: PackPathArg = None,
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
    pack_override: PackPathOpt = None,
    home: HomeOpt = None,
    length_unit: LengthUnitOpt = None,
    angle_unit: AngleUnitOpt = None,
    time_unit: TimeUnitOpt = None,
) -> None:
    """Run a docking session to check whether a Robot Pack's connectors mate.

    This exercises the real semantic pipeline (compatibility, acceptance regions,
    guards, two-phase commit) against the selected backend. On the kinematic mock
    it runs no physics, so it answers "is this pack authored such that these
    connectors would latch?" rather than "will this robot work?".
    """
    preferences = _display_preferences(
        home, length_unit, angle_unit, time_unit, read_saved=output is not OutputFormat.JSON
    )
    resolved_pack = _resolve_cli_pack(pack_path, pack_override, home)
    _require_positive_finite(dt_s, "--dt")
    _require_finite(spacing_m, "--spacing")

    try:
        loaded = RobotPackLoader().load(resolved_pack)
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
        session, tuple(events), output=output, latch=latch, backend=backend_name, units=preferences
    )


@app.command("run", epilog=RUN_EPILOG, cls=UnitAwareCommand)
def run_command(
    pack_path: PackPathArg = None,
    gui: GuiFlag = False,
    view: ViewFlag = False,
    demo: DemoOpt = RuntimeDemo.DOCK,
    module_type: ModuleTypeOpt = None,
    count: RunCountOpt = 2,
    spacing_m: RunSpacingOpt = 0.15,
    fixed_connector: FixedConnectorOpt = None,
    moving_connector: MovingConnectorOpt = None,
    connector_gap_m: ConnectorGapOpt = None,
    orientation_rad: OrientationOpt = 0.0,
    approach_m_s: ApproachOpt = 0.03,
    retract_m_s: RetractOpt = None,
    duration_s: DurationOpt = None,
    dt_s: RunDtOpt = None,
    undock_at_s: UndockAtOpt = None,
    backend: BackendOpt = None,
    gravity: RunGravityOpt = None,
    ground: GroundOpt = None,
    height_m: HeightOpt = None,
    publish_hz: PublishHzOpt = 20.0,
    real_time_factor: RealTimeFactorOpt = 1.0,
    view_id: ModelViewOpt = None,
    viewer: ViewerOpt = None,
    output: OutputOpt = OutputFormat.TEXT,
    pack_override: PackPathOpt = None,
    home: HomeOpt = None,
    length_unit: LengthUnitOpt = None,
    angle_unit: AngleUnitOpt = None,
    time_unit: TimeUnitOpt = None,
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
    seven-module smores_driver_to_snake reconfiguration.
    """
    if gui and view:
        option_error(
            "Use --view for the headless MuJoCo viewer or --gui for the Runtime "
            "Inspector, not both."
        )
    if gui:
        # Share demo defaults and validation with `modsim-robotics runtime`; applying
        # headless defaults first would override the physical demos' settings.
        runtime_command(
            pack_path=pack_path,
            pack_override=pack_override,
            home=home,
            demo=demo,
            module_type=module_type,
            fixed_connector=fixed_connector,
            moving_connector=moving_connector,
            connector_gap_m=connector_gap_m,
            orientation_rad=orientation_rad,
            approach_m_s=approach_m_s,
            retract_m_s=retract_m_s,
            duration_s=duration_s,
            dt_s=dt_s,
            undock_at_s=undock_at_s,
            backend="mujoco" if backend is None else backend,
            gravity=gravity,
            ground=ground,
            height_m=height_m,
            view_id=view_id,
            publish_hz=publish_hz,
            real_time_factor=real_time_factor,
            viewer=viewer,
            length_unit=length_unit,
            angle_unit=angle_unit,
            time_unit=time_unit,
        )
        return
    preferences = _display_preferences(
        home, length_unit, angle_unit, time_unit, read_saved=output is not OutputFormat.JSON
    )
    resolved_pack = _resolve_cli_pack(pack_path, pack_override, home)
    options = ScenarioOptions(
        pack_path=resolved_pack,
        demo=demo,
        module_type=module_type,
        count=count,
        spacing_m=spacing_m,
        fixed_connector=fixed_connector,
        moving_connector=moving_connector,
        connector_gap_m=0.03 if connector_gap_m is None else connector_gap_m,
        orientation_rad=orientation_rad,
        approach_m_s=approach_m_s,
        retract_m_s=retract_m_s,
        duration_s=duration_s,
        dt_s=0.002 if dt_s is None else dt_s,
        undock_at_s=undock_at_s,
        backend=backend,
        gravity=False if gravity is None else gravity,
        ground=False if ground is None else ground,
        height_m=0.0 if height_m is None else height_m,
        publish_hz=publish_hz,
        view_id=view_id,
        viewer=viewer,
        view=view,
        gui=gui,
        output=output,
        display_units=preferences,
    )
    resolved_duration_s = options.validate()
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


def _run_headless_scenario(options: ScenarioOptions, resolved_duration_s: float) -> None:
    """Run a scripted scenario against a physics backend and render a report."""
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
            from modsim_robotics_backend_mujoco.viewer import run_with_viewer
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
        session,
        events,
        output=options.output,
        latch=True,
        backend=backend_name,
        units=options.display_units,
    )
