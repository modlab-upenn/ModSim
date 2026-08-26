# Runtime Inspector

The Runtime Inspector is ModSim Studio's first live runtime visualization. It
runs a named demonstration and displays ModSim's semantic state:

- each module instance is a stable graph node;
- each committed docking connection is an undirected graph edge;
- disconnected modules remain visible;
- parallel connections are drawn as distinct curved edges;
- the canonical event log is shown in sequence order; and
- the header shows backend, scenario phase, module/assembly/connection counts,
  simulation time, and source revision counters.

With the MuJoCo backend, the same command opens MuJoCo's native 3D viewer as a
companion window by default. The two windows show the same authoritative
runtime in parallel: MuJoCo renders the physical model while Studio renders the
generated graph and canonical events. The native viewer is a separate process
and window, not an embedded widget or a second simulation.

The Runtime Inspector itself remains a standalone Qt window rather than an
embedded mode in the Robot Pack authoring window. This keeps the authoring
viewport separate from live simulation.

## Install and launch

Install both optional surfaces from a source checkout:

```bash
python -m pip install -e ".[studio,mujoco]"
```

Launch the generic cube demonstration:

```bash
modsim runtime examples/robot_packs/generic_cube \
  --fixed-connector front \
  --moving-connector front
```

Run the same two-module approach, docking, release, and retraction as an
explicit named demonstration:

```bash
modsim runtime examples/robot_packs/generic_cube \
  --demo dock_undock \
  --fixed-connector front \
  --moving-connector front \
  --duration 6.0
```

The Runtime Inspector uses MuJoCo by default and opens both the semantic window
and native viewer. Use `--no-viewer` when only the semantic graph and event log
are wanted or the backend should run without its native 3D window. Pass
`--backend mock` for a fast kinematic UI check; the mock has no native viewer,
so omitting the viewer option opens only the Runtime Inspector. Explicit
`--viewer` with a non-MuJoCo backend is rejected rather than silently ignored.
The semantic window still requires a desktop display or Xvfb; use the existing
non-GUI `modsim run` workflow for a fully headless process.

The command-line default is backend-aware: omitted means enabled for MuJoCo and
disabled otherwise. Programmatic `RuntimeInspectorConfig` construction keeps
`viewer_enabled=False` as its conservative backward-compatible default, so API
clients opt into creating a native companion explicitly.

The CLI also resolves demo-aware physics defaults. A programmatic launch of
`smores_diff_drive_dock_undock` must explicitly set `backend="mujoco"`,
`gravity=True`, `ground=True`, `height_m>=0.04`, a positive retract speed (or
leave it `None`), `dt_s<=0.005`, and a duration long enough for the cycle (the
CLI uses 12 s). This keeps `RuntimeInspectorConfig` a literal launch request
rather than silently rewriting values supplied by API clients.

If both connector options are omitted, ModSim selects the first declared
connector whose type is self-compatible. Supplying explicit connector IDs is
recommended for real platforms because it makes the scenario intent
reproducible.

A Robot Pack must declare a default runtime `module_topology_graph` recipe. Use
`--model-view RECIPE_ID` to select a different runtime-enabled recipe. The
initial inspector renderer accepts only the module-topology graph result.

The useful scenario controls are:

```text
--demo dock|dock_undock|smores_diff_drive_dock_undock|smores_driver_to_snake|smores_physical_driver_to_snake
--connector-gap METRES
--orientation RADIANS
--approach METRES_PER_SECOND
--duration SECONDS
--dt SECONDS
--undock-at SECONDS
--retract METRES_PER_SECOND
--publish-hz HERTZ
--speed FACTOR
--gravity / --no-gravity
--ground / --no-ground
--height METRES
--viewer / --no-viewer
```

`--speed` (also accepted as `--real-time-factor`) controls wall-clock pacing:
`1` is real time, `2` requests twice real time, and `0.5` requests slow motion.
It never scales the physics timestep, motor limits, controller commands,
simulated duration, or event timestamps. The target is best-effort—when model,
contact, rendering, or hardware throughput is the limit, ModSim runs as fast as
it can without skipping physics steps. `--publish-hz` remains a wall-clock UI
refresh limit; event deltas remain contiguous and lossless at every speed.

`--undock-at` is optional for `dock`. The `dock_undock` demonstration supplies a
release time automatically unless `--undock-at` overrides it. After
`UndockCommitted`, the edge is removed and the moving module retracts at the
requested speed.

## Named demonstrations

The kinematic named demonstrations run through the generic
`ScriptedReconfigurationScenario` engine. The two-module entries are small
dock-only or dock-then-undock plan builders. The physical SMORES entry instead
uses a differential-drive controller that sends bounded joint efforts while
MuJoCo integrates contact and dynamics. Platform dimensions and tuned bootstrap
parameters live in `examples/scenarios/smores_ep_diff_drive_dock_undock.py`.
The seven-module SMORES connector plan is example content at
`examples/scenarios/smores_driver_to_snake.py`, outside the installable
`modsim.runtime` library. The physical platform configuration is likewise kept
under `examples/scenarios`. A source checkout loads this example content beside
the committed SMORES Robot Pack when either named example is selected. The pack itself may be
opened from `examples/robot_packs`, copied, exported, or staged under `.modsim`;
scenario discovery is anchored to the source checkout rather than to the pack's
current directory.

### `dock` and `dock_undock`

The connector-pair demonstrations create exactly two modules. `dock` approaches and
commits one connection. `dock_undock` follows the same path, then releases the
connection and retracts the moving module.

### `smores_diff_drive_dock_undock`

This is the first dynamics-based SMORES-EP locomotion demonstration. MuJoCo
settles two free modules under gravity on the ground, actively brakes the fixed
module, holds both pan/tilt joints, and drives the moving module's two tire
joints toward the fixed module's rear `bottom` connector. Its front `pan`
connector latches only after the measured frames satisfy the ordinary connector
acceptance rules. After a one-second connected hold and an 80 ms face-release
delay, the fixed weld is disabled and the moving module reverses through wheel
effort. No root-pose writes occur after the one-time initial placement.

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_diff_drive_dock_undock \
  --model-view smores_topology
```

Gravity, the ground plane, a 0.05 m initial root height, a 0.02 m connector gap,
a 0.002 s solver step, and a 12 s display duration are defaults for this demo.
Its tire/skid contacts, friction, drivetrain damping/armature, effort limits,
and feedback gains are provisional simulation bootstrap values. The latch is
still an ideal fixed weld; magnetic attraction and electrical face-state
dynamics are not yet modelled. The controller therefore continues inside the
pack's broader 6 mm acceptance region to a 1 mm near-contact gate before
latching. This tuned path requires `0 < --dt <= 0.005` and a positive retract
speed. It starts from a deterministic heading-aligned pose; general navigation
and recovery from arbitrary lateral/yaw offsets remain outside this slice.

### `smores_driver_to_snake`

The included SMORES-EP Robot Pack can run a seven-module Driver-to-Snake
reconfiguration demonstration:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_driver_to_snake \
  --duration 14.0 \
  --dt 0.002 \
  --connector-gap 0.02 \
  --approach 0.03 \
  --no-gravity
```

Do not add `--ground`: this demonstration requires gravity and the ground
plane to remain disabled. The graph holds the initial seven-node, six-edge
Driver topology, then shows four separate `6 → 5 → 6` edge-count
transitions as one connection is released and its replacement is committed.
It finishes as the physical-module chain
`module_1–module_3–module_2–module_4–module_5–module_6–module_7`. The event log
and metrics show every ordinary undock, assembly split, dock, and assembly
merge along the way.

The configuration and four connector replacements are based on Figure 16 and
Table III of Chao Liu, Michael Whitzer, and Mark Yim,
[*A Distributed Reconfiguration Planning Algorithm for Modular Robots*](https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf),
IEEE Robotics and Automation Letters, 2019,
DOI [`10.1109/LRA.2019.2930432`](https://doi.org/10.1109/LRA.2019.2930432).
The paper directly specifies the action pairs; ModSim maps paper face `TOP` to
the pack's `pan` connector and retains `bottom`, `left`, and `right`.

ModSim executes those four pairs sequentially in Table III order using
deterministic kinematic component staging. That ordering and motion staging are
the demonstration's reproducible presentation, not a claim that ModSim has
implemented the paper's autonomous planner or path planner, or that this
seven-module presentation uses physical SMORES locomotion.

### `smores_physical_driver_to_snake`

This demonstration executes the same Table III connector replacements through
MuJoCo contact dynamics and SMORES wheel efforts:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_physical_driver_to_snake \
  --model-view smores_topology \
  --speed 4
```

The initial seven-module Driver tree is staged upright once at simulation time
zero and settles under gravity. After that boundary the scenario cannot write
module root poses or velocities and does not apply root wrenches. It submits one
bounded all-module effort batch each controller step: stationary wheels are
braked, pan/tilt joints are held, and the moving component's left/right wheels
receive differential-drive targets. Actions one and two move individual leaf
modules; actions three and four move connected three-module components.

Every replacement follows the ordinary runtime lifecycle: an 80 ms release
delay, weld removal, wheel-driven clearance/navigation, measured-frame final
approach, compatibility and acceptance evaluation, fixed-weld commit, and a
short connected hold. Active pan-face roll is aligned through its articulated
joint before capture. The graph and event table therefore show four physical
`6 → 5 → 6` transitions and the same final chain as the scripted demo.

The CLI allocates a 210-second simulated display run; the current reference
trace reaches the final connected hold in about 177 simulated seconds. At 4×,
those correspond nominally to 52.5 and about 44 wall-clock seconds. The physics
trajectory is unchanged from 1×, and the native viewer retains the completed
configuration until Stop or close. Actual throughput can be lower than the
requested factor on a compute- or render-limited machine.

The 2019 paper specifies the endpoint topologies and connector replacements,
not an end-to-end hardware trajectory for this Driver-to-Snake task. ModSim's
collision-clearance corridors and feedback tuning are deterministic authored
example data. They use the published differential-drive/unicycle control model
as a basis, but this remains a physics realization rather than autonomous
planning or a replay of measured robot motion. Tire/skid contact, motor effort,
support geometry, magnetic capture, and the post-capture ideal weld retain the
provisional limitations described for the two-module physical demo.

## Reproducible SMORES-EP workflows

The commands in this section are the complete reference for the committed
`examples/robot_packs/smores_ep` pack. Run them from the repository root after
installing the `studio` and `mujoco` extras. The examples use `modsim`; an
unactivated source checkout can use `uv run --no-sync modsim` or
`.venv/bin/modsim` instead.

### Inspect and validate

First inspect the aggregate pack contents:

```bash
modsim pack inspect examples/robot_packs/smores_ep
```

The report should identify pack `smores_ep` at version `0.1.0` and report one
module type, one connector type, two capabilities, one model-view recipe, and
one backend mapping. For machine-readable output, append `--output json`.

Run authoring validation and then the stricter simulation-readiness profile:

```bash
modsim pack validate examples/robot_packs/smores_ep
modsim pack validate \
  examples/robot_packs/smores_ep \
  --profile simulation
```

Both commands should report zero errors. Authoring validation permits certain
incomplete fields as warnings; simulation validation promotes required runtime
metadata and mapping gaps to errors. Neither command launches MuJoCo. Append
`--output json` when the result will be consumed by another tool.

Confirm that MuJoCo is installed and registered before launching a runtime:

```bash
modsim backends
```

### Two modules: dock

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock \
  --fixed-connector pan \
  --moving-connector pan \
  --model-view smores_topology \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 4.0 \
  --dt 0.002 \
  --no-gravity
```

The graph begins with two isolated nodes. After the moving module reaches the
fixed module, one edge appears and the event table contains, in order:

```text
DockCandidateDetected
DockCommitted
AssemblyMerged
```

The final state has two modules, one assembly, one active connection, and one
successful dock. No undock event is expected. Replace both `pan` arguments with
`bottom`, `left`, or `right` to exercise the other same-face pairs.

### Two modules: dock and undock

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock_undock \
  --fixed-connector pan \
  --moving-connector pan \
  --model-view smores_topology \
  --connector-gap 0.02 \
  --approach 0.03 \
  --retract 0.03 \
  --duration 6.0 \
  --dt 0.002 \
  --no-gravity
```

The graph follows `0 → 1 → 0` edges. In addition to the docking events above,
the event table records `UndockCommitted` and `AssemblySplit`; the moving
module then retracts so the released bodies visibly separate. The final state
has two modules, two assemblies, no active connection, one successful dock,
and one successful undock.

### Two modules: physical differential-drive dock and undock

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_diff_drive_dock_undock \
  --model-view smores_topology
```

Unlike the preceding scripted demonstration, this path enables gravity and the
ground by default and moves through the left/right wheel joints. The graph still
follows `0 → 1 → 0` edges, while the native viewer shows tire contact, a physical
approach, the held connection, release, and reverse separation. Expected final
metrics are one successful dock, one successful undock, zero failures, two
assemblies, and no active connection. Keep `--dt` at or below 0.005 s.

For a fully non-GUI version of the scripted lifecycle (not the wheel-driven
physics controller), run:

```bash
modsim run examples/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan \
  --moving-connector pan \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 1.5 \
  --dt 0.002 \
  --undock-at 1.0 \
  --retract 0.03
```

### Seven modules: Driver to Snake

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_driver_to_snake \
  --model-view smores_topology \
  --duration 14.0 \
  --dt 0.002 \
  --connector-gap 0.02 \
  --approach 0.03 \
  --no-gravity
```

Do not add `--ground`; this scripted staging demonstration requires both
gravity and the ground plane to remain disabled. It starts with seven graph
nodes, six connections, and one assembly. Four replacements each remove one
edge and commit another, producing four visible `6 → 5 → 6` transitions:

| Action | Undock | Dock |
|---:|---|---|
| 1 | `module_1/bottom ↔ module_2/pan` | `module_1/pan ↔ module_3/bottom` |
| 2 | `module_7/pan ↔ module_5/bottom` | `module_7/bottom ↔ module_6/pan` |
| 3 | `module_2/right ↔ module_4/left` | `module_2/pan ↔ module_4/bottom` |
| 4 | `module_5/left ↔ module_4/right` | `module_5/bottom ↔ module_4/pan` |

The final topology is:

```text
module_1 — module_3 — module_2 — module_4 — module_5 — module_6 — module_7
```

Expected final metrics are seven modules, six active connections, one
assembly, ten successful docks (six initial connections plus four
replacements), four successful undocks, and zero dock or undock failures. This
is deterministic execution of a predefined plan, not autonomous planning or
SMORES joint/wheel locomotion.

### Viewer and display behavior

Each `modsim runtime` command above opens the Qt Runtime Inspector and MuJoCo's
native viewer, backed by one authoritative runtime. Add `--no-viewer` to hide
only the MuJoCo window; Qt still requires a display or Xvfb. Use the documented
`modsim run` command for a completely non-GUI process. On macOS, the runtime
launcher automatically selects `mjpython` for its viewer child.

## What happens during a connector-pair run

The runtime owner loads and simulation-validates the pack, creates exactly two
module instances, and stages the selected connectors using frames measured
from the loaded backend. It then drives the second module along the selected
approach axis. Only the selected pair is eligible for the scripted dock and
release.

A successful docking run produces:

```text
DockCandidateDetected
DockCommitted
AssemblyMerged
```

An optional release adds:

```text
UndockCommitted
AssemblySplit
```

The graph edge appears only after `DockCommitted`; candidate and failure events
never become edges. The graph layout belongs to the renderer, so node positions
do not jump when the physical modules move or an edge is added. Clicking a node
or edge selects it by stable runtime ID. Selection survives ordinary updates
and is cleared when a selected connection disappears.

## Process, thread, and data boundary

The Qt thread never reads a live `WorldState`, `RuntimeSession`, or MuJoCo
object. With the companion viewer enabled, one child process exclusively owns
pack loading, backend creation, the native viewer, physics stepping, docking,
model-view generation, and backend shutdown. It sends versioned, frozen
`RuntimeInspectorFrame` values to the Qt process. On macOS the launcher
automatically starts that child with the `mjpython` installed beside the active
environment's Python, because MuJoCo's Cocoa viewer must own its process main
thread. The public command remains the same on macOS and Linux; users do not
need to launch or synchronize a second command.

With `--no-viewer`, the existing worker-thread path owns the same runtime
responsibilities inside the Qt process. Both paths publish only frozen frames
containing:

- an immutable `ModuleTopologyGraphView`;
- a `DockingMetrics` snapshot;
- a contiguous, lossless event delta; and
- immutable scenario status.

The Qt-free presenter accumulates event deltas, rejects regressing graph source
stamps, owns stable layout and selection, and supplies renderer-ready graph
geometry. PyQtGraph and Qt remain in `modsim_studio`; neither is imported by
the core package. The companion process does not create a second
`RuntimeSession`; the graph, events, and native 3D image always describe the
same simulation.

## Window lifetime and logging

The scenario is paced to the requested wall-clock real-time factor while both
windows are open. The factor is fixed for one launch and applies equally to the
native-viewer process and `--no-viewer` worker path. When the configured
simulated duration ends, the final state remains visible for inspection.
Closing the Runtime Inspector requests a cooperative child shutdown and closes
the native viewer. Closing the native viewer first ends the simulation while
leaving the last complete graph and event log visible in the Runtime Inspector.
The **Stop** control follows the same cooperative shutdown path.

The Qt process is the only writer that initializes and truncates the Studio
session log. Diagnostics from the native-viewer child are captured and mirrored
into that log, preventing two processes from racing to rewrite the same file.
Expected user actions such as closing the native viewer are recorded as normal
shutdown, while startup, protocol, backend, or display failures produce a
concise dialog and a full local traceback or child diagnostic in the log.

## Current boundary

The Runtime Inspector window shows ModSim semantics, not the backend's 3D
geometry, contacts, or forces. Its separately launched native MuJoCo companion
remains the 3D physics view. On macOS the internal companion must run under
`mjpython`; ModSim resolves and launches it automatically. If it is unavailable,
the launch fails with an installation-oriented message instead of a raw Cocoa
or GLFW traceback. If only the MuJoCo native window is unavailable, use
`--no-viewer`; the Qt semantic window still needs a display or Xvfb.

The first inspector has one graph renderer, one event table, a generic scripted
scenario engine with small connector-pair plan builders, one external
seven-module example plan, one physical SMORES differential-drive controller,
and a Stop control. General scenes, autonomous reconfiguration planning,
pause/restart controls, interactive/manual joint controls, position/velocity
backend command modes, metric plots, docking-lifecycle panels, additional
model-view renderers, and a combined authoring/runtime shell remain later
increments.

Runtime Inspector launches use the same repository-local, truncated Studio log
described in `studio.md`. Worker or child startup, validation, backend,
presentation, protocol, and shutdown failures are logged with tracebacks or
captured native diagnostics.
