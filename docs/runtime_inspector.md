# Runtime Inspector

The Runtime Inspector is ModSim Studio's first live runtime visualization. It
runs a named demonstration and displays ModSim's semantic state through the
selected model-view recipe:

- the topology renderer represents modules as stable nodes and committed
  connections as selectable edges;
- the cubic-lattice renderer represents measured module poses as cubes and
  their nearest integer cells as diagnostic snap targets;
- the canonical event log is shown in sequence order; and
- the header shows backend, scenario phase, module/assembly/connection counts,
  simulation time, and source revision counters.

With the MuJoCo backend, the same command opens MuJoCo's native 3D viewer as a
companion window by default. The two windows show the same authoritative
runtime in parallel: MuJoCo renders the physical model while Studio renders the
selected generated view and canonical events. The native viewer is a separate
process and window, not an embedded widget or a second simulation.

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
and native viewer. Use `--no-viewer` when only the semantic view and event log
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

Likewise, a programmatic `mblocks_momentum_pivot` launch must request MuJoCo,
gravity, ground, `height_m>=0.025`, `dt_s<=0.0005`, and the scenario-defined
connectors. The CLI's reference values are a 0.00025 s step, 0.025 m root
height, five-second duration, and the `mblocks_physics_lattice` view. The
`mblocks_physical_twelve_module_line` launch has the same backend, gravity,
ground, height, timestep ceiling, and connector-override requirements; it
defaults to a 0.0005 s step and 12 simulated seconds and also selects
`mblocks_physics_lattice`. The
`mblocks_twelve_module_line` CLI default is a 24-second, gravity-free kinematic
run using the pack's ordinary `mblocks_lattice` default. The matched staircase
entries use the grounded lattice origin: `mblocks_twelve_module_staircase` is
a 28-second gravity-free reference, while
`mblocks_physical_twelve_module_staircase` uses MuJoCo, gravity, ground, a
0.0005-second step, and a 12-second budget.

If both connector options are omitted, ModSim selects the first declared
connector whose type is self-compatible. Supplying explicit connector IDs is
recommended for real platforms because it makes the scenario intent
reproducible.

A Robot Pack must declare a default runtime model-view recipe. Use
`--model-view RECIPE_ID` to select a different runtime-enabled recipe. The
inspector currently renders results from the generic `module_topology_graph`
and `cubic_lattice` builders; another result type is rejected with the recipe
and unsupported view type in the error.

The useful scenario controls are:

```text
--demo dock|dock_undock|mblocks_five_module_pivot|mblocks_momentum_pivot|mblocks_physical_twelve_module_line|mblocks_physical_twelve_module_staircase|mblocks_twelve_module_line|mblocks_twelve_module_staircase|smores_diff_drive_dock_undock|smores_driver_to_snake|smores_physical_driver_to_snake
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

### Pause and resume

The Runtime Inspector header provides one **Pause**/**Resume** button. With the
native MuJoCo companion open, pressing **Space** in that window toggles the
same authoritative playback state; either input is acknowledged back to the
Inspector, so the button and MuJoCo status overlay stay synchronized.

A pause is applied at the next simulation-step boundary. While paused, ModSim
does not advance physics, `WorldState` time, scenario logic, connector
lifecycle processing, or canonical events, and both the native model and
generated semantic view remain at the same state. Both windows stay responsive
for camera, pan, zoom, selection, and close actions. Resume establishes a new
wall-clock pacing epoch, preventing elapsed pause time from producing a burst
of catch-up steps. **Stop** and either window's close control remain effective
while paused.

MuJoCo's built-in **Run/Pause** menu item remains disabled. That control belongs
to a viewer-owned physics loop, whereas ModSim deliberately uses MuJoCo's
passive viewer and owns every physics and semantic step. The ModSim button,
**Space** shortcut, and `RUNNING`/`PAUSED` native-viewer overlay provide the
equivalent synchronized control without transferring runtime ownership. The
`--no-viewer` worker-thread path exposes the same Inspector button; it simply
has no native window or keyboard shortcut. The native overlay is
feature-detected because older supported MuJoCo releases may not expose public
text overlays; the synchronized controls remain available without it.

`--undock-at` is optional for `dock`. The `dock_undock` demonstration supplies a
release time automatically unless `--undock-at` overrides it. After
`UndockCommitted`, the edge is removed and the moving module retracts at the
requested speed.

## Model-view renderers

The topology renderer keeps a logical layout stable while physical poses
change. Disconnected modules remain visible, parallel connections use separate
curved paths, and clicking a node or edge selects it by stable runtime ID.

The cubic-lattice renderer is a 2.5-D diagnostic view rather than a second 3-D
physics viewport. Its default isometric projection draws solid cubes at the
measured lattice-space poses and dashed ghost cubes at the nearest integer
cells. A dotted tether exposes any measured-to-snap displacement. Normal cubes
are coloured by assembly; amber marks a pose outside the recipe's position or
orientation tolerance, red marks modules and cells involved in an occupancy
conflict, and yellow marks the current selection. Committed connections remain
visible as selectable curves and midpoint diamonds, with lattice-face names in
their hover text.

The projection selector offers **Isometric**, **XY**, **XZ**, and **YZ**. The
**Z layer** selector shows all occupied layers or only modules snapped to one
integer Z coordinate; connections whose other endpoint is hidden are hidden as
well. **Snap cells** and **Local axes** toggle the two diagnostic overlays, and
**Fit** restores the finite grid to the viewport. The always-visible
**Labels: On/Off** button in the Runtime Inspector header, beside **Stop**,
hides or restores module names and cell coordinates for either renderer.
Left-drag pans, right-drag orbits the projected cubes, and the mouse wheel
zooms. The first orbit from a fixed XY,
XZ, or YZ view switches to the isometric/orbit projection; choosing any fixed
projection resets the orbit camera. Camera pitch is bounded before the poles
so the view cannot invert. Labelled global X/Y/Z axes and measured local axes
use red, green, and blue; the global axis letters and occupancy-conflict badges
remain visible when module labels are off. Modules, docking markers/curves, and
warning cells are selectable; module hover text reports its snapped cell,
measured coordinates in cell units, residuals, and assembly. The topology
renderer has the same module-label toggle but no 3-D orbit because its node
layout is an abstract graph.

All projection, orbit-camera, layer, overlay, label, pan, zoom, and selection
state belongs to the presenter/widget. None of those actions modifies the
canonical `WorldState`, the backend, or Robot Pack YAML.

Changing measured lattice geometry is updated in retained graphics items
rather than clearing and recreating the whole Qt scene. Frames that arrive in
one GUI burst are accumulated in order, including every event delta, then only
the newest accepted geometry is painted. This prevents rendering backlog from
blocking normal mouse processing while preserving a lossless canonical event
table. MuJoCo still consumes real CPU/GPU capacity, especially with a small
physics timestep and high `--speed`; `--publish-hz 10` or `5` is available when
a machine cannot sustain the default 20 Hz semantic refresh.

## Named demonstrations

The connector-pair and scripted SMORES demonstrations run through the generic
`ScriptedReconfigurationScenario` engine. The five- and twelve-module
M-Blocks visual benchmarks use the backend-neutral `KinematicPivotScenario`,
which rotates a released assembly through explicitly authored edge arcs. The
two-module M-Blocks physics entry uses `MomentumPivotScenario`, which commands
an internal flywheel and transitions fixed face to hinge to target face while
MuJoCo owns motion. The physical twelve-module entry uses
`MomentumPivotSequenceScenario` to compose eleven of those primitives without
taking ownership of root motion. The mat-to-staircase pair instead shares a
cyclic-capable `CoordinatedPivotPlan`: its reference executor moves a detached
two-module slab along an authored arc, while its physical executor commands
both slab flywheels and keeps one temporary hinge. The two generic module entries are small
dock-only or dock-then-undock plan builders. The physical SMORES entry instead
uses a differential-drive controller that sends bounded joint efforts while
MuJoCo integrates contact and dynamics. Platform dimensions and tuned
bootstrap parameters live in
`examples/scenarios/smores_ep_diff_drive_dock_undock.py`.
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

### `mblocks_five_module_pivot`

The CAD-derived 3D M-Blocks Robot Pack includes an authored five-module
traversal:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_five_module_pivot
```

Four modules form a fixed horizontal base. A fifth begins above the first and
crosses to the fourth through three 90-degree, positive-y edge rotations. Each
replacement publishes a normal `UndockCommitted`/`AssemblySplit` pair before
the arc and a normal candidate/dock/merge sequence at its endpoint. The
default lattice renderer shows `block_5` move continuously along each arc,
with its dashed nearest-cell target and changing local axes, while the
connection overlay shows three `4 → 3 → 4` edge transitions. It finishes as
one five-module assembly. The default eight-second display duration is
sufficient for the authored scenario, which normally completes around 6.0
simulated seconds. Pass `--model-view mblocks_topology` to use the
connectivity-only graph instead.

Gravity and the ground plane remain disabled. The scenario writes the released
module's root pose along each analytical arc while MuJoCo provides mesh
rendering and endpoint weld constraints. It does not model magnetic edge-hinge
forces, flywheel spin-up/braking, impact, or passive target-face capture, and
must not be interpreted as momentum-driven physics. The source-checkout
scenario data lives in `examples/scenarios/mblocks_five_module_pivot.py`; the
pack and fidelity gate are documented in the
[3D M-Blocks integration notes](mblocks_3d.md).

### `mblocks_momentum_pivot`

This is the first momentum-driven M-Blocks physics bootstrap:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_momentum_pivot
```

The CLI supplies MuJoCo, gravity, an infinite ground plane, a 25 mm root
height, a 0.00025 s solver/controller step, a five-second display duration,
and the grounded `mblocks_physics_lattice` recipe. The normal `--speed` option
changes wall-clock pacing only. The native viewer and lattice/event window run
from the same authoritative session.

At time zero, `moving_block` is staged above `support_block` with its initial
fixed face and coincident +Y edge magnets engaged. The controller settles,
spins the one-plane flywheel toward 9,000 RPM using at most 0.03 N m, releases
the face while retaining a two-point hinge, then applies at most 2.6 N m until
the wheel crosses zero. MuJoCo integrates the equal-and-opposite shell motion,
ground/support contact, and the 180-degree edge roll. Once measured target-face
acceptance is within 1 mm, the fixed target face commits and the transient
hinge releases. The event table labels each `DockCommitted` event as `fixed` or
`hinge`; scenario detail shows live flywheel speed, pivot angle, and
accumulated brake impulse. No root pose, velocity, or external wrench is
written after initialization.

The controller uses published reference ceilings: 0.150 kg module mass,
`8.4e-6 kg m^2` flywheel axial inertia, 20,000 RPM maximum speed, 0.03 N m
spin-up effort, and 2.6 N m mechanical-brake effort. A convergence regression
at 0.5, 0.25, and 0.1 ms completes at approximately 0.8585, 0.8595, and 0.8596
simulated seconds. Across those steps, the final moving-root world x coordinate
is about 0.0502–0.0507 m and the integrated brake impulse is
0.00763–0.00767 N m s.

This is a deterministic face-to-hinge-to-face approximation of the passive
magnet mechanism, not continuous magnetic-field fidelity. It models one +Y
actuation plane and does not implement the published three-detent carrier,
force-selected face/edge bond changes, or robust capture from arbitrary state.
Use `0 < --dt <= 0.0005`; the CLI rejects incompatible connector, gravity,
ground, height, backend, and motion-staging overrides.

### `mblocks_twelve_module_line`

Run the larger lattice/graph demonstration with either MuJoCo rendering or the
fast mock backend:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_twelve_module_line
```

Eleven blocks begin as a connected horizontal substrate and `block_12` begins
above its first cell. It executes ten authored 90-degree surface traverses and
one final 180-degree convex roll, finishing as one connected 12-cell line. The
default 24-second display budget includes holds; the current route completes
near 21.30 simulated seconds. Each replacement still uses canonical undock,
split, dock, and merge events, so both the lattice and event log expose all 11
topology transitions.

The 2019 M-Blocks work physically demonstrated decentralized line formation,
but does not publish a replayable per-module move sequence. This benchmark is
therefore paper-inspired, deterministic, and kinematic: it writes the released
module root pose along analytical arcs with gravity disabled. It is not the
exact 2019 hardware sequence, momentum-driven execution, or autonomous
planning.

### `mblocks_physical_twelve_module_line`

Run the physical counterpart through the same lattice/event interface:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_physical_twelve_module_line \
  --speed 4
```

The CLI selects MuJoCo, gravity, an infinite ground plane, a 25 mm minimum
root height, a 0.0005 s solver/controller step, a 12-second simulated-time
budget, and `mblocks_physics_lattice`. It reserves enough runtime constraints
for the eleven simultaneous fixed face bonds and one transient hinge. The
native viewer and Runtime Inspector still observe one authoritative session;
`--speed 4` requests four simulated seconds per wall-clock second without
changing physics or event time.

Eleven substrate blocks begin in a horizontal face-connected chain and
`block_12` begins above `block_1`. The scenario commits that complete initial
tree only at time zero. It then executes ten 90-degree momentum-driven surface
traverses and one final 180-degree roll. Quarter turns use a 6,000 RPM
flywheel target; the final half-turn uses 9,000 RPM. Each action pre-engages
the appropriate two-point +Y edge hinge, spins the internal flywheel, releases
the old fixed face, applies bounded braking while MuJoCo integrates contact and
shell motion, captures the measured target face, releases the hinge, and holds
the new connection before continuing.

No module root pose, velocity, or wrench is written after initialization. The
lattice view therefore follows backend-measured cube poses, while the event
table and connection overlay expose each temporary `hinge`, old-face release,
target `fixed` commit, and hinge release. If an action fails to commit or
disturbs an unrelated baseline connection, the sequence reports failure and
does not advance. A successful final state is one 12-cell line with 11 fixed
connections, no hinge, and one assembly.

The maintained real-MuJoCo full-route regression at the 0.0005 s demo default
verifies all 11 pivots, that exact final topology, 33 `DockCommitted` and 22
`UndockCommitted` events, cells `x = 0..11`, and no node outside the grounded
lattice recipe's 3 mm / 3 degree tolerances. It also installs failing guards on
backend root pose, twist, and wrench controls after scenario creation; the
sequence completes without calling them. A separate 0.00025 s three-block
regression covers the first surface traversal; broader full-route timestep and
perturbation convergence is not yet characterized.

This is a ModSim-authored, one-plane physics realization of the same endpoint
plan as the kinematic benchmark. It is not the exact unpublished 2019 hardware
move sequence, a continuous magnetic-force model, a three-plane carrier, or an
autonomous planner. The 12-second duration is a configured budget, not a
published hardware performance result.

### `mblocks_twelve_module_staircase`

This is the deterministic reference for the mat-to-staircase route:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mock \
  --demo mblocks_twelve_module_staircase \
  --no-viewer
```

Replace `--backend mock --no-viewer` with `--backend mujoco` to show the CAD
meshes beside the semantic window. Gravity and ground remain disabled because
the scenario analytically moves both roots in each released slab. The initial
2×6 mat has 16 fixed connections; the final two-block-deep staircase has 18.
Eleven positive-Y pivots produce column heights 1, 2, and 3 and finish near
25.22 simulated seconds inside the default 28-second budget. The event log
contains the actual multiple-face releases and landings, including assembly
split/merge transitions.

### `mblocks_physical_twelve_module_staircase`

Run the same topology route through full MuJoCo dynamics:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_physical_twelve_module_staircase \
  --speed 4
```

The CLI supplies MuJoCo, gravity, the infinite ground plane, a 25 mm minimum
center height, the grounded lattice recipe, a 0.0005-second step, a 12-second
budget, 24 weld slots, and two hinge slots. For each action the controller
pre-engages one edge hinge, spins both moving slab flywheels, releases two or
four fixed faces, applies bounded brake impulses, waits until all two or four
landing faces pass measured acceptance, commits them, and releases the hinge.
It never writes a module pose, twist, or root wrench after initialization.

The maintained real-MuJoCo regression completes all eleven pivots around 9.5
simulated seconds and ends with 18 fixed connections, one assembly, 53 dock
commits, and 35 undock commits. Some measured cubes can be amber in the strict
fixed-world lattice recipe because accumulated physical drift is larger than
3 mm / 3 degrees; the nearest integer cells and final topology are still
verified. Quarter turns use 6,000 RPM, ordinary half turns 15,000 RPM, and the
elevated half turn 19,500 RPM, below the declared 20,000 RPM limit.

No M-Blocks paper publishes this exact planar-mat-to-staircase trace. It is a
ModSim-designed demonstration based on the 2015 edge-pivot primitives and
planning model. The result occupies three Z layers, but every active pivot is
still about +Y; it is not evidence of an implemented three-plane carrier or an
autonomous planner.

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

- an immutable `ModuleTopologyGraphView` or `CubicLatticeView`;
- a `DockingMetrics` snapshot;
- a contiguous, lossless event delta; and
- immutable scenario status.

The Qt-free presenter accumulates event deltas, rejects regressing model-view
source stamps, owns selection and renderer controls, and supplies
renderer-ready 2-D geometry. It keeps a stable logical topology layout or
projects measured lattice poses through the active fixed/orbit camera. The Qt
window batches already-received frame bursts before asking the presenter for
one paintable result; the presenter consumes every frame's event delta before
projecting the newest accepted state. PyQtGraph and Qt remain in
`modsim_studio`; neither is imported by the core package. The companion process
does not create a second `RuntimeSession`; the semantic view, events, and native
3D image always describe the same simulation.

Pause requests from the Inspector and **Space** presses from the native viewer
converge on one playback state owned by the runtime child. At a pause boundary,
the owner publishes the exact frozen frame before acknowledging the new state.
With `--no-viewer`, the worker thread follows the same step-boundary and
acknowledgement contract without the process boundary.

## Window lifetime and logging

The scenario is paced to the requested wall-clock real-time factor while both
windows are open. The factor is fixed for one launch and applies equally to the
native-viewer process and `--no-viewer` worker path. Pausing freezes simulated
time while the windows continue servicing input; resuming resets the pacing
origin. When the configured simulated duration ends, the final state remains
visible for inspection.
Closing the Runtime Inspector requests a cooperative child shutdown and closes
the native viewer. Closing the native viewer first ends the simulation while
leaving the last complete semantic view and event log visible in the Runtime
Inspector.
The **Stop** control follows the same cooperative shutdown path and remains
available while playback is paused.

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

The inspector has topology and cubic-lattice renderers, one event table, a
generic scripted scenario engine with small connector-pair plan builders,
five- and twelve-module kinematic M-Blocks routes, a two-module one-plane
M-Blocks momentum controller, an eleven-action twelve-module physical M-Blocks
line sequence, matched reference/physical twelve-module staircase routes, one
external seven-module SMORES plan, one physical SMORES
differential-drive controller, synchronized Pause/Resume control, and a Stop
control. General scenes, autonomous reconfiguration planning, continuous
magnetic fields, a three-plane M-Blocks carrier, restart controls,
interactive/manual joint controls,
position/velocity backend command modes, metric plots, docking-lifecycle
panels, further model-view renderers, and a combined authoring/runtime shell
remain later increments.

Runtime Inspector launches use the same repository-local, truncated Studio log
described in `studio.md`. Worker or child startup, validation, backend,
presentation, protocol, and shutdown failures are logged with tracebacks or
captured native diagnostics.
