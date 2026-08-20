# Runtime Inspector

The Runtime Inspector is ModSim Studio's first live runtime visualization. It
runs one two-module docking scenario and displays ModSim's semantic state:

- each module instance is a stable graph node;
- each committed docking connection is an undirected graph edge;
- disconnected modules remain visible;
- parallel connections are drawn as distinct curved edges;
- the canonical event log is shown in sequence order; and
- the header shows backend, scenario phase, module/assembly/connection counts,
  simulation time, and source revision counters.

It is a standalone Qt window rather than an embedded mode in the Robot Pack
authoring window. This keeps the first runtime slice small and prevents the
authoring viewport from becoming a second simulator viewer.

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

The Runtime Inspector uses MuJoCo by default. Pass `--backend mock` for a fast
kinematic UI check. If both connector options are omitted, ModSim selects the
first declared connector whose type is self-compatible. Supplying explicit
connector IDs is recommended for real platforms because it makes the scenario
intent reproducible.

A Robot Pack must declare a default runtime `module_topology_graph` recipe. Use
`--model-view RECIPE_ID` to select a different runtime-enabled recipe. The
initial inspector renderer accepts only the module-topology graph result.

The useful scenario controls are:

```text
--connector-gap METRES
--orientation RADIANS
--approach METRES_PER_SECOND
--duration SECONDS
--dt SECONDS
--undock-at SECONDS
--retract METRES_PER_SECOND
--publish-hz HERTZ
--gravity / --no-gravity
--ground
--height METRES
```

`--undock-at` is optional. Without it, the graph finishes with one committed
edge. With it, the edge is removed after `UndockCommitted`, and the moving
module retracts at the requested speed.

## What happens during a run

The worker loads and simulation-validates the pack, creates exactly two module
instances, and stages the selected connectors using frames measured from the
loaded backend. It then drives the second module along the selected approach
axis. Only the selected pair is eligible for the scripted dock and release.

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

## Thread and data boundary

The Qt thread never reads a live `WorldState`, `RuntimeSession`, or MuJoCo
object. One worker thread exclusively owns pack loading, backend creation,
physics stepping, docking, model-view generation, and backend shutdown. It
publishes frozen `RuntimeInspectorFrame` values containing:

- an immutable `ModuleTopologyGraphView`;
- a `DockingMetrics` snapshot;
- a contiguous, lossless event delta; and
- immutable scenario status.

The Qt-free presenter accumulates event deltas, rejects regressing graph source
stamps, owns stable layout and selection, and supplies renderer-ready graph
geometry. PyQtGraph and Qt remain in `modsim_studio`; neither is imported by
the core package.

## Current boundary

The Runtime Inspector shows ModSim semantics, not the backend's 3D geometry,
contacts, or forces. The existing native MuJoCo viewer remains the tool for a
3D physics view. On macOS it must run as a separate `mjpython -m modsim run
... --view` process because both the passive MuJoCo viewer and Qt require
main-thread ownership.

The first inspector has one graph renderer, one event table, one scripted pair,
and a Stop control. General scenes, pause/restart controls, live joint commands,
metric plots, docking-lifecycle panels, additional model-view renderers, and a
combined authoring/runtime shell remain later increments.

Runtime Inspector launches use the same repository-local, truncated Studio log
described in `studio.md`. Worker startup, validation, backend, presentation, and
shutdown failures are logged with tracebacks.
