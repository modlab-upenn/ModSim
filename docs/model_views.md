# Model views

Model views are immutable, derived representations of ModSim's canonical robot
state. They make hardware and runtime state convenient for algorithms and user
interfaces without turning a graph, matrix, or platform-specific projection
into a second source of truth.

The built-in module-topology graph provides the simplest runtime projection:

- every module instance is a node, including disconnected modules;
- every committed docked connection is an undirected edge;
- parallel connections between the same pair of modules remain separate edges;
- nodes and edges use stable runtime identifiers and deterministic ordering; and
- live poses and connection data are copied into an immutable snapshot.

This representation fits SMORES-EP, but the builder is generic and works with
any Robot Pack whose runtime state uses ModSim modules and connections.

The second built-in view is a cubic-lattice graph. It quantizes each module's
root pose to a nearest integer cell and one of the 24 proper axis-aligned cube
orientations. The result retains the measured world pose, reports translation
and angular residuals, marks poses outside configured tolerances, reports
multiple position-valid modules assigned to one cell, and labels connection
endpoints by their nearest lattice-facing side. It is a derived diagnostic
view; quantization never modifies `WorldState`.

## Architecture

Model-view generation has four separate responsibilities:

1. `WorldState` and the Robot Pack remain canonical state.
2. A `ModelViewBuilder` derives one typed representation from that state.
3. `ModelViewFactory` registers builders, resolves named recipes, and caches the
   latest immutable result for each view.
4. A client such as Studio, a future browser frontend, or an algorithm renders
   or consumes the result.

Builders are registered through Python code. Robot Packs select builders by a
stable identifier; they never contain an import path or executable plugin
code. This keeps packs portable and prevents a data file from executing
arbitrary Python. External plugin discovery can be added later on top of the
same registration contract.

The public result objects are strict, immutable Pydantic models containing only
JSON-safe data. They deliberately do not expose Qt, MuJoCo, NetworkX, NumPy, or
mutable `WorldState` objects. A client may convert the DTO into its preferred
rendering or analysis library.

## Robot Pack recipe

A pack declares named recipes in `robot_pack.yaml`:

```yaml
model_views:
  - id: module_topology
    name: Module Topology
    builder: module_topology_graph
    modes:
      - runtime
    default: true
    configuration: {}
```

The recipe records how the view should be generated, not its current nodes and
edges. The graph itself is regenerated from live `WorldState`, so docking and
undocking are reflected without rewriting the Robot Pack.

Recipe fields are:

- `id`: stable, pack-local recipe identifier;
- `name`: optional display name;
- `builder`: registered builder identifier;
- `modes`: one or both of `authoring` and `runtime`;
- `default`: whether a client should open the view automatically; and
- `configuration`: bounded JSON-compatible builder configuration.

An unknown builder remains valid pack data because builder availability depends
on the installed application and plugins. Attempting to build it produces an
actionable factory error. `module_topology_graph` has no configuration options.
`cubic_lattice` uses the configuration described below. Both built-ins
currently support runtime state only.

### Cubic-lattice recipe

```yaml
model_views:
  - id: module_lattice
    name: Module Lattice
    builder: cubic_lattice
    modes: [runtime]
    default: false
    configuration:
      pitch_m: 0.05
      position_tolerance_m: 0.002
      orientation_tolerance_rad: 0.08726646259971647
      origin_world_m: [0.0, 0.0, 0.0]
      orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0]
```

`pitch_m`, `position_tolerance_m`, and `orientation_tolerance_rad` are
required. Pitch must be positive, position tolerance must be nonnegative and
less than half a cell, and orientation tolerance must be between zero and
pi/4. `origin_world_m` and `orientation_world_wxyz` describe the lattice frame
in world coordinates and default to the world origin and identity rotation;
the quaternion must have unit length. Unknown fields and non-finite values are
rejected with a `ModelViewBuildError`.

The module root frame defines its local cube axes. Position is expressed in the
lattice frame, divided by pitch, and rounded to the nearest integer; exact
half-cell ties round away from zero. The closest proper cube rotation supplies
an orientation index and descriptive orientation ID. The immutable result also
contains the complete 24-orientation catalog, so consumers need not duplicate
that convention. A node is `off_lattice` when either its position or orientation
residual exceeds the configured tolerance.

Occupancy conflicts include only modules whose position residual is within the
configured tolerance. A module with a valid cell but invalid orientation can
therefore participate in a conflict, while a module merely quantized to a
nearby cell from outside the positional tolerance cannot. Connection face
labels are the nearest cardinal lattice directions to the live connector
docking axes; their angular residuals remain in the DTO so consumers can judge
non-axis-aligned measurements.

## Generate a view

List recipes and builders, then generate the initial three-module runtime
snapshot from the command line:

```bash
modsim views examples/robot_packs/generic_cube
modsim views examples/robot_packs/generic_cube \
  --view module_topology \
  --count 3 \
  --output json
```

The command creates canonical in-memory state but does not launch a physics
backend. A runtime client uses the same API with its live world:

```python
from modsim import ModelViewContext, ModelViewFactory

factory = ModelViewFactory()
context = ModelViewContext(pack=session.world.pack, world=session.world)

# Build every default runtime recipe declared by the Robot Pack.
views = factory.build_default_runtime(context)

session.step(0.002)
updated_views = factory.build_default_runtime(context)
```

`ModelViewFactory()` registers the built-in builders for ordinary clients. A
platform package can subclass `ModelViewBuilder` and call `factory.register()`
to add a view implementation. The factory is an ordinary per-client object,
not a singleton, so registrations and cached results cannot leak between
independent runtimes.

## Runtime updates and caching

`WorldState.revision` exposes independent counters for backend samples,
topology changes, docking-state changes, and appended events. A builder states
which counters affect its output. The factory keeps only the latest result for
each source and recipe, so repeated reads of unchanged state reuse the same
immutable object and long simulations do not grow an unbounded cache.

Both built-in graphs observe backend samples because module poses and connector
directions can move, and topology revisions because docking or undocking can
add or remove edges. Only `DockCommitted` and `UndockCommitted` change graph
topology; candidates, failed attempts, and planned connections do not become
graph edges. Each committed edge copies its physical constraint type from the
canonical runtime connection, so a fixed bond and a transient hinge remain
distinguishable in generated topology and lattice DTOs.

## Runtime Inspector consumer

`modsim runtime PACK` is the first live consumer of generated model views. In
semantic-only mode a dedicated worker thread owns `RuntimeSession`, the
backend, and `ModelViewFactory`; with the native MuJoCo viewer enabled, one
companion process owns the same responsibilities. After stepping, either owner
copies the selected supported view, metrics, event delta, and scenario status
into an immutable `RuntimeInspectorFrame`. The Qt thread sees only those
values. The supported frame payload is currently a `ModuleTopologyGraphView`
or `CubicLatticeView`; selecting another builder produces an actionable
renderer error before the window starts publishing updates.

The Qt-free presenter keeps renderer settings and selection as client state.
For topology graphs, module nodes stay in place across physical pose samples,
docking adds an edge without relaying out the graph, and parallel connections
use separate curved paths. For cubic lattices, a Qt-free projector derives
finite 2-D geometry in isometric, XY, XZ, or YZ projection while preserving the
measured module pose independently of its nearest integer snap cell. An
undock clears selection only when the selected edge disappears. Event deltas
are contiguous and deduplicated, so throttling publication does not lose
canonical events. See `runtime_inspector.md` for the executable workflow.

### Cubic-lattice rendering

The lattice widget draws solid, assembly-coloured cubes at measured poses and
dashed ghost cubes at their nearest integer cells. A dotted tether makes a
measured-to-snap offset visible. Amber marks an off-lattice pose, red marks an
occupancy conflict, and yellow outlines the current selection. Global lattice
X/Y/Z axes and optional measured local axes share red, green, and blue colours.
Committed face-labelled connections are selectable curves with a midpoint
marker.

The toolbar switches among isometric, XY, XZ, and YZ projections, filters to
one occupied integer Z layer or all layers, toggles snap cells and local axes,
and fits the finite, auto-expanding grid. Modules, connections, and warning
cells use stable IDs for selection; hover text reports the snapped cell,
measured position in cell units, pose residuals, assembly, and connector-face
mapping. These controls change presentation state only and never write to
`WorldState` or the Robot Pack.

## Current scope

The general factory currently includes the module-topology and cubic-lattice
graphs. The following remain later extensions built through the same interface:

- frame, port, matrix, and hybrid docking views;
- platform-specific configuration-coordinate views;
- third-party builder discovery;
- incremental graph patches or event streaming.

The implemented Runtime Inspector receives and renders complete immutable
module-topology or cubic-lattice snapshots. Incremental patches are not
required for the current two- and twelve-module physical M-Blocks routes, the
five- and twelve-module kinematic M-Blocks references, or the seven-module
SMORES named demonstrations.
