# Model views

Model views are immutable, derived representations of ModSim's canonical robot
state. They make hardware and runtime state convenient for algorithms and user
interfaces without turning a graph, matrix, or platform-specific projection
into a second source of truth.

The first built-in view is a module-topology graph:

- every module instance is a node, including disconnected modules;
- every committed docked connection is an undirected edge;
- parallel connections between the same pair of modules remain separate edges;
- nodes and edges use stable runtime identifiers and deterministic ordering; and
- live poses and connection data are copied into an immutable snapshot.

This representation fits SMORES-EP, but the builder is generic and works with
any Robot Pack whose runtime state uses ModSim modules and connections.

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
actionable factory error. The initial `module_topology_graph` builder has no
configuration options and supports runtime state only.

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

The module-topology graph observes backend samples because node poses can move,
and topology revisions because docking or undocking can add or remove edges.
Only `DockCommitted` and `UndockCommitted` change graph topology; candidates,
failed attempts, and planned connections do not become graph edges.

## Current scope

This increment provides the general factory and the module-topology graph used
by SMORES-EP. The following remain later extensions built through the same
interface:

- frame, port, matrix, and hybrid docking views;
- platform-specific lattice or configuration-coordinate views;
- third-party builder discovery;
- Studio's live Runtime Inspector and graph layout/rendering; and
- incremental graph patches or event streaming.

Runtime visualization should receive complete immutable view snapshots from a
simulation worker. The Qt UI must not traverse or mutate a live `WorldState`
while a backend is stepping.
