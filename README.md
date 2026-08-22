# ModSim

ModSim is a Python-first, backend-agnostic framework for modular and multi-robot
systems. ModSim owns Robot Pack definitions and modular-robot semantics;
the operational MuJoCo adapter owns physics today, and an Isaac Sim adapter is
planned behind the same backend boundary.

## What is implemented

ModSim currently provides an end-to-end Robot Pack authoring and fixed-docking
runtime slice.

Robot Pack and import support:

- strict Pydantic models for assets, module types, joints and limits,
  connector instances and reusable connector types, capabilities, backend
  mappings, and named model-view recipes;
- connector compatibility, gender, allowed orientations, acceptance regions,
  fixed/compliant intent, load limits, undocking intent, and runtime docking
  policy;
- bounded JSON-compatible custom metadata on packs, connectors, and connector
  types;
- secure split-YAML loading, two validation profiles, deterministic export,
  and transactional in-place Save;
- local URDF import for links, tree joints, origins, axes, scalar limits,
  masses, primitive geometry, meshes, and solid-color materials; and
- self-contained draft generation that copies resolved local assets and
  rewrites mesh paths.

ModSim Studio authoring:

- a standalone PySide6/PyVistaQt Robot Pack Builder with project, link, joint,
  connector, connector-type, and model-view trees;
- create-from-URDF, Open, atomic Save, and non-overwriting Export As;
- add/edit/remove workflows for connector types and connector instances,
  including reassignment to an imported URDF body/link;
- editors for custom metadata, connector semantics and docking policy, joint
  control modes and limits, and model-view recipes;
- authoring and simulation-readiness validation plus a read-only canonical
  YAML preview;
- one-module URDF visualization with solid colors, smooth shading, lights,
  shadows, a large grid floor, link selection, collision overlays, frames,
  joint axes, and connector docking/approach axes; and
- a local per-launch sidecar session log with full tracebacks.

Runtime, physics, and model views:

- multi-module `SceneSpec`, canonical `WorldState`, deterministic assembly
  components, revision counters, and an append-only event log;
- backend-neutral connector candidate detection, compatibility and acceptance
  evaluation, docking guards, two-phase dock/undock commits, cooldown and
  overload hooks, and runtime metrics;
- a dependency-free mock backend and an optional MuJoCo backend that composes
  module assets, measures link/connector frames, steps rigid-body physics, and
  implements fixed connections using reserved weld constraints;
- retained URDF visual meshes in MuJoCo, with lightweight collision proxies
  kept physical but hidden in viewer debug group 3 by default;
- a backend-neutral `ModelView`/`GraphModelView` abstraction,
  `ModelViewFactory`, Robot Pack recipes, revision-aware bounded caching, and
  the generic `module_topology_graph` where modules are nodes and committed
  docking connections are distinct edges; and
- a standalone Runtime Inspector with a stable selectable graph, ordered event
  table, plan/phase/action status, live module/assembly/connection and
  dock/undock metrics, and immutable frames with lossless contiguous event
  deltas.

For MuJoCo, one `modsim runtime` command opens the semantic Runtime Inspector
and a separate native 3D viewer in parallel. A single authoritative child
process owns the physics session and streams immutable graph/event frames to
Qt, so the windows cannot diverge. The shipped runtime presets are `dock`,
`dock_undock`, and the seven-module `smores_driver_to_snake` demonstration.

The current feature branch is verified by 421 tests with native MuJoCo enabled,
strict core/Studio/MuJoCo type checks, lint/format checks, and wheel/source
distribution smoke tests.

The current version does **not** provide an Isaac Sim adapter, joint-command or
actuator execution, autonomous reconfiguration planning, welded-contact
exclusion, or MuJoCo constraint-force reporting. MuJoCo currently executes
fixed connections only. Studio does not yet render image textures or provide a
3D connector-placement gizmo. The Runtime Inspector currently has one topology
renderer and named demonstrations rather than arbitrary user-authored scenes;
it does not yet provide pause/restart controls or time-series metric plots. The
model-view factory currently ships only the module-topology builder; frame,
port, hardware-tree, and matrix views remain future extensions.
The `simulation` validation profile is a structural readiness check; it does
not launch a simulator.

For the exact implemented inventory, known defects, verification baseline, and
recommended continuation order, see
[`docs/IMPLEMENTATION_STATUS.md`](docs/IMPLEMENTATION_STATUS.md).

## Install from this repository

ModSim requires Python 3.11 or newer. The distribution/install name is
`modsim-robotics`; the Python import and console command are both `modsim`.

For the complete authoring, MuJoCo runtime, and development environment, run
from the repository root:

```bash
python3.12 -m venv .venv
PIP_NO_CACHE_DIR=1 .venv/bin/python -m pip install --upgrade pip
PIP_NO_CACHE_DIR=1 .venv/bin/python -m pip install -e ".[studio,mujoco,dev]"
.venv/bin/modsim --version
.venv/bin/modsim backends
```

The examples below keep explicit `.venv/bin/...` paths so they work without
shell activation. On POSIX shells, `source .venv/bin/activate` lets you use the
shorter `modsim` command instead.

The locked `uv` equivalent is:

```bash
UV_CACHE_DIR=.uv-cache UV_PYTHON_INSTALL_DIR=.uv-python \
  uv sync --locked --extra dev --extra studio --extra mujoco --python 3.12
uv run --no-sync modsim --version
uv run --no-sync modsim backends
```

The extras are intentionally separate:

- `python -m pip install .` installs the backend-neutral core and CLI;
- `.[studio]` adds the Robot Pack Builder and Runtime Inspector;
- `.[mujoco]` adds the MuJoCo adapter and native viewer support; and
- `.[dev]` adds the repository test, lint, formatting, and type-check tools.

An installed package can launch the Robot Pack Builder with either
`modsim studio` or `modsim-studio`.

Smoke-test the complete checkout with the committed generic pack:

```bash
.venv/bin/modsim pack validate \
  examples/robot_packs/generic_cube --profile simulation
.venv/bin/modsim studio examples/robot_packs/generic_cube
.venv/bin/modsim runtime examples/robot_packs/generic_cube
```

For MuJoCo, that one command opens both the native 3D viewer and the semantic
Runtime Inspector. Use `--no-viewer` to suppress only the native 3D window;
the Qt graph/event window still opens and needs a display or Xvfb. On macOS
ModSim automatically launches the viewer child with the active environment's
`mjpython`; the public command does not change. At the configured duration the
simulation stops stepping and retains its final graph, events, metrics, and 3D
state for inspection until the windows are closed.

## Run the included SMORES-EP Robot Pack

The complete SMORES-EP pack is committed at
`examples/robot_packs/smores_ep`, including the Fusion-exported URDF, five STL
visual meshes, four lightweight OBJ collision proxies, and all Robot Pack YAML.
It is available in every source checkout and source distribution. The project
owner authorized repository inclusion for collaborator access; broader reuse
terms have not yet been expressed in a formal asset license.

The current pack defines module type `smores_ep`, the provisional genderless
and self-compatible connector type `ep_face`, connector IDs `bottom`, `pan`,
`left`, and `right`, `dock` and `undock` capabilities, and the default runtime
model-view recipe `smores_topology`. Schema IDs use lowercase `snake_case`, so
enter `ep_face` as an ID in Studio and put capitalization such as `EP Face` in
the separate display-name field.

The fixed SMORES `bottom` face is authored on the rear `-X` mating plane of
`base_link`, opposite the `pan`/top face—not on the module underside. Its
connector origin is `[-0.010741577148, 0, 0]`, its docking and approach axes
point along `[-1, 0, 0]`, and its local frame uses a π-radian yaw. The matching
collision proxy is a thin rear plate with size
`[0.008, 0.0651, 0.0708856]` m. See
[`docs/smores_ep_mujoco.md`](docs/smores_ep_mujoco.md) for all four connector
frames and proxy dimensions.

Five detailed Fusion-exported STL meshes provide the visible robot geometry.
Four lightweight, 12-triangle OBJ face boxes provide the current collision
geometry; they continue to participate in MuJoCo physics while viewer group 3
hides them by default. The current URDF assigns one solid silver material to
all five visuals; the STL assets also carry no texture coordinates or material
graph. The connector frames, acceptance tolerances, load values, and collision
proxies are provisional demo data, not certified SMORES-EP hardware
specifications.

Inspect and validate it before launching a GUI:

```bash
.venv/bin/modsim pack inspect examples/robot_packs/smores_ep
.venv/bin/modsim pack validate \
  examples/robot_packs/smores_ep --profile simulation
.venv/bin/modsim views examples/robot_packs/smores_ep
.venv/bin/modsim views examples/robot_packs/smores_ep \
  --view smores_topology --count 4 --output json
```

The simulation validation should report zero errors before a runtime is
started. Open the Robot Pack Builder to inspect the detailed meshes, links,
joints, connector semantics, and YAML:

```bash
.venv/bin/modsim studio examples/robot_packs/smores_ep
```

In Studio, select `bottom` and enable connector and collision overlays. The
glyph should be centered on the rigid rear base opposite `pan`, both connector
arrows should point outward along `-X`, and the collision proxy should appear
as a thin vertical rear plate.

### Two modules: dock

```bash
.venv/bin/modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock \
  --fixed-connector pan \
  --moving-connector pan \
  --model-view smores_topology \
  --duration 4.0 \
  --no-gravity
```

This opens MuJoCo's native 3D viewer and the Runtime Inspector together. The
semantic graph starts with two isolated module nodes and gains one edge only
after MuJoCo confirms the physical weld. The event table records candidate,
dock, and assembly-merge events.

### Two modules: dock, then undock

```bash
.venv/bin/modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock_undock \
  --fixed-connector bottom \
  --moving-connector bottom \
  --model-view smores_topology \
  --duration 6.0 \
  --retract 0.03 \
  --no-gravity
```

The graph follows `0 → 1 → 0` edges. The event table adds
`UndockCommitted` and `AssemblySplit`, and the moving module retracts so the
release is visible. This command directly exercises the corrected rear face.
Replace both `bottom` values with `pan`, `left`, or `right` to exercise the
other verified same-face pair demos.

### Seven modules: Driver to Snake

```bash
.venv/bin/modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_driver_to_snake \
  --model-view smores_topology \
  --duration 14.0 \
  --dt 0.002 \
  --connector-gap 0.02 \
  --approach 0.03 \
  --no-gravity
```

Do not add `--ground` to this preset. It starts with seven graph nodes, six
connections, and one assembly. Four scripted replacements each produce a
visible `6 → 5 → 6` edge transition, ending in:

```text
module_1 — module_3 — module_2 — module_4 — module_5 — module_6 — module_7
```

With `bottom` on the rear base, the final MuJoCo module roots form a straight
horizontal nose-to-tail chain in that order instead of the vertically kinked
geometry produced by the former underside placement.

With the defaults above, the verified final runtime metrics are six active
connections, one assembly, ten successful docks (six initial connections plus
four replacements), four successful undocks, and zero dock/undock failures.
The topology and connector replacements follow the seven-module Driver-to-Snake
example in Liu, Whitzer, and Yim's 2019
[*A Distributed Reconfiguration Planning Algorithm for Modular Robots*](https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf).
ModSim currently executes that plan by deterministically staging complete
components through the ordinary dock/undock pipeline. It is not autonomous
planning, collision-free path planning, or SMORES wheel/joint locomotion.

### Window and headless options

Append `--no-viewer` to any `modsim runtime` command to suppress MuJoCo's
native window while keeping the Qt graph, events, and metrics. This is a
headless MuJoCo backend but not a fully non-GUI command: Qt still needs a
display or Xvfb.

Use `modsim run` for a fully non-GUI two-module MuJoCo lifecycle:

```bash
.venv/bin/modsim run examples/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan \
  --moving-connector pan \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 1.5 \
  --dt 0.002 \
  --undock-at 1.0 \
  --retract 0.03 \
  --output json
```

There is no public non-Qt CLI entry point for the seven-module preset yet,
although its reusable controller and runtime runner are covered headlessly in
the test suite. On macOS, use the ordinary `modsim runtime` commands above;
ModSim automatically selects the environment's `mjpython` for its viewer
child. Only the standalone `modsim run --view` workflow requires invoking
`.venv/bin/mjpython -m modsim` explicitly.

Runtime and Studio launches rewrite the example pack's local session log at
`examples/robot_packs/.modsim/logs/smores_ep/studio.log`. Reproduce one issue
per launch when collecting a clean debugging log. When using `uv`, replace
`.venv/bin/modsim` in the examples with `uv run --no-sync modsim`.

Before a public package release, install a locally built wheel in a clean
virtual environment:

```bash
python -m pip install dist/modsim_robotics-0.1.0-py3-none-any.whl
modsim pack validate /absolute/path/to/a/robot_pack
```

Do not substitute `pip install modsim`; that is not this distribution.

## Use from a source checkout

The example and repository documentation paths below exist in a source checkout,
not in an installed wheel. The shorter commands assume the virtual environment
is active; otherwise replace `modsim` with `.venv/bin/modsim`:

```bash
modsim pack inspect examples/robot_packs/generic_cube
modsim pack validate examples/robot_packs/generic_cube
modsim pack validate examples/robot_packs/generic_cube --profile simulation
modsim pack validate examples/robot_packs/generic_cube --output json
modsim views examples/robot_packs/generic_cube
modsim dock examples/robot_packs/generic_cube --count 3
modsim studio examples/robot_packs/generic_cube
modsim runtime examples/robot_packs/generic_cube
```

Use the command that matches the question being answered:

| Command | Purpose | Physics/UI |
|---|---|---|
| `modsim pack inspect` / `pack validate` | Inspect and structurally validate a Robot Pack | No physics, no GUI |
| `modsim studio` | Author a pack and preview one URDF module | Native Qt/PyVista GUI, no physics |
| `modsim views` | List recipes or generate an immutable model-view snapshot | No physics, no GUI |
| `modsim dock` | Exercise connector semantics and report why pairs do or do not latch | Mock by default; text/JSON output |
| `modsim run` | Execute a scripted backend docking lifecycle | Non-GUI unless `--view` is added |
| `modsim runtime` | Run a named demo with graph, events, status, and metrics | Qt Runtime Inspector; MuJoCo viewer opens by default |

A Robot Pack is a self-contained directory. Its root `robot_pack.yaml` refers to
typed specification and backend-mapping documents and keyed local assets. Paths
are portable, relative to the pack, and cannot escape it.

The implemented format is documented in `docs/robot_pack_spec.md`, and the
model-view architecture and recipe workflow are documented in
[`docs/model_views.md`](docs/model_views.md), and the live graph/event workflow
is in [`docs/runtime_inspector.md`](docs/runtime_inspector.md). Current
implementation status and known bugs are tracked in
`docs/IMPLEMENTATION_STATUS.md`. `docs/AGENTS.md` contains implementation rules,
while `docs/HANDOFF.md` is the broader future architecture roadmap.

## Create a Robot Pack from URDF

From the command line:

```bash
modsim pack init \
  --from-urdf /path/to/module.urdf \
  --out /path/to/new_robot_pack

modsim studio /path/to/new_robot_pack
```

Use one or more `--asset-root` options when a URDF uses `package://` paths that
cannot be resolved relative to the URDF:

```bash
modsim pack init \
  --from-urdf /path/to/module.urdf \
  --asset-root /path/to/robot_description_package \
  --out /path/to/new_robot_pack
```

The importer extracts links, joints, joint limits, inertial masses, primitive
geometry, and mesh references. It copies the URDF and resolved meshes into a
self-contained pack, rewrites mesh paths, generates module and URDF mapping
documents, and leaves modular connector/capability semantics for the user. It
accepts URDF rather than Xacro and does not invoke a Xacro expander; convert a
Xacro source to a concrete URDF first. External OBJ/DAE texture and material
sidecars are not yet copied comprehensively, so STL or self-contained geometry
is the safest initial import path.

Studio can also perform this workflow through **File → Create from URDF**. Its
initial editor provides:

- project, module, link, joint, and connector trees;
- Robot Pack custom metadata editing through the Properties panel;
- visual/collision geometry toggles;
- link frames, joint axes, connector frames, docking axes, and approach axes;
- URDF solid-color materials, smooth shaded surfaces, scene lighting, shadows,
  a grid floor, and material-preserving link selection;
- selection from the tree or 3D meshes;
- explicit connector-type create/edit/remove workflows;
- connector reassignment to existing types and imported URDF bodies/links;
- named model-view recipe create/edit/remove workflows;
- numeric connector placement, JSON-compatible custom connector fields, and
  joint metadata editing;
- live authoring/simulation validation and a read-only canonical YAML preview;
- atomic Save and non-overwriting Export As.

**Save** writes the edited Robot Pack metadata and semantic specifications back
to the canonical split-YAML documents. The YAML preview is for inspection only;
make changes through the Properties panel and other editors.

URDF remains the source of geometry, mesh references, and kinematics. For
example, changing a cube's shape or dimensions requires editing its URDF and
reimporting it; Robot Pack metadata does not replace that mechanical source.
Connector placement currently uses numeric pose fields. Studio does not yet
provide a translation or rotation gizmo, render image textures/normal maps, or
preview more than one module at its URDF zero-joint configuration.

Each Studio launch rewrites a local session log. With a pack supplied, the
default sidecar is
`<pack-parent>/.modsim/logs/<pack-folder-name>/studio.log`; without a pack, it
is `<current-working-directory>/.modsim/logs/studio.log`. Set
`MODSIM_STUDIO_LOG` to override the path. The **Session Log** tab mirrors the
same file while Studio is running. During a coupled runtime, the Qt process is
the only log writer and captures the native-viewer child's diagnostics into
that file.

See `docs/studio.md` for the exact workflow and current limitations. The
SMORES-EP assets are an explicitly authorized example; do not commit additional
robot CAD, URDF, or meshes without equivalent owner authorization.

## Python API

```python
from pathlib import Path

from modsim.robot_packs import (
    RobotPackLoader,
    RobotPackValidator,
    RobotPackWriter,
    ValidationProfile,
)

loaded = RobotPackLoader().load(Path("/path/to/robot_pack"))
report = RobotPackValidator().validate(
    loaded,
    profile=ValidationProfile.AUTHORING,
)
report.raise_for_errors()

# Export atomically to a new path.
exported = RobotPackWriter().write(
    loaded,
    Path("/path/to/new_robot_pack_export"),
)

# An explicit editor-style Save stages and validates a complete replacement,
# then atomically swaps the existing pack directory.
saved = RobotPackWriter().update(loaded)
```

## Generate a model view

Robot Packs can select registered model-view builders through named recipes.
The first built-in builder produces a generic module-topology graph: every
module is a node, including isolated modules, and every committed active
docking connection is an undirected edge. Multiple connections between the
same two modules remain distinct.

List a pack's recipes and the builders installed in the current environment:

```bash
modsim views examples/robot_packs/generic_cube
modsim views examples/robot_packs/generic_cube \
  --view module_topology \
  --count 3 \
  --output json
```

Generated views are immutable, JSON-safe snapshots. They are suitable for
algorithms, Studio, a future browser client, or a native renderer without
introducing GUI or physics dependencies into the core. Robot Pack YAML stores
the recipe, not the generated graph or live runtime state. See
[`docs/model_views.md`](docs/model_views.md) for the class boundaries, recipe
fields, and runtime update semantics.

## Inspect a live runtime

The standalone Runtime Inspector runs a named demonstration and shows the
changing semantic graph and event log while the backend steps. MuJoCo is the
default backend. The default `dock` demonstration uses exactly two modules:

```bash
modsim runtime examples/robot_packs/generic_cube \
  --fixed-connector front \
  --moving-connector front \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 4
```

Before docking, the graph contains two isolated module nodes. A committed weld
adds one edge. Use the named `dock_undock` preset for the same approach and
dock followed by a timed release and retraction:

```bash
modsim runtime examples/robot_packs/generic_cube \
  --demo dock_undock \
  --fixed-connector front \
  --moving-connector front \
  --duration 6.0
```

The edge is removed after `UndockCommitted`, and the event log also records
`AssemblySplit`. See **Run the included SMORES-EP Robot Pack** above for the exact
two-module and seven-module SMORES commands, expected topology, metrics, and
staging limitations. Additional pack-specific details are in
[`docs/smores_ep_mujoco.md`](docs/smores_ep_mujoco.md).

The Qt thread receives immutable frames from a worker or viewer child that
exclusively owns MuJoCo and `WorldState`. The default MuJoCo launch opens that
native 3D viewer beside the semantic window. Add `--no-viewer` to retain only
the graph and event log; the mock backend also defaults to semantic-only
operation.

If connector options are omitted, ModSim selects the first self-compatible
connector declared on the module. Explicit IDs are recommended for real robot
packs. The selected model-view recipe must generate a
`module_topology_graph`; `--model-view RECIPE_ID` overrides the default runtime
recipe. See [`docs/runtime_inspector.md`](docs/runtime_inspector.md) for the
complete workflow and current boundary.

## Run a docking session

The quickest check is the `modsim dock` command, which runs a docking session
and reports whether a pack's connectors actually mate:

```bash
modsim dock examples/robot_packs/generic_cube --count 3
modsim dock examples/robot_packs/generic_cube --count 3 --undock
modsim dock examples/robot_packs/generic_cube --output json
```

It places `--count` modules `--spacing` metres apart, runs `--steps` steps of
`--dt` seconds, and prints the event log, the resulting assemblies, and the
runtime metrics. `--latch` (the default) issues a dock command for every pair
whose geometry already satisfies acceptance; `--no-latch` leaves docking to each
connector type's own `auto_latch` policy. When nothing docks, the command
reports which criterion or guard blocked each pair.

This exercises the real semantic pipeline against the kinematic mock backend. It
runs no physics, so it answers "is this pack authored such that these connectors
would latch?" rather than "will this robot work?".

The same thing from Python, which is what to build on:

```python
from pathlib import Path

from modsim import MockBackendAdapter, RobotPackLoader, RuntimeSession, SceneSpec
from modsim.core.ids import ConnectorInstanceId

pack = RobotPackLoader().load(Path("examples/robot_packs/generic_cube")).pack
scene = SceneSpec.grid("generic_cube", 3, spacing_m=0.1)
session = RuntimeSession.create(pack, scene, MockBackendAdapter())

session.request_dock(
    ConnectorInstanceId("generic_cube_0/front"),
    ConnectorInstanceId("generic_cube_1/rear"),
)
for event in session.step(0.01):
    print(event.kind, event.sequence)

print(session.world.assemblies.assemblies)
print(session.metrics().as_dict())
```

ModSim decides whether two connectors may mate and what it means when they do;
the backend decides only whether the requested physical constraint exists. A
logical connection is created only after the backend confirms the constraint, so
semantic state can never claim a connection that no physics engine is
enforcing.

Connector types can declare a `docking_policy` controlling auto-latching,
measured or nominal alignment, redock cooldown, and break force. See
`docs/docking_semantics.md` for the full pipeline and the acceptance criteria.

## Switch between backends

ModSim owns modular-robot semantics; a backend owns physics. Backends are
selected by name, so the same pack and scene can be run against either without
changing code:

```bash
modsim backends                                   # what is registered and installed
modsim dock examples/robot_packs/generic_cube --backend mujoco --no-gravity
MODSIM_BACKEND=mujoco modsim dock examples/robot_packs/generic_cube
```

```python
session = RuntimeSession.create(loaded, scene, "mujoco", gravity=(0.0, 0.0, 0.0))
```

For the generic `RuntimeSession` and `modsim dock` path, resolution order is an
explicit backend, then `MODSIM_BACKEND`, then `mock`. The higher-level
`modsim runtime` command explicitly defaults to MuJoCo because its main purpose
is the coupled physics and semantic demonstration.

The MuJoCo backend is an optional extra and lives in its own package, so
importing `modsim` never imports a physics engine:

```bash
python -m pip install -e ".[mujoco]"
```

Docking and undocking execute under MuJoCo: a scene is compiled with a pool of
reserved weld constraints, and a committed connection claims one, re-points it
at the mating bodies, and activates it. Releasing returns the slot to the pool.

```bash
# on macOS the viewer must run under mjpython, not python
mjpython -m modsim run examples/robot_packs/generic_cube --backend mujoco \
  --count 2 --duration 8 --undock-at 4 --view
```

Without connector options, `modsim run` places modules in a row and drives the
last one along world -X. For a real robot, select the two module-local connector
IDs instead:

```bash
modsim run path/to/pack --backend mujoco \
  --fixed-connector pan --moving-connector pan \
  --connector-gap 0.02 --approach 0.03 \
  --duration 1.5 --undock-at 1.0
```

The runner measures both connector frames after backend load, rotates and
places the second module in a valid mating orientation, and approaches along
the selected docking axis. This also works when a connector belongs to an
articulated child link. Explicit connector mode targets only that selected
pair; row mode can latch every acceptable proposal. Both can release on a
schedule. `--view` opens the MuJoCo passive viewer, paces the run to wall clock,
and holds the window open at the end.

For live 3D physics and ModSim semantics together, use the Runtime Inspector:

```bash
modsim runtime path/to/pack --backend mujoco \
  --fixed-connector pan --moving-connector pan
```

This one command opens the native MuJoCo viewer beside the graph/event window.
One companion process owns the `RuntimeSession`, physics data, and generated
frames, so the two windows cannot diverge into separate simulations. The native
viewer defaults on for MuJoCo and off for the mock backend; explicit `--viewer`
with a non-MuJoCo backend is rejected. Pass `--no-viewer` for the semantic Qt
window with MuJoCo stepping without its native viewer; Qt still requires a
display or Xvfb. Use `modsim run` without `--view` for a fully non-GUI process.
On macOS ModSim finds the environment's `mjpython` automatically for its child
process. The standalone `modsim run --view` workflow still requires an explicit
`mjpython` launcher on macOS.

Note that releasing a constraint does not separate anything: two welded modules
share a velocity, so on release they coast along together still touching. The
runner backs the driven module away at `--retract` (default: the approach speed)
so the undock is visible. `--retract 0` shows the coasting behaviour instead.

`docs/backends.md` covers the adapter contract, the differences between the two
backends, and the cross-backend conformance suite. The committed SMORES-EP example,
provisional connector frames, lightweight collision proxies, and exact demo
commands are recorded in `docs/smores_ep_mujoco.md`.

## Verify a development checkout

After the locked installation above, the repository's review checks are:

```bash
uv lock --check
uv run --no-sync ruff check .
uv run --no-sync ruff format --check .
uv run --no-sync pyright
uv run --no-sync pyright --project pyright-mujoco.json
uv run --no-sync pyright --project pyright-studio.json
uv run --no-sync pytest
git diff --check
```

The current feature baseline is 421 passing tests with native MuJoCo enabled.
GUI smoke tests use Xvfb in CI; local Studio and Runtime Inspector launches need
a working display and OpenGL environment.

## Design boundaries

- URDF and meshes are imported mechanical assets, not canonical ModSim state.
- Robot Pack YAML is the modular-robot semantic layer.
- Generated topology graphs are derived views, not canonical state; additional
  frame, port, and matrix views can use the same factory later.
- Disconnected modules remain first-class entities.
- Docking and undocking are explicit lifecycle events, and the event log is the
  only path that mutates world state.
- Assembly identity is derived from membership, not from a counter, so the same
  configuration produces the same identifiers across runs and replays.
- GUI and simulator dependencies stay outside the core package. Studio is a
  separate optional package and consumes the same public core API that future
  web or native C++ clients can target.
- The current MuJoCo adapter supports fixed docking but not joint commands,
  full backend mapping coverage, named-frame-only connectors, welded-contact
  exclusion, or constraint-force/load feedback.

The package is pre-release. A software license, publication channel, public
project URLs, and a general policy for future robot assets still need to be
chosen. The included SMORES-EP assets are available to repository collaborators,
but no separate license currently grants broader reuse rights.
