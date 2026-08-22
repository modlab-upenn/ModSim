# ModSim implementation status and continuation handoff

This is the canonical snapshot of what the repository implements today, what
is known to be broken or incomplete, and what should be built next. It is
written so that a developer or coding agent can continue from a fresh clone
without relying on prior conversation history.

## Snapshot and document authority

- Snapshot date: 2026-08-22
- Distribution: `modsim-robotics`
- Package version: `0.1.0`
- Robot Pack format: `0.1`
- Supported Python: 3.11 and 3.12; repository default: 3.12
- Active feature branch: `feature/mujoco-runtime-inspector`
- Feature branch base: `feature/model-view-factory` at `ab08ba4`
- Integration parents: `main` at `7d3a3ba7c08f` and
  `origin/docking-exec-aw` at `e2ec0b20a3ef`
- Project status: pre-alpha

When sources disagree, use this order:

1. the code and tests;
2. `robot_pack_spec.md` for the implemented Robot Pack contract;
3. this file for current scope, defects, and continuation priorities;
4. `studio.md` for current desktop workflow and UI details;
5. `HANDOFF.md` for future architecture and milestones.

`HANDOFF.md` is a design roadmap. Parts of its proposed package tree, APIs,
runtime, YAML examples, and milestone status are intentionally ahead of the
code. Do not treat it as an inventory of implemented modules.

## Product decisions already settled

- ModSim is a Python-first, backend-neutral framework. Python 3.11+ is the
  current implementation language; performance-sensitive pieces may later move
  behind stable APIs.
- ModSim owns modular-robot descriptions and semantics. A simulator adapter
  will own physics.
- URDF and meshes are imported mechanical assets. Robot Pack YAML is the
  canonical modular-robot semantic description.
- MuJoCo is the first operational physics backend. Isaac Sim remains a later
  adapter and must implement the same backend contract.
- The first UI is a standalone native Python desktop application using
  PySide6 and PyVistaQt. The core API must remain usable by a future browser
  frontend or native/C++ client.
- Robot assembly is not a manual CAD-style editing task. Future assemblies are
  derived at runtime from modules and connection state produced by
  reconfiguration algorithms.
- The framework must remain generic. SMORES-EP is the first real integration
  target, not a hard-coded platform.
- The project owner authorized the SMORES-EP Fusion export for repository
  inclusion and collaborator access. Do not commit additional robot assets
  without equivalent owner authorization.

## Owner-specified work boundary

Development commands and edits must stay inside the ModSim repository. Do not
modify unrelated files elsewhere on the host, inspect unrelated machine data,
or send local files, logs, paths, credentials, robot assets, or other machine
information to an external service. Public dependency downloads and public
documentation research may be used when needed, without uploading local
content.

Keep virtual environments, dependency caches, test scratch space, build
artifacts, and Studio logs repository-local or use a system sandbox explicitly
provided for temporary data. The relevant ignored locations are `.venv/`,
`.uv-cache/`, `.uv-python/`, `.test-tmp/`, `dist/`, and `.modsim/`. Studio logs
can contain absolute local paths and tracebacks, so review them before sharing
or committing excerpts.

## Current system boundary

The implemented vertical slice now covers Robot Pack authoring plus a
backend-neutral docking runtime:

```text
local URDF + meshes
        |
        v
URDF importer / draft builder
        |
        v
split Robot Pack YAML + copied local assets
        |
        v
loader -> strict typed model -> validator
          |                 |                  |
          v                 v                  v
   canonical writer     CLI reports     named ModelView recipes
          |                                    |
          v                                    v
   ModSim Studio editor                  ModelViewFactory
   + one-module preview                         |
                                               v
SceneSpec -> RuntimeSession <-> mock or MuJoCo backend
                    |
                    v
           WorldState + EventLog
              |              |
              v              v
 connector acceptance   revision counters
   + docking guards           |
                              v
                       ModelViewFactory
                              |
                              v
                  immutable inspector frames
                       |             |
                Qt worker signal   local IPC
                       \             /
                        v           v
                    live graph + event table

MuJoCo runtime owner ------------> native 3D companion window
```

World state, a simulation clock, assemblies, connector acceptance, two-phase
dock/undock commits, runtime metrics, a mock backend, and a MuJoCo weld-backed
adapter are operational. Named Robot Pack model-view recipes, a backend-neutral
model-view factory, the first immutable module-topology graph, and a standalone
Studio Runtime Inspector that renders that graph plus the event log are also
operational. A MuJoCo launch supervises a separate native 3D viewer of that
same authoritative runtime by default. Joint command/control APIs,
reconfiguration planning, and the Isaac adapter remain deferred. The
validation profile named `simulation` still performs structural readiness
checks; it does not launch a backend by itself.

## What is implemented

### Packaging and developer tooling

- A PEP 517/Hatch project with a `src/` layout and typed-package marker.
- Base installation depends only on Pydantic, Rich, ruamel.yaml, and Typer.
- The optional `studio` extra installs NumPy, PySide6, PyQtGraph, PyVista, and
  PyVistaQt.
- The optional `dev` extra installs Ruff, Pyright, pytest, pytest-cov, and
  pre-commit.
- The optional `mujoco` extra installs MuJoCo and NumPy without adding either
  dependency to the core or Studio-only installation.
- A committed `uv.lock` provides reproducible dependency resolution.
- Console commands are `modsim` and `modsim-studio`.
- Wheels contain `modsim`, `modsim_studio`, and the optional
  `modsim_backend_mujoco` adapter; documentation, tests, and examples are
  included in the source distribution.
- CI checks Python 3.11 and 3.12, runs a native Studio smoke under Xvfb, runs a
  separate headless MuJoCo backend/conformance job, builds wheel and source
  distributions, and smoke-tests both artifacts.

### Robot Pack schema

`src/modsim/robot_packs/schema.py` defines strict Pydantic v2 models for:

- the root manifest and local asset catalogs;
- module types, joints, limits, connector instances, and connector types;
- connector compatibility, orientation, acceptance regions, physical
  constraints, compliance, load limits, and undocking intent;
- connector docking policy: explicit/automatic latch, measured/nominal
  alignment, redock cooldown, and optional break force;
- bounded JSON-compatible custom metadata on the manifest, connector
  instances, and connector types;
- named model-view recipes selecting a registered builder, authoring/runtime
  modes, default-open intent, and bounded JSON-compatible configuration;
- advertised capabilities;
- backend mappings; and
- the aggregate loaded Robot Pack.

Format 0.1 uses a root `robot_pack.yaml` plus referenced specification and
mapping YAML documents. Identifiers, relative paths, finite numbers, unit
vectors, unique IDs, and many enum/range constraints are validated. Unknown
fields are rejected. Unit-bearing field names make SI units explicit.

See `robot_pack_spec.md` for every field and its normative meaning.

### Secure loading and canonical persistence

`src/modsim/robot_packs/loader.py`:

- loads only the declared split YAML documents;
- validates each document into strict typed models;
- rejects missing, malformed, escaping, symlinked, and special document
  paths; and
- reports source document and structured validation locations.

`src/modsim/robot_packs/writer.py` provides two distinct operations:

- `write()` stages, reloads, semantically compares, and atomically publishes a
  new pack. It refuses to overwrite an existing destination.
- `update()` stages and verifies a complete sibling copy, atomically swaps an
  existing pack, and attempts rollback if publication fails. Studio Save uses
  this path.

Declared assets are copied without following symlinks. YAML is deterministic
and canonical for a given model. Hand formatting, comments, anchors, quoting,
and key-order preferences are not preserved.

### Validation

`src/modsim/robot_packs/validator.py` returns structured issues with severity,
stable code, source document, JSON-pointer-like location, optional entity, and
suggested fix.

- `authoring` permits incomplete metadata where continued editing is useful
  and reports it as warnings.
- `simulation` promotes completeness requirements to errors and requires
  relevant backend mapping information.
- Both profiles check schema, paths, declared assets, cross-document
  references, uniqueness, joint/connector consistency, and other format
  invariants.

Validation does not run physics or prove that a backend can load a pack. It
also does not currently parse referenced URDFs to verify authored link, joint,
frame, or backend source names.

### URDF import and draft generation

`src/modsim/importers/urdf.py` is a local, dependency-light XML importer. It:

- accepts URDF files up to 32 MiB;
- imports links, tree joints, root links, origins, axes, joint limits, and link
  masses;
- imports visual and collision mesh, box, cylinder, and sphere geometry;
- resolves global and inline URDF solid-color materials, including named
  references and validated RGBA values;
- resolves relative paths, local `file://` references, and locally resolvable
  `package://` references; and
- returns a typed `ImportedRobotAsset`.

`src/modsim/importers/draft_builder.py`:

- refuses an existing destination;
- requires every referenced mesh to resolve locally;
- copies the URDF and meshes into a self-contained pack;
- rewrites URDF mesh paths;
- creates one module type, its joints, and an initial URDF mapping; and
- writes a valid format-0.1 draft ready for semantic annotation.

Xacro expansion, network retrieval, simulator-specific URDF extensions,
transmission catalogs, full inertial tensors, and complete external material
or texture dependency discovery are not implemented.

### Command-line interface

The current CLI supports:

```text
modsim --version
modsim pack init --from-urdf SOURCE --out DESTINATION [--asset-root PATH]
modsim pack inspect PACK [--output text|json]
modsim pack validate PACK [--profile authoring|simulation] [--output text|json]
modsim studio [PACK]
modsim runtime PACK [--demo dock|dock_undock|smores_driver_to_snake]
  [--backend mock|mujoco] [--viewer|--no-viewer]
modsim backends
modsim views PACK [--view RECIPE_ID]
modsim dock PACK [--backend mock|mujoco]
modsim run PACK [--backend mock|mujoco] [--view]
```

`modsim views` lists the pack's recipes and installed builders or generates a
selected immutable view snapshot. It does not launch physics or render a live
graph.

`modsim dock` checks authored docking semantics in a one-shot session.
`modsim run` approaches, docks, optionally releases, and retracts modules under
the selected backend. Supplying `--fixed-connector` and `--moving-connector`
measures those connector frames after backend load, arranges them in a valid
mating orientation, and drives along the actual docking axis instead of
assuming an X-axis layout.

`modsim runtime` opens the standalone Qt Runtime Inspector around a named
demonstration. `dock` and `dock_undock` create exactly two modules and stage a
selected connector pair; `smores_driver_to_snake` creates seven modules and
executes four scripted connection replacements. All variants run through one
runtime owner and display the generated topology graph, canonical event log,
scenario progress, and current metrics. MuJoCo is the default and also opens
its separate native 3D viewer unless `--no-viewer` is supplied. In that coupled
mode the viewer child, rather than the Qt worker, owns the one authoritative
session and sends immutable inspector frames to Qt. Omitting the viewer option
with `--backend mock` keeps it off; explicit `--viewer` with a non-MuJoCo
backend is rejected.

### Runtime, docking, and backends

`src/modsim/core`, `src/modsim/connectors`, `src/modsim/runtime`, and
`src/modsim/backends` provide:

- multi-module `SceneSpec` placement and stable runtime identifiers;
- canonical modules, connectors, logical connections, backend snapshots, and
  an append-only event log;
- `WorldStateRevision` counters for backend samples, topology changes, docking
  state changes, and appended events, so consumers can react only to state
  categories they observe;
- assemblies derived as connected components, including free one-module
  assemblies;
- spatial candidate detection, mutual compatibility and gender rules,
  position/axis/roll/relative-velocity acceptance, and docking guards;
- two-phase commits: logical connections are created only after a backend
  confirms the physical constraint;
- release, cooldown, overload hooks, assembly merge/split events, and modular
  runtime metrics;
- a dependency-free kinematic mock adapter; and
- an optional MuJoCo adapter that composes multiple URDF/MJCF module instances,
  retains URDF visual meshes while hiding separate collision proxies in a
  viewer debug group, reports measured link/site frames, steps rigid-body
  physics, and implements runtime dock/undock with reserved weld equality
  constraints.

`stage_docking_pair` is reusable scenario setup. It derives a whole-module
placement from measured connector and module-root frames, including connectors
on articulated child bodies. Nominal alignment similarly preserves the
measured root-to-parent-body transform before activating a weld.

`DockingPairScenario` owns reusable targeted approach/latch/release/retract
orchestration. Its immutable phase/status is presentation data rather than a
canonical world event. `RuntimeInspectorFrame` copies a generated topology
view, docking metrics, scenario status, and a contiguous event delta from the
runtime-owner session. The public event-detail formatter is shared by CLI text
reports and the GUI table.

`ScriptedReconfigurationScenario` executes a backend-neutral
`ReconfigurationPlan`: it validates the complete tree/action sequence before
mutation, creates the initial tree through ordinary docking commits, replaces
one edge at a time, and rigidly stages every root in a moving assembly. The
built-in seven-module SMORES Driver-to-Snake plan is declarative data over that
generic controller. `RuntimeSession.process_docking()` permits an exact staged
pair to commit without advancing physics time, avoiding an unconstrained
contact step between alignment and weld activation. Both the mock and MuJoCo
pose controls clear stale motion when a component is staged; MuJoCo clears
root and articulated-joint velocities.

The inspector has two execution transports. `--no-viewer` uses the original Qt
worker thread. The default MuJoCo path starts a companion process that owns the
session, physics stepping, native viewer, and model-view factory, then streams
versioned immutable frames to Qt. On macOS the launcher resolves the
environment-local `mjpython` automatically. There is never a duplicate
simulation for the graph, and only the Qt process initializes the truncated
Studio session log; child diagnostics are captured into it.

MuJoCo currently supports only fixed connections. Compliant, hinge, ball, and
custom constraints are refused explicitly. Joint command capability is not
advertised until a real actuator/command API exists.

### Model views

`src/modsim/model_views` provides the first backend-neutral generated-view
subsystem:

- strict, immutable, JSON-safe `ModelView` result objects with a
  `GraphModelView` specialization;
- a `ModelViewBuilder` abstract interface for typed view generators;
- `ModelViewFactory`, which registers builders explicitly, rejects duplicate
  registrations, resolves Robot Pack recipes, and keeps only the latest cached
  result rather than accumulating simulation history; and
- the generic `module_topology_graph` builder.

The module-topology graph creates one node for every module instance, including
isolated modules, and one undirected edge for every committed active docking
connection. Parallel connections between the same two modules remain distinct
edges. Candidates, failed attempts, and planned connections are not edges.
Stable runtime identifiers and deterministic ordering make snapshots suitable
for algorithms, tests, Studio, or a future browser/native renderer.

Robot Pack `model_views` entries are named recipes. They store a builder ID,
supported authoring/runtime modes, default-open intent, and bounded
JSON-compatible configuration; they do not store generated nodes/edges or
executable plugin paths. Builders are registered through Python today, with
external discovery deferred. See `model_views.md` for the contracts and update
semantics.

The first live consumer is the Runtime Inspector. Its Qt-free presenter owns a
deterministic stable layout, selection, distinct curves for parallel edges,
event-delta accumulation, and source-stamp regression checks. Physical pose
samples therefore cannot make the logical graph jitter, while a committed dock
or undock updates the edge set.

### ModSim Studio

`src/modsim_studio` contains two optional standalone Qt applications: the
PySide6/PyVistaQt Robot Pack Builder and a PyQtGraph Runtime Inspector. The
authoring application currently provides:

- create-from-URDF, open, Save, and non-overwriting Export As workflows;
- a tree for the pack, imported assets, connector types, module types, links,
  joints, and connector instances;
- Properties editors for pack and module fields, joint metadata, connector
  instances, connector-type semantics, and custom manifest/connector/type
  metadata;
- explicit connector-type creation and reference-safe deletion;
- connector type reassignment and parent-body selection from imported URDF
  links;
- model-view recipe creation, editing, and removal, including builder, mode,
  default, and configuration fields;
- authoring and simulation-readiness validation panels;
- a read-only preview of all canonical YAML documents;
- an embedded 3D view of one module type in the URDF zero-joint
  configuration;
- visual/collision geometry, link frames, joint axes, connector frames,
  docking axes, and approach axes;
- URDF solid-color materials, smooth PBR surface shading, a three-light scene,
  a large shadowed grid floor that does not affect robot camera framing, and
  material-preserving link-selection bounds;
- opt-in frame and joint-axis overlays so debug glyphs do not obscure the
  default robot view;
- link selection through either the tree or rendered geometry; and
- a timestamped session log mirrored in a GUI tab.

The Runtime Inspector currently provides:

- real-time-paced execution whose sole runtime owner performs pack loading,
  validation, backend/session creation, stepping, view generation, and
  shutdown;
- named `dock` and `dock_undock` two-module presets plus a paper-grounded
  seven-module `smores_driver_to_snake` scripted topology preset;
- a default MuJoCo companion process/window that renders the same session in
  3D and streams immutable frames to Qt, plus a `--no-viewer` worker-thread
  mode;
- automatic environment-local `mjpython` launch for the native child on
  macOS;
- auto-selection of the first self-compatible connector or explicit local
  connector IDs;
- a stable selectable module-topology graph with isolated nodes and distinct
  parallel connections;
- an append-only event table with sequence, simulation time, kind, and detail;
- backend, plan/phase/action, module/assembly/connection, dock/undock/event
  metrics, and exact model-view revision counters;
- an explicit Stop control, coordinated child shutdown, and a final state that
  remains visible after the scenario or an early native-viewer close; and
- concise dialogs plus full local tracebacks when setup or execution fails.

Connector instances and types can be added and removed today. The actions are
selection-dependent:

1. Select **Connector Types** and use **Add connector type**.
2. Select a concrete row under **Module Types → module → Links**.
3. Use **Add connector to this link** in Properties and choose an existing
   type.
4. Select a concrete connector to edit its type, URDF body/link, location,
   axes, or custom fields, or to remove it.
5. Select a concrete connector type to edit or remove it. Referenced types
   cannot be removed.
6. Use **Save** or **Export As** to persist the in-memory edit to YAML.

Selecting only the **Connectors** group row does not expose an add/remove
action. This discoverability problem is tracked below.

### Session logging

Every Studio launch truncates and rewrites one local session log:

- with an initial pack:
  `<pack-parent>/.modsim/logs/<pack-folder>/studio.log`;
- without one: `<working-directory>/.modsim/logs/studio.log`; or
- the path in `MODSIM_STUDIO_LOG`, when set.

Logs contain startup, open/import/save/export/validation operations and full
exception tracebacks. Runtime Inspector launches use the same path and add
backend, worker or child-process, presentation, and shutdown diagnostics. In a
dual-window run the Qt process is the sole log writer and captures the viewer
child's diagnostics, so the child cannot race to truncate the same file. The
`.modsim/` directory is ignored by Git. Reproduce one problem per launch when
collecting a clean debugging log.

### Examples and tests

- `examples/robot_packs/generic_cube` is the committed simulator-neutral
  example. `examples/robot_packs/smores_ep` is the committed real-platform
  example with its complete URDF, visual meshes, collision proxies, semantics,
  backend mapping, and default runtime module-topology recipe.
- Test modules also cover transforms, acceptance, assembly derivation, docking
  lifecycle, pair and seven-module reconfiguration scenarios, backend
  registration/conformance, MuJoCo scene compilation, weld allocation,
  physical dock/undock, articulated-connector
  nominal snapping, world-state revision counters, model-view schemas/factory,
  topology generation, recipe persistence, immutable inspector transport,
  stable graph presentation, native graph/event widgets, reusable inspector
  execution, versioned process-protocol framing, process-controller launch and
  failure handling, viewer-host control and cleanup, worker shutdown, a real
  MuJoCo graph/event dock, a real seven-module/six-weld/four-action
  reconfiguration using public synthetic assets, and Studio project mutations.
- The complete feature-branch verification passes 421 tests with native
  MuJoCo enabled and reports 85% branch-aware core coverage.
- The CI Studio smoke opens the generic cube in a real Qt/PyVista window under
  Xvfb, selects a module tree item, and verifies session logging. The Studio CI
  job also runs the Runtime Inspector widget, worker, and process-controller
  tests; the MuJoCo job verifies that a real dock produces the expected graph
  edge and event delta and exercises the passive-viewer wrapper and native
  viewer host without opening a real display.

The committed SMORES-EP pack at `examples/robot_packs/smores_ep` has a
provisional genderless `ep_face` type; bottom, pan, left, and right connector
instances; docking/undocking capabilities; five Fusion-exported STL visual
meshes; and four 12-triangle OBJ collision proxies. The fixed `bottom`
connector is authored on the rear `-X` plane of `base_link`, opposite `pan`,
with its proxy thin dimension aligned to X. All four same-face pairs complete a
headless MuJoCo dock in the connector-pair scenario. The seven-module
Driver-to-Snake preset also completes all four edge replacements against this
pack with six final welds, no docking failures, and finite module state.

## Actual repository map

```text
.github/workflows/ci.yml       CI matrix, native Studio smoke, package smoke
AGENTS.md                      repository-level pointer for coding agents
README.md                      user-facing installation and quick start
docs/AGENTS.md                 architectural implementation rules
docs/HANDOFF.md                future design roadmap, not current inventory
docs/IMPLEMENTATION_STATUS.md  this canonical continuation snapshot
docs/model_views.md            generated-view contracts and recipe workflow
docs/robot_pack_spec.md        implemented format-0.1 contract
docs/runtime_inspector.md      live graph/event workflow and thread boundary
docs/studio.md                 current desktop workflow and limitations
examples/robot_packs/          committed generic-cube and SMORES-EP examples
src/modsim/cli.py              Typer CLI
src/modsim/core/               runtime identifiers, transforms, state, events, assemblies
src/modsim/model_views/        immutable view DTOs, builders, factory, topology graph
src/modsim/connectors/         compatibility, acceptance, guards, docking execution
src/modsim/backends/           adapter contract, registry, mock backend
src/modsim/runtime/            session, metrics, pair/reconfiguration scenarios,
                               named presets, inspector runner/frames/protocol
src/modsim/importers/          URDF parser and draft builder
src/modsim/robot_packs/        schema, loader, validator, writer, issues
src/modsim_backend_mujoco/     optional scene, adapter, weld pool, viewer/process host
src/modsim_studio/app.py       Qt application lifecycle
src/modsim_studio/main_window.py
                               menus, tree, panels, Properties editors
src/modsim_studio/project.py   GUI-independent immutable editing facade
src/modsim_studio/session_logging.py
                               per-launch local logging
src/modsim_studio/viewport.py  PyVista rendering and mesh picking
src/modsim_studio/runtime_*.py worker/process controllers, presenter, widgets, window
tests/                         core, importer, CLI, and project-model tests
pyproject.toml                 package metadata, extras, tool configuration
pyright-mujoco.json            optional-backend type-check configuration
pyright-studio.json            strict Studio type-check configuration
uv.lock                        committed dependency lock
```

`HANDOFF.md` uses some future package names and layouts that are not literal.
The implemented runtime and backend namespaces above are authoritative.

## Known defects

These are implementation bugs or unsafe UX behaviors, not just future feature
requests. IDs are provided so later work and commits can refer to them.

### High priority

#### PACK-001: connector parent links and source names are not URDF-validated

The normal Add flow derives `parent_link` from a real imported link, and the
Studio project model now rejects connector parent-body edits that are absent
from its imported URDF. `RobotPackValidator` still does not parse the referenced
mechanical source, so externally edited YAML can contain stale connector
parents. The validator likewise does not verify mapping source link/joint
names, named frames, or module root links against the URDF.

Required fix: add an optional reusable mechanical-source validation pass that
loads each referenced URDF once and reports missing link, joint, frame, and
mapping names. Reuse it in CLI validation as well as Studio.

### Medium priority

#### STUDIO-004: connector actions are difficult to discover

Add appears only on a selected concrete link; edit/remove appears only on a
selected concrete connector. Category nodes, menus, toolbars, and context menus
offer no equivalent actions.

#### STUDIO-005: connector identity cannot be renamed

Type and parent URDF body/link can now be changed in Properties. The stable
connector ID remains read-only; renaming still requires remove and recreate,
with no confirmation and no undo.

#### STUDIO-007: incomplete 3D connector interaction

Rendered connector glyphs are not pickable. Placement is numeric only; there
is no translation/rotation gizmo. A connector with only a named frame cannot
be rendered because Studio does not resolve that frame.

#### STUDIO-008: acceptance geometry is not visualized

Acceptance-region values are editable but not drawn in the viewport.

#### STUDIO-009: document edits reset the viewport

A document edit rebuilds the scene and resets the camera. A multi-module
document returns to the first renderable module rather than preserving the
viewed module.

#### STUDIO-010: native mutation paths lack regression coverage

Project-model tests exercise connector/type addition, deletion, reassociation,
custom metadata, and YAML persistence, but no automated MainWindow/viewport
tests cover the corresponding dialogs, tree selection, project switching, or
3D interaction.

## Defects resolved in the current snapshot

- **STUDIO-001:** tree entities have stable selection keys; project/category
  changes clear stale Properties, and mutations restore a valid new/current/
  parent selection.
- **STUDIO-002:** the connector editor exposes named-frame and numeric-local-
  pose modes and no longer silently converts one into the other.
- **STUDIO-003:** connector types are created explicitly, connector dialogs
  select existing types, and deletion is reference-safe rather than inferred
  from free text.
- **STUDIO-006:** Add Connector and Add Connector Type retain their field
  values and reopen after invalid input. Identifier errors now explain the
  lowercase snake_case convention and display-name split; other Pydantic
  failures use field-oriented dialog text while the session log records
  structured diagnostic details and the traceback.
- The Studio project boundary now rejects connector body/link associations
  absent from the imported URDF and removes obsolete connector-frame mappings
  when a connector is deleted.
- **MUJOCO-VIS-001:** URDF visual geometry is retained by default instead of
  being discarded by MuJoCo. Separate collision geoms remain active for
  physics in hidden viewer group 3, so lightweight proxies no longer cover the
  detailed robot meshes.
- **SMORES-PACK-001:** the committed pack's fixed `bottom` connector and collision
  proxy now occupy the rear `-X` mating plane of `base_link`. They were
  previously placed on the underside (`-Z`), which made TOP/BOTTOM connections
  kink vertically and distorted the Driver-to-Snake demonstration. After the
  correction, simulation-profile validation is clean, a real MuJoCo
  bottom-to-bottom dock/release completes without failures, and the committed
  seven-module preset completes as one collinear six-edge chain with ten docks,
  four undocks, and zero failures.

## Known core limitations and technical debt

These are either deliberate format-0.1 boundaries or work not yet implemented:

- Connector compatibility is reciprocal and gender-aware at runtime.
  Connector `active` and capabilities remain declarative; `supports_undocking`,
  physical connection intent, and docking policy are consumed.
- Fixed connections run on both backends. The MuJoCo adapter explicitly refuses
  compliant, hinge, ball, and custom requests; compliant stiffness translation
  and the other constraint types are not implemented.
- The MuJoCo adapter does not yet honor the complete backend mapping document;
  its first slice expects identity-compatible URDF/MJCF body and joint names.
- Named-frame-only connectors are refused by MuJoCo until frame mappings are
  resolved. Connectors with a numeric `local_pose` are materialized as measured
  MuJoCo sites.
- There is no joint-command/actuator API. Pair demos drive a module's root free
  joint; the multi-module demo kinematically stages complete components. These
  are scenario controls, not SMORES wheel or joint actuators.
- MuJoCo does not report equality-constraint forces, so connector break-force
  release and load metrics remain dormant on that backend.
- Welded module contact exclusion is not implemented. Flush lightweight
  collision proxies are suitable for the current demo; recessed or
  interpenetrating connector geometry can make the solver fight the weld.
- Pydantic models are frozen, but keyed dictionaries are shallow-mutable.
  Loader/validator/writer defensively revalidate; callers should still treat
  loaded packs as immutable.
- One draft import creates one module type from one URDF. There is no
  multi-URDF pack-builder workflow.
- The importer does not expand Xacro, fetch network assets, import
  transmissions, preserve material semantics beyond solid URDF RGBA values and
  deferred texture filenames, or import full inertial tensors.
- Mesh files are copied, but external OBJ/DAE material and texture sidecars
  are not comprehensively discovered.
- Studio does not yet display image textures, normal maps, or backend-native
  material graphs; its improved material path is intentionally solid-color
  only.
- PyVista/VTK determines which mesh formats Studio can render successfully.
- Studio edits URDF-derived metadata and semantics; it is not a CAD or
  kinematic-tree editor. Geometry and URDF kinematics must be changed in their
  source and reimported.
- Capabilities and backend mappings have schemas, validation, and YAML, but no
  complete dedicated Studio editors.
- The viewport shows one module at zero joint configuration. It has no joint
  animation, multi-module assembly view, contacts, runtime state, or physics.
- The Runtime Inspector is a separate named-demo window, not a general
  scene/dashboard shell. It has one topology renderer, one event table, live
  summary metrics, Stop, two pair presets, one seven-module preset, and
  final-state inspection; pause/restart, arbitrary user-authored scenes,
  time-series metric plots, joint commands, docking-lifecycle panels, and
  additional view renderers are not implemented.
- The Runtime Inspector window is intentionally semantic and 2D. It does not
  embed backend 3D geometry, contacts, collision-debug views, or force plots.
  The native MuJoCo viewer remains a separate companion process/window,
  particularly on macOS where it and Qt both require main-thread ownership;
  the two windows are launched and shut down together around one runtime.
- Model-view builders require explicit Python registration. Third-party entry
  point or plugin discovery, additional graph/matrix/frame views, cohorts,
  docking controllers, approach planners, and reconfiguration planners are not
  implemented.
- Canonical saves intentionally rewrite YAML formatting.
- There is no migration framework for future Robot Pack format versions.
- There is no project license, publication channel, public project URL, or
  general policy for future robot assets. The SMORES-EP assets were authorized
  for repository inclusion, but no separate asset license grants broader reuse.

## Recommended next implementation iteration

Extend the proven immutable Runtime Inspector boundary without turning it into
a second physics viewer.

1. Add pause/resume/restart through an explicit worker command channel. Commands
   must be thread-safe and must not expose a live session to Qt.
2. Add bounded PyQtGraph time-series plots for the metrics already present in
   `RuntimeInspectorFrame`, including connection/assembly counts and docking
   outcomes. Do not accumulate unbounded simulation history.
3. Generalize the implemented named-preset boundary into user-authored scene
   and controller selection. Keep shipped presets reproducible, and let future
   reconfiguration algorithms drive stable runtime commands rather than
   UI-specific state or direct pose staging.
4. Add connector lifecycle/candidate diagnostics and additional registered
   model-view renderers through the same immutable frame boundary.
5. Then return to backend mappings, mechanical-source validation, real joint
   command/actuator APIs, MuJoCo contact exclusion and constraint forces, and
   richer frame/port/matrix model views.

## SMORES-EP MuJoCo workflow

The committed example pack is:

```bash
modsim pack inspect examples/robot_packs/smores_ep
modsim pack validate examples/robot_packs/smores_ep --profile simulation
modsim views examples/robot_packs/smores_ep --view smores_topology --count 4
modsim studio examples/robot_packs/smores_ep
modsim runtime examples/robot_packs/smores_ep \
  --fixed-connector pan --moving-connector pan
modsim runtime examples/robot_packs/smores_ep \
  --demo dock_undock \
  --fixed-connector pan --moving-connector pan
modsim runtime examples/robot_packs/smores_ep \
  --demo smores_driver_to_snake
```

The last preset uses seven modules and the four connector replacements from
Figure 16 and Table III of Liu, Whitzer, and Yim's 2019 Driver-to-Snake
example. The shipped timing is deterministic sequential kinematic staging for
runtime visualization; it is not the paper's planner, actuator control, or a
physical locomotion reproduction. See `runtime_inspector.md` and
`smores_ep_mujoco.md` for exact topology, provenance, and expected metrics.

Its five detailed Fusion STL meshes are the MuJoCo visual geometry. The four
simple face proxies remain active collision geometry but start hidden in native
viewer group 3. The current visuals use one solid silver URDF material; texture
assets and richer material graphs remain future work.

Run a headless pan-face approach, fixed weld, release, and retract:

```bash
modsim run examples/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan \
  --moving-connector pan \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 1.5 \
  --undock-at 1.0
```

Replace both `pan` values with `bottom`, `left`, or `right` to exercise the
other same-face demos. On macOS, add `--view` and launch the same command with
`.venv/bin/mjpython -m modsim` so MuJoCo owns the main thread.

For live 3D physics and semantics together, use `modsim runtime` without
manually invoking `mjpython`. Qt owns the main process, ModSim automatically
uses environment-local `mjpython` for the native child on macOS, and the graph
edge appears after the same physical weld commit shown in 3D. Add
`--no-viewer` for the headless-worker transport and semantic window alone.

## Fresh-machine setup

After cloning or copying the complete repository, including `uv.lock`:

```bash
cd ModSim
uv sync --locked --extra dev --extra studio --extra mujoco --python 3.12
uv run --no-sync modsim --version
uv run --no-sync modsim backends
uv run --no-sync modsim pack validate examples/robot_packs/generic_cube
uv run --no-sync modsim views examples/robot_packs/generic_cube
uv run --no-sync modsim studio examples/robot_packs/generic_cube
```

The standard-library virtual-environment alternative is:

```bash
cd ModSim
python3.12 -m venv .venv
.venv/bin/python -m pip install -e ".[studio,dev,mujoco]"
.venv/bin/modsim --version
.venv/bin/modsim studio examples/robot_packs/generic_cube
```

Installing a built `modsim-robotics` artifact installs base dependencies
automatically. Installing `modsim-robotics[studio]` also installs the desktop
UI once the project is published. A source checkout uses editable installation
above. Do not install the unrelated PyPI package named `modsim`.

To honor the repository-only write boundary with `uv`, point its cache and any
managed Python installation into ignored repository directories:

```bash
UV_CACHE_DIR=.uv-cache UV_PYTHON_INSTALL_DIR=.uv-python \
  uv sync --locked --extra dev --extra studio --extra mujoco --python 3.12
```

For the pip alternative, use `PIP_NO_CACHE_DIR=1` during installation so pip
does not populate a user-level cache.

Studio requires a working native display/OpenGL environment. On a headless
Linux machine, follow the Xvfb approach in `.github/workflows/ci.yml`.

## Verification commands

Run these before handing work to another machine or reviewer:

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

For a packaging change, also run:

```bash
uv build
uv run --isolated --no-project --with dist/*.whl modsim --help
uv run --isolated --no-project --with dist/*.tar.gz modsim --help
```

## Continuation checklist for another developer or agent

1. Read the root `AGENTS.md` and `docs/AGENTS.md` completely.
2. Read this file, then `docs/robot_pack_spec.md`, `docs/model_views.md`,
   `docs/runtime_inspector.md`, and `docs/studio.md`.
3. Use `HANDOFF.md` only for planned architecture relevant to the next task.
4. Inspect `git status` and preserve unrelated user changes.
5. Reproduce an existing defect before changing it; retain the local Studio
   session log when useful.
6. Keep `modsim` independent from Qt, PyVista, MuJoCo, and Isaac dependencies.
7. Put UI code in `modsim_studio` and reusable document semantics in core or
   the GUI-independent `StudioProject` boundary.
8. Maintain strict validation, safe relative paths, non-overwriting export,
   transactional Save, and deterministic serialization.
9. Add tests with each substantive fix and run the complete verification set.
10. Update this file when a defect is fixed, scope changes, or a new subsystem
    becomes operational.

A useful starting request for the next implementation session is:

```text
Read docs/AGENTS.md, docs/IMPLEMENTATION_STATUS.md,
docs/robot_pack_spec.md, docs/model_views.md, docs/runtime_inspector.md,
docs/studio.md, and docs/docking_semantics.md. Extend the standalone Runtime
Inspector with pause/resume/restart and bounded metric plots over immutable
worker/process frames. Preserve lossless event delivery and stable graph
selection, do not duplicate the runtime for the native viewer, keep Qt out of
the core package, and run core, Studio, MuJoCo, packaging, and cross-backend
checks.
```
