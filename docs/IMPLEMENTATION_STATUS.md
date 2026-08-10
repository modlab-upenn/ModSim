# ModSim implementation status and continuation handoff

This is the canonical snapshot of what the repository implements today, what
is known to be broken or incomplete, and what should be built next. It is
written so that a developer or coding agent can continue from a fresh clone
without relying on prior conversation history.

## Snapshot and document authority

- Snapshot date: 2026-08-10
- Distribution: `modsim-robotics`
- Package version: `0.1.0`
- Robot Pack format: `0.1`
- Supported Python: 3.11 and 3.12; repository default: 3.12
- Branch at the start of this documentation update: `main`
- Implementation baseline commit: `badf8b2c52bc`
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
- MuJoCo is the first intended physics backend; Isaac Sim remains a later
  adapter. Neither adapter exists yet.
- The first UI is a standalone native Python desktop application using
  PySide6 and PyVistaQt. The core API must remain usable by a future browser
  frontend or native/C++ client.
- Robot assembly is not a manual CAD-style editing task. Future assemblies are
  derived at runtime from modules and connection state produced by
  reconfiguration algorithms.
- The framework must remain generic. SMORES-EP is the first real integration
  target, not a hard-coded platform.
- Do not commit SMORES CAD, URDF, meshes, or related assets until their
  redistribution terms are known.

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

The implemented vertical slice is a Robot Pack authoring system, not yet a
simulator:

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
                    |               |
                    v               v
             canonical writer   CLI reports
                    |
                    v
             ModSim Studio editor + one-module 3D preview
```

There is currently no world state, simulation clock, backend protocol,
docking execution, assembly computation, control loop, MuJoCo adapter, Isaac
adapter, or `modsim run` command. The validation profile named `simulation`
only performs stricter structural readiness checks.

## What is implemented

### Packaging and developer tooling

- A PEP 517/Hatch project with a `src/` layout and typed-package marker.
- Base installation depends only on Pydantic, Rich, ruamel.yaml, and Typer.
- The optional `studio` extra installs NumPy, PySide6, PyQtGraph, PyVista, and
  PyVistaQt.
- The optional `dev` extra installs Ruff, Pyright, pytest, pytest-cov, and
  pre-commit.
- A committed `uv.lock` provides reproducible dependency resolution.
- Console commands are `modsim` and `modsim-studio`.
- Wheels contain both `modsim` and `modsim_studio`; documentation, tests, and
  examples are included in the source distribution.
- CI checks Python 3.11 and 3.12, runs a native Studio smoke under Xvfb, builds
  wheel and source distributions, and smoke-tests both artifacts.

### Robot Pack schema

`src/modsim/robot_packs/schema.py` defines strict Pydantic v2 models for:

- the root manifest and local asset catalogs;
- module types, joints, limits, connector instances, and connector types;
- connector compatibility, orientation, acceptance regions, physical
  constraints, compliance, load limits, and undocking intent;
- bounded JSON-compatible custom metadata on the manifest, connector
  instances, and connector types;
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
```

Invalid packs return a nonzero validation exit status. There is no runtime
launch command.

### ModSim Studio

`src/modsim_studio` is an optional standalone PySide6/PyVistaQt application.
It currently provides:

- create-from-URDF, open, Save, and non-overwriting Export As workflows;
- a tree for the pack, imported assets, connector types, module types, links,
  joints, and connector instances;
- Properties editors for pack and module fields, joint metadata, connector
  instances, connector-type semantics, and custom manifest/connector/type
  metadata;
- explicit connector-type creation and reference-safe deletion;
- connector type reassignment and parent-body selection from imported URDF
  links;
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
exception tracebacks. The `.modsim/` directory is ignored by Git. Reproduce one
problem per launch when collecting a clean debugging log.

### Examples and tests

- `examples/robot_packs/generic_cube` is the only committed Robot Pack. It is
  intentionally simulator-neutral and self-contained.
- Test modules cover schemas, loading, writing, validation, CLI behavior, URDF
  import, Studio logging, and the GUI-independent Studio project model.
- The latest verification recorded for this snapshot collects 108 test cases
  from 97 test functions and reports 82% branch-aware core coverage.
- The CI Studio smoke opens the generic cube in a real Qt/PyVista window under
  Xvfb, selects a module tree item, and verifies session logging.

SMORES-EP assets and a SMORES-specific example have not been added.

## Actual repository map

```text
.github/workflows/ci.yml       CI matrix, native Studio smoke, package smoke
AGENTS.md                      repository-level pointer for coding agents
README.md                      user-facing installation and quick start
docs/AGENTS.md                 architectural implementation rules
docs/HANDOFF.md                future design roadmap, not current inventory
docs/IMPLEMENTATION_STATUS.md  this canonical continuation snapshot
docs/robot_pack_spec.md        implemented format-0.1 contract
docs/studio.md                 current desktop workflow and limitations
examples/robot_packs/          committed generic example
src/modsim/cli.py              Typer CLI
src/modsim/importers/          URDF parser and draft builder
src/modsim/robot_packs/        schema, loader, validator, writer, issues
src/modsim_studio/app.py       Qt application lifecycle
src/modsim_studio/main_window.py
                               menus, tree, panels, Properties editors
src/modsim_studio/project.py   GUI-independent immutable editing facade
src/modsim_studio/session_logging.py
                               per-launch local logging
src/modsim_studio/viewport.py  PyVista rendering and mesh picking
tests/                         core, importer, CLI, and project-model tests
pyproject.toml                 package metadata, extras, tool configuration
pyright-studio.json            strict Studio type-check configuration
uv.lock                        committed dependency lock
```

The future packages named `modsim_runtime`, `modsim_models`, and
`modsim_backends` in `HANDOFF.md` do not exist.

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

## Known core limitations and technical debt

These are either deliberate format-0.1 boundaries or work not yet implemented:

- Compatibility checks verify referenced target IDs but not reciprocal
  compatibility or meaningful gender pairing.
- Connector `active`, `gender`, `supports_undocking`, physical constraints,
  and capabilities are declarative. No runtime consumes them.
- Fixed and compliant connection descriptions are shaped for validation.
  `hinge`, `ball`, and `custom` can be named but lack the parameters and
  backend translation needed for simulation.
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
- Canonical saves intentionally rewrite YAML formatting.
- There is no migration framework for future Robot Pack format versions.
- There is no project license, publication channel, public project URL, or
  settled robot-asset redistribution policy.

## Recommended next implementation iteration

Keep the next iteration focused on making the existing authoring loop safe and
usable before adding runtime architecture.

### 1. Establish GUI regression tests

- Add stable Qt tests for category selection, project replacement, connector
  Add/Apply/Remove, type creation, Save/reopen, and named-frame preservation.
- Keep geometry-heavy cases behind an Xvfb/native smoke where appropriate.
- Consider `pytest-qt` only when it materially simplifies lifecycle and signal
  testing; add it to locked dependencies if adopted.

Acceptance: each fixed `STUDIO-*` defect has a regression test and CI stays
green on Python 3.11/3.12 plus the Studio job.

### 2. Complete mechanical-reference validation

- Parse/cache referenced URDF assets for CLI/validator cross-validation.
- Reject or clearly report missing module roots, parent links, source joints,
  named frames, and mapping names even when YAML was edited outside Studio.
- Render resolvable named-frame connectors or report why they cannot render.

Acceptance: all URDF-owned names are checked before simulation-readiness can
pass, independent of which editor produced the YAML.

### 3. Improve connector-action and dialog UX

- Add context-menu or toolbar entry points where they improve discoverability.
- Validate add-dialog input without closing and losing entered values.
- Add confirmation/undo or a clear unsaved-change recovery path for removals.
- Decide whether stable connector IDs need an explicit rename operation.
- Preserve selected module and viewport camera through semantic edits.

Acceptance: connector and type workflows are discoverable, recoverable, and do
not require re-entering data after a validation error.

### 4. Exercise the workflow with SMORES-EP locally

- Start with one redistributable or private local SMORES-EP URDF.
- Generate a draft with `modsim pack init`.
- Record all importer failures before expanding importer scope.
- Author real connector types, instances, tolerances, and mapping data.
- Validate under both profiles, Save, reopen, and visually compare frames.
- Keep private assets outside Git until redistribution is approved. A small
  synthetic fixture may be committed to reproduce a generic bug.

Acceptance: the locally held SMORES-EP pack completes connector-type creation,
URDF-body association, custom-field editing, Save/reopen, and both validation
profiles; all generic failures have tests that do not require private assets.

After these steps, add capability/mapping editor coverage. Only then begin the
roadmap's `ModelViewRegistry`, canonical runtime state, and mock backend. Use a
mock backend to settle runtime APIs before implementing MuJoCo.

## SMORES-EP first-use workflow

From a configured development environment:

```bash
modsim pack init \
  --from-urdf /local/path/to/smores_ep.urdf \
  --asset-root /local/path/to/any/package/root \
  --out /local/path/to/smores_ep_robot_pack

modsim pack inspect /local/path/to/smores_ep_robot_pack
modsim studio /local/path/to/smores_ep_robot_pack
```

Omit `--asset-root` if all mesh paths resolve relative to the URDF. In Studio,
review generated module/joint metadata, create connector types explicitly,
place connector instances, validate with both profiles, Save, close, reopen,
and validate again. A simulation-profile pass means the pack is structurally
complete; it still cannot run until a backend/runtime exists.

## Fresh-machine setup

After cloning or copying the complete repository, including `uv.lock`:

```bash
cd ModSim
uv sync --locked --extra dev --extra studio --python 3.12
uv run --no-sync modsim --version
uv run --no-sync modsim pack validate examples/robot_packs/generic_cube
uv run --no-sync modsim studio examples/robot_packs/generic_cube
```

The standard-library virtual-environment alternative is:

```bash
cd ModSim
python3.12 -m venv .venv
.venv/bin/python -m pip install -e ".[studio,dev]"
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
  uv sync --locked --extra dev --extra studio --python 3.12
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
2. Read this file, then `docs/robot_pack_spec.md` and `docs/studio.md`.
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
docs/robot_pack_spec.md, and docs/studio.md. Add stable Qt regression coverage
for connector and connector-type create/edit/remove, custom metadata,
URDF-body reassociation, category/project selection, and Save/reopen. Preserve
the core/Studio dependency boundary and run all repository checks.
```
