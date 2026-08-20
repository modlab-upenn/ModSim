# AGENTS.md — ModSim Implementation Instructions v0.4

## Read before changing code

Read `IMPLEMENTATION_STATUS.md` first for the current inventory, known bugs,
verification baseline, and recommended next iteration. Read
`robot_pack_spec.md` for the implemented format-0.1 contract, `model_views.md`
for derived-view contracts, `runtime_inspector.md` for the worker/UI boundary,
and `studio.md` for the current desktop workflow. `HANDOFF.md` is the future
architecture roadmap; do not infer that its proposed packages, APIs, CLI
commands, or milestones are implemented.

## Project goal

Implement ModSim, a Python-first backend-agnostic framework for modular
robotics, with native PySide6 Studio applications for Robot Pack authoring and
runtime inspection.

ModSim should let users:

1. start from CAD-generated URDF/meshes;
2. import those assets into ModSim;
3. interactively build a Robot Pack in ModSim Studio, a standalone desktop GUI built with PySide6/Qt and PyVistaQt/PyVista;
4. define hardware descriptions, joints, connectors, docking interfaces, capabilities, and constraints;
5. validate and save Robot Pack YAML;
6. generate mathematical model views;
7. run a simulation where MuJoCo handles physics and ModSim manages modular-robot state;
8. view modular-robot model views, events, and metrics in ModSim Studio Runtime Inspector using Qt panels and PyQtGraph plots.

## Core design rules

- ModSim is not a physics engine.
- URDF is an imported mechanical asset, not the canonical ModSim state.
- CAD or external tooling should generate URDF/meshes.
- Robot Pack YAML is the modular-robot semantic layer.
- Canonical state is `HardwareCatalog + CapabilityCatalog + WorldState + EventLog`.
- Graphs are generated views, not the foundation.
- Robot Packs store named model-view recipes, never generated graph state or
  executable Python import paths.
- Keep model-view builders and result objects backend-neutral and GUI-free;
  rendering belongs in clients such as Studio.
- All modules remain first-class even when disconnected.
- Assemblies are derived connected components.
- Cohorts are logical groups that may contain disconnected modules.
- Docking and undocking are first-class hybrid events.
- MuJoCo and Isaac Sim are backend adapters, not core dependencies.
- SMORES-like robots are examples only; keep the framework general.
- ModSim Studio is a standalone desktop app, not a web GUI for MVP.
- The primary Studio GUI stack is PySide6/Qt + PyVistaQt/PyVista + PyQtGraph.
- Do not make the primary GUI an Isaac Sim extension. Isaac integration comes later through an adapter or optional extension.
- Do not embed MuJoCo/Isaac viewer into Studio for MVP; use simulator viewer separately and Studio for semantic/model-view inspection.

## Language and style

- Use Python 3.11+ or 3.12+.
- Use type hints everywhere.
- Use Pydantic v2 for validated configuration schemas.
- Use dataclasses or Pydantic models for runtime data structures.
- Use `Enum` for lifecycle states.
- Use `Protocol` or abstract base classes for interfaces.
- Avoid unstructured dictionary-heavy core code.
- Keep `modsim-core` independent from GUI and backend packages.
- Add tests with every substantive change.

## Recommended packages

The lists below are roadmap candidates, not the current installed dependency
set. Add a package only when the active milestone requires it, and treat
`pyproject.toml` plus `uv.lock` as authoritative for what is installed now.

Core:

```text
pydantic
ruamel.yaml
orjson
numpy
scipy
networkx
typer
rich
platformdirs
```

Geometry/import:

```text
yourdfpy
trimesh
lxml
meshio
```

GUI / ModSim Studio:

```text
PySide6            # main desktop app and editor widgets
pyvista            # 3D mesh/model visualization
pyvistaqt          # embedded PyVista viewport inside Qt
pyqtgraph          # runtime metrics, matrices, plots
qtawesome optional # icons/theme only
pytest-qt          # GUI tests after shell stabilizes
```

Backend:

```text
mujoco optional
Isaac Sim later, inside Isaac's own Python environment
```

Dev:

```text
uv
ruff
pyright or mypy
pytest
pytest-cov
hypothesis
pre-commit
```

## Roadmap implementation order and current progress

1. Project skeleton and tooling — implemented.
2. Robot Pack schema, loader, writer, validator — implemented for format 0.1.
3. URDF importer and draft Robot Pack builder — implemented with the
   limitations in `IMPLEMENTATION_STATUS.md`.
4. ModSim Studio shell using PySide6 with an embedded PyVistaQt viewport —
   implemented.
5. URDF/mesh visualization, link/joint/frame tree, and selection model —
   implemented for one-module authoring, with known selection defects.
6. Connector, docking-interface, joint, and hardware constraint editors —
   partially implemented; connector lifecycle UX and capability/mapping
   editors remain incomplete.
7. Model-view recipe schema, factory/registry, and first generic module-topology
   graph — implemented and consumed by the first Runtime Inspector renderer.
8. Runtime core and mock backend — implemented.
9. Runtime Inspector mode in Studio with PyQtGraph metrics and model-view
   rendering — first standalone two-module graph/event slice implemented;
   general scenes, metric plots, and richer controls remain.
10. MuJoCo backend MVP — implemented for fixed docking constraints.
11. Docking/undocking with backend bridge — implemented for the mock and
    MuJoCo backends, with the limitations in `IMPLEMENTATION_STATUS.md`.

## ModSim Studio implementation rules

- Keep GUI code in `modsim_studio`; never import PySide6, PyVista, PyVistaQt, PyQtGraph, VTK, MuJoCo, or Isaac from `modsim_core`.
- Treat Robot Pack YAML as the editable document model.
- Use view-model or controller classes between widgets and core schema objects.
- Make user edits explicit where practical: `AddConnector`, `UpdateJointSpec`, `SetAcceptanceRegion`, `SetConnectorCompatibility`, etc.
- The main layout should start with: left project/URDF tree, center PyVistaQt viewport, right property editor, bottom validation/YAML/event panel.
- PyVistaQt overlays should show link frames, joint axes, connector frames, docking approach directions, and acceptance regions.
- Runtime Inspector should show ModSim semantics, not replace the simulator viewer: WorldState, assemblies, connections, docking lifecycle, ModelViews, metrics, and backend status.
- Runtime Inspector renderers must consume immutable generated model-view
  snapshots; they must not traverse or mutate a live `WorldState` while a
  backend is stepping.
- Use PyQtGraph for live metrics and simple matrix/model-view panels.
- Keep Studio optional through packaging extras:
  `modsim-robotics[studio]` after publication or `.[studio]` from a source
  checkout.

## Do not build first

- Do not build a full physics engine.
- Do not build CAD-to-URDF generation.
- Do not implement full Isaac Sim adapter yet.
- Do not implement a web GUI first.
- Do not implement an Isaac Sim extension as the primary UI.
- Do not build a custom CAD-to-URDF generator.
- Do not implement full behavior editor yet.
- Do not implement formal mission planning yet.
- Do not make graph the canonical state.
- Do not make the framework SMORES-specific.

## Quality bar

Every milestone should include:

```text
unit tests
clear schemas
validation errors with useful messages
sample data/fixtures
minimal CLI path
no hidden global state
```

Use this preferred Codex starting prompt for the next iteration:

```text
Read docs/AGENTS.md, docs/IMPLEMENTATION_STATUS.md,
docs/robot_pack_spec.md, docs/model_views.md, docs/runtime_inspector.md, and
docs/studio.md. Extend the standalone Runtime Inspector with pause/restart
controls and bounded live metric plots while preserving immutable worker
frames, lossless event delivery, and stable graph selection. Do not read live
WorldState from the Qt thread. Preserve the core/Studio dependency boundary and
run all repository checks.
```
